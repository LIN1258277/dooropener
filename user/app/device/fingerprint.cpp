#include "fingerprint.h"
#include "main.h"
#include <cstring>

/* =========================================================================
 * 协议常量（《指纹模组产品用户手册 V1.5.1》）
 * ========================================================================= */
namespace {

constexpr uint8_t FP_HEAD0 = 0xEF;
constexpr uint8_t FP_HEAD1 = 0x01;
constexpr uint8_t FP_PID_ACK = 0x07;

constexpr uint8_t FP_CMD_GET_IMAGE     = 0x01; // 验证用获取图像
constexpr uint8_t FP_CMD_GEN_CHAR      = 0x02; // 生成特征
constexpr uint8_t FP_CMD_SEARCH        = 0x04; // 搜索指纹（1:N）
constexpr uint8_t FP_CMD_READ_SYS_PARA = 0x0F; // 读模组基本参数

// 确认码
constexpr uint8_t FP_ACK_OK           = 0x00;
constexpr uint8_t FP_ACK_NO_FINGER    = 0x02;
constexpr uint8_t FP_ACK_NOT_SEARCHED = 0x09;
constexpr uint8_t FP_ACK_HANGOVER     = 0x17;

// 匹配得分下限（模组自身已有安全等级，这里只做兜底过滤）
constexpr uint16_t FP_MATCH_MIN_SCORE = 10;
// 读不到库容量时的兜底搜索页数（足够覆盖当前 0/1/2 及后续新增 ID）
constexpr uint16_t FP_FALLBACK_PAGE_NUM = 100;

// 时间参数
constexpr uint32_t FP_BOOT_WAIT_MS          = 300;
constexpr uint32_t FP_PARA_TIMEOUT_MS       = 500;
constexpr uint32_t FP_IMG_TIMEOUT_MS        = 500;
constexpr uint32_t FP_GEN_TIMEOUT_MS        = 1000;
constexpr uint32_t FP_SEARCH_TIMEOUT_MS     = 2500;
constexpr uint32_t FP_SCAN_INTERVAL_MS      = 100;  // 无手指时重试间隔
constexpr uint32_t FP_RETRY_INTERVAL_MS     = 300;  // 出错退避
constexpr uint32_t FP_SEARCH_RETRY_MS       = 200;  // 未搜索到后重试间隔
constexpr uint32_t FP_FINGER_OFF_INTERVAL_MS = 200; // 等手指离开的采样间隔
constexpr uint8_t  FP_PARA_RETRY_MAX        = 3;

// 帧缓冲：最大应答为 ReadSysPara（28 字节）
constexpr uint16_t FP_MAX_FRAME = 64;
constexpr uint16_t FP_RX_BUF_SIZE = 256;

} // namespace

/* =========================================================================
 * 内部状态
 * ========================================================================= */
static Uart *s_uart = nullptr;

// 接收环形缓冲：中断里只入队，poll 里解析
static volatile uint8_t  s_rx_buf[FP_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0;
static volatile uint16_t s_rx_tail = 0;
static volatile uint32_t s_rx_overflow = 0;

// 帧解析
static uint8_t  s_frame[FP_MAX_FRAME];
static uint16_t s_frame_len = 0;
static uint16_t s_frame_total = 0;

enum parse_state_t {
    PS_HEAD0 = 0,
    PS_HEAD1,
    PS_FIXED,
    PS_DATA
};
static parse_state_t s_parse_state = PS_HEAD0;

// 最近一条应答（含确认码）
struct fp_resp_t {
    uint8_t ack;
    uint8_t len;
    uint8_t data[FP_MAX_FRAME];
};
static fp_resp_t s_resp;
static bool s_resp_ready = false;

// 状态机
static bool          s_active = false;   // false = 休眠，状态机不发送任何指令
static fp_state_t    s_state = FP_STATE_BOOT_WAIT;
static fp_inflight_t s_inflight = FP_INFLIGHT_NONE;
static uint32_t      s_deadline = 0;
static uint8_t       s_para_retry = 0;

static uint16_t s_page_num = FP_FALLBACK_PAGE_NUM;

// 匹配事件
static bool     s_match_pending = false;
static uint16_t s_last_id = 0;
static uint16_t s_last_score = 0;

// 统计
static uint32_t s_match_count = 0;
static uint32_t s_timeout_count = 0;
static uint32_t s_error_count = 0;

/* =========================================================================
 * 基础工具
 * ========================================================================= */

// 中断上下文：只把字节塞进环形缓冲
static void fp_rx_byte_isr(uint8_t b)
{
    uint16_t next = (uint16_t)((s_rx_head + 1) & (FP_RX_BUF_SIZE - 1));
    if (next == s_rx_tail) {
        s_rx_overflow++;
        return;
    }
    s_rx_buf[s_rx_head] = b;
    s_rx_head = next;
}

// 从包标识开始累加，取低 16 位
static uint16_t fp_calc_sum(const uint8_t *p, uint16_t n)
{
    uint16_t sum = 0;
    while (n--) {
        sum = (uint16_t)(sum + *p++);
    }
    return sum;
}

/**
 * @brief 组一条命令包并发送。
 * 包格式：EF 01 FF FF FF FF 01 LEN_H LEN_L CMD PARAMS... SUM_H SUM_L
 * LEN = 指令码 + 参数 + 校验和
 */
static void fp_send_cmd(uint8_t cmd, const uint8_t *params, uint8_t param_len)
{
    if (s_uart == nullptr) {
        return;
    }

    uint8_t buf[FP_MAX_FRAME];
    uint16_t total = (uint16_t)(12 + param_len);   // 9 字节头 + 指令码 + 参数 + 2 校验
    uint16_t len_field = (uint16_t)(param_len + 3);

    buf[0] = FP_HEAD0;
    buf[1] = FP_HEAD1;
    buf[2] = 0xFF;              // 设备地址默认 0xFFFFFFFF（大端）
    buf[3] = 0xFF;
    buf[4] = 0xFF;
    buf[5] = 0xFF;
    buf[6] = 0x01;              // 命令包
    buf[7] = (uint8_t)(len_field >> 8);
    buf[8] = (uint8_t)(len_field);
    buf[9] = cmd;

    for (uint8_t i = 0; i < param_len; i++) {
        buf[10 + i] = params[i];
    }

    uint16_t sum = fp_calc_sum(&buf[6], (uint16_t)(total - 8));
    buf[total - 2] = (uint8_t)(sum >> 8);
    buf[total - 1] = (uint8_t)(sum);

    s_uart->send_data(buf, total);
}

static void fp_send_search(void)
{
    uint8_t params[5];
    params[0] = 1;                              // BufferID
    params[1] = 0;                              // StartPage 高字节
    params[2] = 0;                              // StartPage 低字节
    params[3] = (uint8_t)(s_page_num >> 8);     // PageNum 高字节
    params[4] = (uint8_t)(s_page_num);          // PageNum 低字节
    fp_send_cmd(FP_CMD_SEARCH, params, sizeof(params));
}

/* =========================================================================
 * 接收解析
 * ========================================================================= */

static void fp_parse_reset(void)
{
    s_frame_len = 0;
    s_parse_state = PS_HEAD0;
}

static void fp_frame_done(void)
{
    // 只关心模组应答（包标识 0x07）
    if (s_frame[6] != FP_PID_ACK) {
        return;
    }

    uint16_t calc = fp_calc_sum(&s_frame[6], (uint16_t)(s_frame_total - 8));
    uint16_t recv = (uint16_t)(((uint16_t)s_frame[s_frame_total - 2] << 8) |
                               s_frame[s_frame_total - 1]);
    if (calc != recv) {
        s_error_count++;
        return;
    }

    // data 从确认码开始，长度 = 包长度字段 - 2
    uint8_t data_len = (uint8_t)(s_frame_total - 11);
    if (data_len > sizeof(s_resp.data)) {
        s_error_count++;
        return;
    }

    s_resp.ack = s_frame[9];
    s_resp.len = data_len;
    memcpy(s_resp.data, &s_frame[9], data_len);
    s_resp_ready = true;
}

static void fp_parse_byte(uint8_t b)
{
    switch (s_parse_state) {
    case PS_HEAD0:
        if (b == FP_HEAD0) {
            s_frame[0] = b;
            s_frame_len = 1;
            s_parse_state = PS_HEAD1;
        }
        break;

    case PS_HEAD1:
        if (b == FP_HEAD1) {
            s_frame[1] = b;
            s_frame_len = 2;
            s_parse_state = PS_FIXED;
        } else if (b == FP_HEAD0) {
            // 连续 0xEF，重新同步
            s_frame[0] = b;
            s_frame_len = 1;
        } else {
            fp_parse_reset();
        }
        break;

    case PS_FIXED:
        s_frame[s_frame_len++] = b;
        if (s_frame_len >= 9) {
            uint16_t len_field = (uint16_t)(((uint16_t)s_frame[7] << 8) | s_frame[8]);
            uint16_t total = (uint16_t)(9 + len_field);
            if (len_field < 3 || total > FP_MAX_FRAME) {
                fp_parse_reset();
            } else {
                s_frame_total = total;
                s_parse_state = PS_DATA;
            }
        }
        break;

    case PS_DATA:
        s_frame[s_frame_len++] = b;
        if (s_frame_len >= s_frame_total) {
            fp_frame_done();
            fp_parse_reset();
        }
        break;

    default:
        fp_parse_reset();
        break;
    }
}

static void fp_drain_rx(void)
{
    while (s_rx_tail != s_rx_head) {
        uint8_t b = s_rx_buf[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1) & (FP_RX_BUF_SIZE - 1));
        fp_parse_byte(b);
    }
}

/* =========================================================================
 * 状态机
 * ========================================================================= */

static void fp_go_scan(uint32_t now, uint32_t delay_ms)
{
    s_inflight = FP_INFLIGHT_NONE;
    s_state = FP_STATE_SCAN_IDLE;
    s_deadline = now + delay_ms;
}

static void fp_send_wait_ack(uint32_t now, fp_inflight_t inflight,
                             uint32_t timeout_ms, uint8_t cmd,
                             const uint8_t *params, uint8_t param_len)
{
    s_inflight = inflight;
    s_state = FP_STATE_WAIT_ACK;
    s_deadline = now + timeout_ms;
    fp_send_cmd(cmd, params, param_len);
}

static void fp_start_finger_off(uint32_t now)
{
    fp_send_wait_ack(now, FP_INFLIGHT_FINGER_OFF, FP_IMG_TIMEOUT_MS,
                     FP_CMD_GET_IMAGE, nullptr, 0);
}

static void fp_start_scan(uint32_t now)
{
    fp_send_wait_ack(now, FP_INFLIGHT_GET_IMAGE, FP_IMG_TIMEOUT_MS,
                     FP_CMD_GET_IMAGE, nullptr, 0);
}

static void fp_handle_ack(uint32_t now)
{
    uint8_t ack = s_resp.ack;

    switch (s_inflight) {
    case FP_INFLIGHT_READ_PARA:
        if (ack == FP_ACK_OK && s_resp.len >= 17) {
            // 应答参数布局：ack + EnrollTimes(2) + TempSize(2) + DataBaseSize(2) + ...
            uint16_t db_size = (uint16_t)(((uint16_t)s_resp.data[5] << 8) |
                                          s_resp.data[6]);
            if (db_size > 0 && db_size <= 1000) {
                s_page_num = db_size;
            } else {
                s_page_num = FP_FALLBACK_PAGE_NUM;
            }
            s_para_retry = 0;
            fp_go_scan(now, 50);
        } else if (++s_para_retry < FP_PARA_RETRY_MAX) {
            fp_send_wait_ack(now, FP_INFLIGHT_READ_PARA, FP_PARA_TIMEOUT_MS,
                             FP_CMD_READ_SYS_PARA, nullptr, 0);
        } else {
            s_page_num = FP_FALLBACK_PAGE_NUM;
            fp_go_scan(now, 50);
        }
        break;

    case FP_INFLIGHT_GET_IMAGE:
        if (ack == FP_ACK_OK) {
            // 图像采集成功 → 生成特征（BufferID=1）
            uint8_t buffer_id = 1;
            fp_send_wait_ack(now, FP_INFLIGHT_GEN_CHAR, FP_GEN_TIMEOUT_MS,
                             FP_CMD_GEN_CHAR, &buffer_id, 1);
        } else if (ack == FP_ACK_NO_FINGER) {
            fp_go_scan(now, FP_SCAN_INTERVAL_MS);
        } else if (ack == FP_ACK_HANGOVER) {
            fp_go_scan(now, FP_RETRY_INTERVAL_MS);
        } else {
            s_error_count++;
            fp_go_scan(now, FP_RETRY_INTERVAL_MS);
        }
        break;

    case FP_INFLIGHT_GEN_CHAR:
        if (ack == FP_ACK_OK) {
            // 特征生成成功 → 在指纹库中搜索（Search 带 5 字节参数）
            s_inflight = FP_INFLIGHT_SEARCH;
            s_state = FP_STATE_WAIT_ACK;
            s_deadline = now + FP_SEARCH_TIMEOUT_MS;
            fp_send_search();
        } else {
            s_error_count++;
            fp_go_scan(now, FP_RETRY_INTERVAL_MS);
        }
        break;

    case FP_INFLIGHT_SEARCH:
        if (ack == FP_ACK_OK && s_resp.len >= 5) {
            uint16_t id = (uint16_t)(((uint16_t)s_resp.data[1] << 8) |
                                     s_resp.data[2]);
            uint16_t score = (uint16_t)(((uint16_t)s_resp.data[3] << 8) |
                                        s_resp.data[4]);
            if (score >= FP_MATCH_MIN_SCORE) {
                s_last_id = id;
                s_last_score = score;
                s_match_count++;
                s_match_pending = true;
            }
            // 无论得分是否达标，都先等手指离开，
            // 避免同一次按压被反复匹配/反复触发。
            fp_start_finger_off(now);
        } else if (ack == FP_ACK_NOT_SEARCHED) {
            fp_go_scan(now, FP_SEARCH_RETRY_MS);
        } else if (ack == FP_ACK_HANGOVER) {
            fp_go_scan(now, FP_RETRY_INTERVAL_MS);
        } else {
            s_error_count++;
            fp_go_scan(now, FP_RETRY_INTERVAL_MS);
        }
        break;

    case FP_INFLIGHT_FINGER_OFF:
        if (ack == FP_ACK_NO_FINGER) {
            // 手指已离开，重新武装，等待下一次触摸
            fp_go_scan(now, FP_SCAN_INTERVAL_MS);
        } else {
            // 手指还在（0x00）或偶发错误，稍后再采一次
            s_inflight = FP_INFLIGHT_NONE;
            s_state = FP_STATE_FINGER_OFF_DELAY;
            s_deadline = now + FP_FINGER_OFF_INTERVAL_MS;
        }
        break;

    default:
        fp_go_scan(now, FP_RETRY_INTERVAL_MS);
        break;
    }
}

static void fp_handle_timeout(uint32_t now)
{
    s_timeout_count++;

    switch (s_inflight) {
    case FP_INFLIGHT_READ_PARA:
        if (++s_para_retry < FP_PARA_RETRY_MAX) {
            fp_send_wait_ack(now, FP_INFLIGHT_READ_PARA, FP_PARA_TIMEOUT_MS,
                             FP_CMD_READ_SYS_PARA, nullptr, 0);
        } else {
            // 读不到参数也能工作：用兜底页数继续
            s_page_num = FP_FALLBACK_PAGE_NUM;
            fp_go_scan(now, 50);
        }
        break;

    case FP_INFLIGHT_FINGER_OFF:
        // 等手指离开期间超时：继续采样，但不要退回扫描
        s_inflight = FP_INFLIGHT_NONE;
        s_state = FP_STATE_FINGER_OFF_DELAY;
        s_deadline = now + FP_FINGER_OFF_INTERVAL_MS;
        break;

    case FP_INFLIGHT_GET_IMAGE:
    case FP_INFLIGHT_GEN_CHAR:
    case FP_INFLIGHT_SEARCH:
    default:
        fp_go_scan(now, FP_RETRY_INTERVAL_MS);
        break;
    }
}

/* =========================================================================
 * 对外接口
 * ========================================================================= */

void fingerprint_init(Uart *uart)
{
    s_uart = uart;

    // 清空接收与解析状态
    s_rx_head = 0;
    s_rx_tail = 0;
    s_rx_overflow = 0;
    fp_parse_reset();
    s_resp_ready = false;

    s_inflight = FP_INFLIGHT_NONE;
    s_para_retry = 0;
    s_page_num = FP_FALLBACK_PAGE_NUM;
    s_match_pending = false;

    if (s_uart == nullptr) {
        return;
    }

    // 先挂字节回调，再启动中断接收
    s_uart->set_byte_handler(fp_rx_byte_isr);
    s_uart->init();

    // 上电默认休眠：等外部触摸中断把模组电源拉高后再唤醒
    s_active = false;
    s_state = FP_STATE_BOOT_WAIT;
    s_deadline = HAL_GetTick();
}

void fingerprint_wake(void)
{
    if (s_uart == nullptr) {
        return;
    }

    // 丢掉休眠期间线路上的噪声/残留字节
    s_rx_head = 0;
    s_rx_tail = 0;
    s_rx_overflow = 0;
    fp_parse_reset();
    s_resp_ready = false;

    s_inflight = FP_INFLIGHT_NONE;
    s_para_retry = 0;
    s_page_num = FP_FALLBACK_PAGE_NUM;
    s_match_pending = false;

    // 重新走一遍"上电等待 → 读库容量 → 扫描"
    s_state = FP_STATE_BOOT_WAIT;
    s_deadline = HAL_GetTick() + FP_BOOT_WAIT_MS;
    s_active = true;
}

void fingerprint_sleep(void)
{
    s_active = false;

    s_inflight = FP_INFLIGHT_NONE;
    s_state = FP_STATE_BOOT_WAIT;
    s_resp_ready = false;
    s_match_pending = false;

    // 清掉休眠期间的接收残留，避免唤醒后解析到半帧
    s_rx_head = 0;
    s_rx_tail = 0;
    fp_parse_reset();
}

bool fingerprint_is_active(void)
{
    return s_active;
}

void fingerprint_pause(void)
{
    s_active = false;

    s_inflight = FP_INFLIGHT_NONE;
    s_state = FP_STATE_BOOT_WAIT;
    s_resp_ready = false;
    s_match_pending = false;

    // 丢掉暂停瞬间可能还没处理完的应答/半帧
    s_rx_head = 0;
    s_rx_tail = 0;
    fp_parse_reset();
}

void fingerprint_resume(void)
{
    if (s_uart == nullptr || s_active) {
        return;
    }

    // 模组没有断电，不需要重新等待上电/读参数；
    // 先确认手指是否还按着，避免恢复瞬间对同一根手指重复匹配。
    s_rx_head = 0;
    s_rx_tail = 0;
    fp_parse_reset();
    s_resp_ready = false;
    s_inflight = FP_INFLIGHT_NONE;
    s_match_pending = false;

    s_state = FP_STATE_FINGER_OFF_DELAY;
    s_deadline = HAL_GetTick() + 50;
    s_active = true;
}

void fingerprint_poll(void)
{
    // 休眠时不发任何指令，也不解析线路噪声
    if (s_uart == nullptr || !s_active) {
        return;
    }

    fp_drain_rx();

    uint32_t now = HAL_GetTick();

    // 没有在等应答时，丢掉迟到的应答，避免状态串台
    if (s_resp_ready && s_state != FP_STATE_WAIT_ACK) {
        s_resp_ready = false;
    }

    if (s_resp_ready) {
        s_resp_ready = false;
        fp_handle_ack(now);
        return;
    }

    switch (s_state) {
    case FP_STATE_BOOT_WAIT:
        if ((int32_t)(now - s_deadline) >= 0) {
            fp_send_wait_ack(now, FP_INFLIGHT_READ_PARA, FP_PARA_TIMEOUT_MS,
                             FP_CMD_READ_SYS_PARA, nullptr, 0);
        }
        break;

    case FP_STATE_SCAN_IDLE:
        if ((int32_t)(now - s_deadline) >= 0) {
            fp_start_scan(now);
        }
        break;

    case FP_STATE_WAIT_ACK:
        if ((int32_t)(now - s_deadline) >= 0) {
            fp_handle_timeout(now);
        }
        break;

    case FP_STATE_FINGER_OFF_DELAY:
        if ((int32_t)(now - s_deadline) >= 0) {
            fp_start_finger_off(now);
        }
        break;

    default:
        fp_go_scan(now, FP_SCAN_INTERVAL_MS);
        break;
    }
}

bool fingerprint_take_match(uint16_t *page_id, uint16_t *score)
{
    if (!s_match_pending) {
        return false;
    }

    s_match_pending = false;
    if (page_id != nullptr) {
        *page_id = s_last_id;
    }
    if (score != nullptr) {
        *score = s_last_score;
    }
    return true;
}

void fingerprint_get_status(fp_status_t *status)
{
    if (status == nullptr) {
        return;
    }

    status->state         = (uint8_t)s_state;
    status->inflight      = (uint8_t)s_inflight;
    status->page_num      = s_page_num;
    status->last_id       = s_last_id;
    status->last_score    = s_last_score;
    status->match_count   = s_match_count;
    status->timeout_count = s_timeout_count;
    status->error_count   = s_error_count;
    status->rx_overflow   = s_rx_overflow;
}
