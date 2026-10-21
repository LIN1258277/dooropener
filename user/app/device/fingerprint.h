#pragma once

#include "device/uart.h"
#include <cstdint>

/* =========================================================================
 * Synochip（指纹模组）精简协议驱动
 *
 * 通信：USART2, 57600 8N1（与协议手册默认值一致）
 * 方式：非阻塞轮询
 *     PS_GetImage(0x01) → PS_GenChar(0x02, BufferID=1) → PS_Search(0x04)
 * 匹配到任意已录入指纹（返回确认码 0x00）即产生一次 match 事件。
 *
 * 说明：
 *  - Search 的 PageNum 从上电时 PS_ReadSysPara(0x0F) 读取的"指纹库大小"得到，
 *    因此后续新增的 ID 无需改代码即可被搜到。
 *  - 一次按压只上报一次：匹配后持续采图直到返回 0x02（无手指）才恢复扫描。
 * ========================================================================= */

// 状态枚举（供 Live Watch 观察）
typedef enum {
    FP_STATE_BOOT_WAIT = 0,    // 上电后等待模组启动
    FP_STATE_SCAN_IDLE,        // 空闲，等待下一次采图
    FP_STATE_WAIT_ACK,         // 已发指令，等待应答
    FP_STATE_FINGER_OFF_DELAY  // 匹配后等手指离开（两次采图之间的间隔）
} fp_state_t;

// 在途指令（供 Live Watch 观察）
typedef enum {
    FP_INFLIGHT_NONE = 0,
    FP_INFLIGHT_READ_PARA,
    FP_INFLIGHT_GET_IMAGE,
    FP_INFLIGHT_GEN_CHAR,
    FP_INFLIGHT_SEARCH,
    FP_INFLIGHT_FINGER_OFF
} fp_inflight_t;

typedef struct {
    uint8_t  state;         // fp_state_t
    uint8_t  inflight;      // fp_inflight_t
    uint16_t page_num;      // Search 的 PageNum（= 模组指纹库容量）
    uint16_t last_id;       // 最近一次匹配的指纹 ID
    uint16_t last_score;    // 最近一次匹配得分
    uint32_t match_count;   // 匹配成功次数
    uint32_t timeout_count; // 应答超时次数
    uint32_t error_count;   // 协议/确认码错误次数
    uint32_t rx_overflow;   // 接收环形缓冲溢出次数
} fp_status_t;

/**
 * @brief 绑定串口、注册字节回调并启动接收。只需在 main_task() 中调用一次。
 *        初始化后驱动处于"休眠"状态，不会发送任何指令，直到调用 fingerprint_wake()。
 * @param uart 指纹模组所在串口（&huart2）
 */
void fingerprint_init(Uart *uart);

/**
 * @brief 唤醒模组：清空休眠期间的接收残留，重新执行上电等待与参数读取。
 *        需由外部在给模组上电（IO_OUT 拉高）后调用。
 */
void fingerprint_wake(void);

/**
 * @brief 进入休眠：停止状态机发送，并清掉未处理的匹配事件。
 *        需由外部在给模组断电（IO_OUT 拉低）前调用。
 */
void fingerprint_sleep(void);

/**
 * @brief 当前驱动是否处于唤醒（扫描）状态。
 */
bool fingerprint_is_active(void);

/**
 * @brief 暂停扫描（模组保持供电）：清空在途指令与匹配事件，
 *        暂停期间不会再产生指纹匹配。用于电机运动期间。
 */
void fingerprint_pause(void);

/**
 * @brief 恢复扫描：先发一次采图确认手指已离开，再恢复正常轮询。
 *        必须在模组仍供电时调用；已处于唤醒状态则直接返回。
 */
void fingerprint_resume(void);

/**
 * @brief 主循环中反复调用，推进非阻塞状态机。
 */
void fingerprint_poll(void);

/**
 * @brief 取走一次匹配事件（取走即清除）。
 * @return true 表示自上次调用以来发生过匹配
 */
bool fingerprint_take_match(uint16_t *page_id, uint16_t *score);

/**
 * @brief 读取调试状态。
 */
void fingerprint_get_status(fp_status_t *status);
