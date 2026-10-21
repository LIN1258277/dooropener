#include "main_task.h"
#include "usart.h"
#include "device/uart.h"
#include "device/fingerprint.h"
#include "stm32f103xb.h"
#include <cstring>
#include <cstdlib>

// 注意：这里刻意不放 dt。dt 是 PID 内部按控制周期设定的量（PID::dt = 0.02f），
// 写在这里只会让人误以为"改了它就能改采样周期"，实际上从来没被读走过。
typedef struct
{
    float kp;
    float ki;
    float kd;
} PID_t;

PID_t pid_speed = {60.0, 9.0, 1.0};
PID_t pid_angle = {4.0, 0.4, 0.2};

Motor motor1(GPIOB, AIN1_Pin, AIN2_Pin, &htim1, &htim2);
Motor motor2(GPIOB, BIN1_Pin, BIN2_Pin, &htim4, &htim3);

// UART通信实例 (USART1用于Vofa+上位机通信)
Uart uart(&huart1);
Uart uart_fingerprint(&huart2);
/* =========================================================================
 * Live Watch 调试变量（只用于观察，不参与任何控制逻辑）
 *
 * 必须是 volatile：否则编译器可能把它优化进寄存器或直接删掉，
 * 而 Live Watch 是周期性从内存读值，没有固定内存地址就读不到。
 *
 * 在 VS Code 左侧 LIVE WATCH 面板里 Add To Live Watch：
 *     g_enc1_cnt   g_enc2_cnt      ← 最关键的 raw 数据
 *     g_enc1_delta g_enc2_delta
 *     g_enc1_rpm   g_loop_count
 * ========================================================================= */
volatile uint32_t g_enc1_cnt   = 0;     // 电机1 编码器(TIM2) 计数器原始值 raw
volatile uint32_t g_enc2_cnt   = 0;     // 电机2 编码器(TIM3) 计数器原始值 raw
volatile int32_t  g_enc1_delta = 0;     // 电机1 本 20ms 窗口的脉冲增量
volatile int32_t  g_enc2_delta = 0;     // 电机2 本 20ms 窗口的脉冲增量
volatile float    g_enc1_rpm   = 0.0f;  // 电机1 换算出的转速
volatile float    g_enc2_rpm   = 0.0f;  // 电机2 换算出的转速
volatile int32_t  g_m1_mode    = 0;     // 电机1 当前模式 (motor_mode_t)
volatile int32_t  g_m2_mode    = 0;     // 电机2 当前模式
volatile int32_t  g_m1_pwm     = 0;     // 电机1 最近写入 CCR 的有符号 PWM
volatile int32_t  g_m2_pwm     = 0;     // 电机2 最近写入 CCR 的有符号 PWM
volatile float    g_m1_scmd    = 0.0f;  // 电机1 角度环输出的转速指令 (rpm)
volatile float    g_m2_scmd    = 0.0f;  // 电机2 角度环输出的转速指令 (rpm)
volatile int32_t  g_m1_astate  = 0;     // 电机1 到位状态 (angle_state_t: 0=RUN 1=BRAKE 2=HOLD)
volatile int32_t  g_m2_astate  = 0;     // 电机2 到位状态
volatile uint32_t g_loop_count = 0;     // 主循环心跳，停下说明程序卡住
volatile uint8_t  g_fp_wake    = 0;     // 指纹模组唤醒状态：0=休眠 1=唤醒
// 发送到Vofa+的数据缓冲区 (6通道)
static float vofa_data[6];

/* =========================================================================
 * 指纹开锁时序（电机2）
 *
 *   指纹匹配成功
 *      → 记录当前位置 home（首次/偏离过大时）
 *      → 角度模式转到 home + 990 count
 *      → 到位（ANGLE_HOLD）后保持 M2_DWELL_MS
 *      → 角度模式返回 home
 *      → **位置清零 + 原点归零 + 切 IDLE**（一轮结束，误差不跨轮累积）
 *
 * 说明：
 *  - 编码器有固定测量偏差，位置逐轮累加会越来越偏；所以每轮回位结束后
 *    调用 motor2.reset_position() 把位置清零、复用 home = 0，下一轮从 0 重新计。
 *  - 静止（IDLE/HOLD）时角度环关闭、电机输出 0；位置也只在电机真的在转的
 *    窗口里累计（set_position_gate(true)），待机时的编码器抖动不会让位置漂移。
 *  - **识别成功的那一刻**开始 M2_CYCLE_COOLDOWN_MS 冷却；
 *    整个时序期间 + 冷却期内都会忽略新的指纹匹配，避免短时间连续触发。
 *  - 用 m2_seen_moving 过滤"目标刚下发时 state 还停在上一轮的 HOLD"
 *    造成的假到位；到位再用位置容差兜底。
 *  - 运行中做堵转检测：有输出但一段时间无编码器位移就立即断输出，
 *    避免撞限位/卡死时一直满 PWM 顶着，把电机或 TB6612 烧热。
 *  - 15s 超时同样直接断电中止，而不是继续强行驱动。
 *  - 中止（堵转/超时）时把当前位置当作新原点：位置清零、home 归零，
 *    残留行程不会带到下一轮。
 * ========================================================================= */
static constexpr float    M2_OPEN_COUNTS    = 990.0f;  // 开门行程
static constexpr uint32_t M2_DWELL_MS       = 1000;    // 到位后保持时间
static constexpr uint32_t M2_MOVE_TIMEOUT_MS = 15000;  // 单段移动超时兜底
static constexpr int32_t  M2_ARRIVE_TOL_CNT = 20;      // 到位位置容差
static constexpr uint32_t M2_CYCLE_COOLDOWN_MS = 10000; // 识别成功后 10s 内不再驱动电机
static constexpr uint32_t M2_STALL_TIMEOUT_MS = 800;   // 有输出但无位移多久判堵转
static constexpr int32_t  M2_STALL_MIN_DELTA = 2;      // 位移阈值(count)
static constexpr int32_t  M2_HOME_RESYNC_TOL_CNT = 60; // 偏离原位超过此值视为人为移动，重设原位

typedef enum {
    M2_SEQ_IDLE = 0,   // 空闲，可接受指纹触发
    M2_SEQ_OPEN,       // 正在转到 home + 990
    M2_SEQ_DWELL,      // 已到位，保持 M2_DWELL_MS
    M2_SEQ_RETURN      // 正在返回 home
} m2_seq_phase_t;

static m2_seq_phase_t m2_phase = M2_SEQ_IDLE;
static float    m2_home_angle = 0.0f;    // 固定原点（不再每轮重取，避免误差累积）
static bool     m2_home_valid = false;   // 是否已经确定过原点
static uint32_t m2_move_start = 0;       // 本段移动开始时刻
static uint32_t m2_dwell_start = 0;      // 到位时刻
static bool     m2_seen_moving = false;  // 本段是否真正离开过 HOLD
static uint32_t m2_ready_tick = 0;       // 允许下一次指纹触发的最早时刻
static int32_t  m2_stall_pos = 0;        // 堵转检测：上一采样位置
static uint32_t m2_stall_tick = 0;       // 堵转检测：上次有位移的时刻
static uint32_t m2_stall_count = 0;      // 堵转/超时中止次数（调试）

static inline int32_t m2_abs_i32(int32_t v)
{
    return (v < 0) ? -v : v;
}

// 开始一段新的移动：复位到位判定与堵转检测
static void m2_begin_move(uint32_t now)
{
    m2_seen_moving = false;
    m2_move_start = now;
    m2_stall_pos = motor2.get_encoder();
    m2_stall_tick = now;
}

// 中止整个时序并立即断电（防止堵转时持续满 PWM 把驱动/电机烧热）
static void m2_abort_sequence(uint32_t now, const char *msg)
{
    motor2.set_target_speed(0.0f);
    motor2.set_mode(MOTOR_IDLE);
    // 中止时不知道机械停在哪，就把当前位置当成新的原点：位置清零、home 归零。
    // 这样上一段残留的行程不会带到下一轮，也不会因为"偏离原点"再触发一次重同步。
    motor2.reset_position();
    motor2.set_target_angle(0.0f);
    m2_home_angle = 0.0f;
    m2_home_valid = true;
    m2_phase = M2_SEQ_IDLE;
    m2_ready_tick = now + M2_CYCLE_COOLDOWN_MS;
    m2_stall_count++;
    uart.send_text(msg);
}

// 返回 true 表示检测到堵转：处于 RUN 但连续一段时间没有编码器位移
static bool m2_is_stalled(uint32_t now, int32_t pos)
{
    if (motor2.get_angle_state() != ANGLE_RUN) {
        // HOLD/BRAKE 阶段不判堵转，并持续刷新基准
        m2_stall_pos = pos;
        m2_stall_tick = now;
        return false;
    }

    if (m2_abs_i32(pos - m2_stall_pos) >= M2_STALL_MIN_DELTA) {
        m2_stall_pos = pos;
        m2_stall_tick = now;
        return false;
    }

    return (int32_t)(now - m2_stall_tick) >= (int32_t)M2_STALL_TIMEOUT_MS;
}

/* =========================================================================
 * 指纹模组唤醒 / 休眠管理
 *
 *   上电：IO_OUT = 低（模组断电休眠，不扫描）
 *   IO_IN(PA5) 上升沿（外部触摸芯片）→ IO_OUT = 高 + fingerprint_wake()
 *   10 秒内没有新的触摸 → fingerprint_sleep() + IO_OUT = 低
 *
 * 每次触摸都会把 10 秒计时重置，所以按住/连续触摸期间不会中途断电。
 * ========================================================================= */
static constexpr uint32_t FP_AWAKE_TIMEOUT_MS = 10000; // 唤醒后保持时间
static constexpr uint32_t FP_WAKE_DEBOUNCE_MS = 0;   // 触摸信号去抖

static volatile bool     fp_wake_request = false;  // EXTI 置位，主循环处理
static volatile uint32_t fp_wake_tick = 0;         // 上次触摸时刻（去抖）
static bool              fp_module_powered = false;
static uint32_t          fp_power_on_tick = 0;

static void fp_module_power_on(uint32_t now)
{
    if (!fp_module_powered) {
        HAL_GPIO_WritePin(IO_OUT_GPIO_Port, IO_OUT_Pin, GPIO_PIN_SET);
        fingerprint_wake();
        fp_module_powered = true;
        uart.send_text("fp module wake\r\n");
    }
    // 必须用调用者传入的 now：如果这里重新 HAL_GetTick()，
    // 可能比上一行的 now 大 1ms，后面的无符号相减会下溢，
    // 导致刚上电就在同一轮循环里被判定为"已超时"并立刻断电。
    fp_power_on_tick = now;
}

static void fp_module_power_off(void)
{
    if (!fp_module_powered) {
        return;
    }
    fingerprint_sleep();
    HAL_GPIO_WritePin(IO_OUT_GPIO_Port, IO_OUT_Pin, GPIO_PIN_RESET);
    fp_module_powered = false;
    uart.send_text("fp module sleep\r\n");
}

// PA5(IO_IN) 上升沿：外部触摸芯片的唤醒信号
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == IO_IN_Pin) {
        uint32_t now = HAL_GetTick();
        if ((now - fp_wake_tick) >= FP_WAKE_DEBOUNCE_MS) {
            fp_wake_tick = now;
            fp_wake_request = true;
        }
    }
}

/* =========================================================================
 * PID 参数统一下发
 *
 * 原来只有"二进制 6×float32"那一条路会调 Motor::set_pid()，
 * 文本命令 $kp/$ki/$kd 只是把 pid_speed.kp 回显出来（读取，不写入），
 * 所以用串口助手发 $kp 100 时电机那边的 PID 根本没变。
 *
 * 现在所有改参数的入口（文本 $kp/$pid、二进制帧）都走这里下发。
 * ========================================================================= */
static void apply_pid_to_motors(void){
    motor1.set_pid(pid_speed.kp, pid_speed.ki, pid_speed.kd,
                   pid_angle.kp, pid_angle.ki, pid_angle.kd);
    motor2.set_pid(pid_speed.kp, pid_speed.ki, pid_speed.kd,
                   pid_angle.kp, pid_angle.ki, pid_angle.kd);
}

/* =========================================================================
 * 控制模式已上移进 Motor 内部（motor_mode_t + Motor::update()）
 *
 * 架构：
 *     串口命令   ──► set_target_speed() / set_target_angle() / set_mode()
 *                        （只改设定值和模式，绝不写 PWM）
 *     20ms 节拍  ──► Motor::update()
 *                        MOTOR_SPEED  : 速度环 → PWM
 *                        MOTOR_ANGLE  : 角度环 → 速度环 → PWM
 *                        MOTOR_IDLE   : 输出 0
 *                        MOTOR_OPENLOOP: PWM 由 set_output() 直给
 *
 * 原来的问题：set_speed() 既"设目标"又"立刻写 PWM"（命令一到达电机就跳变），
 * 而且主循环无条件只跑速度环（角度环从来没形成闭环）。
 * ========================================================================= */

void main_task(void){
    // 初始化电机
    motor1.init();
    motor2.init();

    // 电机2（指纹门）开启位置门控：只有电机真的在转的 20ms 窗口才累计位置，
    // 静止时不把编码器的抖动/干扰脉冲算进去，避免长时间待机把位置"漂"出去。
    // 电机1 保持原来的"始终累加"行为。
    motor2.set_position_gate(true);

    // 初始化PID参数
    motor1.set_pid(pid_speed.kp, pid_speed.ki, pid_speed.kd,
                   pid_angle.kp, pid_angle.ki, pid_angle.kd);
    motor2.set_pid(pid_speed.kp, pid_speed.ki, pid_speed.kd,
                   pid_angle.kp, pid_angle.ki, pid_angle.kd);

    // 初始化UART并启动中断接收
    uart.init();

    // 指纹模组：USART2（57600 8N1），驱动内部注册字节回调并启动接收。
    // 上电默认休眠：IO_OUT 拉低给模组断电，等 IO_IN 触摸上升沿再唤醒。
    HAL_GPIO_WritePin(IO_OUT_GPIO_Port, IO_OUT_Pin, GPIO_PIN_RESET);
    fp_module_powered = false;
    fp_power_on_tick = HAL_GetTick();
    fingerprint_init(&uart_fingerprint);

    // 主循环
    uint32_t last_send_tick = 0;
    uint32_t last_read_tick = 0;

    while (1) {
        uint32_t now = HAL_GetTick();
        g_fp_wake = HAL_GPIO_ReadPin(IO_OUT_GPIO_Port, IO_OUT_Pin) ? 1 : 0;
        g_loop_count++;   // Live Watch 心跳：停下不动说明程序卡住了

        // ===== 指纹模组唤醒/休眠：触摸触发，10 秒无新触摸后休眠 =====
        if (fp_wake_request) {
            fp_wake_request = false;
            fp_module_power_on(now);       // 已唤醒时只重置计时
        }
        fingerprint_poll();
        if (fp_module_powered &&
            (int32_t)(now - fp_power_on_tick) >= (int32_t)FP_AWAKE_TIMEOUT_MS) {
            fp_module_power_off();
        }

        // 任意已录入指纹匹配成功 → 启动一次完整开锁时序：
        // 转到 home+990 → 到位保持 M2_DWELL_MS → 返回 home。
        // 时序进行中（或电机2 正被别人驱动）时忽略新的匹配。
        uint16_t fp_id = 0;
        uint16_t fp_score = 0;
        if (fingerprint_take_match(&fp_id, &fp_score)) {
            bool m2_busy = (motor2.get_mode() == MOTOR_ANGLE) &&
                           (motor2.get_angle_state() != ANGLE_HOLD);
            if (m2_phase == M2_SEQ_IDLE && !m2_busy &&
                (int32_t)(now - m2_ready_tick) >= 0) {
                int32_t cur_pos = motor2.get_encoder();

                // 固定原点：只有首次触发、或当前位置明显偏离原点（人为移动/中止）时
                // 才重新取点。正常回位即使有 ±容差误差，也不会把误差逐轮累积。
                if (!m2_home_valid ||
                    m2_abs_i32(cur_pos - (int32_t)m2_home_angle) > M2_HOME_RESYNC_TOL_CNT) {
                    m2_home_angle = (float)cur_pos;
                    m2_home_valid = true;
                }

                motor2.set_target_angle(m2_home_angle + M2_OPEN_COUNTS);
                motor2.set_mode(MOTOR_ANGLE);
                m2_begin_move(now);
                m2_phase = M2_SEQ_OPEN;
                // 运动期间暂停指纹扫描：途中即使触摸/匹配成功也不会产生事件，
                // 避免影响当前时序和下一轮触发。
                fingerprint_pause();
                // 冷却从"识别成功"这一刻开始计时，而不是回位后：
                // 短时间内连续识别成功时，后续匹配在这 5 秒内一律忽略。
                m2_ready_tick = now + M2_CYCLE_COOLDOWN_MS;
                fp_power_on_tick = now;   // 识别成功后再保留 10 秒

                uart.send_named_int("fp_match_id", (int32_t)fp_id);
                uart.send_named_int("fp_match_score", (int32_t)fp_score);
                uart.send_named_float("m2_home_angle", m2_home_angle);
                uart.send_named_float("m2_target_angle", motor2.target_angle);
            } else {
                uart.send_text("fp match ignored: m2 sequence busy\r\n");
            }
        }

        // ===== 每20ms读取编码器/速度 =====
        if (now - last_read_tick >= 20) {
            last_read_tick = now;
            // update_encoder() 才是"推进测量窗口"的动作，只在这里调用；
            // 查询命令走的是 get_encoder()（纯读取），不会打断统计
            motor1.update_encoder();
            motor2.update_encoder();

            // ===== 控制节拍：每个电机只执行一次控制律 =====
            // SPEED 模式跑速度环；ANGLE 模式跑 角度环→速度环。
            // 必须放在 20ms 节拍里，角度环才是真正的闭环 —— 原来 set_angle()
            // 只在收到 $ang1 的那一刻算了一拍，之后电机就以那一拍算出的转速
            // 一直转下去，永远不会停在目标角度。
            motor1.update();
            motor2.update();

            // ===== 指纹开锁时序推进（在 update 之后判断状态，数据最新）=====
            // 急停（$stop / $stop2 会把模式切到 IDLE）时放弃后续的保持/回位
            if (m2_phase != M2_SEQ_IDLE && motor2.get_mode() == MOTOR_IDLE) {
                m2_phase = M2_SEQ_IDLE;
                uart.send_text("m2 sequence cancelled by stop\r\n");
            }

            switch (m2_phase) {
            case M2_SEQ_OPEN: {
                int32_t pos = motor2.get_encoder();
                int32_t tgt = (int32_t)(m2_home_angle + M2_OPEN_COUNTS);

                // 先确认真的离开过 HOLD，避免复用到上一轮的到位状态
                if (motor2.get_angle_state() != ANGLE_HOLD) {
                    m2_seen_moving = true;
                }

                if (m2_seen_moving &&
                    motor2.get_angle_state() == ANGLE_HOLD &&
                    m2_abs_i32(pos - tgt) <= M2_ARRIVE_TOL_CNT) {
                    m2_phase = M2_SEQ_DWELL;
                    m2_dwell_start = now;
                } else if (m2_is_stalled(now, pos)) {
                    // 撞限位/卡死：立即断输出，否则会一直满 PWM 顶着烧驱动
                    m2_abort_sequence(now, "m2 open stall, motor stopped\r\n");
                } else if ((int32_t)(now - m2_move_start) >= (int32_t)M2_MOVE_TIMEOUT_MS) {
                    m2_abort_sequence(now, "m2 open timeout, motor stopped\r\n");
                }
                break;
            }

            case M2_SEQ_DWELL:
                if ((int32_t)(now - m2_dwell_start) >= (int32_t)M2_DWELL_MS) {
                    motor2.set_target_angle(m2_home_angle);
                    motor2.set_mode(MOTOR_ANGLE);
                    m2_begin_move(now);
                    m2_phase = M2_SEQ_RETURN;
                }
                break;

            case M2_SEQ_RETURN: {
                int32_t pos = motor2.get_encoder();
                int32_t home = (int32_t)m2_home_angle;

                if (motor2.get_angle_state() != ANGLE_HOLD) {
                    m2_seen_moving = true;
                }

                if (m2_seen_moving &&
                    motor2.get_angle_state() == ANGLE_HOLD &&
                    m2_abs_i32(pos - home) <= M2_ARRIVE_TOL_CNT) {
                    // 一轮"开出 → 保持 → 返回"完成：先把位置记录清零，再断电回 IDLE。
                    // 编码器每轮都有微小测量偏差，位置若一直累加，转动次数多了以后
                    // 记录位置会严重偏离机械原位；清零后每轮都从 0 重新计。
                    // 静止后角度环完全关闭、输出 0，既不追外力也不会嗡嗡抖。
                    motor2.reset_position();
                    motor2.set_target_angle(0.0f);
                    motor2.set_mode(MOTOR_IDLE);
                    m2_home_angle = 0.0f;
                    m2_home_valid = true;
                    m2_phase = M2_SEQ_IDLE;
                    uart.send_text("m2 cycle done, position reset\r\n");
                } else if (m2_is_stalled(now, pos)) {
                    m2_abort_sequence(now, "m2 return stall, motor stopped\r\n");
                } else if ((int32_t)(now - m2_move_start) >= (int32_t)M2_MOVE_TIMEOUT_MS) {
                    m2_abort_sequence(now, "m2 return timeout, motor stopped\r\n");
                }
                break;
            }

            default:
                break;
            }

            // 时序结束/中止后恢复指纹扫描；期间若模组已因 10s 超时断电，
            // 则等下一次触摸重新上电唤醒。
            if (m2_phase == M2_SEQ_IDLE && fp_module_powered &&
                !fingerprint_is_active()) {
                fingerprint_resume();
            }

            // ===== 刷新 Live Watch 的 raw 数据 =====
            // cnt 直接读寄存器原始值，不经过任何换算，最能看出
            // "编码器到底有没有脉冲进来"；delta/rpm 是软件换算的结果。
            g_enc1_cnt   = __HAL_TIM_GET_COUNTER(&htim2);
            g_enc1_delta = motor1.get_last_encoder();
            g_enc1_rpm   = motor1.get_speed();
            g_m1_mode    = (int32_t)motor1.get_mode();
            g_m1_pwm     = motor1.get_pwm();
            g_m1_scmd    = motor1.get_speed_cmd();
            g_m1_astate  = (int32_t)motor1.get_angle_state();
            g_enc2_cnt   = __HAL_TIM_GET_COUNTER(&htim3);
            g_enc2_delta = motor2.get_last_encoder();
            g_enc2_rpm   = motor2.get_speed();
            g_m2_mode    = (int32_t)motor2.get_mode();
            g_m2_pwm     = motor2.get_pwm();
            g_m2_scmd    = motor2.get_speed_cmd();
            g_m2_astate  = (int32_t)motor2.get_angle_state();
        }

        // 通道 2/5 用 get_speed_cmd()：速度模式下它等于 target_speed，
        // 角度模式下它是角度环输出的转速指令 —— 两种模式都能看到"当前指令"
        vofa_data[0] = (float)motor1.get_last_encoder();
        vofa_data[1] = motor1.get_speed();
        vofa_data[2] = motor1.get_speed_cmd();

        vofa_data[3] = (float)motor2.get_last_encoder();
        vofa_data[4] = motor2.get_speed();
        vofa_data[5] = motor2.get_speed_cmd();

        // ===== 每20ms向Vofa+发送数据 (JustFloat格式) =====
        if (now - last_send_tick >= 20) {
            last_send_tick = now;

            // 6通道: [M1编码器, M1当前速度, M1目标速度,
            //         M2编码器, M2当前速度, M2目标速度]
            // 使用 get_last_encoder() 避免二次复位硬件计数器
           
            uart.send_justfloat(vofa_data, 6);
        }

        // ===== 检查串口是否收到新的PID参数 =====
        if (uart.has_new_pid_data()) {
            float speed_kp, speed_ki, speed_kd;
            float angle_kp, angle_ki, angle_kd;

            uart.get_pid_data(&speed_kp, &speed_ki, &speed_kd,
                               &angle_kp, &angle_ki, &angle_kd);

            // 更新本地PID参数记录
            pid_speed.kp = speed_kp;
            pid_speed.ki = speed_ki;
            pid_speed.kd = speed_kd;
            pid_angle.kp = angle_kp;
            pid_angle.ki = angle_ki;
            pid_angle.kd = angle_kd;


           
            // 应用到两个电机
            motor1.set_pid(speed_kp, speed_ki, speed_kd,
                           angle_kp, angle_ki, angle_kd);
            motor2.set_pid(speed_kp, speed_ki, speed_kd,
                           angle_kp, angle_ki, angle_kd);
        }


        // ===== 处理文本命令（串口助手/网页请求） =====
        if (uart.has_command()) {
            char cmd[32];
            uart.get_command(cmd, sizeof(cmd));

            char *arg = strchr(cmd, ' ');
            if (arg != nullptr) {
                *arg = '\0';
                arg++;
            }

            if (strcmp(cmd, "help") == 0) {
                uart.send_text("--- Available commands ---\r\n");
                uart.send_text("spd1 [val] / spd2 [val]    Speed mode + target speed (rpm)\r\n");
                uart.send_text("ang1 [val] / ang2 [val]    Angle mode + target angle (counts)\r\n");
                uart.send_text("ang1r [val] / ang2r [val]  Angle mode, target = current + val\r\n");
                uart.send_text("mode1/mode2 spd|ang|stop   Switch mode only\r\n");
                uart.send_text("stop / stop1 / stop2       Emergency stop (IDLE)\r\n");
                uart.send_text("pwm1 [val] / pwm2 [val]    Open-loop PWM (debug)\r\n");
                uart.send_text("enc1 / enc2                Motor encoder\r\n");
                uart.send_text("kp/ki/kd [val]             Speed PID (val=write, none=read)\r\n");
                uart.send_text("akp/aki/akd [val]          Angle PID (val=write, none=read)\r\n");
                uart.send_text("pid skp ski skd akp aki akd  Set all 6 PID params\r\n");
                uart.send_text("aslew [rpm/s]              Angle PID speed-cmd slew limit\r\n");
                uart.send_text("lpf [alpha]                Speed 1st-order LPF (0<a<=1, 1=off)\r\n");
                uart.send_text("fp                         Fingerprint status\r\n");
                uart.send_text("home2                      Capture current M2 position as home\r\n");
                uart.send_text("rst                        Reset PID internal state\r\n");
                uart.send_text("all                        All values\r\n");
            }
            // ===== 速度模式 + 目标转速 (rpm) =====
            else if (strcmp(cmd, "spd1") == 0 || strcmp(cmd, "spd2") == 0) {
                Motor *mot = (cmd[3] == '1') ? &motor1 : &motor2;
                if (arg != nullptr) {
                    // 只设目标 + 选模式，绝不直接写 PWM：
                    // 输出交给 20ms 节拍，否则命令一到达电机就立刻跳变
                    mot->set_target_speed((float)atof(arg));
                    mot->set_mode(MOTOR_SPEED);           // 自动切到速度模式
                    uart.send_named_float((cmd[3] == '1') ? "motor1_target_speed"
                                                           : "motor2_target_speed",
                                           mot->target_speed);
                }
                uart.send_named_float((cmd[3] == '1') ? "motor1_speed" : "motor2_speed",
                                       mot->get_speed());
            }
            else if (strcmp(cmd, "enc1") == 0) {
                uart.send_named_int("motor1_encoder", motor1.get_encoder());
            }
            else if (strcmp(cmd, "enc2") == 0) {
                uart.send_named_int("motor2_encoder", motor2.get_encoder());
            }
            // ===== 角度模式 + 目标位置（绝对，单位 = 编码器计数）=====
            else if (strcmp(cmd, "ang1") == 0 || strcmp(cmd, "ang2") == 0) {
                Motor *mot = (cmd[3] == '1') ? &motor1 : &motor2;
                if (arg != nullptr) {
                    // 只设目标；闭环由 20ms 节拍里的 Motor::update() 持续推进
                    mot->set_target_angle((float)atof(arg));
                    mot->set_mode(MOTOR_ANGLE);
                }
                uart.send_named_float((cmd[3] == '1') ? "motor1_angle" : "motor2_angle",
                                       mot->target_angle);
                uart.send_named_float((cmd[3] == '1') ? "motor1_pos" : "motor2_pos",
                                       (float)mot->get_encoder());
            }
            // ===== 相对当前位置转动：$ang1r <counts> =====
            else if (strcmp(cmd, "ang1r") == 0 || strcmp(cmd, "ang2r") == 0) {
                Motor *mot = (cmd[3] == '1') ? &motor1 : &motor2;
                if (arg != nullptr) {
                    mot->set_target_angle_relative((float)atof(arg));
                    mot->set_mode(MOTOR_ANGLE);
                }
                uart.send_named_float((cmd[3] == '1') ? "motor1_angle" : "motor2_angle",
                                       mot->target_angle);
            }
            // ===== 只切模式、不改目标：$mode1 spd|ang|stop|open =====
            else if (strcmp(cmd, "mode1") == 0 || strcmp(cmd, "mode2") == 0) {
                Motor *mot = (cmd[4] == '1') ? &motor1 : &motor2;
                if (arg != nullptr) {
                    if (strncmp(arg, "ang", 3) == 0) {
                        // 基准同步到当前位置：否则 target_angle 还是旧值(比如 0)，
                        // 一进角度模式就以限幅速度冲过去，门被猛拽一下
                        mot->set_target_angle((float)mot->get_encoder());
                        mot->set_mode(MOTOR_ANGLE);
                    } else if (strncmp(arg, "spd", 3) == 0) {
                        // 角度模式里 target_speed 只是内环指令、值已无意义，
                        // 切换时清零，避免"切回速度模式电机自己跑起来"
                        mot->set_target_speed(0.0f);
                        mot->set_mode(MOTOR_SPEED);
                    } else if (strncmp(arg, "open", 4) == 0) {
                        mot->set_mode(MOTOR_OPENLOOP);
                    } else {
                        mot->set_target_speed(0.0f);
                        mot->set_mode(MOTOR_IDLE);        // stop / 其它
                    }
                }
                uart.send_named_int((cmd[4] == '1') ? "mode1" : "mode2",
                                     (int32_t)mot->get_mode());
            }
            // ===== 急停：$stop / $stop1 / $stop2 =====
            else if (strcmp(cmd, "stop") == 0 || strcmp(cmd, "stop1") == 0 || strcmp(cmd, "stop2") == 0) {
                if (cmd[4] != '2') { motor1.set_target_speed(0.0f); motor1.set_mode(MOTOR_IDLE); }
                if (cmd[4] != '1') { motor2.set_target_speed(0.0f); motor2.set_mode(MOTOR_IDLE); }
                uart.send_text("stopped\r\n");
            }
            // ===== 开环直给 PWM：区分"电机不转"还是"编码器不计数" =====
            else if (strcmp(cmd, "pwm1") == 0 || strcmp(cmd, "pwm2") == 0) {
                Motor *mot = (cmd[3] == '1') ? &motor1 : &motor2;
                if (arg != nullptr) {
                    mot->set_mode(MOTOR_OPENLOOP);
                    mot->set_output((int)atof(arg));
                }
                uart.send_named_int((cmd[3] == '1') ? "motor1_pwm" : "motor2_pwm",
                                     mot->get_pwm());
            }
            // ===== 清 PID 内部状态（积分/历史误差）=====
            else if (strcmp(cmd, "rst") == 0) {
                motor1.reset_pid();
                motor2.reset_pid();
                uart.send_text("pid state reset\r\n");
            }
            // ===== 速度一阶低通系数：带参数=写入，不带参数=查询 =====
            // α ∈ (0,1]：越小越平滑、滞后越大；α=1 等于关闭滤波。
            // 默认 0.30（τ≈47ms），非法值会被 Motor 忽略并保持原值。
            else if (strcmp(cmd, "lpf") == 0) {
                if (arg != nullptr) {
                    float alpha = (float)atof(arg);
                    motor1.set_speed_lpf(alpha);
                    motor2.set_speed_lpf(alpha);
                }
                uart.send_named_float("speed_lpf", motor1.get_speed_lpf());
            }
            // ===== PID 参数：带参数=写入，不带参数=查询 =====
            else if (strcmp(cmd, "kp") == 0) {
                if (arg != nullptr) { pid_speed.kp = (float)atof(arg); apply_pid_to_motors(); }
                uart.send_named_float("speed_kp", pid_speed.kp);
            }
            else if (strcmp(cmd, "ki") == 0) {
                if (arg != nullptr) { pid_speed.ki = (float)atof(arg); apply_pid_to_motors(); }
                uart.send_named_float("speed_ki", pid_speed.ki);
            }
            else if (strcmp(cmd, "kd") == 0) {
                if (arg != nullptr) { pid_speed.kd = (float)atof(arg); apply_pid_to_motors(); }
                uart.send_named_float("speed_kd", pid_speed.kd);
            }
            else if (strcmp(cmd, "akp") == 0) {
                if (arg != nullptr) { pid_angle.kp = (float)atof(arg); apply_pid_to_motors(); }
                uart.send_named_float("angle_kp", pid_angle.kp);
            }
            else if (strcmp(cmd, "aki") == 0) {
                if (arg != nullptr) { pid_angle.ki = (float)atof(arg); apply_pid_to_motors(); }
                uart.send_named_float("angle_ki", pid_angle.ki);
            }
            else if (strcmp(cmd, "akd") == 0) {
                if (arg != nullptr) { pid_angle.kd = (float)atof(arg); apply_pid_to_motors(); }
                uart.send_named_float("angle_kd", pid_angle.kd);
            }
            // ===== 一行下发全部 6 个参数：$pid skp ski skd akp aki akd =====
            else if (strcmp(cmd, "pid") == 0) {
                float v[6] = {0};
                int n = 0;
                if (arg != nullptr) {
                    for (char *t = strtok(arg, " "); t != nullptr && n < 6; t = strtok(nullptr, " ")) {
                        v[n++] = (float)atof(t);
                    }
                }
                if (n == 6) {
                    pid_speed.kp = v[0]; pid_speed.ki = v[1]; pid_speed.kd = v[2];
                    pid_angle.kp = v[3]; pid_angle.ki = v[4]; pid_angle.kd = v[5];
                    apply_pid_to_motors();
                    uart.send_text("pid updated\r\n");
                } else {
                    uart.send_text("usage: $pid skp ski skd akp aki akd\r\n");
                }
                uart.send_named_float("speed_kp", pid_speed.kp);
                uart.send_named_float("speed_ki", pid_speed.ki);
                uart.send_named_float("speed_kd", pid_speed.kd);
                uart.send_named_float("angle_kp", pid_angle.kp);
                uart.send_named_float("angle_ki", pid_angle.ki);
                uart.send_named_float("angle_kd", pid_angle.kd);
            }
            // ===== 角度环输出斜率限制：$aslew <rpm/s> =====
            // 位置 PID 算出的转速指令会经这个斜率变成斜坡，替代旧 S 曲线的加减速约束
            // 不带参数就只回显当前值（默认值写在 motor.h 的 ANGLE_SPEED_SLEW_DEFAULT）
            else if (strcmp(cmd, "aslew") == 0) {
                if (arg != nullptr) {
                    float v = (float)atof(arg);
                    motor1.set_angle_slew(v);
                    motor2.set_angle_slew(v);
                }
                uart.send_named_float("angle_slew", motor1.get_angle_slew());
            }
            else if (strcmp(cmd, "fp") == 0) {
                fp_status_t st;
                fingerprint_get_status(&st);
                uart.send_named_int("fp_state", (int32_t)st.state);
                uart.send_named_int("fp_inflight", (int32_t)st.inflight);
                uart.send_named_int("fp_page_num", (int32_t)st.page_num);
                uart.send_named_int("fp_match_count", (int32_t)st.match_count);
                uart.send_named_int("fp_last_id", (int32_t)st.last_id);
                uart.send_named_int("fp_last_score", (int32_t)st.last_score);
                uart.send_named_int("fp_timeout", (int32_t)st.timeout_count);
                uart.send_named_int("fp_error", (int32_t)st.error_count);
                uart.send_named_int("fp_rx_overflow", (int32_t)st.rx_overflow);
                uart.send_named_int("fp_powered", fp_module_powered ? 1 : 0);
                uart.send_named_int("fp_drv_active", fingerprint_is_active() ? 1 : 0);
                uart.send_named_int("fp_io_in", (HAL_GPIO_ReadPin(IO_IN_GPIO_Port, IO_IN_Pin) == GPIO_PIN_SET) ? 1 : 0);
            }
            else if (strcmp(cmd, "home2") == 0) {
                // 手动把当前位置设为电机2的原位（例如机构被人工移动过）
                m2_home_angle = (float)motor2.get_encoder();
                m2_home_valid = true;
                uart.send_named_float("m2_home_angle", m2_home_angle);
            }
            else if (strcmp(cmd, "all") == 0) {
                uart.send_named_int("m1_mode", (int32_t)motor1.get_mode());
                uart.send_named_int("m1_astate", (int32_t)motor1.get_angle_state());
                uart.send_named_int("m1_pwm", motor1.get_pwm());
                uart.send_named_float("m1_speed_cmd", motor1.get_speed_cmd());
                uart.send_named_float("m1_pos_err", motor1.target_angle - (float)motor1.get_encoder());
                uart.send_named_float("m1_speed", motor1.get_speed());
                uart.send_named_float("m1_speed_raw", motor1.get_speed_raw());
                uart.send_named_float("m1_encoder", (float)motor1.get_encoder());
                uart.send_named_float("m1_target_speed", motor1.target_speed);
                uart.send_named_float("m1_target_angle", motor1.target_angle);
                uart.send_named_int("m2_mode", (int32_t)motor2.get_mode());
                uart.send_named_int("m2_astate", (int32_t)motor2.get_angle_state());
                uart.send_named_int("m2_pwm", motor2.get_pwm());
                uart.send_named_float("m2_speed_cmd", motor2.get_speed_cmd());
                uart.send_named_float("m2_speed", motor2.get_speed());
                uart.send_named_float("m2_speed_raw", motor2.get_speed_raw());
                uart.send_named_float("m2_pos_err", motor2.target_angle - (float)motor2.get_encoder());
                uart.send_named_float("m2_encoder", (float)motor2.get_encoder());
                uart.send_named_float("m2_target_speed", motor2.target_speed);
                uart.send_named_float("m2_target_angle", motor2.target_angle);
                uart.send_named_int("m2_seq_phase", (int32_t)m2_phase);
                uart.send_named_float("m2_home_angle", m2_home_angle);
                uart.send_named_int("m2_pos_gate", motor2.get_position_gate() ? 1 : 0);
                uart.send_named_int("m2_stall_count", (int32_t)m2_stall_count);
                uart.send_named_float("speed_kp", pid_speed.kp);
                uart.send_named_float("speed_ki", pid_speed.ki);
                uart.send_named_float("speed_kd", pid_speed.kd);
                uart.send_named_float("angle_kp", pid_angle.kp);
                uart.send_named_float("angle_ki", pid_angle.ki);
                uart.send_named_float("angle_kd", pid_angle.kd);
            }
            else {
                uart.send_text("unknown cmd, send $help\r\n");
            }
        }

        
    }
}
