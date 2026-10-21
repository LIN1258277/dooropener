#include "main_task.h"
#include "usart.h"
#include "device/uart.h"
#include "device/fingerprint.h"
#include "stm32f103xb.h"

/* ===== PID 参数 =====
 * 原本可以在线整定（串口命令），现在固化为常量：改参数直接改这里并重新编译。
 * speed: 转速误差(rpm) → PWM        angle: 位置误差(count) → 转速指令(rpm)
 *
 * 注意：这里刻意不放 dt。dt 是 PID 内部按控制周期设定的量（PID::dt = 0.02f），
 * 写在这里只会让人误以为"改了它就能改采样周期"，实际上从来没被读走过。
 */
static constexpr float PID_SPEED_KP = 60.0f;
static constexpr float PID_SPEED_KI = 9.0f;
static constexpr float PID_SPEED_KD = 1.0f;
static constexpr float PID_ANGLE_KP = 4.0f;
static constexpr float PID_ANGLE_KI = 0.4f;
static constexpr float PID_ANGLE_KD = 0.2f;

// 指纹门电机（BIN1/BIN2 方向，TIM4 PWM，TIM3 编码器）。
// 电机1 已不再使用：TIM1/TIM2 与 AIN 引脚仍由 CubeMX 初始化，但应用层不再驱动。
Motor motor2(GPIOB, BIN1_Pin, BIN2_Pin, &htim4, &htim3);

// 指纹模组串口（USART2）。USART1 只保留 CubeMX 初始化，应用层不再使用。
Uart uart_fingerprint(&huart2);

/* =========================================================================
 * Live Watch 调试变量（只用于观察，不参与任何控制逻辑）
 *
 * 必须是 volatile：否则编译器可能把它优化进寄存器或直接删掉，
 * 而 Live Watch 是周期性从内存读值，没有固定内存地址就读不到。
 *
 * 在 VS Code 左侧 LIVE WATCH 面板里 Add To Live Watch：
 *     g_enc2_cnt   g_enc2_delta   g_enc2_rpm
 *     g_m2_mode    g_m2_pwm       g_m2_scmd   g_m2_astate
 *     g_loop_count g_fp_wake
 * ========================================================================= */
volatile uint32_t g_enc2_cnt   = 0;     // 电机2 编码器(TIM3) 计数器原始值 raw
volatile int32_t  g_enc2_delta = 0;     // 电机2 本 20ms 窗口的脉冲增量
volatile float    g_enc2_rpm   = 0.0f;  // 电机2 换算出的转速
volatile int32_t  g_m2_mode    = 0;     // 电机2 当前模式
volatile int32_t  g_m2_pwm     = 0;     // 电机2 最近写入 CCR 的有符号 PWM
volatile float    g_m2_scmd    = 0.0f;  // 电机2 角度环输出的转速指令 (rpm)
volatile int32_t  g_m2_astate  = 0;     // 电机2 到位状态 (angle_state_t: 0=RUN 1=BRAKE 2=HOLD)
volatile uint32_t g_loop_count = 0;     // 主循环心跳，停下说明程序卡住
volatile uint8_t  g_fp_wake    = 0;     // 指纹模组唤醒状态：0=休眠 1=唤醒

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
static void m2_abort_sequence(uint32_t now)
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
 *   5 秒内没有新的触摸 → fingerprint_sleep() + IO_OUT = 低
 *
 * 模组正常工作要几十 mA，是整机最大的耗电项之一，所以唤醒窗口不宜太长：
 * 5 秒足够一次按压完成"采图 → 比对"，超时就断电。
 * 每次触摸都会把 5 秒计时重置，所以按住/连续触摸期间不会中途断电。
 * ========================================================================= */
static constexpr uint32_t FP_AWAKE_TIMEOUT_MS = 5000; // 唤醒后保持时间
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
 * 控制模式已上移进 Motor 内部（motor_mode_t + Motor::update()）
 *
 * 架构：
 *     指纹事件   ──► set_target_angle() / set_mode()
 *                        （只改设定值和模式，绝不写 PWM）
 *     20ms 节拍  ──► Motor::update()
 *                        MOTOR_ANGLE  : 角度环 → 速度环 → PWM
 *                        MOTOR_IDLE   : 输出 0
 *                        MOTOR_SPEED / MOTOR_OPENLOOP 仍保留，但已无串口入口
 *
 * 原来的问题：set_speed() 既"设目标"又"立刻写 PWM"（命令一到达电机就跳变），
 * 而且主循环无条件只跑速度环（角度环从来没形成闭环）。
 * ========================================================================= */

void main_task(void){
    motor2.init();

    // 开启位置门控：只有电机真的在转的 20ms 窗口才累计位置，
    // 静止时不把编码器的抖动/干扰脉冲算进去，避免长时间待机把位置"漂"出去。
    motor2.set_position_gate(true);

    // 初始化PID参数（常量写在文件顶部，不再支持串口在线整定）
    motor2.set_pid(PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD,
                   PID_ANGLE_KP, PID_ANGLE_KI, PID_ANGLE_KD);

    // 指纹模组：USART2（57600 8N1），驱动内部注册字节回调并启动接收。
    // 上电默认休眠：IO_OUT 拉低给模组断电，等 IO_IN 触摸上升沿再唤醒。
    HAL_GPIO_WritePin(IO_OUT_GPIO_Port, IO_OUT_Pin, GPIO_PIN_RESET);
    fp_module_powered = false;
    fp_power_on_tick = HAL_GetTick();
    fingerprint_init(&uart_fingerprint);

    // 主循环
    uint32_t last_read_tick = 0;

    while (1) {
        uint32_t now = HAL_GetTick();
        g_fp_wake = HAL_GPIO_ReadPin(IO_OUT_GPIO_Port, IO_OUT_Pin) ? 1 : 0;
        g_loop_count++;   // Live Watch 心跳：停下不动说明程序卡住了

        // ===== 指纹模组唤醒/休眠：触摸触发，FP_AWAKE_TIMEOUT_MS 无新触摸后休眠 =====
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
        if (fingerprint_take_match(nullptr, nullptr)) {
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
                // 短时间内连续识别成功时，后续匹配在冷却期内一律忽略。
                m2_ready_tick = now + M2_CYCLE_COOLDOWN_MS;
                fp_power_on_tick = now;   // 识别成功后再保留 FP_AWAKE_TIMEOUT_MS
            }
        }

        // ===== 每20ms读取编码器/速度 =====
        if (now - last_read_tick >= 20) {
            last_read_tick = now;
            // update_encoder() 才是"推进测量窗口"的动作，只在这里调用
            motor2.update_encoder();

            // ===== 控制节拍：每个电机只执行一次控制律 =====
            // ANGLE 模式跑 角度环→速度环，必须放在 20ms 节拍里才是真正的闭环。
            motor2.update();

            // ===== 指纹开锁时序推进（在 update 之后判断状态，数据最新）=====
            // 电机被强制切到 IDLE（例如中止）时放弃后续的保持/回位
            if (m2_phase != M2_SEQ_IDLE && motor2.get_mode() == MOTOR_IDLE) {
                m2_phase = M2_SEQ_IDLE;
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
                    m2_abort_sequence(now);
                } else if ((int32_t)(now - m2_move_start) >= (int32_t)M2_MOVE_TIMEOUT_MS) {
                    m2_abort_sequence(now);
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
                } else if (m2_is_stalled(now, pos)) {
                    m2_abort_sequence(now);
                } else if ((int32_t)(now - m2_move_start) >= (int32_t)M2_MOVE_TIMEOUT_MS) {
                    m2_abort_sequence(now);
                }
                break;
            }

            default:
                break;
            }

            // 时序结束/中止后恢复指纹扫描；期间若模组已因唤醒超时断电，
            // 则等下一次触摸重新上电唤醒。
            if (m2_phase == M2_SEQ_IDLE && fp_module_powered &&
                !fingerprint_is_active()) {
                fingerprint_resume();
            }

            // ===== 刷新 Live Watch 的 raw 数据 =====
            // cnt 直接读寄存器原始值，不经过任何换算，最能看出
            // "编码器到底有没有脉冲进来"；delta/rpm 是软件换算的结果。
            g_enc2_cnt   = __HAL_TIM_GET_COUNTER(&htim3);
            g_enc2_delta = motor2.get_last_encoder();
            g_enc2_rpm   = motor2.get_speed();
            g_m2_mode    = (int32_t)motor2.get_mode();
            g_m2_pwm     = motor2.get_pwm();
            g_m2_scmd    = motor2.get_speed_cmd();
            g_m2_astate  = (int32_t)motor2.get_angle_state();
        }

        /* ===== 空闲休眠 =====
         * 本轮活干完了就睡，等中断再醒。能唤醒 CPU 的有：
         *   SysTick (1ms)  → HAL_GetTick() 与 20ms 控制节拍照常推进
         *   USART2 RX      → 指纹模组字节（中断里直接收，不会丢）
         *   EXTI (PA5)     → 触摸唤醒
         * 最长只睡 1ms，控制节拍和指纹协议时序都不受影响；
         * 但 CPU 不再 100% 空转（72MHz 空转约 36mA，Sleep 约 14mA）。 */
        __WFI();
    }
}
