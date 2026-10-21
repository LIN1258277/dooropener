//采用TB6612芯片操控电机

#pragma once

#include "tim.h"
#include <cstdint>

/* =========================================================================
 * 速度一阶低通滤波（软件）—— 默认系数 α ∈ (0,1]
 *
 *   y[n] = y[n-1] + α * (x[n] - y[n-1])
 *
 * 为什么需要：硬件编码器输入滤波是关闭的（tim.c 里 IC1Filter = IC2Filter = 0），
 * 而且 330 count/圈、20ms 窗口下 1 个计数 = 9.09rpm —— 速度曲线是"台阶状"的，
 * 量化噪声会直接进速度环，表现为 PWM 反复翻、电机嗡嗡抖。
 *
 * 每 20ms 一拍：
 *   α = 0.15 → 时间常数 τ = dt*(1-α)/α ≈ 113ms，等效截止频率 ≈ 1.4Hz
 *   α = 0.30 → τ ≈ 47ms，截止 ≈ 3.4Hz（默认，响应与降噪的折中）
 *   α = 1.00 → 等于关闭滤波，直接用原始转速
 * 注意：α 越小越平滑，但相位滞后越大 —— 速度环的响应会变慢。
 * 改参数：直接改 SPEED_LPF_ALPHA_DEFAULT 并重新编译（串口在线调整入口已删除）。
 * ========================================================================= */
static constexpr float SPEED_LPF_ALPHA_DEFAULT = 0.3f;

/* 角度环输出（转速指令）的斜率限制，单位 rpm/s。
 * 位置 PID 在大误差时会立刻要求很高的转速，若不加限制，下发目标的瞬间
 * 转速指令就是阶跃，门会被猛拽一下。默认 200rpm/s（与旧 S 曲线的 a_max 一致），
 * 改参数请直接改这个常量。 */
static constexpr float ANGLE_SPEED_SLEW_DEFAULT = 200.0f;

typedef enum
{
    MOTOR_ERROR = -1,
    MOTOR_SUCCESS = 0
} err_t;

// ===== 控制模式 =====
typedef enum
{
    MOTOR_IDLE = 0,   // 空闲：输出 0，电机不加电
    MOTOR_SPEED,      // 速度模式：速度环 → PWM
    MOTOR_ANGLE,      // 角度模式：位置 PID → 速度环 → PWM
    MOTOR_OPENLOOP    // 开环模式：set_output() 直接给 PWM（调试用）
} motor_mode_t;




class PID
{
    public:
        PID(): speed_kp(0), speed_ki(0), speed_kd(0), angle_kp(0), angle_ki(0), angle_kd(0),
               speed_integral(0), angle_integral(0), speed_last_error(0),
               angle_last_position(0), angle_primed(false), dt(0.02f) {};
        err_t set_s_P(float P);
        err_t set_s_I(float I);
        err_t set_s_D(float D);
        err_t set_a_P(float P);
        err_t set_a_I(float I);
        err_t set_a_D(float D);
        err_t set_total_PID(float speed_kp, float speed_ki, float speed_kd, float angle_kp, float angle_ki, float angle_kd);
        // use_deadband: 误差绝对值小于速度死区时直接输出 0。
        //   速度模式传 true —— 抑制 9rpm 量化台阶导致的 PWM 来回翻 / 停机嗡嗡抖；
        //   角度模式的内环传 false —— 否则角度环给出的小转速指令会被死区吃掉，
        //   电机永远蠕动不到目标（表现为"角度环卡死"）。
        int speed_calculate(float target, float current, bool use_deadband = true);
        // 位置 PID：输入目标/当前位置 (count)，输出转速指令 (rpm)，
        // 结果限幅 ±MOTOR_SPEED_CMD_MAX，作为速度内环的设定值。
        //   u = kp*e + ki*∫e·dt - kd*(实测转速, rpm)
        //   kp: rpm/count   ki: rpm/(count·s)   kd: rpm/rpm（阻尼系数）
        // D 用「测量值微分」而不是误差微分：目标阶跃时不会产生微分冲击。
        float angle_calculate(float target, float current);
        void set_target_speed(float speed);
        void set_target_angle(float angle);
        // 清零积分与历史误差：切模式、改参数后调用，避免残留量"踢一脚"
        void reset();
        // 到位断输出（BRAKE/HOLD）期间调用：清掉两个积分余量，并把角度环的
        // 微分基准对齐到当前位置 —— 否则离开 HOLD 的那一拍会拿一个过时的
        // 位置算微分，产生一次假的"速度冲击"。
        void reset_angle(float current_position);

    private:
        float speed_kp;
        float speed_ki;
        float speed_kd;

        float angle_kp;
        float angle_ki;
        float angle_kd;

        float speed_integral;
        float angle_integral;
        float speed_last_error;
        float angle_last_position;   // 上一拍位置 (count)，用于测量值微分
        bool  angle_primed;          // false = 微分基准还没有对齐过
        float dt;
};
//电机参数：最大转速330RPM，最大扭矩5.0kg/cm,减速比30，输出分辨率为330 PPR

// ===== 角度模式的到位状态 =====
typedef enum
{
    ANGLE_RUN = 0,   // 位置 PID 正在闭环逼近目标
    ANGLE_BRAKE,     // 已进到位带，短接制动中（持续 brake_ms）
    ANGLE_HOLD       // 保持在位：断输出，靠滞环判断何时重新启动
} angle_state_t;

class Motor
{
    public:
        Motor(GPIO_TypeDef *MOTOR_DIR_GPIO_Port, uint16_t MOTOR_DIR_PIN_1, uint16_t MOTOR_DIR_PIN_2, TIM_HandleTypeDef *htim_pwm, TIM_HandleTypeDef *htim_encoder):
            target_speed(0.0f), target_angle(0.0f),
            MOTOR_DIR_GPIO_Port(MOTOR_DIR_GPIO_Port), MOTOR_DIR_PIN_1(MOTOR_DIR_PIN_1), MOTOR_DIR_PIN_2(MOTOR_DIR_PIN_2),
            htim_pwm(htim_pwm), htim_encoder(htim_encoder),
            mode(MOTOR_IDLE), speed_cmd(0.0f),
            encoder(0), enc_total(0), last_cnt(0),
            position_gate(false), pos_track(false),
            speed(0.0f), speed_raw(0.0f), speed_lpf(SPEED_LPF_ALPHA_DEFAULT),
            speed_lpf_primed(false), last_pwm(0),
            pid(), angle_state(ANGLE_HOLD), brake_start_tick(0),
            angle_slew(ANGLE_SPEED_SLEW_DEFAULT) {};
        err_t init();
        err_t forward();
        err_t backward();
        err_t stop();

        // ===== 控制模式 =====
        // 切换模式会清零 PID 的积分与历史误差，避免残留量在切换瞬间变成一记突跳。
        err_t set_mode(motor_mode_t m);
        motor_mode_t get_mode() const { return mode; }

        // ===== 设定值：纯赋值，零副作用 =====
        // 只改设定值，真正的输出统一交给 update() 在控制节拍里完成。
        void set_target_speed(float rpm);              // 速度模式的目标转速 (rpm)
        void set_target_angle(float counts);           // 角度模式的目标位置 (编码器计数)
        void set_target_angle_relative(float counts);  // 目标 = 当前位置 + counts

        // ===== 唯一执行入口 =====
        // 必须每 20ms 调一次：按 mode 选择控制律，算完直接写 PWM。
        void update();

        void set_output(int pwm);                      // 开环：直接写 PWM（调试用）

        // 观测：最近一次实际写进 CCR 的有符号 PWM
        int get_pwm() const { return last_pwm; }
        // 观测：角度模式下它是角度环输出的转速指令
        float get_speed_cmd() const { return speed_cmd; }

        // 只清 PID 内部状态，不改模式
        void reset_pid();

        // ===== 角度模式 =====
        err_t brake();                                   // 短接制动（TB6612 两 IN 都拉高）
        angle_state_t get_angle_state() const { return angle_state; }
        // 角度环输出（转速指令）的斜率限制 (rpm/s)：越小越柔和，越大跟得越快
        void set_angle_slew(float rpm_per_s);
        float get_angle_slew() const { return angle_slew; }

        // ===== 编码器 =====
        // 必须每 20ms 调用一次：根据硬件计数器算出本窗口的增量。
        // 注意：它不属于"读取"，会推进测量窗口，所以不要在查询里调它。
        void update_encoder();

        // 纯读取，无任何副作用：返回累计脉冲数（位置），查询命令随便调，
        // 不会破坏测速窗口。这也是原来速度恒为 0 的根源所在。
        int get_encoder();

        // 上一个采样窗口的脉冲增量
        int get_last_encoder() const { return encoder; }

        /* ===== 位置记录 =====
         * 编码器总存在固定的测量偏差，位置（enc_total）逐轮累加后必然越走越偏。
         * 两条措施：
         *   1) set_position_gate(true)：只有电机被实际驱动的窗口才把本窗口增量
         *      计入位置，静止窗口一概不计 —— 编码器的抖动/干扰脉冲不会再把位置
         *      慢慢"漂"出去。默认关闭（保持"始终累加"的旧行为）。
         *   2) reset_position()：在机械确实回到原点的时机（一轮开合结束）把位置
         *      清零，让每轮都从 0 重新计，误差不再跨轮累积。
         */
        void set_position_gate(bool en) { position_gate = en; }
        bool get_position_gate() const { return position_gate; }
        void reset_position();

        // 实测转速 (rpm)：一阶低通滤波后的值。控制环与上位机都用它。
        // 纯读取、无副作用 —— 观测代码随便调，不会推进滤波器。
        float get_speed();

        // 滤波前的原始转速 (rpm)：1 count = 9.09rpm 的量化台阶全在里面，
        // 只用于调试对比（看滤波到底压掉了多少抖动）。
        float get_speed_raw() const { return speed_raw; }

        // 设置一阶低通系数 α ∈ (0,1]：α=1 视为关闭滤波。
        // 非法值(≤0 或 >1)会被忽略并保持原值。
        void set_speed_lpf(float alpha);
        float get_speed_lpf() const { return speed_lpf; }

        //PID参数设置
        void set_pid(float speed_kp, float speed_ki, float speed_kd, float angle_kp, float angle_ki, float angle_kd);

        float target_speed;
        float target_angle;

    private:
        GPIO_TypeDef *MOTOR_DIR_GPIO_Port;
        uint16_t MOTOR_DIR_PIN_1;
        uint16_t MOTOR_DIR_PIN_2;

        TIM_HandleTypeDef *htim_pwm;
        TIM_HandleTypeDef *htim_encoder;

        motor_mode_t mode;    // 当前控制模式
        float speed_cmd;      // 角度环的输出 = 速度环的设定值（与 target_speed 分离）

        int encoder;          // 本采样窗口的脉冲增量
        int32_t enc_total;    // 累计脉冲（代表位置），受 pos_track 门控
        uint16_t last_cnt;    // 上次读到的硬件计数值，用于做差
        bool position_gate;   // true = 只在电机被驱动的窗口累加位置
        bool pos_track;       // 本窗口是否允许把增量累加进 enc_total
        float speed;          // 一阶低通后的转速 (rpm)，控制环用它
        float speed_raw;      // 本窗口未滤波的原始转速 (rpm)，仅调试观察
        float speed_lpf;      // 低通系数 α ∈ (0,1]，1 = 关闭滤波
        bool  speed_lpf_primed; // 首拍是否已用原始值初始化滤波状态
        int last_pwm;         // 最近一次写进 CCR 的有符号 PWM
        PID pid;

        angle_state_t angle_state;  // 角度模式的到位状态机
        uint32_t brake_start_tick;  // 进入 ANGLE_BRAKE 的时刻
        float angle_slew;           // 角度环输出斜率限制 (rpm/s)

        // 把有符号 PWM 落到硬件：判方向 + 限幅 + 写 CCR
        void apply_pwm(int pwm);
};

