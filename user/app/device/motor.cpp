#include "motor.h"
#include <cmath>

static constexpr float MOTOR_PWM_MAX = 7200.0f;
// 角度外环输出的"转速指令"上限 (rpm)。门控场景必须有限速，
// 否则一个大角度误差会直接得到几千 rpm 的指令。
static constexpr float MOTOR_SPEED_CMD_MAX = 200.0f;
// 速度死区 (rpm)：|目标 - 实测| 小于它就认为"到位了"，直接断输出。
// 为什么需要：编码器 1 个计数 = 9.09rpm 的量化台阶，稳态时反馈会在目标
// 上下跳 1 个计数，PID 就会跟着把 PWM 翻来翻去 —— 表现为电机嗡嗡抖。
// 代价：稳态会有约 ±5rpm 的怠速波动（输出 0 时电机自由滑行）。
static constexpr float SPEED_DEADBAND_RPM = 5.0f;
// 角度到位死区 (count)：|目标位置 - 当前位置| 小于它就认为"到位了"。
// 为什么需要：编码器 1 count ≈ 1.09°，目标附近总差 1~2 个计数时，
// 角度环会不停推、速度环也会为了"把转速压到 0"而输出大 PWM，
// 在量化台阶上来回翻 —— 表现为到位后电机持续嗡嗡抖。
// 330 count = 1 圈，所以 10 count ≈ 10.9°。
static constexpr float MOTOR_ANGLE_BAND_CNT = 10.0f;
// 到位滞环宽度 (count)：进入 HOLD 后，必须偏出 BAND + HYST 才重新启动运动。
// 没有它的话，边界上一点点位移就会触发"推一下 → 又进带 → 再断"的来回抖。
static constexpr float MOTOR_ANGLE_HYST_CNT = 10.0f;
// 进到位带后短接制动的持续时间 (ms)
static constexpr uint32_t MOTOR_ANGLE_BRAKE_MS = 200;

// 编码器装在减速箱输出轴：330 count/圈（见仓库笔记，不要再去 ÷30）
static constexpr float ENCODER_PPR = 330.0f;
// 1 count/s = 60/330 rpm（编码器 330 count/圈）
static constexpr float RPM_PER_COUNT_S = 60.0f / ENCODER_PPR;
// 控制周期 (s)，必须与主循环的 20ms 节拍一致
static constexpr float CTRL_DT_S = 0.020f;

//实现函数

err_t Motor::init(){
    if(HAL_TIM_PWM_Start(htim_pwm, TIM_CHANNEL_1) != HAL_OK){
        return MOTOR_ERROR;
    }
    if(HAL_TIM_Encoder_Start(htim_encoder, TIM_CHANNEL_ALL) != HAL_OK){
        return MOTOR_ERROR;
    }

    // 记录起始计数值，作为第一次做过差的基准。
    // 计数器从此自由运行，再也不会被清零。
    last_cnt  = (uint16_t)__HAL_TIM_GET_COUNTER(htim_encoder);
    encoder   = 0;
    enc_total = 0;

    // 滤波状态一起清零，并标记"未初始化"：第一次 update_encoder() 会把
    // 滤波输出直接置成原始值，避免从 0 慢慢爬升造成起步测速偏低。
    speed            = 0.0f;
    speed_raw        = 0.0f;
    speed_lpf_primed = false;

    return MOTOR_SUCCESS;
}

/**
 * @brief 每 20ms 调用一次：把硬件计数器换算成本窗口的脉冲增量。
 *
 * 关键点：这里绝不写硬件计数器。
 * 原来的实现在 get_encoder() 里做了 __HAL_TIM_SET_COUNTER(..., 0)，
 * 而 get_encoder() 又被 $enc1 / $enc2 / $all / $ang1 / $ang2 这些
 * 查询命令调用 —— 于是每查询一次，硬件计数器就被清零一次，
 * 20ms 窗口里的脉冲被抹掉。只要查询频率接近 20ms（比如上位机
 * 轮询 $all），主循环读到的基本就是"刚清零后的 0"，
 * 于是速度恒为 0.000。这就是根本原因。
 */
void Motor::update_encoder(){
    uint16_t cnt   = (uint16_t)__HAL_TIM_GET_COUNTER(htim_encoder);
    int16_t  delta = (int16_t)(cnt - last_cnt);   // 16位无符号相减，自动处理回绕
    last_cnt  = cnt;
    encoder   = (int)delta;

    /* 位置只在"电机真的在转"的窗口里累加（门控开启时）。
     * 静止时编码器仍会零星跳动（振动、干扰），这些脉冲没有任何物理位移，
     * 一旦被累加，位置就会缓慢漂移，角度环随后又为了纠正这个假误差把电机推走。
     * 测速不受影响：speed_raw 用的始终是本窗口的原始增量。 */
    if (pos_track) {
        enc_total += delta;
    }

    // ---- 原始转速 (rpm)：本窗口脉冲数 → rpm ----
    //   speed = 脉冲数 * 60 / (330 * 0.020)
    // 1 count 就是 9.09rpm，这条曲线是台阶状的，量化噪声全在这里。
    speed_raw = (float)encoder * 60.0f / (ENCODER_PPR * CTRL_DT_S);

    /* ===== 一阶低通（软件滤波）=====
     *   y[n] = y[n-1] + α * (x[n] - y[n-1])
     *
     * 必须只在 update_encoder() 里推进 —— 它和 20ms 采样窗口一一对应。
     * 千万不要放进 get_speed()：$all/$spd1/$ang1 这些查询命令每秒可能来好多次，
     * 那样 α 会被重复施加，实际滤波强度随查询频率漂移，采样点也不再均匀。
     *
     * 首拍特殊处理：直接把 y 置成 x。否则从 0 开始爬升，起步那几拍测速偏低，
     * 速度环会误判"还没转起来"而加大 PWM。
     *
     * 注意：此处不做重置（切模式也不重置）—— 电机转速是连续的物理量，
     * 把滤波状态清零会让切模式瞬间出现一次假的"从 0 爬升"。
     */
    if (!speed_lpf_primed) {
        speed            = speed_raw;
        speed_lpf_primed = true;
    } else {
        speed += speed_lpf * (speed_raw - speed);
    }
}

// 纯读取，无副作用：返回累计脉冲数（位置）
int Motor::get_encoder(){
    return (int)enc_total;
}

/**
 * @brief 把位置记录清零。
 *
 * 编码器存在固定的测量偏差，脉冲增量逐轮累加后位置会越走越偏。
 * 在"机械确实回到原点"的时机（例如一次完整开合结束）调用它，
 * 让下一轮从 0 重新计，误差不再跨轮累积。
 *
 * 同时把角度环的微分基准对齐到 0：否则清零后的第一拍会拿清零前的位置
 * 做微分，产生一次假的速度冲击。
 */
void Motor::reset_position(){
    enc_total = 0;
    pid.reset_angle(0.0f);
}

//参数
//实际速度 单位：rpm,     330 PPR
// speed(rpm) = 本窗口脉冲数 * 60秒/分钟 / (330脉冲/圈 * 采样周期_秒)
//              结果再经一阶低通（在 update_encoder() 里推进）
float Motor::get_speed(){
    // 纯读取：滤波值在 update_encoder() 每 20ms 更新一次，
    // 这里重复调用不会改动任何状态，也不会额外推进滤波器。
    return speed;
}

// 一阶低通系数 α ∈ (0,1]；α=1 等价于关闭滤波
void Motor::set_speed_lpf(float alpha){
    // 非法值忽略并保持原值：α<=0 会让滤波输出冻结，α>1 会变成过冲/发散
    if (alpha <= 0.0f || alpha > 1.0f) {
        return;
    }
    speed_lpf = alpha;
}


/* =========================================================================
 * 设定值：纯赋值，零副作用
 * 串口命令只改这两个值，绝不在这里算 PID、更不写 PWM。
 * ========================================================================= */
void Motor::set_target_speed(float rpm){
    target_speed = rpm;
}

void Motor::set_target_angle(float counts){
    target_angle = counts;
}

void Motor::set_target_angle_relative(float counts){
    target_angle = (float)get_encoder() + counts;
}

/* =========================================================================
 * 模式切换
 * ========================================================================= */
err_t Motor::set_mode(motor_mode_t m){
    if (m == mode) {
        return MOTOR_SUCCESS;
    }

    // 切模式必须清 PID 状态：否则上一次留下的积分/历史误差会在切换后的
    // 第一拍直接变成 PWM 输出，表现为电机"被踢一脚"。
    pid.reset();

    // 角度环输出从 0 起步（pid.reset() 已清积分与微分基准），到位状态机回到 RUN
    speed_cmd        = 0.0f;
    angle_state      = ANGLE_RUN;
    brake_start_tick = 0;

    if (m == MOTOR_IDLE) {
        last_pwm = 0;
        stop();
    }

    mode = m;
    return MOTOR_SUCCESS;
}

/* =========================================================================
 * 唯一执行入口：每 20ms 调一次，按模式选择控制律并写 PWM
 * ========================================================================= */
void Motor::update(){
    switch (mode) {
    case MOTOR_SPEED:
        // 速度模式：目标转速 → 速度环 → PWM
        // 带速度死区：|目标-实测| <= SPEED_DEADBAND_RPM 时断输出。
        // 停机时不会为了"精确到 0"而嗡嗡抖；稳态也不会被 9rpm 的量化台阶
        // 逼着把 PWM 翻来翻去。
        speed_cmd = target_speed;
        apply_pwm(pid.speed_calculate(target_speed, get_speed(), true));
        break;

    case MOTOR_ANGLE: {
        float pos_err = target_angle - (float)get_encoder();
        float e_abs   = std::fabs(pos_err);
        uint32_t now  = HAL_GetTick();

        switch (angle_state) {
        case ANGLE_RUN: {
            // 进到位带 → 转制动
            if (e_abs <= MOTOR_ANGLE_BAND_CNT) {
                speed_cmd        = 0.0f;
                angle_state      = ANGLE_BRAKE;
                brake_start_tick = now;
                pid.reset_angle((float)get_encoder());
                brake();
                break;
            }

            // ===== 位置 PID → 转速指令 =====
            // u = akp*e + aki*∫e·dt - akd*(实测转速)，限幅 ±MOTOR_SPEED_CMD_MAX
            float cmd = pid.angle_calculate(target_angle, (float)get_encoder());

            // 输出斜率限制：把 PID 的转速指令变成斜坡，替代旧 S 曲线的
            // 加减速约束。没有它，目标一下发就是"满速阶跃"，门会被猛拽。
            float slew_step = angle_slew * CTRL_DT_S;
            if (cmd > speed_cmd + slew_step)      cmd = speed_cmd + slew_step;
            else if (cmd < speed_cmd - slew_step) cmd = speed_cmd - slew_step;
            speed_cmd = cmd;

            // 内环必须关掉速度死区：接近目标时转速指令会小于死区，
            // 一旦被吃掉就输出 0，电机会永远差一点、蠕动不进去。
            apply_pwm(pid.speed_calculate(speed_cmd, get_speed(), false));
            break;
        }

        case ANGLE_BRAKE:
            // 短接制动：持续 BRAKE_MS，确保真的停稳了再放手
            speed_cmd = 0.0f;
            pid.reset_angle((float)get_encoder());
            brake();
            if ((now - brake_start_tick) >= MOTOR_ANGLE_BRAKE_MS) {
                angle_state = ANGLE_HOLD;
            }
            break;

        case ANGLE_HOLD:
        default:
            // 保持在位：断输出。何时重新启动由滞环决定，
            // 否则在带边界上一点点位移就会"推一下 → 又进带 → 再断"地来回抖。
            // 注意：位置门控开启后 HOLD 期间位置是冻结的，外力推动不会被记录下来，
            // 所以这里的滞环实际上只在"目标被改写"（新的角度指令）时触发重启。
            speed_cmd = 0.0f;
            pid.reset_angle((float)get_encoder());
            apply_pwm(0);
            if (e_abs > (MOTOR_ANGLE_BAND_CNT + MOTOR_ANGLE_HYST_CNT)) {
                angle_state = ANGLE_RUN;
            }
            break;
        }
        break;
    }

    case MOTOR_OPENLOOP:
        // 开环：PWM 由 set_output() 直接写，这里不动
        break;

    case MOTOR_IDLE:
    default:
        speed_cmd = 0.0f;
        last_pwm  = 0;
        __HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_1, 0);
        break;
    }

    /* ===== 位置记账使能 =====
     * 只在"电机确实被驱动"的窗口里把编码器增量计入位置：
     *   ANGLE       : RUN/BRAKE 记账 —— 制动阶段转子还在惯性滑行，漏计会引入
     *                 比噪声更大的误差；HOLD 已经停稳，不记。
     *   SPEED/OPENLOOP : 仅当上一拍真的写了非零 PWM。
     *   IDLE        : 不记。
     * 门控关闭时恒为 true，保持电机1 原来的"始终累加"行为。
     */
    if (!position_gate) {
        pos_track = true;
    } else if (mode == MOTOR_ANGLE) {
        pos_track = (angle_state != ANGLE_HOLD);
    } else if (mode == MOTOR_SPEED || mode == MOTOR_OPENLOOP) {
        pos_track = (last_pwm != 0);
    } else {
        pos_track = false;
    }
}

void Motor::reset_pid(){
    pid.reset();
}

/* =========================================================================
 * 把"有符号 PWM"落到硬件：判方向 + 限幅 + 写 CCR
 * （原来 set_speed() 的尾部逻辑，现在被 speed/angle 两条路共用）
 * ========================================================================= */
void Motor::apply_pwm(int pwm){
    if (pwm > (int)MOTOR_PWM_MAX) {
        pwm = (int)MOTOR_PWM_MAX;
    } else if (pwm < -(int)MOTOR_PWM_MAX) {
        pwm = -(int)MOTOR_PWM_MAX;
    }

    last_pwm = pwm;

    if (pwm > 0) {
        forward();
    } else if (pwm < 0) {
        backward();
        pwm = -pwm;
    } else {
        // 输出为 0 时两个方向脚都拉低，避免 H 桥同侧导通
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_PIN_2, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_1, (uint32_t)pwm);
}

void Motor::set_output(int pwm){
    if (pwm > (int)MOTOR_PWM_MAX) {
        pwm = (int)MOTOR_PWM_MAX;
    } else if (pwm < -(int)MOTOR_PWM_MAX) {
        pwm = -(int)MOTOR_PWM_MAX;
    }

    last_pwm = pwm;

    if (pwm >= 0) {
        forward();
    } else {
        backward();
        pwm = -pwm;
    }
    __HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_1, (uint32_t)pwm);
}


err_t Motor::forward(){
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_PIN_1, GPIO_PIN_SET); 
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_PIN_2, GPIO_PIN_RESET);
    return MOTOR_SUCCESS;
}

err_t Motor::backward(){
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_PIN_1, GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_PIN_2, GPIO_PIN_SET);
    return MOTOR_SUCCESS;
}

err_t Motor::stop(){
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_PIN_1, GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_PIN_2, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_1, 0);
    last_pwm = 0;
    return MOTOR_SUCCESS;
}

/**
 * @brief 短接制动：TB6612 的 IN1 = IN2 = 1 会把电机绕组短路。
 *        停车距离比"滑停"短得多，用于进到位带后确保停稳。
 *
 * 注意：本板 .ioc 里没有 STBY 引脚（应该是板上直接拉高），所以没有
 * "彻底释放输出"的手段 —— PWM = 0 时 TB6612 本身就处于 short brake。
 * 也就是说 stop() / apply_pwm(0) / brake() 在这块板上都会制动，
 * 区别只是措辞与是否有明确的 IN 组合。
 */
err_t Motor::brake(){
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_PIN_2, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_1, 0);
    last_pwm = 0;
    return MOTOR_SUCCESS;
}

void Motor::set_angle_slew(float rpm_per_s){
    // 参数保护：<=0 会让转速指令永远爬不动（等于把电机锁死）
    if (rpm_per_s <= 0.0f) {
        return;
    }
    angle_slew = rpm_per_s;
}

void Motor::set_pid(float speed_kp, float speed_ki, float speed_kd, float angle_kp, float angle_ki, float angle_kd){
    pid.set_total_PID(speed_kp, speed_ki, speed_kd, angle_kp, angle_ki, angle_kd);
}


//output的范围是-7200到7200
int PID::speed_calculate(float target, float actual, bool use_deadband){
    float error = target - actual;

    // ---- 速度死区：误差足够小 → 直接断输出 ----
    // 死区内"冻结"积分（不清零）：出了死区还能立刻接上原来的稳态输出，
    // 不会被重新积起来的那一下踢一脚。last_error 照常更新，保证 D 项连续。
    if (use_deadband && std::fabs(error) <= SPEED_DEADBAND_RPM) {
        this->speed_last_error = error;
        return 0;
    }

    this->speed_integral += error * this->dt;
    float derivative = (error - this->speed_last_error) / this->dt;
    float output = this->speed_kp * error + this->speed_ki * this->speed_integral + this->speed_kd * derivative;

    // 限幅 + 条件积分（饱和时回退本拍积分，抗 windup）
    if (output > MOTOR_PWM_MAX) {
        output = MOTOR_PWM_MAX;
        this->speed_integral -= error * this->dt;
    } else if (output < -MOTOR_PWM_MAX) {
        output = -MOTOR_PWM_MAX;
        this->speed_integral -= error * this->dt;
    }

    this->speed_last_error = error;
    return (int)std::lround(output);
}

// 清零积分与历史误差：切模式/改参数后调用
void PID::reset(){
    this->speed_integral   = 0.0f;
    this->angle_integral   = 0.0f;
    this->speed_last_error = 0.0f;
    this->angle_primed     = false;   // 下一拍重新对齐角度微分基准
}

// 到位断输出（BRAKE/HOLD）期间调用：清积分 + 微分基准对齐到当前位置
void PID::reset_angle(float current_position){
    this->angle_integral      = 0.0f;
    this->speed_integral      = 0.0f;
    this->angle_last_position = current_position;
    this->angle_primed        = true;
}

float PID::angle_calculate(float target, float current){
    float error = target - current;

    // 微分基准：还没初始化时先对齐，本拍微分为 0（避免从 0 位置跳变）
    if (!this->angle_primed) {
        this->angle_last_position = current;
        this->angle_primed        = true;
    }

    // 用「测量值微分」求实测转速 (rpm)，而不是微分误差：
    // 目标阶跃时误差导数会是几千，kd 一乘就直接打满转速指令。
    float meas_speed_rpm = (current - this->angle_last_position) * RPM_PER_COUNT_S / this->dt;
    this->angle_last_position = current;

    this->angle_integral += error * this->dt;

    //   u = kp*e + ki*∫e·dt - kd*v_meas
    //   kp: rpm/count   ki: rpm/(count·s)   kd: rpm/rpm（阻尼）
    float output = this->angle_kp * error
                 + this->angle_ki * this->angle_integral
                 - this->angle_kd * meas_speed_rpm;

    // 注意：角度外环的输出是"转速指令(rpm)"，不是 PWM！
    // 按 ±MOTOR_SPEED_CMD_MAX 限幅，饱和时回退本拍积分（抗 windup）。
    if (output > MOTOR_SPEED_CMD_MAX) {
        output = MOTOR_SPEED_CMD_MAX;
        this->angle_integral -= error * this->dt;
    } else if (output < -MOTOR_SPEED_CMD_MAX) {
        output = -MOTOR_SPEED_CMD_MAX;
        this->angle_integral -= error * this->dt;
    }

    return output;
}



err_t PID::set_total_PID(float speed_kp, float speed_ki, float speed_kd, float angle_kp, float angle_ki, float angle_kd){
    this->speed_kp = speed_kp;
    this->speed_ki = speed_ki;
    this->speed_kd = speed_kd;
    this->angle_kp = angle_kp;
    this->angle_ki = angle_ki;
    this->angle_kd = angle_kd;
    return MOTOR_SUCCESS;
}