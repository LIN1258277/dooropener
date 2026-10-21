#include "motor.h"

//实现函数

err_t Motor::init(){
    if(HAL_TIM_PWM_Start(htim_pwm, TIM_CHANNEL_1) != HAL_OK){
        return MOTOR_ERROR;
    }
    if(HAL_TIM_Encoder_Start(htim_encoder, TIM_CHANNEL_ALL) != HAL_OK){
        return MOTOR_ERROR;
    }
    return MOTOR_SUCCESS;
}

int Motor::get_encoder(){
    encoder = __HAL_TIM_GET_COUNTER(htim_encoder);
    __HAL_TIM_SET_COUNTER(htim_encoder, 0);
    return encoder;
}

//参数
//实际速度 单位：rpm,     330 PPR
// 调用前需已调用 get_encoder()（每20ms），定时器计数器已复位
// speed(rpm) = 脉冲数 * 60秒/分钟 / (330脉冲/圈 * 采样周期_秒)
float Motor::get_speed(){
    // 采样周期 20ms = 0.020s
    this->speed = (float)encoder * 60.0f / (330.0f * 0.020f);
    return speed;
}


void Motor::set_speed(float tar_speed){
    //速度环控制
    //pid计算
    if(tar_speed > 0){
        forward();
    }else{
        backward();
    }
    uint32_t pwm = pid.speed_calculate(tar_speed, get_speed());
    __HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_1, pwm);
}

void Motor::set_output(int pwm){
    if(pwm > 0){
        forward();
    }
    else {backward();
        pwm = -pwm;
    }
    __HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_1, pwm);
}

//target_angle 单位：弧度
void Motor::set_angle(float tar_angle){
    //角度环控制
    this->target_speed = pid.angle_calculate(tar_angle, get_encoder());
    set_speed(target_speed);
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
    return MOTOR_SUCCESS;
}

void Motor::set_pid(float speed_kp, float speed_ki, float speed_kd, float angle_kp, float angle_ki, float angle_kd){
    pid.set_total_PID(speed_kp, speed_ki, speed_kd, angle_kp, angle_ki, angle_kd);
}


//output的范围是0-7200
int PID::speed_calculate(float target, float actual){
    float error = target - actual;
    this->speed_integral += error * this->dt;
    float derivative = (error - this->last_error) / this->dt;
    float output = this->speed_kp * error + this->speed_ki * this->speed_integral + this->speed_kd * derivative;
    if (output > 7200) output = 7200;
    if (output < 0) output = 0;
    this->last_error = error;
    return (int)output;
}

float PID::angle_calculate(float target, float current){
    float error = target - current;
    this->angle_integral += error * this->dt;
    float derivative = (error - this->last_error) / this->dt;
    float output = this->angle_kp * error + this->angle_ki * this->angle_integral + this->angle_kd * derivative;
    this->last_error = error;
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