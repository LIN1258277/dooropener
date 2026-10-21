//采用TB6612芯片操控电机

#pragma once

#include "tim.h"
#include <cstdint>

typedef enum
{
    MOTOR_ERROR = -1,
    MOTOR_SUCCESS = 0
} err_t;




class PID
{
    public:
        PID(): speed_kp(0), speed_ki(0), speed_kd(0), angle_kp(0), angle_ki(0), angle_kd(0), speed_integral(0), angle_integral(0), last_error(0), dt(0.01f) {};
        err_t set_s_P(float P);
        err_t set_s_I(float I);
        err_t set_s_D(float D);
        err_t set_a_P(float P);
        err_t set_a_I(float I);
        err_t set_a_D(float D);
        err_t set_total_PID(float speed_kp, float speed_ki, float speed_kd, float angle_kp, float angle_ki, float angle_kd);
        int speed_calculate(float target, float current);
        float angle_calculate(float target, float current);
        void set_target_speed(float speed);
        void set_target_angle(float angle);

    private:
        float speed_kp;
        float speed_ki;
        float speed_kd;

        float angle_kp;
        float angle_ki;
        float angle_kd;

        float speed_integral;
        float angle_integral;
        float last_error;
        float dt;
       
        

};
//电机参数：最大转速330RPM，最大扭矩5.0kg/cm,减速比30，输出分辨率为330 PPR
class Motor
{
    public:
        Motor(GPIO_TypeDef *MOTOR_DIR_GPIO_Port, uint16_t MOTOR_DIR_PIN_1, uint16_t MOTOR_DIR_PIN_2, TIM_HandleTypeDef *htim_pwm, TIM_HandleTypeDef *htim_encoder): MOTOR_DIR_GPIO_Port(MOTOR_DIR_GPIO_Port), MOTOR_DIR_PIN_1(MOTOR_DIR_PIN_1), MOTOR_DIR_PIN_2(MOTOR_DIR_PIN_2), htim_pwm(htim_pwm), htim_encoder(htim_encoder), encoder(0), speed(0.0f) {};
        err_t init();
        err_t forward();
        err_t backward();
        err_t stop();
    //角度环控制电机
        void set_angle(float angle);
        //速度环控制
        void set_speed(float speed);

        void set_output(int pwm);
        //编码器
        int get_encoder();
        int get_last_encoder() const { return encoder; }  // 仅读取上次值，不复位硬件
        float get_speed();

        //PID参数设置
        void set_pid(float speed_kp, float speed_ki, float speed_kd, float angle_kp, float angle_ki, float angle_kd);

        float target_speed;
        float target_angle;
        
    private:
        int encoder;
        float speed;
        PID pid;
        GPIO_TypeDef *MOTOR_DIR_GPIO_Port;
        uint16_t MOTOR_DIR_PIN_1;
        uint16_t MOTOR_DIR_PIN_2;

        TIM_HandleTypeDef *htim_pwm;
        TIM_HandleTypeDef *htim_encoder;
};

