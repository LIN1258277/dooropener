#include "main_task.h"
#include "usart.h"
#include "device/uart.h"
#include "stm32f103xb.h"
#include <cstring>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float dt;
} PID_t;

PID_t pid_speed = {100, 0.0, 0.0, 0.01};
PID_t pid_angle = {0.5, 0.0, 0.0, 0.01};

Motor motor1(GPIOB, AIN1_Pin, AIN2_Pin, &htim1, &htim2);
Motor motor2(GPIOB, BIN1_Pin, BIN2_Pin, &htim4, &htim3);

// UART通信实例 (USART2用于Vofa+上位机通信)
Uart uart2(&huart2);

// 发送到Vofa+的数据缓冲区 (6通道)
static float vofa_data[6];

void main_task(void){
    // 初始化电机
    motor1.init();
    motor2.init();

    // 初始化PID参数
    motor1.set_pid(pid_speed.kp, pid_speed.ki, pid_speed.kd,
                   pid_angle.kp, pid_angle.ki, pid_angle.kd);
    motor2.set_pid(pid_speed.kp, pid_speed.ki, pid_speed.kd,
                   pid_angle.kp, pid_angle.ki, pid_angle.kd);

    // 初始化UART并启动中断接收
    uart2.init();

    // 主循环
    uint32_t last_send_tick = 0;
    uint32_t last_read_tick = 0;

    // 开环测试：直接给 PWM，不依赖 PID
    // motor1.set_speed(20.0f); // 100 RPM
    motor1.target_speed = 20.0f; // 100 RPM
    
    while (1) {
        uint32_t now = HAL_GetTick();

        // ===== 每20ms读取编码器/速度 =====
        if (now - last_read_tick >= 20) {
            last_read_tick = now;
            motor1.get_encoder();
            motor1.get_speed();
            motor2.get_encoder();
            motor2.get_speed();
           
        }
            vofa_data[0] = (float)motor1.get_last_encoder();
            vofa_data[1] = motor1.get_speed();
            vofa_data[2] = motor1.target_speed;
            
            vofa_data[3] = (float)motor2.get_last_encoder();
            vofa_data[4] = motor2.get_speed();
            vofa_data[5] = motor2.target_speed;

        
        // ===== 每20ms向Vofa+发送数据 (JustFloat格式) =====
        if (now - last_send_tick >= 20) {
            last_send_tick = now;

            // 6通道: [M1编码器, M1当前速度, M1目标速度,
            //         M2编码器, M2当前速度, M2目标速度]
            // 使用 get_last_encoder() 避免二次复位硬件计数器
           
            uart2.send_justfloat(vofa_data, 6);
        }

        // ===== 检查串口是否收到新的PID参数 =====
        if (uart2.has_new_pid_data()) {
            float speed_kp, speed_ki, speed_kd;
            float angle_kp, angle_ki, angle_kd;

            uart2.get_pid_data(&speed_kp, &speed_ki, &speed_kd,
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
        if (uart2.has_command()) {
            char cmd[32];
            uart2.get_command(cmd, sizeof(cmd));

            if (strcmp(cmd, "help") == 0) {
                uart2.send_text("--- Available commands ---\r\n");
                uart2.send_text("spd1 / spd2    Motor speed (rpm)\r\n");
                uart2.send_text("enc1 / enc2    Motor encoder\r\n");
                uart2.send_text("ang1 / ang2    Motor target angle\r\n");
                uart2.send_text("kp / ki / kd   Speed PID params\r\n");
                uart2.send_text("akp/ aki/ akd  Angle PID params\r\n");
                uart2.send_text("all            All values\r\n");
            }
            else if (strcmp(cmd, "spd1") == 0) {
                uart2.send_named_float("motor1_speed", motor1.get_speed());
            }
            else if (strcmp(cmd, "spd2") == 0) {
                uart2.send_named_float("motor2_speed", motor2.get_speed());
            }
            else if (strcmp(cmd, "enc1") == 0) {
                uart2.send_named_int("motor1_encoder", motor1.get_encoder());
            }
            else if (strcmp(cmd, "enc2") == 0) {
                uart2.send_named_int("motor2_encoder", motor2.get_encoder());
            }
            else if (strcmp(cmd, "ang1") == 0) {
                uart2.send_named_float("motor1_angle", motor1.target_angle);
            }
            else if (strcmp(cmd, "ang2") == 0) {
                uart2.send_named_float("motor2_angle", motor2.target_angle);
            }
            else if (strcmp(cmd, "kp") == 0) {
                uart2.send_named_float("speed_kp", pid_speed.kp);
            }
            else if (strcmp(cmd, "ki") == 0) {
                uart2.send_named_float("speed_ki", pid_speed.ki);
            }
            else if (strcmp(cmd, "kd") == 0) {
                uart2.send_named_float("speed_kd", pid_speed.kd);
            }
            else if (strcmp(cmd, "akp") == 0) {
                uart2.send_named_float("angle_kp", pid_angle.kp);
            }
            else if (strcmp(cmd, "aki") == 0) {
                uart2.send_named_float("angle_ki", pid_angle.ki);
            }
            else if (strcmp(cmd, "akd") == 0) {
                uart2.send_named_float("angle_kd", pid_angle.kd);
            }
            else if (strcmp(cmd, "all") == 0) {
                uart2.send_named_float("m1_speed", motor1.get_speed());
                uart2.send_named_float("m1_encoder", (float)motor1.get_encoder());
                uart2.send_named_float("m1_target_speed", motor1.target_speed);
                uart2.send_named_float("m1_target_angle", motor1.target_angle);
                uart2.send_named_float("m2_speed", motor2.get_speed());
                uart2.send_named_float("m2_encoder", (float)motor2.get_encoder());
                uart2.send_named_float("m2_target_speed", motor2.target_speed);
                uart2.send_named_float("m2_target_angle", motor2.target_angle);
                uart2.send_named_float("speed_kp", pid_speed.kp);
                uart2.send_named_float("speed_ki", pid_speed.ki);
                uart2.send_named_float("speed_kd", pid_speed.kd);
                uart2.send_named_float("angle_kp", pid_angle.kp);
                uart2.send_named_float("angle_ki", pid_angle.ki);
                uart2.send_named_float("angle_kd", pid_angle.kd);
            }
            else {
                uart2.send_text("unknown cmd, send $help\r\n");
            }
        }

        
    }
}
