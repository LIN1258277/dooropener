#pragma once
#include "main.h"
#include "device/motor.h"
#include <cstdio>
#include <cstring>

class Uart {
public:
    Uart(UART_HandleTypeDef *huart);
    err_t init();

    // 基础发送/接收
    err_t send_data(uint8_t *data, uint16_t size);
    err_t receive_data(uint8_t *data, uint16_t size);

    // JustFloat格式发送 (Vofa+ 上位机)
    // data: float数组, channel_count: 通道数
    err_t send_justfloat(float *data, uint8_t channel_count);

    // === 文本格式发送（新增）===
    err_t send_text(const char *str);                              // 发送纯文本
    err_t send_named_float(const char *name, float value);         // 发送 "name: value\r\n"
    err_t send_named_int(const char *name, int32_t value);         // 发送 "name: value\r\n"

    // === 命令接收（新增）===
    bool has_command();                  // 检查是否有文本命令到达
    void get_command(char *buf, uint8_t max_len); // 获取命令内容（不含前导 '$'）

    // 启动中断接收 (接收6个float: speed_kp, ki, kd, angle_kp, ki, kd)
    err_t start_receive();

    // 检查是否有新的PID数据
    bool has_new_pid_data();

    // 获取接收到的PID参数
    void get_pid_data(float *speed_kp, float *speed_ki, float *speed_kd,
                      float *angle_kp, float *angle_ki, float *angle_kd);

    // 由HAL_UART_RxCpltCallback调用
    void rx_callback();

    // 获取UART句柄
    UART_HandleTypeDef *get_handle() { return huart; }

    // 用于HAL C回调桥接的实例指针
    static Uart *instance;

private:
    UART_HandleTypeDef *huart;
    uint8_t rx_buffer[24];      // 6 floats * 4 bytes
    uint8_t rx_index;           // 当前接收索引
    uint8_t rx_byte;            // 单字节接收缓存
    float pid_buffer[6];        // 解析后的PID参数
    bool new_pid_ready;         // 新数据就绪标志

    // === 文本命令相关（新增）===
    char cmd_buffer[32];        // 文本命令缓冲区
    uint8_t cmd_index;          // 命令索引
    uint8_t cmd_length;         // 完成命令的长度
    bool cmd_ready;             // 命令就绪标志
    bool text_mode;             // 当前是否处于文本命令模式（以 '$' 开头）
};