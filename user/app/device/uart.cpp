#include "uart.h"
#include <cstring>

// 静态实例指针，用于HAL C回调桥接
Uart *Uart::instance = nullptr;

// JustFloat尾帧: 0x00 0x00 0x80 0x7f
static const uint8_t justfloat_tail[4] = {0x00, 0x00, 0x80, 0x7f};

Uart::Uart(UART_HandleTypeDef *huart)
    : huart(huart), rx_index(0), rx_byte(0), new_pid_ready(false),
      cmd_index(0), cmd_length(0), cmd_ready(false), text_mode(false) {
    instance = this;
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(pid_buffer, 0, sizeof(pid_buffer));
    memset(cmd_buffer, 0, sizeof(cmd_buffer));
}

err_t Uart::init() {
    // 初始化UART外设由CubeMX完成，这里启动中断接收
    return start_receive();
}

err_t Uart::send_data(uint8_t *data, uint16_t size) {
    if (HAL_UART_Transmit(huart, data, size, 1000) == HAL_OK) {
        return MOTOR_SUCCESS;
    }
    return MOTOR_ERROR;
}

err_t Uart::receive_data(uint8_t *data, uint16_t size) {
    if (HAL_UART_Receive(huart, data, size, 1000) == HAL_OK) {
        return MOTOR_SUCCESS;
    }
    return MOTOR_ERROR;
}

// ==================== 文本发送 ====================

err_t Uart::send_text(const char *str) {
    uint16_t len = (uint16_t)strlen(str);
    if (len == 0) return MOTOR_SUCCESS;
    return send_data((uint8_t *)str, len);
}

err_t Uart::send_named_float(const char *name, float value) {
    char buf[48];
    int len = snprintf(buf, sizeof(buf), "%s: %.3f\r\n", name, (double)value);
    if (len > 0) {
        return send_data((uint8_t *)buf, (uint16_t)len);
    }
    return MOTOR_ERROR;
}

err_t Uart::send_named_int(const char *name, int32_t value) {
    char buf[48];
    int len = snprintf(buf, sizeof(buf), "%s: %ld\r\n", name, (long)value);
    if (len > 0) {
        return send_data((uint8_t *)buf, (uint16_t)len);
    }
    return MOTOR_ERROR;
}

// ==================== 命令接收 ====================

bool Uart::has_command() {
    bool ret = cmd_ready;
    if (ret) cmd_ready = false;  // 自动清除
    return ret;
}

void Uart::get_command(char *buf, uint8_t max_len) {
    uint8_t copy_len = (cmd_length < max_len - 1) ? cmd_length : (max_len - 1);
    memcpy(buf, cmd_buffer, copy_len);
    buf[copy_len] = '\0';
    cmd_length = 0;
    cmd_index = 0;
}

// ==================== JustFloat格式发送 ====================

/**
 * @brief 以JustFloat格式发送数据到Vofa+
 * @param data         float数据数组
 * @param channel_count 通道数量
 * @return err_t
 *
 * JustFloat协议: [ch1_4bytes][ch2_4bytes]...[chN_4bytes][0x00 0x00 0x80 0x7f]
 */
err_t Uart::send_justfloat(float *data, uint8_t channel_count) {
    uint16_t data_bytes = channel_count * 4;
    uint8_t buffer[data_bytes + 4];

    // 复制float数据
    memcpy(buffer, data, data_bytes);
    // 添加JustFloat尾帧
    memcpy(buffer + data_bytes, justfloat_tail, 4);

    return send_data(buffer, data_bytes + 4);
}

/**
 * @brief 启动中断接收（单字节模式）
 *        默认接收24字节=6个float PID参数；
 *        若收到'$'开头则为文本命令，直到'\n'结束
 */
err_t Uart::start_receive() {
    rx_index = 0;
    cmd_index = 0;
    cmd_length = 0;
    new_pid_ready = false;
    cmd_ready = false;
    text_mode = false;
    if (HAL_UART_Receive_IT(huart, &rx_byte, 1) == HAL_OK) {
        return MOTOR_SUCCESS;
    }
    return MOTOR_ERROR;
}

bool Uart::has_new_pid_data() {
    bool ret = new_pid_ready;
    if (ret) {
        new_pid_ready = false; // 自动清除标志
    }
    return ret;
}

void Uart::get_pid_data(float *speed_kp, float *speed_ki, float *speed_kd,
                         float *angle_kp, float *angle_ki, float *angle_kd) {
    *speed_kp  = pid_buffer[0];
    *speed_ki  = pid_buffer[1];
    *speed_kd  = pid_buffer[2];
    *angle_kp  = pid_buffer[3];
    *angle_ki  = pid_buffer[4];
    *angle_kd  = pid_buffer[5];
}

/**
 * @brief 中断接收回调：每收到一个字节
 *   - 若当前未处于文本模式且 rx_index == 0 且字节为 '$'，
 *     则进入文本命令模式
 *   - 文本模式：累积到 cmd_buffer，收到 '\n' 或 '\r' 即完成命令
 *   - 二进制模式（默认）：原有 JustFloat PID 接收逻辑
 */
void Uart::rx_callback() {
    // ---- 检测文本命令起始 ----
    if (!text_mode && rx_index == 0 && rx_byte == '$') {
        text_mode = true;
        cmd_index = 0;
        memset(cmd_buffer, 0, sizeof(cmd_buffer));
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
        return;
    }

    if (text_mode) {
        // ---- 文本命令模式：累积到换行结束 ----
        if (rx_byte == '\n' || rx_byte == '\r') {
            cmd_buffer[cmd_index] = '\0';
            cmd_length = cmd_index;
            cmd_ready = true;
            text_mode = false;
            rx_index = 0;   // 重置二进制接收索引
        } else if (cmd_index < sizeof(cmd_buffer) - 1) {
            cmd_buffer[cmd_index++] = (char)rx_byte;
        }
    } else {
        // ---- 二进制 PID 模式（原有逻辑） ----
        if (rx_index < sizeof(rx_buffer)) {
            rx_buffer[rx_index++] = rx_byte;
        }

        if (rx_index >= 24) {
            rx_index = 0;
            memcpy(pid_buffer, rx_buffer, 24);
            new_pid_ready = true;
        }
    }

    // 继续接收下一字节
    HAL_UART_Receive_IT(huart, &rx_byte, 1);
}


// ==================== HAL 全局回调函数 ====================
// 由 stm32f1xx_it.c 中的 HAL_UART_IRQHandler 自动调用
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // 判断是哪个UART实例触发的中断
    if (Uart::instance != nullptr && huart->Instance == USART2) {
        Uart::instance->rx_callback();
    }
}

