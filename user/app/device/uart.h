#pragma once
#include "main.h"
#include "device/motor.h"

/* =========================================================================
 * 串口驱动（只负责发字节 + 收字节回调）
 *
 * 现在唯一的用户是指纹模组（USART2）。原来的文本命令解析、Vofa+ JustFloat
 * 上行、6×float PID 参数帧都随调试入口一并删除；接新协议时用
 * set_byte_handler() 注册字节回调，自己解析即可。
 * ========================================================================= */
class Uart {
public:
    Uart(UART_HandleTypeDef *huart);
    err_t init();

    // 发送一段数据（阻塞，超时 1s）
    err_t send_data(uint8_t *data, uint16_t size);

    // 由 HAL_UART_RxCpltCallback 调用
    void rx_callback();

    UART_HandleTypeDef *get_handle() { return huart; }

    // ===== 原始字节回调（私有协议使用）=====
    // 设置后，每收到一个字节都直接回调 handler。
    typedef void (*ByteHandler)(uint8_t byte);
    void set_byte_handler(ByteHandler handler) { byte_handler = handler; }

    // 多实例分发：HAL 回调只拿到 huart 句柄，由这里按句柄找到对应实例
    static void dispatch(UART_HandleTypeDef *handle);

    // 串口错误（溢出/噪声/帧错误）后重新 arm 接收，
    // 避免指纹模组断电期间浮空 RX 引入一次错误就让接收永久停摆。
    static void recover(UART_HandleTypeDef *handle);

private:
    enum { UART_MAX_INSTANCES = 2 };
    static Uart *instances[UART_MAX_INSTANCES];
    static uint8_t instance_count;

    err_t start_receive();

    UART_HandleTypeDef *huart;
    uint8_t rx_byte;            // 单字节接收缓存
    ByteHandler byte_handler;   // 非空时走原始字节回调（私有协议）
};
