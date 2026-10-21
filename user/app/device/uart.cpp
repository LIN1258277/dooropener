#include "uart.h"

// 多实例注册表：HAL 的 RxCplt 回调只有 huart 句柄，
// 必须靠这张表把字节派发给正确的 Uart 对象。
Uart *Uart::instances[Uart::UART_MAX_INSTANCES] = {nullptr};
uint8_t Uart::instance_count = 0;

Uart::Uart(UART_HandleTypeDef *huart)
    : huart(huart), rx_byte(0), byte_handler(nullptr) {
    if (instance_count < UART_MAX_INSTANCES) {
        instances[instance_count++] = this;
    }
}

err_t Uart::init() {
    // UART 外设由 CubeMX 初始化，这里只启动中断接收
    return start_receive();
}

err_t Uart::send_data(uint8_t *data, uint16_t size) {
    if (HAL_UART_Transmit(huart, data, size, 1000) == HAL_OK) {
        return MOTOR_SUCCESS;
    }
    return MOTOR_ERROR;
}

// 启动单字节中断接收
err_t Uart::start_receive() {
    if (HAL_UART_Receive_IT(huart, &rx_byte, 1) == HAL_OK) {
        return MOTOR_SUCCESS;
    }
    return MOTOR_ERROR;
}

// 每收到一个字节：交给注册的 byte_handler（私有协议），然后重新 arm 接收
void Uart::rx_callback() {
    if (byte_handler != nullptr) {
        byte_handler(rx_byte);
    }
    HAL_UART_Receive_IT(huart, &rx_byte, 1);
}

// ==================== HAL 全局回调函数 ====================
// 由 stm32f1xx_it.c 中的 HAL_UART_IRQHandler 自动调用
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    Uart::dispatch(huart);
}

void Uart::dispatch(UART_HandleTypeDef *handle) {
    for (uint8_t i = 0; i < instance_count; i++) {
        if (instances[i] != nullptr && instances[i]->get_handle() == handle) {
            instances[i]->rx_callback();
            return;
        }
    }
}

// 串口错误回调：ORE/NE/FE 等错误会让 HAL 中止中断接收，
// 这里重新启动接收，保证指纹模组重新上电后还能收到数据。
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    Uart::recover(huart);
}

void Uart::recover(UART_HandleTypeDef *handle) {
    for (uint8_t i = 0; i < instance_count; i++) {
        if (instances[i] != nullptr && instances[i]->get_handle() == handle) {
            instances[i]->start_receive();
            return;
        }
    }
}
