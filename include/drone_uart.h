#ifndef DRONE_UART_H
#define DRONE_UART_H

#include "stm32f4xx_hal.h"

typedef struct {
    USART_TypeDef *instance;
    uint32_t baudrate;
    GPIO_TypeDef *gpio_port;
    uint16_t tx_pin;
    uint16_t rx_pin;
    uint8_t gpio_af;
} DroneUartConfig_t;

#define DRONE_UART_DEFAULT_CONFIG { \
    .instance = USART1,            \
    .baudrate = 115200,            \
    .gpio_port = GPIOA,            \
    .tx_pin = GPIO_PIN_9,          \
    .rx_pin = GPIO_PIN_10,         \
    .gpio_af = GPIO_AF7_USART1,    \
}

void drone_uart_init(const DroneUartConfig_t *config);

#endif
