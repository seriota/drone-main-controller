#include "drone_uart.h"
#include <errno.h>

static UART_HandleTypeDef uart_handle;

void drone_uart_init(const DroneUartConfig_t *config)
{
    DroneUartConfig_t cfg;
    if (config == NULL) {
        DroneUartConfig_t defaults = DRONE_UART_DEFAULT_CONFIG;
        cfg = defaults;
    } else {
        cfg = *config;
    }

    /* Enable clocks */
    if (cfg.instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
    } else if (cfg.instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
    }

    if (cfg.gpio_port == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (cfg.gpio_port == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }

    /* Configure TX and RX pins as alternate function */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = cfg.tx_pin | cfg.rx_pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = cfg.gpio_af;
    HAL_GPIO_Init(cfg.gpio_port, &gpio);

    /* Configure UART */
    uart_handle.Instance = cfg.instance;
    uart_handle.Init.BaudRate = cfg.baudrate;
    uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
    uart_handle.Init.StopBits = UART_STOPBITS_1;
    uart_handle.Init.Parity = UART_PARITY_NONE;
    uart_handle.Init.Mode = UART_MODE_TX_RX;
    uart_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&uart_handle);
}

/* Retarget printf to UART via newlib _write */
int _write(int fd, char *ptr, int len)
{
    (void)fd;
    HAL_UART_Transmit(&uart_handle, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
