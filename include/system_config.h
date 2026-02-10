#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "stm32f4xx_hal.h"

#define LED_PIN1 GPIO_PIN_13
#define LED_PIN2 GPIO_PIN_12
#define BUILTIN_LED GPIO_PIN_13

#define SYSTEMCLOCK_FREQUENCY 84000000 // 84 MHz for STM32F4 series

typedef enum DroneError_t
{
    DRONE_ERROR_NONE,
    DRONE_ERROR_TIMEOUT,
    DRONE_ERROR_INVALID_PARAM,
    DRONE_ERROR_COMMUNICATION,
    DRONE_ERROR_SYSTEM_INITIALIZATION
} DroneError_t;

void Error_Handler(void);
void SystemClock_Config(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void system_init(void);
void gpio_init(void);
void propellers_init(void);

#endif