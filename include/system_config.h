#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "stm32f4xx_hal.h"
#include "helper.h"
#include "drone_propeller.h"
#include "cmsis_os.h"
#include "drone_task.h"
#include "drone_imu.h"

#define LED_PIN1 GPIO_PIN_13
#define LED_PIN2 GPIO_PIN_12
#define BUILTIN_LED GPIO_PIN_13

#define SYSTEMCLOCK_FREQUENCY 84000000 // 84 MHz for STM32F4 series

void SystemClock_Config(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void system_init(void);
void gpio_init(void);
void tasks_init(void);

#endif