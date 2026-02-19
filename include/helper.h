#ifndef HELPER_H
#define HELPER_H

#include "stm32f4xx_hal.h"

typedef enum DroneError_t
{
    DRONE_ERROR_NONE,
    DRONE_ERROR_TIMEOUT,
    DRONE_ERROR_INVALID_PARAM,
    DRONE_ERROR_COMMUNICATION,
    DRONE_ERROR_SYSTEM_INITIALIZATION
} DroneError_t;

void Error_Handler(void);

#endif /* HELPER_H */