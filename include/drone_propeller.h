#ifndef DRONE_PROPELLER_H
#define DRONE_PROPELLER_H

#include "stm32f4xx.h"
#include "system_config.h"
#include <stdint.h>
#include <stdbool.h>

#define DRONE_PROPELLER_TIMER TIM1
#define DRONE_PROPELLER_DMA_TIM1_CHANNEL1 DMA2_Stream1
#define DRONE_PROPELLER_DMA_TIM1_CHANNEL2 DMA2_Stream2
#define DRONE_PROPELLER_DMA_TIM1_CHANNEL3 DMA2_Stream6
#define DRONE_PROPELLER_DMA_TIM1_CHANNEL4 DMA2_Stream4
#define DRONE_PROPELLER_MIN_SPEED 0          // Minimum speed in percentage (0%)
#define DRONE_PROPELLER_MAX_SPEED 100        // Maximum speed in percentage (100%)
#define DRONE_PROPELLER_PWM_FREQ 50          // PWM frequency in Hz (50 Hz for typical servos
#define DRONE_PROPELLER_DSHOT600_FREQ 600000 // DShot frequency in Hz (600 kHz for DShot 600)
// #define PWM_PROTOCOL // PWM protocol identifier
#define DRONE_PROPELLER_MIN_DUTYCYCLE_PERIOD 1000    // Minimum duty cycle period in microseconds (1000 us)
#define DRONE_PROPELLER_MAX_DUTYCYCLE_PERIOD 2000    // Maximum duty cycle period in microseconds (2000 us)
#define DRONE_PROPELLER_DSHOT_HIGH_BIT_DUTYCYCLE 75  // High bit duty cycle in percentage (75%)
#define DRONE_PROPELLER_DSHOT_LOW_BIT_DUTYCYCLE 37.5 // Low bit duty cycle in percentage (37.5%)

typedef enum
{
    DRONE_PROPELLER_FRONT_RIGHT = 0x00,
    DRONE_PROPELLER_FRONT_LEFT = 0x01,
    DRONE_PROPELLER_BACK_LEFT = 0x02,
    DRONE_PROPELLER_BACK_RIGHT = 0x03,
} DronePropellerChannel_t;

typedef enum
{
    DRONE_PROPELLER_DSHOT_PROTOCOL = 0x00,
    DRONE_PROPELLER_PWM_PROTOCOL = 0x01,
} DronePropellerCommunicationProtocol_t;

typedef struct
{
    uint8_t ID;                                                  // Propeller ID
    int Pin;                                                     // GPIO pin number
    TIM_TypeDef *PropellerTimer;                                 // Timer used for PWM
    uint32_t TimerChannel;                                       // Timer channel used for PWM
    uint32_t PWMPeriod;                                          // PWM period in microseconds
    float DshotPeriod;                                           // minimum duty cycle period in microseconds
    uint32_t DshotTickPeriod;                                    // DShot tick period in microseconds
    uint32_t DshotHighBitTick;                                   // High bit tick in microseconds
    uint32_t DshotLowBitTick;                                    // Low bit tick in microseconds
    uint32_t MaxDutycyclePeriod;                                 // maximum period in microseconds
    uint32_t MinDutycyclePeriod;                                 // minimum period in microseconds
    uint32_t CurrentDutycyclePeriode;                            // current period in microseconds
    DronePropellerCommunicationProtocol_t CommunicationProtocol; // Communication protocol used
} DronePropeller_t;

extern DronePropeller_t *drone_propeller_array[4];
extern uint16_t drone_propeller_length;
extern uint32_t drone_propeller_dma_buffer[4][16];
extern volatile bool drone_propeller_dma_is_busy[4];
extern TIM_HandleTypeDef drone_propeller_timer;
extern DMA_HandleTypeDef drone_propeller_dma[4];

extern DronePropeller_t propellerFR, propellerFL, propellerBL, propellerBR;

void drone_propeller_init(DronePropeller_t *propeller);
void drone_propeller_set_speed(DronePropellerChannel_t channel, uint32_t speed);
void drone_propeller_send_dshot(DronePropellerChannel_t channel, uint32_t speed);
void DMA2_Stream1_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);
void DMA2_Stream4_IRQHandler(void);
void HAL_DMA_TxCpltCallback(DMA_HandleTypeDef *hdma);
DroneError_t drone_propeller_start();

#endif