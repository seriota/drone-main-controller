#ifndef DRONE_BAROMETER_H
#define DRONE_BAROMETER_H

#include "stm32f4xx_hal.h"
#include "drone_spi_communication.h"

/* Chip-select pin (manual NSS, GPIOA) */
#define DRONE_BAROMETER_CS_PIN GPIO_PIN_3
#define DRONE_BAROMETER_CS_GPIO GPIOA
#define DRONE_BAROMETER_CS_LOW() HAL_GPIO_WritePin(DRONE_BAROMETER_CS_GPIO, DRONE_BAROMETER_CS_PIN, GPIO_PIN_RESET)
#define DRONE_BAROMETER_CS_HIGH() HAL_GPIO_WritePin(DRONE_BAROMETER_CS_GPIO, DRONE_BAROMETER_CS_PIN, GPIO_PIN_SET)

#define DRONE_BAROMETER_PRESSURE_SENSITIVITY_REG 0xA2
#define DRONE_BAROMETER_PRESSURE_OFFSET_REG 0xA4
#define DRONE_BAROMETER_TEMP_COEFFICIENT_REG 0xA6
#define DRONE_BAROMETER_TEMP_OFFSET_REG 0xA8
#define DRONE_BAROMETER_REF_TEMP_REG 0xAA
#define DRONE_BAROMETER_TEMP_COEFFICIENT_OF_TEMP_REG 0xAC
#define DRONE_BAROMETER_CRC_REG 0xAE

#define DRONE_BAROMETER_CONVERT_D1_OSR256 0x40
#define DRONE_BAROMETER_CONVERT_D1_OSR512 0x42
#define DRONE_BAROMETER_CONVERT_D1_OSR1024 0x44
#define DRONE_BAROMETER_CONVERT_D1_OSR2048 0x46
#define DRONE_BAROMETER_CONVERT_D1_OSR4096 0x48

#define DRONE_BAROMETER_CONVERT_D2_OSR256 0x50
#define DRONE_BAROMETER_CONVERT_D2_OSR512 0x52
#define DRONE_BAROMETER_CONVERT_D2_OSR1024 0x54
#define DRONE_BAROMETER_CONVERT_D2_OSR2048 0x56
#define DRONE_BAROMETER_CONVERT_D2_OSR4096 0x58

#define DRONE_BAROMETER_ADC_READ 0x00

typedef struct
{
    uint16_t PRESSURE_SENSITIVITY;
    uint16_t PRESSURE_OFFSET;
    uint16_t TEMP_COEFFICIENT;
    uint16_t TEMP_OFFSET;
    uint16_t REF_TEMP;
    uint16_t TEMP_COEFFICIENT_OF_TEMP;
    uint16_t DATA_CRC;
} DroneBarometerCoefficients_t;

typedef struct
{
    float pressure;
    float temperature;
} DroneBarometerData_t;

typedef struct {
    void (*spi_read)(uint8_t reg, uint8_t *data, uint8_t length);
    void (*spi_send_command)(uint8_t cmd);
    uint8_t d1_osr_command;
    uint8_t d2_osr_command;
    uint16_t conversion_delay_ms;
    float sea_level_pressure_mbar;
} DroneBarometerConfig_t;

#define DRONE_BAROMETER_DEFAULT_CONFIG {                \
    .spi_read = NULL,                                   \
    .spi_send_command = NULL,                           \
    .d1_osr_command = DRONE_BAROMETER_CONVERT_D1_OSR4096, \
    .d2_osr_command = DRONE_BAROMETER_CONVERT_D2_OSR4096, \
    .conversion_delay_ms = 10,                          \
    .sea_level_pressure_mbar = 1013.25f,                \
}

typedef struct {
    int32_t temperature;    /* 0.01 degC (2007 = 20.07C) */
    int32_t pressure;       /* 0.01 mbar (100009 = 1000.09 mbar) */
    float altitude_m;
} DroneBarometerResult_t;

extern DroneBarometerData_t drone_barometer_data;

void drone_barometer_init(const DroneBarometerConfig_t *config);
HAL_StatusTypeDef drone_barometer_read_raw(uint32_t *d1, uint32_t *d2);
void drone_barometer_convert_first_order(uint32_t d1, uint32_t d2, DroneBarometerResult_t *result);
void drone_barometer_convert_second_order(uint32_t d1, uint32_t d2, DroneBarometerResult_t *result);
float drone_barometer_pressure_to_altitude(int32_t pressure_001mbar);

#endif
