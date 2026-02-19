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
    uint16_t PRESSURE_SENSITIVITY;     // Pressure sensitivity
    uint16_t PRESSURE_OFFSET;          // Pressure offset
    uint16_t TEMP_COEFFICIENT;         // Temperature coefficient of pressure sensitivity
    uint16_t TEMP_OFFSET;              // Temperature coefficient of pressure offset
    uint16_t REF_TEMP;                 // Reference temperature
    uint16_t TEMP_COEFFICIENT_OF_TEMP; // Temperature coefficient of the temperature
    uint16_t DATA_CRC;                 // CRC value for coefficients
} DroneBarometerCoefficients_t;

/*Raw sensor data*/
typedef struct
{
    float pressure;    // Pressure in hectopascals
    float temperature; // Temperature in degrees Celsius (if available)
} DroneBarometerData_t;

extern DroneBarometerData_t drone_barometer_data;

void drone_barometer_init(void);

#endif