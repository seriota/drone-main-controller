#include "drone_barometer.h"

DroneBarometerData_t drone_barometer_data = {0};
static DroneBarometerCoefficients_t drone_barometer_coefficients = {0};
static uint16_t barometer_coefficients_buffer[8] = {0};
static const uint16_t DRONE_BAROMETER_REGISTERS[7] = {
    DRONE_BAROMETER_PRESSURE_SENSITIVITY_REG,
    DRONE_BAROMETER_PRESSURE_OFFSET_REG,
    DRONE_BAROMETER_TEMP_COEFFICIENT_REG,
    DRONE_BAROMETER_TEMP_OFFSET_REG,
    DRONE_BAROMETER_REF_TEMP_REG,
    DRONE_BAROMETER_TEMP_COEFFICIENT_OF_TEMP_REG,
    DRONE_BAROMETER_CRC_REG};

static void drone_barometer_read_register(uint8_t reg, uint8_t *data, uint8_t length)
{
    DRONE_BAROMETER_CS_LOW();

    drone_spi_transmit_blocking(&reg, 1, 100);
    drone_spi_receive_blocking(data, length, 100);

    DRONE_BAROMETER_CS_HIGH();
}

static void drone_barometer_write_register(uint8_t reg, uint8_t data)
{
    DRONE_BAROMETER_CS_LOW();

    drone_spi_transmit_blocking(&reg, 1, 100);

    DRONE_BAROMETER_CS_HIGH();
}

static void drone_barometer_read_coefficients(void)
{
    for (uint8_t i = 0; i < 7; i++)
    {
        uint8_t buff[2];
        drone_barometer_read_register(DRONE_BAROMETER_REGISTERS[i], buff, 2);
        uint16_t value = (uint16_t)((buff[0] << 8) | buff[1]);
        barometer_coefficients_buffer[i + 1] = value;
        switch (i)
        {
        case 0:
            drone_barometer_coefficients.PRESSURE_SENSITIVITY = value;
            break;
        case 1:
            drone_barometer_coefficients.PRESSURE_OFFSET = value;
            break;
        case 2:
            drone_barometer_coefficients.TEMP_COEFFICIENT = value;
            break;
        case 3:
            drone_barometer_coefficients.TEMP_OFFSET = value;
            break;
        case 4:
            drone_barometer_coefficients.REF_TEMP = value;
            break;
        case 5:
            drone_barometer_coefficients.TEMP_COEFFICIENT_OF_TEMP = value;
            break;
        case 6:
            drone_barometer_coefficients.DATA_CRC = value;
            break;
        }
    }
}

void drone_barometer_init(void)
{
    if (!is_spi_initialized())
    {
        drone_spi_init();
    }

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DRONE_BAROMETER_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(DRONE_BAROMETER_CS_GPIO, &GPIO_InitStruct);
    DRONE_BAROMETER_CS_HIGH();
}