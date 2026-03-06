#include "drone_barometer.h"
#include <math.h>

DroneBarometerData_t drone_barometer_data = {0};
static DroneBarometerCoefficients_t drone_barometer_coefficients = {0};
static DroneBarometerConfig_t baro_cfg;

static const uint16_t DRONE_BAROMETER_REGISTERS[7] = {
    DRONE_BAROMETER_PRESSURE_SENSITIVITY_REG,
    DRONE_BAROMETER_PRESSURE_OFFSET_REG,
    DRONE_BAROMETER_TEMP_COEFFICIENT_REG,
    DRONE_BAROMETER_TEMP_OFFSET_REG,
    DRONE_BAROMETER_REF_TEMP_REG,
    DRONE_BAROMETER_TEMP_COEFFICIENT_OF_TEMP_REG,
    DRONE_BAROMETER_CRC_REG};

/* ---------- default blocking SPI helpers ---------- */

static void default_spi_read(uint8_t reg, uint8_t *data, uint8_t length)
{
    DRONE_BAROMETER_CS_LOW();
    drone_spi_transmit_blocking(&reg, 1, 100);
    drone_spi_receive_blocking(data, length, 100);
    DRONE_BAROMETER_CS_HIGH();
}

static void default_spi_send_command(uint8_t cmd)
{
    DRONE_BAROMETER_CS_LOW();
    drone_spi_transmit_blocking(&cmd, 1, 100);
    DRONE_BAROMETER_CS_HIGH();
}

static void drone_barometer_read_coefficients(void)
{
    for (uint8_t i = 0; i < 7; i++)
    {
        uint8_t buff[2];
        baro_cfg.spi_read(DRONE_BAROMETER_REGISTERS[i], buff, 2);
        uint16_t value = (uint16_t)((buff[0] << 8) | buff[1]);
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

void drone_barometer_init(const DroneBarometerConfig_t *config)
{
    if (config != NULL) {
        baro_cfg = *config;
    } else {
        DroneBarometerConfig_t defaults = DRONE_BAROMETER_DEFAULT_CONFIG;
        baro_cfg = defaults;
    }

    if (baro_cfg.spi_read == NULL) baro_cfg.spi_read = default_spi_read;
    if (baro_cfg.spi_send_command == NULL) baro_cfg.spi_send_command = default_spi_send_command;

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

    drone_barometer_read_coefficients();
}

HAL_StatusTypeDef drone_barometer_read_raw(uint32_t *d1, uint32_t *d2)
{
    uint8_t buf[3];

    /* Start D1 (pressure) conversion */
    baro_cfg.spi_send_command(baro_cfg.d1_osr_command);
    HAL_Delay(baro_cfg.conversion_delay_ms);

    /* Read ADC result */
    baro_cfg.spi_read(DRONE_BAROMETER_ADC_READ, buf, 3);
    *d1 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];

    /* Start D2 (temperature) conversion */
    baro_cfg.spi_send_command(baro_cfg.d2_osr_command);
    HAL_Delay(baro_cfg.conversion_delay_ms);

    /* Read ADC result */
    baro_cfg.spi_read(DRONE_BAROMETER_ADC_READ, buf, 3);
    *d2 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];

    return HAL_OK;
}

void drone_barometer_convert_first_order(uint32_t d1, uint32_t d2, DroneBarometerResult_t *result)
{
    int64_t dT = (int64_t)d2 - ((int64_t)drone_barometer_coefficients.REF_TEMP << 8);
    int64_t TEMP = 2000 + (dT * (int64_t)drone_barometer_coefficients.TEMP_COEFFICIENT_OF_TEMP) / (1LL << 23);

    int64_t OFF = ((int64_t)drone_barometer_coefficients.PRESSURE_OFFSET << 16)
                + ((int64_t)drone_barometer_coefficients.TEMP_OFFSET * dT) / (1LL << 7);

    int64_t SENS = ((int64_t)drone_barometer_coefficients.PRESSURE_SENSITIVITY << 15)
                 + ((int64_t)drone_barometer_coefficients.TEMP_COEFFICIENT * dT) / (1LL << 8);

    int64_t P = ((int64_t)d1 * SENS / (1LL << 21) - OFF) / (1LL << 15);

    result->temperature = (int32_t)TEMP;
    result->pressure = (int32_t)P;
    result->altitude_m = drone_barometer_pressure_to_altitude(result->pressure);
}

void drone_barometer_convert_second_order(uint32_t d1, uint32_t d2, DroneBarometerResult_t *result)
{
    int64_t dT = (int64_t)d2 - ((int64_t)drone_barometer_coefficients.REF_TEMP << 8);
    int64_t TEMP = 2000 + (dT * (int64_t)drone_barometer_coefficients.TEMP_COEFFICIENT_OF_TEMP) / (1LL << 23);

    int64_t OFF = ((int64_t)drone_barometer_coefficients.PRESSURE_OFFSET << 16)
                + ((int64_t)drone_barometer_coefficients.TEMP_OFFSET * dT) / (1LL << 7);

    int64_t SENS = ((int64_t)drone_barometer_coefficients.PRESSURE_SENSITIVITY << 15)
                 + ((int64_t)drone_barometer_coefficients.TEMP_COEFFICIENT * dT) / (1LL << 8);

    /* Second order compensation */
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;

    if (TEMP < 2000)
    {
        T2 = (dT * dT) / (1LL << 31);
        int64_t temp_diff = TEMP - 2000;
        OFF2 = 5 * (temp_diff * temp_diff) / 2;
        SENS2 = 5 * (temp_diff * temp_diff) / 4;

        if (TEMP < -1500)
        {
            int64_t temp_diff2 = TEMP + 1500;
            OFF2 += 7 * (temp_diff2 * temp_diff2);
            SENS2 += 11 * (temp_diff2 * temp_diff2) / 2;
        }
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;

    int64_t P = ((int64_t)d1 * SENS / (1LL << 21) - OFF) / (1LL << 15);

    result->temperature = (int32_t)TEMP;
    result->pressure = (int32_t)P;
    result->altitude_m = drone_barometer_pressure_to_altitude(result->pressure);
}

float drone_barometer_pressure_to_altitude(int32_t pressure_001mbar)
{
    float pressure_mbar = (float)pressure_001mbar / 100.0f;
    return 44330.0f * (1.0f - powf(pressure_mbar / baro_cfg.sea_level_pressure_mbar, 0.1903f));
}
