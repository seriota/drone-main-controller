#include "drone_imu.h"

SPI_HandleTypeDef drone_imu_spi;
DroneImuOffset_t imu_offsets = {0};
DroneImuData_t imu_accel_calibrated = {0};
DroneImuData_t imu_gyro_calibrated = {0};

/* ---------- private helper: burst read n bytes starting at reg ---------- */
static void drone_imu_read_burst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx = reg | MPU9250_READ_FLAG;
    DRONE_IMU_CS_LOW();
    HAL_SPI_Transmit(&drone_imu_spi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&drone_imu_spi, buf, len, HAL_MAX_DELAY);
    DRONE_IMU_CS_HIGH();
}

void drone_imu_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg & MPU9250_WRITE_FLAG, data};
    DRONE_IMU_CS_LOW();
    HAL_SPI_Transmit(&drone_imu_spi, buf, 2, HAL_MAX_DELAY);
    DRONE_IMU_CS_HIGH();
}

uint8_t drone_imu_read_reg(uint8_t reg)
{
    uint8_t tx = reg | MPU9250_READ_FLAG;
    uint8_t rx = 0;
    DRONE_IMU_CS_LOW();
    HAL_SPI_Transmit(&drone_imu_spi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&drone_imu_spi, &rx, 1, HAL_MAX_DELAY);
    DRONE_IMU_CS_HIGH();
    return rx;
}

/* ---------- raw reads (register counts, ±2g / ±250dps default) ---------- */

void drone_imu_read_accel_raw(DroneImuRaw_t *out)
{
    uint8_t buf[6];
    /* Burst-read 6 bytes: ACCEL_XOUT_H, _L, ACCEL_YOUT_H, _L, ACCEL_ZOUT_H, _L */
    drone_imu_read_burst(MPU9250_REG_ACCEL_XOUT_H, buf, 6);
    out->x = (int16_t)((buf[0] << 8) | buf[1]);
    out->y = (int16_t)((buf[2] << 8) | buf[3]);
    out->z = (int16_t)((buf[4] << 8) | buf[5]);
}

void drone_imu_read_gyro_raw(DroneImuRaw_t *out)
{
    uint8_t buf[6];
    /* Burst-read 6 bytes: GYRO_XOUT_H, _L, GYRO_YOUT_H, _L, GYRO_ZOUT_H, _L */
    drone_imu_read_burst(MPU9250_REG_GYRO_XOUT_H, buf, 6);
    out->x = (int16_t)((buf[0] << 8) | buf[1]);
    out->y = (int16_t)((buf[2] << 8) | buf[3]);
    out->z = (int16_t)((buf[4] << 8) | buf[5]);
}

/* ---------- scaled reads (g and dps) ---------- */

void drone_imu_read_accel(DroneImuData_t *out)
{
    DroneImuRaw_t raw;
    drone_imu_read_accel_raw(&raw);
    out->x = (float)raw.x / MPU9250_ACCEL_SENS_2G;
    out->y = (float)raw.y / MPU9250_ACCEL_SENS_2G;
    out->z = (float)raw.z / MPU9250_ACCEL_SENS_2G;
}

void drone_imu_read_gyro(DroneImuData_t *out)
{
    DroneImuRaw_t raw;
    drone_imu_read_gyro_raw(&raw);
    out->x = (float)raw.x / MPU9250_GYRO_SENS_250;
    out->y = (float)raw.y / MPU9250_GYRO_SENS_250;
    out->z = (float)raw.z / MPU9250_GYRO_SENS_250;
}

void drone_imu_read_accel_calibrated(DroneImuData_t *out)
{
    DroneImuData_t raw;
    drone_imu_read_accel(&raw);
    out->x = raw.x - imu_offsets.accel_x;
    out->y = raw.y - imu_offsets.accel_y;
    out->z = raw.z - imu_offsets.accel_z;
}

void drone_imu_read_gyro_calibrated(DroneImuData_t *out)
{
    DroneImuData_t raw;
    drone_imu_read_gyro(&raw);
    out->x = raw.x - imu_offsets.gyro_x;
    out->y = raw.y - imu_offsets.gyro_y;
    out->z = raw.z - imu_offsets.gyro_z;
}

void drone_imu_calibrate_offsets(DroneImuOffset_t *offsets)
{
    const int num_samples = 200;
    DroneImuData_t accel_sum = {0}, gyro_sum = {0};

    for (int i = 0; i < num_samples; i++)
    {
        DroneImuData_t accel, gyro;
        drone_imu_read_accel(&accel);
        drone_imu_read_gyro(&gyro);
        accel_sum.x += accel.x;
        accel_sum.y += accel.y;
        accel_sum.z += accel.z;
        gyro_sum.x += gyro.x;
        gyro_sum.y += gyro.y;
        gyro_sum.z += gyro.z;
        HAL_Delay(10); // Delay between samples
    }

    offsets->accel_x = accel_sum.x / num_samples;
    offsets->accel_y = accel_sum.y / num_samples;
    offsets->accel_z = (accel_sum.z / num_samples) - 1.0f; // Subtract gravity
    offsets->gyro_x = gyro_sum.x / num_samples;
    offsets->gyro_y = gyro_sum.y / num_samples;
    offsets->gyro_z = gyro_sum.z / num_samples;
}

void drone_imu_init(void)
{
    /* Enable clocks */
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* SCK, MISO, MOSI — alternate function */
    GPIO_InitStruct.Pin = DRONE_SCK_PIN | DRONE_MISO_PIN | DRONE_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CS pin — manual output, idle HIGH */
    GPIO_InitStruct.Pin = DRONE_IMU_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(DRONE_IMU_CS_GPIO, &GPIO_InitStruct);
    DRONE_IMU_CS_HIGH();

    /* SPI peripheral config */
    drone_imu_spi.Instance = DRONE_IMU_SPI_INSTANCE;
    drone_imu_spi.Init.Mode = SPI_MODE_MASTER;
    drone_imu_spi.Init.Direction = SPI_DIRECTION_2LINES;
    drone_imu_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    drone_imu_spi.Init.CLKPolarity = SPI_POLARITY_HIGH;               // CPOL=1 (Mode 3)
    drone_imu_spi.Init.CLKPhase = SPI_PHASE_2EDGE;                    // CPHA=1 (Mode 3)
    drone_imu_spi.Init.NSS = SPI_NSS_SOFT;                            // CS managed manually
    drone_imu_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; // 84MHz/128 = ~656KHz (<1MHz limit)
    drone_imu_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    drone_imu_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    drone_imu_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

    if (HAL_SPI_Init(&drone_imu_spi) != HAL_OK)
    {
        Error_Handler();
    }

    /* Disable I2C interface, force SPI-only (USER_CTRL: I2C_IF_DIS) */
    drone_imu_write_reg(MPU9250_REG_USER_CTRL, 0x10);

    /* Verify device identity via WHO_AM_I (expected 0x71) */
    if (drone_imu_read_reg(MPU9250_REG_WHO_AM_I) != MPU9250_WHO_AM_I_VAL)
    {
        Error_Handler();
    }

    /* Wake the device (clear SLEEP bit in PWR_MGMT_1, select PLL clock) */
    drone_imu_write_reg(MPU9250_REG_PWR_MGMT_1, 0x01);

    /* Set accelerometer full-scale range to ±2g */
    drone_imu_write_reg(MPU9250_REG_ACCEL_CONFIG, MPU9250_ACCEL_FS_2G);

    /* Set gyroscope full-scale range to ±250 dps */
    drone_imu_write_reg(MPU9250_REG_GYRO_CONFIG, MPU9250_GYRO_FS_250);

    drone_imu_calibrate_offsets(&imu_offsets);

    /* Optional: read calibrated values into global variables for easy access */
    drone_imu_read_accel_calibrated(&imu_accel_calibrated);
    drone_imu_read_gyro_calibrated(&imu_gyro_calibrated);
}