#include "drone_imu.h"

DroneImuOffset_t imu_offsets = {0};
DroneImuData_t imu_accel_calibrated = {0};
DroneImuData_t imu_gyro_calibrated = {0};

static DroneImuConfig_t imu_cfg;

/* ---------- default blocking SPI helpers (used when config function pointers are NULL) ---------- */

static void default_spi_read_burst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    spi_tx_buffer[0] = reg | MPU9250_READ_FLAG;

    DRONE_IMU_CS_LOW();
    drone_spi_transmit_blocking(spi_tx_buffer, 1, 100);
    drone_spi_receive_blocking(buf, len, 100);
    DRONE_IMU_CS_HIGH();
}

static void default_spi_write_reg(uint8_t reg, uint8_t data)
{
    spi_tx_buffer[0] = reg & MPU9250_WRITE_FLAG;
    spi_tx_buffer[1] = data;

    DRONE_IMU_CS_LOW();
    drone_spi_transmit_blocking(spi_tx_buffer, 2, 100);
    DRONE_IMU_CS_HIGH();
}

static uint8_t default_spi_read_reg(uint8_t reg)
{
    uint8_t rx_data;
    spi_tx_buffer[0] = reg | MPU9250_READ_FLAG;

    DRONE_IMU_CS_LOW();
    drone_spi_transmit_blocking(spi_tx_buffer, 1, 100);
    drone_spi_receive_blocking(&rx_data, 1, 100);
    DRONE_IMU_CS_HIGH();

    return rx_data;
}

/* ---------- raw reads ---------- */

static void drone_imu_read_accel_raw(DroneImuRaw_t *out)
{
    uint8_t buf[6];
    imu_cfg.spi_read_burst(MPU9250_REG_ACCEL_XOUT_H, buf, 6);
    out->x = (int16_t)((buf[0] << 8) | buf[1]);
    out->y = (int16_t)((buf[2] << 8) | buf[3]);
    out->z = (int16_t)((buf[4] << 8) | buf[5]);
}

static void drone_imu_read_gyro_raw(DroneImuRaw_t *out)
{
    uint8_t buf[6];
    imu_cfg.spi_read_burst(MPU9250_REG_GYRO_XOUT_H, buf, 6);
    out->x = (int16_t)((buf[0] << 8) | buf[1]);
    out->y = (int16_t)((buf[2] << 8) | buf[3]);
    out->z = (int16_t)((buf[4] << 8) | buf[5]);
}

/* ---------- scaled reads ---------- */

static void drone_imu_read_accel(DroneImuData_t *out)
{
    DroneImuRaw_t raw;
    drone_imu_read_accel_raw(&raw);
    out->x = (float)raw.x / imu_cfg.accel_sensitivity;
    out->y = (float)raw.y / imu_cfg.accel_sensitivity;
    out->z = (float)raw.z / imu_cfg.accel_sensitivity;
}

static void drone_imu_read_gyro(DroneImuData_t *out)
{
    DroneImuRaw_t raw;
    drone_imu_read_gyro_raw(&raw);
    out->x = (float)raw.x / imu_cfg.gyro_sensitivity;
    out->y = (float)raw.y / imu_cfg.gyro_sensitivity;
    out->z = (float)raw.z / imu_cfg.gyro_sensitivity;
}

/* ---------- calibrated reads (public) ---------- */

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

static void drone_imu_calibrate_offsets(DroneImuOffset_t *offsets)
{
    DroneImuData_t accel_sum = {0}, gyro_sum = {0};

    for (int i = 0; i < imu_cfg.calibration_samples; i++)
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
        HAL_Delay(10);
    }

    offsets->accel_x = accel_sum.x / imu_cfg.calibration_samples;
    offsets->accel_y = accel_sum.y / imu_cfg.calibration_samples;
    offsets->accel_z = (accel_sum.z / imu_cfg.calibration_samples) - 1.0f;
    offsets->gyro_x = gyro_sum.x / imu_cfg.calibration_samples;
    offsets->gyro_y = gyro_sum.y / imu_cfg.calibration_samples;
    offsets->gyro_z = gyro_sum.z / imu_cfg.calibration_samples;
}

void drone_imu_init(const DroneImuConfig_t *config)
{
    if (config != NULL) {
        imu_cfg = *config;
    } else {
        DroneImuConfig_t defaults = DRONE_IMU_DEFAULT_CONFIG;
        imu_cfg = defaults;
    }

    /* Fill in default SPI functions if not injected */
    if (imu_cfg.spi_read_burst == NULL) imu_cfg.spi_read_burst = default_spi_read_burst;
    if (imu_cfg.spi_write_reg == NULL) imu_cfg.spi_write_reg = default_spi_write_reg;
    if (imu_cfg.spi_read_reg == NULL) imu_cfg.spi_read_reg = default_spi_read_reg;

    if (!is_spi_initialized())
    {
        drone_spi_init();
    }

    /* CS pin — manual output, idle HIGH */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DRONE_IMU_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(DRONE_IMU_CS_GPIO, &GPIO_InitStruct);
    DRONE_IMU_CS_HIGH();

    /* Disable I2C interface, force SPI-only (USER_CTRL: I2C_IF_DIS) */
    imu_cfg.spi_write_reg(MPU9250_REG_USER_CTRL, 0x10);

    /* Verify device identity via WHO_AM_I (expected 0x71) */
    if (imu_cfg.spi_read_reg(MPU9250_REG_WHO_AM_I) != MPU9250_WHO_AM_I_VAL)
    {
        Error_Handler();
    }

    /* Wake the device (clear SLEEP bit in PWR_MGMT_1, select PLL clock) */
    imu_cfg.spi_write_reg(MPU9250_REG_PWR_MGMT_1, 0x01);

    /* Set accelerometer full-scale range */
    imu_cfg.spi_write_reg(MPU9250_REG_ACCEL_CONFIG, imu_cfg.accel_fs_config);

    /* Set gyroscope full-scale range */
    imu_cfg.spi_write_reg(MPU9250_REG_GYRO_CONFIG, imu_cfg.gyro_fs_config);

    drone_imu_calibrate_offsets(&imu_offsets);

    drone_imu_read_accel_calibrated(&imu_accel_calibrated);
    drone_imu_read_gyro_calibrated(&imu_gyro_calibrated);
}
