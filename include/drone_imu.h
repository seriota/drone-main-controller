#ifndef DRONE_IMU_H
#define DRONE_IMU_H

#include "stm32f4xx_hal.h"
#include "system_config.h"

/* SPI peripheral */
#define DRONE_IMU_SPI_INSTANCE SPI1

/* SPI GPIO pins (SPI1, GPIOA) */
#define DRONE_SCK_PIN GPIO_PIN_5
#define DRONE_MISO_PIN GPIO_PIN_6
#define DRONE_MOSI_PIN GPIO_PIN_7

/* Chip-select pin (manual NSS, GPIOA) */
#define DRONE_IMU_CS_PIN GPIO_PIN_4
#define DRONE_IMU_CS_GPIO GPIOA
#define DRONE_IMU_CS_LOW() HAL_GPIO_WritePin(DRONE_IMU_CS_GPIO, DRONE_IMU_CS_PIN, GPIO_PIN_RESET)
#define DRONE_IMU_CS_HIGH() HAL_GPIO_WritePin(DRONE_IMU_CS_GPIO, DRONE_IMU_CS_PIN, GPIO_PIN_SET)

/* MPU-9250 register addresses */
#define MPU9250_REG_USER_CTRL 0x6A
#define MPU9250_REG_WHO_AM_I 0x75
#define MPU9250_WHO_AM_I_VAL 0x71

/* Accelerometer output registers (high byte first) */
#define MPU9250_REG_ACCEL_XOUT_H 0x3B /* ACCEL_XOUT[15:8] */

/* Gyroscope output registers (high byte first) */
#define MPU9250_REG_GYRO_XOUT_H 0x43 /* GYRO_XOUT[15:8] */

/* Configuration registers */
#define MPU9250_REG_ACCEL_CONFIG 0x1C /* Accel full-scale select */
#define MPU9250_REG_GYRO_CONFIG 0x1B  /* Gyro full-scale select  */
#define MPU9250_REG_PWR_MGMT_1 0x6B   /* Power management        */

/* Accel full-scale range options (ACCEL_CONFIG bits [4:3]) */
#define MPU9250_ACCEL_FS_2G 0x00  /* ±2g  — 16384 LSB/g */
#define MPU9250_ACCEL_FS_4G 0x08  /* ±4g  —  8192 LSB/g */
#define MPU9250_ACCEL_FS_8G 0x10  /* ±8g  —  4096 LSB/g */
#define MPU9250_ACCEL_FS_16G 0x18 /* ±16g —  2048 LSB/g */

/* Gyro full-scale range options (GYRO_CONFIG bits [4:3]) */
#define MPU9250_GYRO_FS_250 0x00  /* ±250  dps — 131.0 LSB/dps */
#define MPU9250_GYRO_FS_500 0x08  /* ±500  dps —  65.5 LSB/dps */
#define MPU9250_GYRO_FS_1000 0x10 /* ±1000 dps —  32.8 LSB/dps */
#define MPU9250_GYRO_FS_2000 0x18 /* ±2000 dps —  16.4 LSB/dps */

/* Sensitivity scale factors */
#define MPU9250_ACCEL_SENS_2G 16384.0f
#define MPU9250_GYRO_SENS_250 131.0f
#define MPU9250_ACCEL_SENS_4G 8192.0f
#define MPU9250_GYRO_SENS_500 65.5f

/* SPI frame helpers */
#define MPU9250_READ_FLAG 0x80  /* MSB=1 for read  */
#define MPU9250_WRITE_FLAG 0x7F /* mask MSB=0 for write */

/* Raw sensor data (straight from registers) */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} DroneImuRaw_t;

/* Scaled sensor data in physical units */
typedef struct
{
    float x; /* Accelerometer: g  |  Gyroscope: dps */
    float y;
    float z;
} DroneImuData_t;

typedef struct
{
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} DroneImuOffset_t;

extern SPI_HandleTypeDef drone_imu_spi;
extern DroneImuOffset_t imu_offsets;
extern DroneImuData_t imu_accel_calibrated;
extern DroneImuData_t imu_gyro_calibrated;

void drone_imu_init(void);
void drone_imu_write_reg(uint8_t reg, uint8_t data);
uint8_t drone_imu_read_reg(uint8_t reg);
void drone_imu_read_accel_raw(DroneImuRaw_t *out);
void drone_imu_read_gyro_raw(DroneImuRaw_t *out);
void drone_imu_read_accel(DroneImuData_t *out);
void drone_imu_read_gyro(DroneImuData_t *out);
void drone_imu_calibrate_offsets(DroneImuOffset_t *offsets);
void drone_imu_read_accel_calibrated(DroneImuData_t *out);
void drone_imu_read_gyro_calibrated(DroneImuData_t *out);
#endif