#include "drone_task.h"
#include "drone_propeller.h"
#include "drone_imu.h"
#include "drone_barometer.h"
#include "drone_attitude.h"
#include "drone_spi_communication.h"
#include <stdio.h>

const osThreadAttr_t propeller_task_attributes = {
    .name = "propellerTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t imu_task_attributes = {
    .name = "imuTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

const osThreadAttr_t barometer_task_attributes = {
    .name = "baroTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t sensor_fusion_task_attributes = {
    .name = "fusionTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

/* Shared sensor data (written by sensor tasks, read by fusion task) */
static volatile DroneImuData_t shared_accel = {0};
static volatile DroneImuData_t shared_gyro = {0};
static volatile float shared_baro_altitude = 0.0f;
static volatile int32_t shared_baro_temperature = 0;
static volatile int32_t shared_baro_pressure = 0;

void propeller_task(void *argument)
{
    for (;;)
    {
        drone_propeller_set_speed(DRONE_PROPELLER_FRONT_RIGHT, 8);
        osDelay(5000);
        drone_propeller_set_speed(DRONE_PROPELLER_FRONT_RIGHT, 0);
        osDelay(5000);
    }
}

void imu_task(void *argument)
{
    (void)argument;

    for (;;)
    {
        DroneImuData_t accel, gyro;

        /* Read via SPI manager (DMA-based, RTOS-safe) */
        uint8_t accel_buf[6];
        if (spi_manager_burst_read(DRONE_SPI_SENSOR_IMU, MPU9250_REG_ACCEL_XOUT_H, accel_buf, 6, 100) == HAL_OK)
        {
            DroneImuRaw_t raw;
            raw.x = (int16_t)((accel_buf[0] << 8) | accel_buf[1]);
            raw.y = (int16_t)((accel_buf[2] << 8) | accel_buf[3]);
            raw.z = (int16_t)((accel_buf[4] << 8) | accel_buf[5]);
            accel.x = (float)raw.x / MPU9250_ACCEL_SENS_4G - imu_offsets.accel_x;
            accel.y = (float)raw.y / MPU9250_ACCEL_SENS_4G - imu_offsets.accel_y;
            accel.z = (float)raw.z / MPU9250_ACCEL_SENS_4G - imu_offsets.accel_z;
            shared_accel = accel;
        }

        uint8_t gyro_buf[6];
        if (spi_manager_burst_read(DRONE_SPI_SENSOR_IMU, MPU9250_REG_GYRO_XOUT_H, gyro_buf, 6, 100) == HAL_OK)
        {
            DroneImuRaw_t raw;
            raw.x = (int16_t)((gyro_buf[0] << 8) | gyro_buf[1]);
            raw.y = (int16_t)((gyro_buf[2] << 8) | gyro_buf[3]);
            raw.z = (int16_t)((gyro_buf[4] << 8) | gyro_buf[5]);
            gyro.x = (float)raw.x / MPU9250_GYRO_SENS_500 - imu_offsets.gyro_x;
            gyro.y = (float)raw.y / MPU9250_GYRO_SENS_500 - imu_offsets.gyro_y;
            gyro.z = (float)raw.z / MPU9250_GYRO_SENS_500 - imu_offsets.gyro_z;
            shared_gyro = gyro;
        }

        osDelay(2); /* ~500Hz */
    }
}

void barometer_task(void *argument)
{
    (void)argument;

    for (;;)
    {
        uint8_t adc_buf[3] = {0};

        /* Start D1 (pressure) conversion */
        spi_manager_send_command(DRONE_SPI_SENSOR_BAROMETER, DRONE_BAROMETER_CONVERT_D1_OSR4096, 100);
        osDelay(10);

        uint32_t d1 = 0;
        spi_manager_burst_read(DRONE_SPI_SENSOR_BAROMETER, DRONE_BAROMETER_ADC_READ, adc_buf, 3, 100);
        d1 = ((uint32_t)adc_buf[0] << 16) | ((uint32_t)adc_buf[1] << 8) | adc_buf[2];

        /* Start D2 (temperature) conversion */
        spi_manager_send_command(DRONE_SPI_SENSOR_BAROMETER, DRONE_BAROMETER_CONVERT_D2_OSR4096, 100);
        osDelay(10);

        uint32_t d2 = 0;
        spi_manager_burst_read(DRONE_SPI_SENSOR_BAROMETER, DRONE_BAROMETER_ADC_READ, adc_buf, 3, 100);
        d2 = ((uint32_t)adc_buf[0] << 16) | ((uint32_t)adc_buf[1] << 8) | adc_buf[2];

        /* Convert with second-order compensation */
        DroneBarometerResult_t result;
        drone_barometer_convert_second_order(d1, d2, &result);

        shared_baro_altitude = result.altitude_m;
        shared_baro_temperature = result.temperature;
        shared_baro_pressure = result.pressure;

        osDelay(20); /* ~25Hz (40ms total with two 10ms conversion delays) */
    }
}

void sensor_fusion_task(void *argument)
{
    (void)argument;

    drone_attitude_init(NULL);

    uint32_t last_tick = osKernelGetTickCount();

    for (;;)
    {
        uint32_t now = osKernelGetTickCount();
        float dt = (float)(now - last_tick) / 1000.0f;
        last_tick = now;

        /* Copy volatile shared data */
        DroneImuData_t accel = {shared_accel.x, shared_accel.y, shared_accel.z};
        DroneImuData_t gyro = {shared_gyro.x, shared_gyro.y, shared_gyro.z};
        float baro_alt = shared_baro_altitude;

        drone_attitude_update(&accel, &gyro, baro_alt, dt);

        printf("R:%6.1f P:%6.1f Y:%6.1f A:%6.2f T:%ld P:%ld\r\n",
               drone_attitude_state.roll,
               drone_attitude_state.pitch,
               drone_attitude_state.yaw,
               drone_attitude_state.altitude,
               (long)shared_baro_temperature,
               (long)shared_baro_pressure);

        osDelay(10); /* ~100Hz */
    }
}
