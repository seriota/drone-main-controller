#include "drone_attitude.h"
#include <math.h>

#define RAD_TO_DEG 57.2957795131f

DroneAttitudeState_t drone_attitude_state = {0};

static DroneAttitudeConfig_t att_cfg;

void drone_attitude_init(const DroneAttitudeConfig_t *config)
{
    if (config != NULL) {
        att_cfg = *config;
    } else {
        DroneAttitudeConfig_t defaults = DRONE_ATTITUDE_DEFAULT_CONFIG;
        att_cfg = defaults;
    }

    drone_attitude_state.roll = 0.0f;
    drone_attitude_state.pitch = 0.0f;
    drone_attitude_state.yaw = 0.0f;
    drone_attitude_state.altitude = 0.0f;
}

void drone_attitude_update(const DroneImuData_t *accel, const DroneImuData_t *gyro,
                           float baro_altitude, float dt)
{
    /* Accelerometer-based angles */
    float accel_roll = atan2f(accel->y, accel->z) * RAD_TO_DEG;
    float accel_pitch = atan2f(-accel->x, sqrtf(accel->y * accel->y + accel->z * accel->z)) * RAD_TO_DEG;

    /* Complementary filter: fuse gyro integration with accelerometer */
    float a = att_cfg.alpha;
    drone_attitude_state.roll = a * (drone_attitude_state.roll + gyro->x * dt) + (1.0f - a) * accel_roll;
    drone_attitude_state.pitch = a * (drone_attitude_state.pitch + gyro->y * dt) + (1.0f - a) * accel_pitch;

    /* Yaw: gyro integration only (no magnetometer correction) */
    drone_attitude_state.yaw += gyro->z * dt;

    /* Normalize yaw to -180..180 */
    if (drone_attitude_state.yaw > 180.0f) {
        drone_attitude_state.yaw -= 360.0f;
    } else if (drone_attitude_state.yaw < -180.0f) {
        drone_attitude_state.yaw += 360.0f;
    }

    /* Altitude complementary filter */
    float aa = att_cfg.altitude_alpha;
    drone_attitude_state.altitude = aa * drone_attitude_state.altitude + (1.0f - aa) * baro_altitude;
}
