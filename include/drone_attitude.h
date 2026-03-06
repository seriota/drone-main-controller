#ifndef DRONE_ATTITUDE_H
#define DRONE_ATTITUDE_H

#include "drone_imu.h"

typedef struct {
    float alpha;            /* complementary filter coeff (default 0.98) */
    float altitude_alpha;   /* altitude filter coeff (default 0.95) */
} DroneAttitudeConfig_t;

#define DRONE_ATTITUDE_DEFAULT_CONFIG { \
    .alpha = 0.98f,                     \
    .altitude_alpha = 0.95f,            \
}

typedef struct {
    float roll;     /* degrees */
    float pitch;    /* degrees */
    float yaw;      /* degrees (gyro-integrated, drifts without magnetometer) */
    float altitude; /* meters (fused) */
} DroneAttitudeState_t;

extern DroneAttitudeState_t drone_attitude_state;

void drone_attitude_init(const DroneAttitudeConfig_t *config);
void drone_attitude_update(const DroneImuData_t *accel, const DroneImuData_t *gyro,
                           float baro_altitude, float dt);

#endif
