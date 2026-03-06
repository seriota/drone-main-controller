#ifndef DRONE_TASK_H
#define DRONE_TASK_H

#include "cmsis_os.h"

extern osThreadId_t propeller_task_handler;
extern const osThreadAttr_t propeller_task_attributes;

extern const osThreadAttr_t imu_task_attributes;
extern const osThreadAttr_t barometer_task_attributes;
extern const osThreadAttr_t sensor_fusion_task_attributes;

void propeller_task(void *argument);
void imu_task(void *argument);
void barometer_task(void *argument);
void sensor_fusion_task(void *argument);
void drone_initialize_tasks(void);

#endif /* DRONE_TASK_H */
