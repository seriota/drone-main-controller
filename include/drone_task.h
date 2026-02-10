#ifndef DRONE_TASK_H
#define DRONE_TASK_H

#include "cmsis_os.h"

osThreadId_t propeller_task_handler;
extern const  osThreadAttr_t propeller_task_attributes;

void propeller_task(void *argument);
void drone_initialize_tasks();
#endif /* DRONE_TASK_H */