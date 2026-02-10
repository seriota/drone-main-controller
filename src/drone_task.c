#include "drone_task.h"
#include "drone_propeller.h"

const osThreadAttr_t propeller_task_attributes = {
    .name = "propellerTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void propeller_task(void *argument)
{
    for (;;)
    {
        drone_propeller_set_speed(DRONE_PROPELLER_FRONT_RIGHT, 8); // Set speed to 50%
        osDelay(5000);                                             // Delay for 1 second
        drone_propeller_set_speed(DRONE_PROPELLER_FRONT_RIGHT, 0); // Set speed to 50%
        osDelay(5000);                                             // Delay for 1 second
                                                                   // drone_propeller_set_speed(DRONE_PROPELLER_FRONT_RIGHT, 60); // Set speed to 0%
                                                                   // osDelay(5000); // Delay for 1 second
                                                                   // drone_propeller_set_speed(DRONE_PROPELLER_FRONT_RIGHT, 80); // Set speed to 0%
                                                                   // osDelay(5000); // Delay for 1 second
                                                                   // drone_propeller_set_speed(DRONE_PROPELLER_FRONT_RIGHT, 100); // Set speed to 0%
                                                                   // osDelay(5000); // Delay for 1 second
    }
}

void drone_initialize_tasks()
{
    osKernelInitialize();
    osThreadNew(propeller_task, NULL, &propeller_task_attributes);
    osKernelStart();
}