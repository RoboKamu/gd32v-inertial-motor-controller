#ifndef TASKS_H
#define TASKS_H

#include "LIS2DW12.h"   /* acceletometer lib */
#include "timer.h"      /* PWM functions */

#include "FreeRTOS.h" /* Must come first. */
#include "task.h" /* RTOS task related API prototypes. */
#include "queue.h" /* RTOS queue related API prototypes. */

void freertos_init(void); 

void AccelSensorTask(void *argument);
void MotorControllerTask(void *argument);

extern QueueHandle_t accelQueue;

typedef enum {
    MOTOR_IDLE,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
} MotorDirection;

typedef struct {
    float speed;
    MotorDirection direction;
} TiltData;

#endif
