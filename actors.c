#include "actors.h"
#include "i2c.h"

#define THRESH_NEG -0.2f
#define THRESH_POS 0.2f
#define MAX_TILT 10.0f   // m/s² maximum tilt to consider
#define MAX_SPEED (4320.0f-1.0f)  // max PWM

QueueHandle_t accelQueue = NULL;

/**
 * @brief Function implementing the Sensor thread.
 */
void AccelSensorTask(void *argument)
{
    LIS2DW12 acc;
    acc.i2c_instance = I2C0;
    LIS2DW12_Init(&acc, acc.i2c_instance);

    TiltData msg;

    for (;;) {
        /* reset to IDLE */
        msg.direction = MOTOR_IDLE;
        msg.speed = 0.0f;

        /* update new data */
        int err = LIS2DW12_ReadAccelerations(&acc);
        if (err != 0) {
            /* error with reading data */

            /* tell the motor to stop to reliably reset i2c */
            TiltData killMsg = { .direction = MOTOR_IDLE, .speed = 0 };
            xQueueOverwrite(accelQueue, &killMsg);
            vTaskDelay(pdMS_TO_TICKS(20));

            /* begin reset of i2c and driver */
            i2c_software_reset_config(acc.i2c_instance, I2C_SRESET_SET);
            i2c_software_reset_config(acc.i2c_instance, I2C_SRESET_RESET);

            i2c_deinit(acc.i2c_instance);
            i2c0_rcu_config();
            i2c0_setup(); /* implementation dependent */

            if (LIS2DW12_Init(&acc, acc.i2c_instance) != 0) {
                gpio_bit_write(GPIOC, GPIO_PIN_13, !gpio_output_bit_get(GPIOC, GPIO_PIN_13));
            }

            vTaskDelay(pdMS_TO_TICKS(10));
            continue; 
        }

        float val = acc.acc_mps2[0];

        /* calculate the sped and direction */
        if(val > MAX_TILT) val = MAX_TILT;
        if(val < -MAX_TILT) val = -MAX_TILT;

        if (val < THRESH_NEG) {
            /* counter clockwise */ 
            msg.direction = MOTOR_BACKWARD;
            msg.speed = (-val / MAX_TILT) * MAX_SPEED;
        } else if (val > THRESH_POS) {
            /* clockwise */
            msg.direction = MOTOR_FORWARD;
            msg.speed = (val / MAX_TILT) * MAX_SPEED;
        } else {
            msg.direction = MOTOR_IDLE;
            msg.speed = 0.0;
        }

        xQueueOverwrite(accelQueue, &msg);
        vTaskDelay(pdMS_TO_TICKS(20));

    }
}

/**
 * @brief Function implementing the Motor thread.
 */
void MotorControllerTask(void *argument)
{
    TiltData msg;
    uint32_t current_speed = 0;

    for (;;) {
        if(xQueueReceive(accelQueue, &msg, pdMS_TO_TICKS(40)) == pdFALSE) {
            /**
             * Have not received anything from the I2C sensor for 40ms 
             * some unknown error has occured, reset the struct and 
             * make motor IDLE again.
             */
            msg.direction = MOTOR_IDLE;
            msg.speed = 0.0f;
        }
        
        /* speed already converted to duty cycle, update CCR reg */ 
        switch (msg.direction) {
            case MOTOR_FORWARD:
                timer_channel_output_pulse_value_config(PWM_TIMER, PWM_TIMER_CH2, 0);
                timer_channel_output_pulse_value_config(PWM_TIMER, PWM_TIMER_CH1, (uint32_t) msg.speed);
                break;
            case MOTOR_BACKWARD:
                timer_channel_output_pulse_value_config(PWM_TIMER, PWM_TIMER_CH2, (uint32_t) msg.speed);
                timer_channel_output_pulse_value_config(PWM_TIMER, PWM_TIMER_CH1, 0);
                break;
            case MOTOR_IDLE:
                timer_channel_output_pulse_value_config(PWM_TIMER, PWM_TIMER_CH2, 0);
                timer_channel_output_pulse_value_config(PWM_TIMER, PWM_TIMER_CH1, 0);
                break;
        }

    }
}

/**
  * @brief  FreeRTOS initialization
  */
void freertos_init(void) {

  /* creation of accelQueue */
  accelQueue = xQueueCreate(1, sizeof(TiltData));

  /* creation of Sensor */
  xTaskCreate(AccelSensorTask, NULL, 512, ( void * ) 1, tskIDLE_PRIORITY + 1, NULL);

  /* creation of Motor */
  xTaskCreate(MotorControllerTask, NULL, 256, ( void * ) 1, tskIDLE_PRIORITY + 2, NULL);

}
