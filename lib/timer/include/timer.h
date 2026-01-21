/**
 * This file uses timer1 and configures ch1 and ch2 as PWM output
 */

#ifndef TIMER_H
#define TIMER_H

#include "gd32vf103.h"

#define PWM_TIMER           TIMER2
#define RCU_PWM_TIMER       RCU_TIMER2

#define PWM_PIN_1           GPIO_PIN_0
#define PWM_TIMER_CH1       TIMER_CH_2
 
#define PWM_PIN_2           GPIO_PIN_1
#define PWM_TIMER_CH2       TIMER_CH_3


void pwm_gpio_config(void);
void pwm_setup(void);

#endif 