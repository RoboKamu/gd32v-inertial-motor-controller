#include "gd32vf103.h"
#include "i2c.h"
#include "timer.h"

#include "FreeRTOS.h"
#include "actors.h"

//#define NDBG

int32_t main (void) {

    /* init pwm */
    pwm_gpio_config();
    pwm_setup();
    /* init i2c */
    i2c0_rcu_config();
    i2c0_setup();


#ifndef NDBG
    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
#endif

    freertos_init();
    vTaskStartScheduler();
    
    while(1);

}