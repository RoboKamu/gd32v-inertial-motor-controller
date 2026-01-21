#include "timer.h"

#define PWM_TIMER           TIMER2
#define RCU_PWM_TIMER       RCU_TIMER2

#define PWM_PIN_1           GPIO_PIN_0
#define PWM_TIMER_CH1       TIMER_CH_2
 
#define PWM_PIN_2           GPIO_PIN_1
#define PWM_TIMER_CH2       TIMER_CH_3

void pwm_gpio_config(void) {
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_PIN_1);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_PIN_2);
}

void pwm_setup(void) {

    /**
     * Simple setup of timer 
     * SYSCLK = 108 Mhz 
     * Freq aim for motor = 25 khz
     * Prescaler = 0 because we want good resolution 
     * => TIMERCLK = 108 mhz / (prescaler + 1) = 108 mhz 
     * 25Khz in steps : 108 mhz / 25 khz = 4320
     * -count up to 4320.. reset to 0.. count up again.-
     * PWM duty cycle is then what percentage of those steps are high values  
     */

    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_PWM_TIMER);

    timer_deinit(PWM_TIMER);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* PWM_TIMER configuration */
    timer_initpara.prescaler         = 1-1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 4320-1;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(PWM_TIMER, &timer_initpara);

    /* initialize TIMER channel output parameter struct */
    timer_channel_output_struct_para_init(&timer_ocinitpara);
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(PWM_TIMER,PWM_TIMER_CH1,&timer_ocinitpara);
    timer_channel_output_config(PWM_TIMER,PWM_TIMER_CH2,&timer_ocinitpara);

    /* PWM_TIMER_ch1 configuration in PWM mode1 */
    timer_channel_output_pulse_value_config(PWM_TIMER,PWM_TIMER_CH1, 0);
    timer_channel_output_mode_config(PWM_TIMER,PWM_TIMER_CH1,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(PWM_TIMER,PWM_TIMER_CH1,TIMER_OC_SHADOW_DISABLE);

    /* PWM_TIMER_ch2 configuration in PWM mode1 */
    timer_channel_output_pulse_value_config(PWM_TIMER,PWM_TIMER_CH2, 0);
    timer_channel_output_mode_config(PWM_TIMER,PWM_TIMER_CH2,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(PWM_TIMER,PWM_TIMER_CH2,TIMER_OC_SHADOW_DISABLE);

    timer_auto_reload_shadow_enable(PWM_TIMER);
    timer_enable(PWM_TIMER);

}
