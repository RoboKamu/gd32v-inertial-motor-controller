#include "i2c.h"
#include "gd32vf103.h"


void i2c0_rcu_config(void);
void i2c0_setup(void);

void i2c0_rcu_config(void)
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* enable I2C0 clock */
    rcu_periph_clock_enable(RCU_I2C0);
    /* enable the clock for alterantive functions on gpio */
    rcu_periph_clock_enable(RCU_AF);
}

/* setups i2c @ 100khz, SCL = PPB6, SDA = PB7 */
void i2c0_setup(void) {
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7 );

    /* i2c normal 100khz speed with scl as 2:1 LWO:HIGH */
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);

    /* If using multi-master mode this is essential but irrelevant for my application */
    // i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_SLAVE_ADDRESS7);

    /* enable I2C0 */
    i2c_enable(I2C0);
    /* enable ACK for first the inital usage */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}