/*
 *
 * LIS2DW12 accelerometer I2C driver
 *
 * Author: Karzan M.
 * Edited on: 14 Jan, 2025
 * Datasheet download link: https://www.st.com/resource/en/datasheet/lis2dw12.pdf
 *
 */

#include "LIS2DW12.h"
#include <stdint.h>

#define LIS2DW12_OFFSET_TEMP    25.0f                   /* Reference temp for offset */
#define LIS2DW12_OFFSET_LSB     0.0f                    /* 0 LSB offset @ 25 degC */
#define LIS2DW12_SLOPE          0.0625f                 /* inverse of LSB/degC = degC/LSB (p. 8, TSDr) */
#define LIS2DW12_SENSITIVITY    (0.000244f * 9.81f)     /* page 6 sensitivity 0.244 */

/**
 * The I2C transactions can get stuck if any loop fails 
 * This is the reason for having a timeout everytime you wait for a flag 
 * If I2C is 100KHz, simply counting to 100 cycles should be enough of a 
 * bare-bones timeout. 
 */

#define I2C_TIMEOUT_CYCLES 100000

/* wait UNTIL the flag is SET */
#define I2C_WAIT_FLAG(instance, flag) \
    { \
        volatile int32_t timeout = I2C_TIMEOUT_CYCLES; \
        while (!i2c_flag_get(instance, flag)) { \
            if (--timeout <= 0) return -1; /* ABORT! */ \
        } \
    }

/* wait WHILE the flag is SET (1) */
#define I2C_WAIT_FLAG_SET(instance, flag) \
    { \
        volatile int32_t timeout = I2C_TIMEOUT_CYCLES; \
        while (i2c_flag_get(instance, flag)) { \
            if (--timeout <= 0) return -1; /* ABORT! */ \
        } \
    }

static int8_t LIS2DW12_ReadRegister( LIS2DW12 *dev, uint8_t reg, uint8_t *data );

/*
 * DEINITIALIZATION
 */
void LIS2DW12_DeInit( LIS2DW12 *dev ) {
    /*
     * DATASHEET: page 37
     * Soft reset control registers
     */
    uint8_t reg_data = 0x40;
    LIS2DW12_WriteRegister(dev, LIS2DW12_REG_CTRL2, &reg_data, 1);
}

/*
 * INITIALIZATION
 */

/**
 * \brief initialize accelerometer 
 * \param dev instane of accelerometer struct 
 * \param i2c_user_instance i2c instance to use 
 * \retval uint8_t : 0 on success, 255 on fail
 */
uint8_t LIS2DW12_Init( LIS2DW12 *dev, I2C_Inst_t i2c_user_instance ) {

    // ultra simple function contract stuff 
    if ( dev == NULL ) return 255;
    if ( i2c_user_instance != I2C_INSTANCE_0 && i2c_user_instance != I2C_INSTANCE_1 ) return 255;

    /* Setup struct parameters */
    dev->i2c_instance = i2c_user_instance;
    dev->acc_mps2[0] = dev->acc_mps2[1] = dev->acc_mps2[2] = 0.0f;
    dev->temp_C = 0.0f;
    
    /* reset all control registers */
    LIS2DW12_DeInit(dev);
    /* small delay before writing to registers after a reset */
    volatile int32_t n = 540000; /* @ 108Mhz ~ 1ms */
    while (n--) __asm volatile("nop");

    /* Ensure correct device ID */
    uint8_t reg_data;

    LIS2DW12_ReadRegister(dev, LIS2DW12_REG_WHO_AM_I, &reg_data);

    if ( reg_data != LIS2DW12_ID ) {

        return 255;     /* Can't talk to the Accelerometer */

    }

    int err = 0; /* error count, if any of the write fail then something is wrong */

    /*
     * Page 15 and 36 DATASHEET
     * Setup ODR and power mode ( ODR 200 Hz, Low power mode 4, 14 bit 77uA supply current )
     */
     reg_data = 0x63;
     err += LIS2DW12_WriteRegister(dev, LIS2DW12_REG_CTRL1, &reg_data, 1);

     /*
      * Page 37 DATASHEET
      * Turn on Block Data Update so data does not get updated while reading
      */
     reg_data = 0x0;
     err += LIS2DW12_ReadRegister(dev, LIS2DW12_REG_CTRL2, &reg_data);
     reg_data |= (1 << 2);
     err += LIS2DW12_WriteRegister(dev, LIS2DW12_REG_CTRL2, &reg_data, 1);

     /*
      * Page 41 DATASHEET
      * Setup filters and scale selectoin ( low noise config, LP route, bandwidth = ODR/4, sensitivity= +-2g )
      */
     reg_data = 0x44;
     err += LIS2DW12_WriteRegister(dev, LIS2DW12_REG_CTRL6, &reg_data, 1);

     /**
      * Page 39 and 55 DATASHEET
      * Add this if you want an interrupt driven approach
      * INT1 will have a data-ready interrupt, add little
      * code for EXTI on MCU and you can read data every 20ms 
      */
	// reg_data = 0x01;
	// LIS2DW12_WriteRegister(dev, LIS2DW12_REG_CTRL4_INT1_PAD_CTRL, &reg_data, 1);
	// reg_data = 0xB0;
	// LIS2DW12_WriteRegister(dev, LIS2DW12_REG_CTRL7, &reg_data, 1);

     return err;
}

/*
 * DATA AQUISITION
 */

int8_t LIS2DW12_ReadTemperature( LIS2DW12 *dev ) {

    /* DATASHEET: page 35 */

    /* Raw 16 bit word in twos complement for temperature, 12 bit resolution */
    uint8_t reg_data[2];

    /* Read 2 bytes starting from the LSB output register [LSB, MSB] */
    int err = LIS2DW12_ReadRegisters(dev, LIS2DW12_REG_OUT_T_L, reg_data, 2);
    if (err == -1) return err;

    /* combine to 12 bit data (SIGNED 2 complement) */
    int16_t temp_raw = (int16_t)((int16_t)reg_data[1] << 8 | (reg_data[0] & 0xF0)) >> 4;

    /*
     * DATASHEET: page 8
     * convert to degC ( offset @ 25 degC = 0 LSB, slope @ 12 bits resolution = 16 LSB/degC
     */
    dev->temp_C = LIS2DW12_SLOPE * ( (float) temp_raw - LIS2DW12_OFFSET_LSB ) + LIS2DW12_OFFSET_TEMP;

    return 0;
}

int8_t LIS2DW12_ReadAccelerations( LIS2DW12 *dev ) {

    /* DATASHEET: page 43 */

    /* raw 16 bit word in twos complement for accleration, 14 bit resolution */
    uint8_t reg_data[6];

    /* read 2 bytes starting from the LSB output register [LSB, MSB] */
    int err = LIS2DW12_ReadRegisters(dev, LIS2DW12_REG_OUT_X_L, reg_data, 6);
    if (err == -1) return err;

    /* Combine register readings to combine raw 14 bit accelerometer readings */
    int16_t acc_raw_signed[3];

    /* combine to 14 bit singed data for x axis acceleration */
    acc_raw_signed[0] = (int16_t)((int16_t)reg_data[1] << 8 | (reg_data[0] & 0xFC )) >> 2;
    acc_raw_signed[1] = (int16_t)((int16_t)reg_data[3] << 8 | (reg_data[2] & 0xFC )) >> 2;
    acc_raw_signed[2] = (int16_t)((int16_t)reg_data[5] << 8 | (reg_data[4] & 0xFC )) >> 2;

    /* convert raw data to acceleration ( m/s^2 ) */
    dev->acc_mps2[0] = LIS2DW12_SENSITIVITY * acc_raw_signed[0];
    dev->acc_mps2[1] = LIS2DW12_SENSITIVITY * acc_raw_signed[1];
    dev->acc_mps2[2] = LIS2DW12_SENSITIVITY * acc_raw_signed[2];

    return 0;
}


/*
 * LOW-LEVEL implementation based funtions ( change this if using another microcontroller )
 */

/** 
 * \brief simple simple helper for readregisters 
 */
static int8_t LIS2DW12_ReadRegister( LIS2DW12 *dev, uint8_t reg, uint8_t *data ) {

    return LIS2DW12_ReadRegisters( dev, reg, data, 1 );

}

/**
 * \brief read one or multiple registers 
 * \param dev pointer to acceleromtert struct instance 
 * \param reg register to read 
 * \param data pointer to buffer to save to 
 * \param length length of the data buffer 
 * \return 0 if sucess, -1 if fail; when failed the the driver has to be reinitialized 
 */
int8_t LIS2DW12_ReadRegisters( LIS2DW12 *dev, uint8_t reg, uint8_t *data, uint8_t length ){

    /* Based on the master read sequence described @ p. 346 solution A */

    /** 
     * Since an actual sensor is connected; the address has to be sent first, 
     * then a second START condition can be initiated to read data 
     * 
     * Not stopping before a new start ensures one master ( this onw ) is
     * the only one active on the bus
     */

    /* wait until the i2c is no longer busy to not override previous writes */
    I2C_WAIT_FLAG_SET(dev->i2c_instance, I2C_FLAG_I2CBSY);

    /* request i2c to generate a START condition on the i2c bus */ 
    i2c_start_on_bus(dev->i2c_instance);
    /* wait for it to enter master mode */
    I2C_WAIT_FLAG(dev->i2c_instance, I2C_FLAG_SBSEND);

    /* Send the slave address */
    i2c_master_addressing(dev->i2c_instance, LIS2DW12_I2C_ADDR, I2C_TRANSMITTER);
    /* once the address is sent, clear the ADDSEND (ADD10SEND if 10 bit address) */
    I2C_WAIT_FLAG(dev->i2c_instance, I2C_FLAG_ADDSEND);
    i2c_flag_clear(dev->i2c_instance, I2C_FLAG_ADDSEND);

    /* wait until TBE is set to be able to transmit a byte */
    I2C_WAIT_FLAG(dev->i2c_instance, I2C_FLAG_TBE);
    /* send command byte */
    i2c_data_transmit(dev->i2c_instance, reg);
    I2C_WAIT_FLAG(dev->i2c_instance, I2C_FLAG_TBE);

    /* generate a new START sigal */
    i2c_start_on_bus(dev->i2c_instance);
    /* wait for it to enter master mode */
    I2C_WAIT_FLAG(dev->i2c_instance, I2C_FLAG_SBSEND);

    /* Send the slave address */
    i2c_master_addressing(dev->i2c_instance, LIS2DW12_I2C_ADDR, I2C_RECEIVER);
    /* once the address is sent, clear the ADDSEND (ADD10SEND if 10 bit address) */
    I2C_WAIT_FLAG(dev->i2c_instance, I2C_FLAG_ADDSEND);
    i2c_flag_clear(dev->i2c_instance, I2C_FLAG_ADDSEND);
   
    /* Here you can continously read bytes/data everytime RBNE flag is set */
    while (length--) {
        /* wait until we get the RBNE flag to read the data */

        if (length == 0) {
            /* Prepare for last byte with NACK */
            i2c_ack_config(dev->i2c_instance, I2C_ACK_DISABLE);
            /* Set STOP bit before the last byte */
            i2c_stop_on_bus(dev->i2c_instance);
        }

        I2C_WAIT_FLAG(dev->i2c_instance, I2C_FLAG_RBNE);
        /* read one byte and move down the array 1 step */
        *(data++) = i2c_data_receive(dev->i2c_instance);
    }

    /* re-enable ACK and wait until the STOP bit has been set */
    i2c_ack_config(dev->i2c_instance, I2C_ACK_ENABLE);

    return 0;
}

/**
 * \brief write to one or multiple registers 
 * \param dev pointer to accelerometer instance
 * \param reg register to write to 
 * \param data pointer to data buffer with values to write 
 * \param length size of the data buffer 
 * \return 0 if sucess, -1 if fail; when failed the the driver has to be reinitialized 
 */
int8_t LIS2DW12_WriteRegister( LIS2DW12 *dev, uint8_t reg, uint8_t *data, uint8_t length ){

    /* based on the master transmit sequence descried @ p. 344 from the gd32vf103 user manual rev 1.8 */

    /* wait until the i2c is no longer busy to not override previous writes */
    I2C_WAIT_FLAG_SET(dev->i2c_instance, I2C_FLAG_I2CBSY);

    /* following the sequence from user manual @ p. 347  */ 

    /* request i2c to generate a START condition on the i2c bus */ 
    i2c_start_on_bus(dev->i2c_instance);
    /* wait for it to enter master mode */
    I2C_WAIT_FLAG(dev->i2c_instance, I2C_FLAG_SBSEND);

    /* Send the slave address */
    i2c_master_addressing(dev->i2c_instance, LIS2DW12_I2C_ADDR, I2C_TRANSMITTER);
    /* once the address is sent, clear the ADDSEND (ADD10SEND if 10 bit address) */
    I2C_WAIT_FLAG(dev->i2c_instance, I2C_FLAG_ADDSEND);
    i2c_flag_clear(dev->i2c_instance, I2C_FLAG_ADDSEND);

    /* wait until TBE is set to be able to transmit a byte */
    I2C_WAIT_FLAG(dev->i2c_instance, I2C_FLAG_TBE);
    /* send command byte */
    i2c_data_transmit(dev->i2c_instance, reg);
    I2C_WAIT_FLAG(dev->i2c_instance, I2C_FLAG_TBE);

    // /* Here you can continously send bytes/data everytime TBE flag is set */
    while (length-- > 0) {
        /* write one byte */
        i2c_data_transmit(dev->i2c_instance, *data++);
        /* wait until TBE is set again */
        I2C_WAIT_FLAG(dev->i2c_instance, I2C_FLAG_TBE);
    }

    /* send STOP signal and end the transmission */
    i2c_stop_on_bus(dev->i2c_instance);
    /* wait until the STOP bit has been set */
    // while( I2C_CTL0(dev->i2c_instance) & I2C_CTL0_STOP );
    return 0;

}
