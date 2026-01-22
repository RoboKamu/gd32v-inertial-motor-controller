<h1 align="center"> Real-Time Inertial Motor Controller (GD32V RISC-V) </h1>

<div align="center">
<i>
A fault-tolerant, event-driven motor control system on the GD32V (RISC-V) Longan Nano.
Featuring a custom non-blocking driver for the LIS2DW12 accelerometer capable of automatic bus recovery during EMI events.
</i>
</div>

## Overview

This project demonstrates a robust embedded architecture using **FreeRTOS** and the **Active Object** pattern. The core focus was to create a "crash-proof" sensor driver that can recover from I2C bus lockups caused by DC motor electrical noise (EMI) without resetting the entire microcontroller.

It serves as a study in concurrency, hardware abstraction, and fault-tolerance on the RISC-V architecture.

## Hardware

* **MCU:** Sipeed Longan Nano (GD32VF103CBT6 - RISC-V)
* **Sensor:** LIS2DW12 3-Axis Accelerometer
* **Actuator:** Standard DC Motor via DRV8833 H-Bridge
* **Wiring:** No custom PCB was used; modules are connected via breadboard for prototyping.

| Module       | Pin | Function | 
| --------     | -------  | ------- | 
| I2C sensor   | PB6      | SCL |  
|              | PB7      | SDA | 
| Motor Driver | PB0      | PWM (Timer 2 CH2) | 
|              | PB1      | PWM (Timer 2 CH3) | 
| Debug        | PC13     | Status LED | 

## Software Architecture

The system runs on **FreeRTOS** with two primary Active Objects (Tasks) communicating via a thread-safe Queue:

1. **Sensor Actor:** Polls the accelerometer every 20ms and converts raw tilt vectors into velocity commands.
2. **Motor Actor:** Receives commands and updates the PWM duty cycle. It implements a **failsafe timeout**: if no new data arrives for >100ms (due to sensor failure), the motor performs an emergency stop.

### Fault-Tolerant Driver Design

The LIS2DW12 driver was written from scratch to handle **I2C Bus Lockups**. Standard drivers often hang in infinite `while` loops if a motor spark corrupts the clock line. This driver uses macro-based timeouts to detect these hangs and trigger a peripheral reset.

**Key Implementation Details:**

```c
#define I2C_TIMEOUT_CYCLES 100000

/* Robust Wait Macro: Returns error (-1) instead of hanging forever */
#define I2C_WAIT_FLAG(instance, flag) \
    { \
        volatile int32_t timeout = I2C_TIMEOUT_CYCLES; \
        while (!i2c_flag_get(instance, flag)) { \
            if (--timeout <= 0) return -1; /* ABORT TRANSACTION */ \
        } \
    }

```

If `I2C_WAIT_FLAG` returns `-1`, the Sensor Actor initiates a **Software Reset** sequence on the I2C peripheral to clear the bus and resume operation automatically.

## Build & Flash

This project uses the GD32V Linux toolchain.

1. Clone repository into your projects folder.
2. Put the Longan Nano into DFU Bootloader mode.
3. Compile and flash:

```bash
make dfu
```

## Demo

https://github.com/user-attachments/assets/111c7d91-e7e5-4653-87cf-200e6e2b7cbc

This video demonstrates real-time motor velocity control based on accelerometer tilt vectors. 

**Key Feature Showcase: Fault Injection & Recovery**
The video specifically highlights the system's robustness against connection failures:
1.  **Signal Loss:** The I2C lines (green wires) are physically disconnected while the motor is running.
2.  **Failsafe Activation:** The system detects the timeout immediately, triggering an **Emergency Stop** (Motor Speed = 0) and entering an error state (Blinking LED).
3.  **Automatic Recovery:** Upon reconnection, the driver detects the restored bus, re-initializes the sensor, and resumes normal operation without requiring a hard reset.

## Acknowledgements

* Based on the open-source [GD32V Examples by Linus Remahl](https://github.com/linusreM/RISC-V-IO-card-examples-Makefile).
