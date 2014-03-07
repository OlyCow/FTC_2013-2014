// For collecting most of the data used in teleop (temp, IMU, etc.).
#ifndef DATA_CONTROLLER_H
#define DATA_CONTROLLER_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/twi>
#ifndef F_CPU
#define F_CPU 1000000UL
#endif
#include <util/delay.h>
#include <math.h>
#include "../../Libraries/i2cmaster.h"
//#include "../../Libraries/I2C.h"
#include "../../Libraries/MPU6050.h"

const unsigned int debounce_delay  = 25*1000; // 25 milliseconds.

void setupPins(void);

#endif // DATA_CONTROLLER_H