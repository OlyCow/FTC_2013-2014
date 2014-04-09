// For collecting most of the data used in teleop (temp, IMU, etc.).
#ifndef DATA_CONTROLLER_H
#define DATA_CONTROLLER_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/twi.h>
#ifndef F_CPU
#define F_CPU 1000000UL
#endif
#include <util/delay.h>
#include <math.h>
#include "../../Libraries/i2cmaster.h"
//#include "../../Libraries/I2C.h"
#include "../../Libraries/MPU6050.h"

#define STATUS_W_INIT			0x01
#define STATUS_W_ACK			0x02
#define STATUS_W_READY			0x03

#define STATUS_R_INIT			0x01
#define STATUS_R_ACK			0x02
#define STATUS_R_RESET_GYRO		0x03
#define STATUS_R_REQ_GYRO_X		0x04
#define STATUS_R_REQ_GYRO_Y		0x05
#define STATUS_R_REQ_GYRO_Z_L	0x06
#define STATUS_R_REQ_GYRO_Z_H	0x07
#define STATUS_R_LED_A_ON		0x10
#define STATUS_R_LED_B_ON		0x11
#define STATUS_R_LED_C_ON		0x12
#define STATUS_R_LED_D_ON		0x13
#define STATUS_R_LED_A_BLINK	0x14
#define STATUS_R_LED_B_BLINK	0x15
#define STATUS_R_LED_C_BLINK	0x16
#define STATUS_R_LED_D_BLINK	0x17
#define STATUS_R_LED_A_OFF		0x18
#define STATUS_R_LED_B_OFF		0x19
#define STATUS_R_LED_C_OFF		0x1A
#define STATUS_R_LED_D_OFF		0x1B
#define STATUS_R_END			0xFF

const unsigned int debounce_delay  = 25*1000; // 25 milliseconds.
const unsigned int LED_port[4] = {PORTD, PORTD, PORTD, PORTB};
const unsigned int LED_pin[4] = {PORTD5, PORTD6, PORTD7, PORTB0};

void setupPins();
void alertA();
void clearA();
void alertB();
void clearB();
void alertC();
void clearC();
void alertD();
void clearD();

#endif // DATA_CONTROLLER_H