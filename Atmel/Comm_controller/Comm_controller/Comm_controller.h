// For communicating with the NXT (via the SuperPro board).
#ifndef COMM_CONTROLLER_H
#define COMM_CONTROLLER_H

#include <avr/io.h>
#include <avr/interrupt.h>	// Include <util/atomic.h> for proper use.
#include <util/atomic.h>	// For interrupts^.
#include <util/twi.h>
#ifndef F_CPU
#define F_CPU 1000000UL
#endif
#include <util/delay.h>
#include <math.h>
#include "../../Libraries/i2cmaster.h"
//#include "../../Libraries/I2C.h"
#include "../../Libraries/MPU6050.h"

// Number of I/O lines.
#define NXT_LINE_NUM		6

// Each NXT line.
#define NXT_LINE_F			PB6
#define NXT_LINE_E			PB7
#define NXT_LINE_D			PD5
#define NXT_LINE_C			PD6
#define NXT_LINE_B			PD7
#define NXT_LINE_A			PB0

// Ports of each NXT line.
#define NXT_LINE_F_PORT		PORTB
#define NXT_LINE_E_PORT		PORTB
#define NXT_LINE_D_PORT		PORTD
#define NXT_LINE_C_PORT		PORTD
#define NXT_LINE_B_PORT		PORTD
#define NXT_LINE_A_PORT		PORTB

// NXT read codes. These are long because they're 32 bits.
#define NXT_CODE_COMM_RESET		0xFFFFFFFF
#define NXT_CODE_ROT_RESET		0x00000001
#define NXT_CODE_CUBE_RESET		0x00000002

// Yah.
#define STATUS_R_INIT			0x01
#define STATUS_R_ACK			0x02
#define STATUS_R_READY			0x03

#define STATUS_W_INIT			0x01
#define STATUS_W_ACK			0x02
#define STATUS_W_RESET_GYRO		0x03
#define STATUS_W_REQ_GYRO_X		0x04
#define STATUS_W_REQ_GYRO_Y		0x05
#define STATUS_W_REQ_GYRO_Z		0x06
#define STATUS_W_LED_A_ON		0x10
#define STATUS_W_LED_B_ON		0x11
#define STATUS_W_LED_C_ON		0x12
#define STATUS_W_LED_D_ON		0x13
#define STATUS_W_LED_A_BLINK	0x14
#define STATUS_W_LED_B_BLINK	0x15
#define STATUS_W_LED_C_BLINK	0x16
#define STATUS_W_LED_D_BLINK	0x17
#define STATUS_W_LED_A_OFF		0x18
#define STATUS_W_LED_B_OFF		0x19
#define STATUS_W_LED_C_OFF		0x1A
#define STATUS_W_LED_D_OFF		0x1B
#define STATUS_W_END			0xFF

const unsigned int debounce_delay  = 25*1000; // 25 milliseconds.

void setupPins();

#endif // COMM_CONTROLLER_H
