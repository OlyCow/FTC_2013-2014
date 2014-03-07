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

const unsigned int debounce_delay  = 25*1000; // 25 milliseconds.

void setupPins();
void alert();
void clear();

#endif // COMM_CONTROLLER_H
