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
#include "../../Libraries/I2C.h"
#include "../../Libraries/MPU6050.h"

// Number of I/O lines.
#define NXT_LINE_NUM		6

// TODO: Enum this?
// Each NXT line.
#define NXT_LINE_A			PB6
#define NXT_LINE_B			PB7
#define NXT_LINE_C			PD5
#define NXT_LINE_D			PD6
#define NXT_LINE_E			PD7
#define NXT_LINE_F			PB0

// Ports of each NXT line.
#define NXT_LINE_A_PORT		PORTB
#define NXT_LINE_B_PORT		PORTB
#define NXT_LINE_C_PORT		PORTD
#define NXT_LINE_D_PORT		PORTD
#define NXT_LINE_E_PORT		PORTD
#define NXT_LINE_F_PORT		PORTB

// Clock counts for debounce to trigger.
// 9999 = 10 ms
#define DEBOUNCE_COUNTS		9999

void setupPins(void);
void alert(void);
void clear(void);

#endif // COMM_CONTROLLER_H
