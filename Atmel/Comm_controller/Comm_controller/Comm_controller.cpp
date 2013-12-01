// For communicating with the NXT (via the SuperPro board).

#include <avr/io.h>
#include <avr/interrupt.h>	// Include <util/atomic.h> for proper use.
#include <util/atomic.h>	// For interrupts^.
#include <util/twi.h>

#ifndef F_CPU
#define F_CPU 1000000UL
#endif
#include <util/delay.h>

#include <math.h>

#include "Comm_controller.h"

int main(void)
{
	setupPins();
	while (true) {
		// TODO: Write actual code.
		// Initialize.
		// Read data from other AVRs.
		// Read data from NXT.
		// Write data to NXT.
	}
}