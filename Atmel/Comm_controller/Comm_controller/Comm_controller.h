// For communicating with the NXT (via the SuperPro board).
#ifndef COMM_CONTROLLER_H
#define COMM_CONTROLLER_H

void setupPins();

void setupPins() {
	
	// Set up I/O port directions with the DDRx registries. 1=out, 0=in.
	// These can be changed later in the program (and some sensors need
	// to do this, e.g. ultrasonic sensors).
	//----------------SCHEMATIC----------------
	//  1-PC6: RESET			28-PC5: LED_A (SCL)
	//  2-PD0: SCLK_NXT			27-PC4: LED_B (SDA)
	//  3-PD1: MISO_NXT			26-PC3: LIGHT_SEL_A
	//  4-PD2: SS_SEL_A			25-PC2: LIGHT_SEL_B
	//  5-PD3: SS_SEL_B			24-PC1: LIGHT_SEL_C
	//  6-PD4: SS_SEL_C			23-PC0: LIGHT_READ
	//  7-[VCC]					22-[GND]
	//  8-[GND]					21-[AREF]
	//  9-PB6: MOSI_NXT_A		20-[AVCC]
	// 10-PB7: MOSI_NXT_B		19-PB5: SCLK_MCU
	// 11-PD5: MOSI_NXT_C		18-PB4: MISO_MCU
	// 12-PD6: MOSI_NXT_D		17-PB3: MOSI_MCU
	// 13-PD7: MOSI_NXT_E		16-PB2: SS_MCU_WRITE
	// 14-PB0: MOSI_NXT_F		15-PB1: LIFT_RESET (cube counter)
	DDRB = ((1<<PB0) |
			(0<<PB1) |
			(1<<PB2) |
			(1<<PB3) |
			(0<<PB4) |
			(1<<PB5) |
			(1<<PB6) |
			(1<<PB7));
	DDRC = ((0<<PC0) |
			(1<<PC1) |
			(1<<PC2) |
			(1<<PC3) |
			(1<<PC4) |
			(1<<PC5) |
			(0<<PC6)); // No bit 7.
	DDRD = ((0<<PD0) |
			(0<<PD1) |
			(1<<PD2) |
			(1<<PD3) |
			(1<<PD4) |
			(1<<PD5) |
			(1<<PD6) |
			(1<<PD7));
	
	// (PORTx registries) Initialize outputs to 0 (LOW), and enable internal
	// pull-up resistors for the appropriate inputs (most notably the SDA &
	// SCL pins). 1=pull-up resistor enabled. For details, see schematic for
	// the DDRx registries' set-up.
	PORTB = ((0<<PB0) |
			 (1<<PB1) |
			 (0<<PB2) |
			 (0<<PB3) |
			 (1<<PB4) | // TODO: Not sure of this one. (MISO)
			 (0<<PB5) |
			 (0<<PB6) |
			 (0<<PB7));
	PORTC = ((1<<PC0) | // TODO: Unnecessary? (multiplexer read pin)
			 (0<<PC1) |
			 (0<<PC2) |
			 (0<<PC3) |
			 (0<<PC4) |
			 (0<<PC5) |
			 (1<<PC6)); // No bit 7. TODO: Is pull-up necessary here?
	PORTD = ((1<<PD0) | // TODO: Unnecessary? (SCLK_NXT)
			 (1<<PD1) | // TODO: Unnecessary? (MISO_NXT)
			 (0<<PD2) |
			 (0<<PD3) |
			 (0<<PD4) |
			 (0<<PD5) |
			 (0<<PD6) |
			 (0<<PD7));
	
	// TODO: Set up TWI. Remember pull-up resistors!
}

#endif // COMM_CONTROLLER_H
