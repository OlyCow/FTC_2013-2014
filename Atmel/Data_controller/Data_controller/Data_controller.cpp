// For collecting most of the data used in teleop (temp, IMU, etc.).
#include "Data_controller.h"

int main()
{
    while (true) {
		// TODO :P
    }
}

void setupPins()
{
	// Set up I/O port directions with the DDRx registers. 1=out, 0=in.
	// These can be changed later in the program (and some sensors need
	// to do this, e.g. ultrasonic sensors).
	//----------------SCHEMATIC----------------
	//  1-PC6: RESET			28-PC5: GYRO_SCL
	//  2-PD0: BUMP_READ		27-PC4: GYRO_SDA
	//  3-PD1: DATA_SEL_A		26-PC3: BUMP_SEL_A
	//  4-PD2: DATA_SEL_B		25-PC2: BUMP_SEL_B
	//  5-PD3: DATA_SEL_C		24-PC1: BUMP_SEL_C
	//  6-PD4: DATA_SEL_D		23-PC0: DATA_READ
	//  7-[VCC]					22-[GND]
	//  8-[GND]					21-[AREF]
	//  9-PB6: [UNUSED]			20-[AVCC]
	// 10-PB7: [UNUSED]			19-PB5: SCLK
	// 11-PD5: LED_A			18-PB4: MISO
	// 12-PD6: LED_B			17-PB3: MOSI
	// 13-PD7: LED_C			16-PB2: SS
	// 14-PB0: LED_D			15-PB1: (pushbutton)
	DDRB = ((1<<PB0) |
			(0<<PB1) |
			(0<<PB2) |
			(0<<PB3) |
			(1<<PB4) |
			(0<<PB5) |
			(0<<PB6) |	// Initialize unused pins to input (tristated).
			(0<<PB7));	// Initialize unused pins to input (tristated).
	DDRC = ((0<<PC0) |
			(1<<PC1) |
			(1<<PC2) |
			(0<<PC3) |
			//(1<<PC3) |
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
	
	// (PORTx registers) Initialize outputs to 0 (LOW), and enable internal
	// pull-ups for the appropriate inputs. 1=pull-up resistor enabled. For
	// details, see the schematic for the DDRx registers' set-up.
	// SPI shouldn't need pull-up resistors. Nor do multiplexer read pins.
	PORTB = ((0<<PB0) |
	(1<<PB1) |
	(0<<PB2) |
	(0<<PB3) |
	(0<<PB4) |
	(0<<PB5) |
	(0<<PB6) |
	(0<<PB7));
	PORTC = ((0<<PC0) |
	(0<<PC1) |
	(0<<PC2) |
	(0<<PC3) |
	(0<<PC4) |
	(0<<PC5) |
	(0<<PC6)); // Pull-up unnecessary for RESET pin. No bit 7.
	PORTD = ((0<<PD0) |
	(0<<PD1) |
	(0<<PD2) |
	(0<<PD3) |
	(0<<PD4) |
	(0<<PD5) |
	(0<<PD6) |
	(0<<PD7));
}