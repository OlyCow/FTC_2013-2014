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
	TWI::setup();
	
	TWI::start();
	
	while (true) {
		// TODO: Write actual code.
		// Initialize.
		// Read data from other AVRs.
		// Read data from NXT.
		// Write data to NXT.
	}
}
	
void TWI::setup(void)
{
	TWSR = 0x00; // No prescalar.
	TWBR = 0x0C; // Bitrate = 400kHz.
	TWCR = (1<<TWEN); // Enable TWI.
}
void TWI::start(void)
{
	// Send a START condition.
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while ( (TWCR&(1<<TWINT))==0 ) {
		; // Wait for the interrupt flag to be set (transmit done).
	}
	while ( (TWSR&0xF8) != 0x08 ) {
		; // Wait for ACK.
	}
}
void TWI::stop(void)
{
	// Send a STOP condition.
	TWCR = (1<<TWINT)|(1<<TWSTO)|(TWEN);
}
void TWI::write_address(uint8_t u8data)
{
	TWDR = u8data; // Load input into data-shift register (SLA+R/W).
	TWCR = (1<<TWINT)|(1<<TWEN); // Clear interrupt bit to send data.
	while ( (TWCR&(1<<TWINT))==0 ) {
		; // Wait for the interrupt flag to be set (transmit done).
	}
	while ( (TWSR&0xF8) != 0x40 ) {
		; // Wait for ACK.
	}
}
void TWI::write_data(uint8_t u8data)
{
	TWDR = u8data;
	TWCR = (1<<TWINT)|(1<<TWEN); // Clear interrupt bit to send data.
	while ( (TWCR&(1<<TWINT))==0 ) {
		; // Wait for the interrupt flag to be set (transmit done).
	}
	while ( (TWSR&0xF8) != 0x28 ) {
		; // Wait for ACK.
	}
}
uint8_t TWI::read_data_once()
{
	TWCR = (1<<TWINT)|(1<<TWEN); // Clear interrupt bit to send data.
	while ( (TWCR&(1<<TWINT))==0 ) {
		; // Wait for the interrupt flag to be set (transmit done).
	}
	while ( (TWSR&0xF8) != 0x58 ) {
		; // Wait for ACK.
	}
	return TWDR;
}

uint8_t TWI::status(void)
{
	// Mask out the prescalar bits from the register.
	uint8_t status = TWSR & 0xF8;
	return status;
}
