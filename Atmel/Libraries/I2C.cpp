// Not-so-primitive I2C library-maybe even better than Arduino's
// Wire library, if I do say so myself.
#include "I2C.h"

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
void TWI::read_data_once(uint8_t &u8data)
{
	TWCR = (1<<TWINT)|(1<<TWEN); // Clear interrupt bit to send data.
	while ( (TWCR&(1<<TWINT))==0 ) {
		; // Wait for the interrupt flag to be set (transmit done).
	}
	while ( (TWSR&0xF8) != 0x58 ) {
		; // Wait for ACK.
	}
	u8data = TWDR;
}

void TWI::write_SLAW(uint8_t address)
{
	TWI::write_address(address<<1); // W bit is 0, no need to set anything.
}
void TWI::write_SLAR(uint8_t address)
{
	TWI::write_address((address<<1)|1); // R bit is 1, 1=0b00000001.
}

uint8_t TWI::status(void)
{
	// Mask out the prescalar bits from the register.
	uint8_t status = TWSR & 0xF8;
	return status;
}
