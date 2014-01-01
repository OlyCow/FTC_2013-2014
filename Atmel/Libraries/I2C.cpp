// Not-so-primitive I2C library-maybe even better than Arduino's
// Wire library, if I do say so myself.
#include "I2C.h"

// TODO: These are the status codes (master transmit):
// 0x08 - START condition transmitted
// 0x10 - repeated START condition transmitted
// 0x18 - SLA+W transmitted; ACK received
// 0x20 - SLA+W transmitted; NOT ACK received
// 0x28 - data byte transmitted; ACK received
// 0x30 - data byte transmitted; NOT ACK received
// 0x38 - arbitration lost (SLA+W or data)

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
void TWI::hold(void)
{
	// Send a START condition.
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while ( (TWCR&(1<<TWINT))==0 ) {
		; // Wait for the interrupt flag to be set (transmit done).
	}
	while ( (TWSR&0xF8) != 0x10 ) {
		; // Wait for ACK.
	}
}
void TWI::stop(void)
{
	// Send a STOP condition.
	TWCR = (1<<TWINT)|(1<<TWSTO)|(TWEN);
}
void TWI::write_SLA_W(uint8_t address)
{
	// W bit is 0, no need to set anything.
	TWDR = address<<1; // Load input into data-shift register (SLA+R/W).
	TWCR = (1<<TWINT)|(1<<TWEN); // Clear interrupt bit to send data.
	while ( (TWCR&(1<<TWINT))==0 ) {
		; // Wait for the interrupt flag to be set (transmit done).
	}
	// TODO: FIX STATUS CODES
	while ( (TWSR&0xF8) != 0x18 ) {
		; // Wait for ACK.
	}
}
void TWI::write_SLA_R(uint8_t address)
{
	// R bit is 1, 1=0b00000001.
	TWDR = (address<<1)|1; // Load input into data-shift register (SLA+R/W).
	TWCR = (1<<TWINT)|(1<<TWEN); // Clear interrupt bit to send data.
	while ( (TWCR&(1<<TWINT))==0 ) {
		; // Wait for the interrupt flag to be set (transmit done).
	}
	// TODO: FIX STATUS CODES
	while ( (TWSR&0xF8) != 0x40 ) {
		; // Wait for ACK.
	}
	// TODO: This must be followed immediately by the read operation, I think.
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
void TWI::read_data(uint8_t u8data[], int byte_num)
{
	// TODO: status codes. Again.
	for (int byte_read=0; byte_read<byte_num; byte_read++) {
		TWCR = (1<<TWINT)|(1<<TWEN);
		while ( (TWCR&(1<<TWINT))==0 ) {
			; // Wait for the interrupt flag to be set (transmit done).
		}
		if (byte_read<byte_num-1) {
			while ( (TWSR&0xF8) != 0x50 ) {
				; // Wait for ACK.
			}
		} else {
			while ( (TWSR&0xF8) != 0x58 ) {
				; // Wait for ACK.
			}
		}
		u8data[byte_read] = TWDR;
	}
}
