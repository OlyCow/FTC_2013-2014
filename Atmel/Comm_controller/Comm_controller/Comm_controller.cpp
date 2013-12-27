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
	
	//// TODO: Uncomment this stuff when we get the ATmega328s.
	//// TODO: Move this interrupt registry stuff over to the
	//// header file when they've been tested to work.
	//PCMSK0 = (1<<PCINT1); 
	//PCICR = (1<<PCIE0);
	
	// When we're ready, enable interrupts.
	sei();
	
	// Setting up a timer for debouncing.
	// When CS10=1 and CS11, CS12=0, clock prescaling = 1.
	// TODO: Is this the best way to set 3 different bits?
	TCCR1B |= (1 << CS10); // Set CS10 in control registry.
	
	// Variables for I/O with the NXT (prototype board).
	bool clock_NXT_current = false;				// TODO: I don't think this initialization matters... Does it?
	bool clock_NXT_prev = false;				// DERP
	bool header_read = false;					// We want to default to "normal" data.
	bool header_write[NXT_LINE_NUM] = {false,false,false,false,false,false};
	uint32_t data_read = 0;
	uint32_t data_write[NXT_LINE_NUM] = {0,0,0,0,0,0};
	bool parity_read = false;					// TODO: Leaving this undefined really isn't a good idea either... Oh well.
	bool parity_read_check = parity_read;		// DERP
	bool parity_write[NXT_LINE_NUM] = {false,false,false,false,false,false}; // TODO: Same problem as `parity_read`.
		
	// Data gathered from various pins to report back.
	uint8_t cube_num = 0;
	
	// Variables to process pin inputs.
	bool cube_counter_current = false;
	bool cube_counter_prev = false;
	bool isDebouncing = false;
	
	while (true) {
		// Process NXT (prototype board) I/O.
		clock_NXT_current = (PIND & (1<<PD0));
		if (clock_NXT_current != clock_NXT_prev) {
			clock_NXT_prev = clock_NXT_current;
			// Process some input.
		}
		
		// Process cube counting.
		cube_counter_current = (PINB & (1<<PINB));
		if (cube_counter_current!=cube_counter_prev) {
			switch (isDebouncing) {
				case false :
					isDebouncing = true;
					TCNT1 = 0; // Clear this timer; start counting.
				case true :
					if (TCNT1 >= DEBOUNCE_COUNTS) {
						// Under the correct conditions, increment cube count.
						if (((~cube_counter_current)&cube_counter_prev) == true) {
							if (cube_num<4) {
								cube_num++; // Some really hackish error handling here :)
							}
						}
						// Get ready for the next cycle.
						TCNT1 = 0; // Clear clock.
						isDebouncing = false;
						cube_counter_prev = cube_counter_current;
					}
					break;
			}
		}
		
		// Process gyro data.
		// TODO: This is temporary!
		TWI::start();
		
		TWI::stop();
	}
}

//// TODO: Enable this when we get the ATmega328s.
//ISR(PCINT0_vect)
//{
	//
//}


	
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
