#ifndef TWI_H
#define TWI_H

#include <avr/io.h>

namespace TWI
{
	// Common uses:
	// start-write_address-write_data-stop
	// start-write_address-read_data_once-stop
	
	void setup(void);
	void start(void); // REPEATED START is same as START (according to datasheet).
	void stop(void);
	void write_address(uint8_t u8data);
	void write_data(uint8_t u8data);
	void read_data_once(uint8_t &u8data);
	
	//// TODO!
	//void read_data_cont(int size);
	
	void write_SLAW(uint8_t address);
	void write_SLAR(uint8_t address);
	
	uint8_t status(void);
}

#endif // TWI_H
