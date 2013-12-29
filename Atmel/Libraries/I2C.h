// Not-so-primitive I2C library-maybe even better than Arduino's
// Wire library, if I do say so myself.
#ifndef TWI_H
#define TWI_H

#include <avr/io.h>

namespace TWI
{
	// Common uses:
	// start-write_address-write_data-stop
	// start-write_address-read_data_once-stop
	
	void setup(void);
	void start(void);
	void hold(void); // only difference between this and `start()` is status ACK.
	void stop(void);
	void write_SLA_W(uint8_t u8data);
	void write_SLA_R(uint8_t u8data);
	void write_data(uint8_t u8data);
	void read_data(uint8_t u8data[], int byte_num);
}

#endif // TWI_H
