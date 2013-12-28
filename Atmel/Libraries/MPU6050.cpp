// Basic acquiring and processing of the data fetched from the MPU-6050.
// A lot of stuff is taken from Jeff Rowberg's library. <jeff@rowberg.net>
#include "MPU6050.h"

void MPU::read(uint8_t address, uint8_t request, uint8_t &data)
{
	TWI::start();
	TWI::write_SLAW(address);
	TWI::write_data(request);
	TWI::start(); // Actually, repeated start. :)
	TWI::write_SLAR(address);
	TWI::read_data_once(data);
	TWI::stop();
}
