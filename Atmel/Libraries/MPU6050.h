// Basic acquiring and processing of the data fetched from the MPU-6050.
// A lot of stuff is taken from Jeff Rowberg's library. <jeff@rowberg.net>
#ifndef MPU6050_H
#define MPU6050_H

#include <avr/io.h>
#include "I2C.h"

#define MPU6050_ADDRESS_AD0_LOW		0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH	0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS		MPU6050_ADDRESS_AD0_LOW

#define MPU6050_ADDRESS				MPU6050_ADDRESS_AD0_LOW

#define MPU6050_RA_WHO_AM_I			0x75

namespace MPU
{
	void read(uint8_t address, uint8_t request, uint8_t &data);
}

#endif // MPU6050_H
