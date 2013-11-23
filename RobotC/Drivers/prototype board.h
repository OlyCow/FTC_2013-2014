// This header provides an API for the HiTechnic SuperPro Prototype Board.
// Version 0.1, made by Xander Soldaat.
#ifndef __HTSPB_H__
#define __HTSPB_H__
#pragma systemFile
#ifndef __COMMON_H__
#include "common.h"
#endif // __COMMON_H__



#define HTSPB_I2C_ADDR	0x10	// SuperPro prototype board I2C device address
#define HTSPB_OFFSET	0x42
#define HTSPB_A0_U		0x00	// Address of upper bits of first ADC, bits 9-2
#define HTSPB_A0_L		0x01	// Address of lower bits of first ADC, bits 1-0
#define HTSPB_DIGIN		0x0A	// Address of digital inputs
#define HTSPB_DIGOUT	0x0B	// Address of digital outputs
#define HTSPB_DIGCTRL	0x0C	// Controls direction of digital ports
#define HTSPB_STROBE	0x0E	// Address of strobe outputs, bits 6-0
#define HTSPB_LED		0x0F	// Address of on-board led outputs, bits 1-0
#define HTSPB_O0MODE	0x10	// Address of analog output 0 mode
#define HTSPB_O0FREQ	0x11	// Address of analog output 0 frequency
#define HTSPB_O0VOLT	0x13	// Address of analog output 0 voltage
#define HTSPB_O1MODE	0x15	// Address of analog output 1 mode
#define HTSPB_O1FREQ	0x16	// Address of analog output 1 frequency
#define HTSPB_O1VOLT	0x18	// Address of analog output 1 voltage
#define HTSPB_DACO0		0x10	// Address of analog parameters output O0
#define HTSPB_DACO1		0x15	// Address of analog parameters output O1
// SuperPro Analog output modes:
#define DAC_MODE_DCOUT			0	// Steady (DC) voltage output.
#define DAC_MODE_SINEWAVE		1	// Sine wave output.
#define DAC_MODE_SQUAREWAVE		2	// Square wave output.
#define DAC_MODE_SAWPOSWAVE		3	// Positive going sawtooth output.
#define DAC_MODE_SAWNEGWAVE		4	// Negative going sawtooth output.
#define DAC_MODE_TRIANGLEWAVE	5	// Triangle wave output.
#define DAC_MODE_PWMVOLTAGE		6	// PWM square wave output.



tByteArray HTSPB_I2CRequest;	// array to hold I2C command data
tByteArray HTSPB_I2CReply;		// array to hold I2C reply data

ubyte HTSPBreadIO(tSensors link, ubyte mask);
bool HTSPBwriteIO(tSensors link, ubyte mask);
bool HTSPBsetupIO(tSensors link, ubyte mask);
int HTSPBreadADC(tSensors link, byte channel, byte width);
bool HTSPBreadAllADC(tSensors link, int &adch0, int &adch1, int &adch2, int &adch3, int &adch4, byte width);
bool HTSPBsetSamplingTime(tSensors link, byte interval);



// Read the values of the digital inputs as specified by the mask.
// Returns eight bits of digital input (&'d with specified mask).
// `link`:	Port number.
// `mask`:	Digital ports to read.
ubyte HTSPBreadIO(tSensors link, ubyte mask) {
	memset(HTSPB_I2CRequest, 0, sizeof(tByteArray));

	HTSPB_I2CRequest[0] = 2;				// message size
	HTSPB_I2CRequest[1] = HTSPB_I2C_ADDR;	// I2C address
	HTSPB_I2CRequest[2] = HTSPB_OFFSET + HTSPB_DIGIN; // start digital output read address

	if (!writeI2C(link, HTSPB_I2CRequest, HTSPB_I2CReply, 1))
		return 0;

	return HTSPB_I2CReply[0] & mask;
}

// Write the values to digital outputs as specified by the mask.
// If a bit is masked out (not set; `0`) it is set to false (`0`).
// Returns true if no error occurred, false if it did.
// `link`:	Port number.
// `mask`:	Digital ports to write to.
bool HTSPBwriteIO(tSensors link, ubyte mask) {
	memset(HTSPB_I2CRequest, 0, sizeof(tByteArray));

	HTSPB_I2CRequest[0] = 3;				// message size
	HTSPB_I2CRequest[1] = HTSPB_I2C_ADDR;	// I2C address
	HTSPB_I2CRequest[2] = HTSPB_OFFSET + HTSPB_DIGOUT; // start digital output write address
	HTSPB_I2CRequest[3] = mask;				// the specified digital ports

	return writeI2C(link, HTSPB_I2CRequest);
}

// Configure digital ports for input or output according to a mask.
// Returns true if no error occurred, false if it did.
// `link`:	Port number.
// `mask`:	Specified digital ports: 0->input, 1->output
bool HTSPBsetupIO(tSensors link, ubyte mask) {
	memset(HTSPB_I2CRequest, 0, sizeof(tByteArray));

	HTSPB_I2CRequest[0] = 3;				// message size
	HTSPB_I2CRequest[1] = HTSPB_I2C_ADDR;	// I2C address
	HTSPB_I2CRequest[2] = HTSPB_OFFSET + HTSPB_DIGCTRL; // start digital I/O control address
	HTSPB_I2CRequest[3] = mask;				// the specified digital ports

	return writeI2C(link, HTSPB_I2CRequest);
}

// Read the value of the specified analog (ADC) channel.
// If an error occurs, returns -1.
// `link`:		Port number.
// `channel`:	ADC channel.
// `width`:		Resolution of the result, either 8 or 10.
int HTSPBreadADC(tSensors link, byte channel, byte width) {
	memset(HTSPB_I2CRequest, 0, sizeof(tByteArray));

	int _adcVal = 0;
	HTSPB_I2CRequest[0] = 2;				// message size
	HTSPB_I2CRequest[1] = HTSPB_I2C_ADDR;	// I2C address
	// Start digital output read address with channel offset.
	HTSPB_I2CRequest[2] = HTSPB_OFFSET + HTSPB_A0_U + (channel * 2);

	if (!writeI2C(link, HTSPB_I2CRequest, HTSPB_I2CReply, 2))
		return -1;

	// Convert bytes to int:
	// 1st byte contains bits 9-2 of the channel's value;
	// 2nd byte contains bits 1-0 of the channel's value.
	// We'll need to shift the 1st byte left by 2 and OR the 2nd byte onto it.
	// If 8 bits is all we want, we just return the first byte we're done.
	if (width == 8)
		_adcVal = HTSPB_I2CReply[0];
	else
		_adcVal = (HTSPB_I2CReply[0] * 4) + HTSPB_I2CReply[1];

	return _adcVal;
}

// Reads the value of all of the analog (ADC) channels into buffers.
// Returns true if no error occurred, false if it did.
// `link`:	Port number.
// `adch0`:	Buffer to hold value for ADC channel 0.
// `adch1`:	Buffer to hold value for ADC channel 1.
// `adch2`:	Buffer to hold value for ADC channel 2.
// `adch3`:	Buffer to hold value for ADC channel 3.
// `width`:	Resolution of the result; either 8 or 10.
bool HTSPBreadAllADC(tSensors link, int &adch0, int &adch1, int &adch2, int &adch3, byte width) {
	memset(HTSPB_I2CRequest, 0, sizeof(tByteArray));

	HTSPB_I2CRequest[0] = 2;				// message size
	HTSPB_I2CRequest[1] = HTSPB_I2C_ADDR;	// I2C address
	HTSPB_I2CRequest[2] = HTSPB_OFFSET + HTSPB_A0_U; // start digital output read address

	if (!writeI2C(link, HTSPB_I2CRequest, HTSPB_I2CReply, 10))
		return false;

	// Convert bytes to int:
	// 1st byte contains bits 9-2 of the channel's value;
	// 2nd byte contains bits 1-0 of the channel's value.
	// We'll need to shift the 1st byte left by 2 and OR the 2nd byte onto it.
	// If 8 bits is all we want, we just return the first byte we're done.
	if (width == 8) {
		adch0 = (int)HTSPB_I2CReply[0];
		adch1 = (int)HTSPB_I2CReply[2];
		adch2 = (int)HTSPB_I2CReply[4];
		adch3 = (int)HTSPB_I2CReply[6];
		} else {
		adch0 = ((int)HTSPB_I2CReply[0] << 2) + (int)HTSPB_I2CReply[1];
		adch1 = ((int)HTSPB_I2CReply[2] << 2) + (int)HTSPB_I2CReply[3];
		adch2 = ((int)HTSPB_I2CReply[4] << 2) + (int)HTSPB_I2CReply[5];
		adch3 = ((int)HTSPB_I2CReply[6] << 2) + (int)HTSPB_I2CReply[7];
	}
	return true;
}

// Write to an analog output.
// Returns true if no error occurred, false if it did.
// `link`:	Port number.
// `dac`:	Specified analog output; HTSPB_DACO0 or HTSPB_DACO1.
// `mode`:	Analog mode (see macros).
// `freq`:	Output frequency (1~8193).
// `volt`:	Outoyt voltage from (0~1023, for 0~3.3V).
bool HTSPBwriteAnalog(tSensors link, byte dac, byte mode, int freq, int volt) {
	memset(HTSPB_I2CRequest, 0, sizeof(tByteArray));

	HTSPB_I2CRequest[0] = 7;				// message size
	HTSPB_I2CRequest[1] = HTSPB_I2C_ADDR;	// I2C address
	HTSPB_I2CRequest[2] = HTSPB_OFFSET + dac; // start analog output write address
	HTSPB_I2CRequest[3] = mode;				// the specified analog mode
	HTSPB_I2CRequest[4] = freq/256;			// high byte of frequency
	HTSPB_I2CRequest[5] = freq&255;			// low byte of frequency
	HTSPB_I2CRequest[6] = volt/4;			// high 8 bits of voltage
	HTSPB_I2CRequest[7] = volt&3;			// low 2 bits of voltage

	return writeI2C(link, HTSPB_I2CRequest);
}



#endif // __HTSPB_H__
