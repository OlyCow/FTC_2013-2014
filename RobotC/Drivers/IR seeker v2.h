// This header provides an API for the HiTechnic IR Seeker V2.
// version 0.5, made by Xander Soldaat.
#ifndef __HTIRS2_H__
#define __HTIRS2_H__
#pragma systemFile
#ifndef __COMMON_H__
#include "common.h"
#endif // __COMMON_H__



#define HTIRS2_I2C_ADDR		0x10	// IR Seeker I2C device address
#define HTIRS2_DSP_MODE		0x41	// AC DSP mode: 0 = 1200Hz, 1 = 600Hz
#define HTIRS2_OFFSET		0x42	// Offset for data registers
#define HTIRS2_DC_DIR		0x00	// DC Direction data
#define HTIRS2_DC_SSTR1		0x01	// DC Sensor 0 signal strength above avg
#define HTIRS2_DC_SSTR2		0x02	// DC Sensor 1 signal strength above avg
#define HTIRS2_DC_SSTR3		0x03	// DC Sensor 2 signal strength above avg
#define HTIRS2_DC_SSTR4		0x04	// DC Sensor 3 signal strength above avg
#define HTIRS2_DC_SSTR5		0x05	// DC Sensor 4 signal strength above avg
#define HTIRS2_DC_SAVG		0x06	// DC sensor signal strength avg
#define HTIRS2_AC_DIR		0x07	// AC Direction data
#define HTIRS2_AC_SSTR1		0x08	// AC Sensor 0 signal strength above avg
#define HTIRS2_AC_SSTR2		0x09	// AC Sensor 1 signal strength above avg
#define HTIRS2_AC_SSTR3		0x0A	// AC Sensor 2 signal strength above avg
#define HTIRS2_AC_SSTR4		0x0B	// AC Sensor 3 signal strength above avg
#define HTIRS2_AC_SSTR5		0x0C	// AC Sensor 4 signal strength above avg



// AC DSP modes
typedef enum {
	DSP_1200 = 0,
	DSP_600 = 1
} tHTIRS2DSPMode;

tByteArray HTIRS2_I2CRequest;	// array to hold I2C command data
tByteArray HTIRS2_I2CReply;		// array to hold I2C reply data



bool HTIRS2readEnhanced(tSensors link, int &dir, int &strength);
// DC Signal processing functions:
int HTIRS2readDCDir(tSensors link);
bool HTIRS2readAllDCStrength(tSensors link, int &dcS1, int &dcS2, int &dcS3, int &dcS4, int &dcS5);
int HTIRS2readDCAverage(tSensors link);
// AC Signal processing functions:
bool HTIRS2setDSPMode(tSensors link, tHTIRS2DSPMode mode);
int HTIRS2readACDir(tSensors link);
bool HTIRS2readAllACStrength(tSensors link, int &acS1, int &acS2, int &acS3, int &acS4, int &acS5);

#ifdef __HTSMUX_SUPPORT__
tConfigParams HTIRS2_config = {HTSMUX_CHAN_I2C, 13, 0x10, 0x42};	// array to hold SMUX config data for sensor
bool HTIRS2readEnhanced(tMUXSensor muxsensor, int &dir, int &strength);
// DC Signal processing functions:
int HTIRS2readDCDir(tMUXSensor muxsensor);
bool HTIRS2readAllDCStrength(tMUXSensor muxsensor, int &dcS1, int &dcS2, int &dcS3, int &dcS4, int &dcS5);
int HTIRS2readDCAverage(tMUXSensor muxsensor);
// AC Signal processing functions:
int HTIRS2readACDir(tMUXSensor muxsensor);
bool HTIRS2readAllACStrength(tMUXSensor muxsensor, int &acS1, int &acS2, int &acS3, int &acS4, int &acS5);
#endif // __HTSMUX_SUPPORT__



// Read value of DC direction data register and return it.
// Value ranges from 0~9 or -1 if an error occurred.
// `link`:	Port number.
int HTIRS2readDCDir(tSensors link) {
	memset(HTIRS2_I2CRequest, 0, sizeof(tByteArray));
	HTIRS2_I2CRequest[0] = 2; 				// message size
	HTIRS2_I2CRequest[1] = HTIRS2_I2C_ADDR;	// I2C address
	HTIRS2_I2CRequest[2] = HTIRS2_OFFSET + HTIRS2_DC_DIR; // start direction register
	
	if (!writeI2C(link, HTIRS2_I2CRequest, HTIRS2_I2CReply, 1))
		return -1;
		
	return HTIRS2_I2CReply[0];
}

// Read the value of the all internal sensors (DC mode).
// Returns true if no error occurred, false if it did.
// `link`:	Port number.
// `dcS1`:	Internal sensor 1.
// `dcS2`:	Internal sensor 2.
// `dcS3`:	Internal sensor 3.
// `dcS4`:	Internal sensor 4.
// `dcS5`:	Internal sensor 5.
bool HTIRS2readAllDCStrength(tSensors link, int &dcS1, int &dcS2, int &dcS3, int &dcS4, int &dcS5) {
	memset(HTIRS2_I2CRequest, 0, sizeof(tByteArray));

	HTIRS2_I2CRequest[0] = 2;				// message size
	HTIRS2_I2CRequest[1] = HTIRS2_I2C_ADDR;	// I2C address
	HTIRS2_I2CRequest[2] = HTIRS2_OFFSET + HTIRS2_DC_SSTR1; // sensor 0 signal strength

	if (!writeI2C(link, HTIRS2_I2CRequest, HTIRS2_I2CReply, 5))
		return false;

	dcS1 = HTIRS2_I2CReply[0];
	dcS2 = HTIRS2_I2CReply[1];
	dcS3 = HTIRS2_I2CReply[2];
	dcS4 = HTIRS2_I2CReply[3];
	dcS5 = HTIRS2_I2CReply[4];

	return true;
}

// Read the value of the average data register and return it.
// Returns -1 if error occurs.
// `link`:	Port number.
int HTIRS2readDCAverage(tSensors link) {
	memset(HTIRS2_I2CRequest, 0, sizeof(tByteArray));

	HTIRS2_I2CRequest[0] = 2;				// message size
	HTIRS2_I2CRequest[1] = HTIRS2_I2C_ADDR;	// I2C address
	HTIRS2_I2CRequest[2] = HTIRS2_OFFSET + HTIRS2_DC_SAVG; // DC sensor signal strength avg

	if (!writeI2C(link, HTIRS2_I2CRequest, HTIRS2_I2CReply, 1))
		return -1;

	return HTIRS2_I2CReply[0];
}

// Set the DSP mode of the AC carrier wave detector.
// Returns true if no error occurred, false if it did.
// `link`:	Port number.
// `mode`:	Frequency to detect: either DSP_1200 or DSP_600.
bool HTIRS2setDSPMode(tSensors link, tHTIRS2DSPMode mode) {
	memset(HTIRS2_I2CRequest, 0, sizeof(tByteArray));

	HTIRS2_I2CRequest[0] = 3;				// message size
	HTIRS2_I2CRequest[1] = HTIRS2_I2C_ADDR;	// I2C address
	HTIRS2_I2CRequest[2] = HTIRS2_DSP_MODE;	// start direction register
	HTIRS2_I2CRequest[3] = (ubyte)mode;

	return writeI2C(link, HTIRS2_I2CRequest);
}

// Read value of AC direction data register and return it.
// Value ranges from 0~9 or -1 if an error occurred.
// `link`:	Port number.
int HTIRS2readACDir(tSensors link) {
	memset(HTIRS2_I2CRequest, 0, sizeof(tByteArray));

	HTIRS2_I2CRequest[0] = 2;				// message size
	HTIRS2_I2CRequest[1] = HTIRS2_I2C_ADDR;	// I2C address
	HTIRS2_I2CRequest[2] = HTIRS2_OFFSET + HTIRS2_AC_DIR; // start direction register

	if (!writeI2C(link, HTIRS2_I2CRequest, HTIRS2_I2CReply, 1))
		return -1;

	return HTIRS2_I2CReply[0];
}

// Read the value of the all internal sensors (AC mode).
// Returns true if no error occurred, false if it did.
// `link`:	Port number.
// `acS1`:	Internal sensor 1.
// `acS2`:	Internal sensor 2.
// `acS3`:	Internal sensor 3.
// `acS4`:	Internal sensor 4.
// `acS5`:	Internal sensor 5.
bool HTIRS2readAllACStrength(tSensors link, int &acS1, int &acS2, int &acS3, int &acS4, int &acS5) {
	memset(HTIRS2_I2CRequest, 0, sizeof(tByteArray));

	HTIRS2_I2CRequest[0] = 2;				// message size
	HTIRS2_I2CRequest[1] = HTIRS2_I2C_ADDR;	// I2C address
	HTIRS2_I2CRequest[2] = HTIRS2_OFFSET + HTIRS2_AC_SSTR1; // sensor 0 signal strength

	if (!writeI2C(link, HTIRS2_I2CRequest, HTIRS2_I2CReply, 5))
		return false;

	acS1 = HTIRS2_I2CReply[0];
	acS2 = HTIRS2_I2CReply[1];
	acS3 = HTIRS2_I2CReply[2];
	acS4 = HTIRS2_I2CReply[3];
	acS5 = HTIRS2_I2CReply[4];

	return true;
}

// Calculate the strength and direction based on both DC and AC signals.
// Returns true if no error occurred, false if it did.
// `link`:		Port number.
// `dir`:		Buffer for direction detected; has value of 0-9 (0 = nothing detected).
// `strength`:	Strength of detected IR signal.
bool HTIRS2readEnhanced(tSensors  link, int &dir, int &strength)
{
	ubyte iMax = 0;
	long dcSigSum = 0;

	// Read DC mode:
	memset(HTIRS2_I2CRequest, 0, sizeof(tByteArray));

	HTIRS2_I2CRequest[0] = 2;				// message size
	HTIRS2_I2CRequest[1] = HTIRS2_I2C_ADDR;	// I2C address
	HTIRS2_I2CRequest[2] = HTIRS2_OFFSET + HTIRS2_DC_SSTR1; // sensor 0 signal strength

	if (!writeI2C(link, HTIRS2_I2CRequest, HTIRS2_I2CReply, 6))
		return false;

	// Find max DC signal strength.
	for (int i = 1; i < 5; i++)
	{
		if (HTIRS2_I2CReply[i] > HTIRS2_I2CReply[iMax])
		{
			iMax = i;
		}
	}

	// Calculate base DC direction value.
	dir = iMax * 2 + 1;
	// Set base dcStrength based on max signal and average.
	dcSigSum = HTIRS2_I2CReply[iMax] + HTIRS2_I2CReply[5];

	// Check signal strength of neighbouring sensor elements.
	if ((iMax > 0) && (HTIRS2_I2CReply[iMax - 1] > (ubyte)(HTIRS2_I2CReply[iMax] / 2)))
	{
		dir--;
		dcSigSum += HTIRS2_I2CReply[iMax - 1];
	}

	if ((iMax < 4) && (HTIRS2_I2CReply[iMax + 1] > (ubyte)(HTIRS2_I2CReply[iMax] / 2)))
	{
		dir++;
		dcSigSum += HTIRS2_I2CReply[iMax + 1];
	}

	// Make DC strength compatible with AC strength.
	strength = sqrt(dcSigSum * 500);

	// Decide if using AC strength.
	if (strength <= 200)
	{
		writeDebugStreamLine("switching to AC");
		// Use AC Dir:
		HTIRS2_I2CRequest[2] = HTIRS2_OFFSET + HTIRS2_AC_DIR; // Recycle rest of cmdBuf

		if (!writeI2C(link, HTIRS2_I2CRequest, HTIRS2_I2CReply, 6))
			return false;

		dir = HTIRS2_I2CReply[0];

		// Sum the sensor elements to get strength.
		if (dir > 0)
		{
			strength =	HTIRS2_I2CReply[1] + HTIRS2_I2CReply[2] +
						HTIRS2_I2CReply[3] + HTIRS2_I2CReply[4] +
						HTIRS2_I2CReply[5];
		}
	}
	return true;
}



#ifdef __HTSMUX_SUPPORT__
// Read value of DC direction data register and return it.
// Value ranges from 0~9 or -1 if an error occurred.
// `muxsensor`:	SMUX sensor port number.
int HTIRS2readDCDir(tMUXSensor muxsensor) {
	memset(HTIRS2_I2CReply, 0, sizeof(tByteArray));
	if (HTSMUXSensorTypes[muxsensor] != HTSMUXSensorCustom)
		HTSMUXconfigChannel(muxsensor, HTIRS2_config);
	if (!HTSMUXreadPort(muxsensor, HTIRS2_I2CReply, 1, HTIRS2_DC_DIR)) {
		return -1;
	}
	return HTIRS2_I2CReply[0];
}
// Read the value of the all internal sensors (DC mode).
// Returns true if no error occurred, false if it did.
// `muxsensor`:	SMUX sensor port number.
// `dcS1`:	Internal sensor 1.
// `dcS2`:	Internal sensor 2.
// `dcS3`:	Internal sensor 3.
// `dcS4`:	Internal sensor 4.
// `dcS5`:	Internal sensor 5.
bool HTIRS2readAllDCStrength(tMUXSensor muxsensor, int &dcS1, int &dcS2, int &dcS3, int &dcS4, int &dcS5) {
	memset(HTIRS2_I2CReply, 0, sizeof(tByteArray));

	if (HTSMUXSensorTypes[muxsensor] != HTSMUXSensorCustom)
		HTSMUXconfigChannel(muxsensor, HTIRS2_config);

	if (!HTSMUXreadPort(muxsensor, HTIRS2_I2CReply, 5, HTIRS2_DC_SSTR1)) {
		return false;
	}

	dcS1 = HTIRS2_I2CReply[0];
	dcS2 = HTIRS2_I2CReply[1];
	dcS3 = HTIRS2_I2CReply[2];
	dcS4 = HTIRS2_I2CReply[3];
	dcS5 = HTIRS2_I2CReply[4];

	return true;
}

// Read the value of the average data register and return it.
// Returns -1 if error occurs.
// `muxsensor`:	SMUX sensor port number.
int HTIRS2readDCAverage(tMUXSensor muxsensor) {
	memset(HTIRS2_I2CReply, 0, sizeof(tByteArray));

	if (HTSMUXSensorTypes[muxsensor] != HTSMUXSensorCustom)
		HTSMUXconfigChannel(muxsensor, HTIRS2_config);

	if (!HTSMUXreadPort(muxsensor, HTIRS2_I2CReply, 1, HTIRS2_DC_SAVG)) {
		return -1;
	}

	return HTIRS2_I2CReply[0];
}

// Read value of AC direction data register and return it.
// Value ranges from 0~9 or -1 if an error occurred.
// `muxsensor`:	SMUX sensor port number.
int HTIRS2readACDir(tMUXSensor muxsensor) {
	memset(HTIRS2_I2CReply, 0, sizeof(tByteArray));

	if (HTSMUXSensorTypes[muxsensor] != HTSMUXSensorCustom)
		HTSMUXconfigChannel(muxsensor, HTIRS2_config);

	if (!HTSMUXreadPort(muxsensor, HTIRS2_I2CReply, 1, HTIRS2_AC_DIR)) {
		return -1;
	}

	return HTIRS2_I2CReply[0];
}

// Read the value of the all internal sensors (AC mode).
// Returns true if no error occurred, false if it did.
// `muxsensor`:	SMUX sensor port number.
// `acS1`:	Internal sensor 1.
// `acS2`:	Internal sensor 2.
// `acS3`:	Internal sensor 3.
// `acS4`:	Internal sensor 4.
// `acS5`:	Internal sensor 5.
bool HTIRS2readAllACStrength(tMUXSensor muxsensor, int &acS1, int &acS2, int &acS3, int &acS4, int &acS5) {
	memset(HTIRS2_I2CReply, 0, sizeof(tByteArray));

	if (HTSMUXSensorTypes[muxsensor] != HTSMUXSensorCustom)
		HTSMUXconfigChannel(muxsensor, HTIRS2_config);

	if (!HTSMUXreadPort(muxsensor, HTIRS2_I2CReply, 5, HTIRS2_AC_SSTR1)) {
		return false;
	}

	acS1 = HTIRS2_I2CReply[0];
	acS2 = HTIRS2_I2CReply[1];
	acS3 = HTIRS2_I2CReply[2];
	acS4 = HTIRS2_I2CReply[3];
	acS5 = HTIRS2_I2CReply[4];

	return true;
}

// Calculate the strength and direction based on both DC and AC signals.
// Returns true if no error occurred, false if it did.
// `muxsensor`:	SMUX sensor port number.
// `dir`:		Buffer for direction detected; has value of 0-9 (0 = nothing detected).
// `strength`:	Strength of detected IR signal.
bool HTIRS2readEnhanced(tMUXSensor muxsensor, int &dir, int &strength)
{
	ubyte iMax = 0;
	long dcSigSum = 0;

	memset(HTIRS2_I2CReply, 0, sizeof(tByteArray));

	if (HTSMUXSensorTypes[muxsensor] != HTSMUXSensorCustom)
		HTSMUXconfigChannel(muxsensor, HTIRS2_config);

	if (!HTSMUXreadPort(muxsensor, HTIRS2_I2CReply, 13, HTIRS2_DC_DIR)) {
		return false;
	}

	// Find the max DC signal strength:
	for (int i = 1; i < 5; i++)
	{
		if (HTIRS2_I2CReply[HTIRS2_DC_SSTR1+i] > HTIRS2_I2CReply[HTIRS2_DC_SSTR1+iMax])
		{
			iMax = i;
		}
	}

	// Calculate base DC direction.
	dir = iMax * 2 + 1;
	// Set base dcStrength based on max signal and average.
	dcSigSum = HTIRS2_I2CReply[HTIRS2_DC_SSTR1+iMax] + HTIRS2_I2CReply[HTIRS2_DC_SSTR1+5];

	// Check signal strength of neighbouring sensor elements.
	if ((iMax > 0) && (HTIRS2_I2CReply[HTIRS2_DC_SSTR1+iMax - 1] > (ubyte)(HTIRS2_I2CReply[HTIRS2_DC_SSTR1+iMax] / 2)))
	{
		dir--;
		dcSigSum += HTIRS2_I2CReply[HTIRS2_DC_SSTR1+iMax - 1];
	}

	if ((iMax < 4) && (HTIRS2_I2CReply[HTIRS2_DC_SSTR1+iMax + 1] > (ubyte)(HTIRS2_I2CReply[HTIRS2_DC_SSTR1+iMax] / 2)))
	{
		dir++;
		dcSigSum += HTIRS2_I2CReply[HTIRS2_DC_SSTR1+iMax + 1];
	}

	// Make DC strength compatible with AC strength.
	strength = sqrt(dcSigSum * 500);

	// Decide if using AC strength:
	if (strength <= 200)
	{
		writeDebugStreamLine("switching to AC");
		// Use AC Dir:
		dir = HTIRS2_I2CReply[HTIRS2_AC_DIR];

		// Sum the sensor elements to get strength.
		if (dir > 0)
		{
			strength =	HTIRS2_I2CReply[HTIRS2_AC_SSTR1] + HTIRS2_I2CReply[HTIRS2_AC_SSTR2] +
						HTIRS2_I2CReply[HTIRS2_AC_SSTR3] + HTIRS2_I2CReply[HTIRS2_AC_SSTR4] +
						HTIRS2_I2CReply[HTIRS2_AC_SSTR5];
		}
	}
	return true;
}
#endif // __HTSMUX_SUPPORT__



#endif // __HTIRS2_H__
