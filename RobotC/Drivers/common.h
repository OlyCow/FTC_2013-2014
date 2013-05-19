// This header provides some functions frequently used in drivers.
// Version 0.16, made by Xander Soldaat.
#ifndef __COMMON_H__
#define __COMMON_H__
#pragma systemFile
#include "..\Headers\includes.h"



#undef __COMMON_H_DEBUG__

// Define this as 0 to remove the check.
#ifndef __COMMON_H_SENSOR_CHECK__
#define __COMMON_H_SENSOR_CHECK__ 1
#else
#warn "Sensor checking disabled, I hope you know what you're doing!"
#endif

#include "firmwareVersion.h"
#if (kRobotCVersionNumeric < 359)
#error "These drivers are only supported on RobotC 3.59 or higher :("
#endif

#ifndef MAX_ARR_SIZE
// Max buffer size for byte_array, can be overridden in your program.
// It's 17 bytes big because the max I2C buffer size is 16,
// plus 1 byte to denote packet length.
#define MAX_ARR_SIZE 17
#endif


// Returns the smaller of 2 numbers.
#define min2(a, b) (a < b ? a : b)
// Returns the smaller of 3 numbers.
#define min3(a, b, c) (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c)
// Returns the larger of 2 numbers.
#define max2(a, b) (a > b ? a : b)
// Returns the larger of 3 numbers.
#define max3(a, b, c) (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c)
// Returns x if it is between min and max.
// If it is outside the range, returns min or max.
#define clip(a, b, c) min2(c, max2(b, a))


// Array of bytes as a struct, this is a workaround for RobotC's
// inability to pass an array to a function.
typedef ubyte tByteArray[MAX_ARR_SIZE];
typedef sbyte tsByteArray[MAX_ARR_SIZE];
typedef ubyte tMassiveArray[128];	// 128 byte array for lots of data
typedef ubyte tHugeByteArray[64];	// 64 byte array for lots of data
typedef ubyte tBigByteArray[32];	// 32 byte array for lots of data
typedef ubyte tIPaddr[4];			// struct for holding IPs

// Array of ints as a struct, this is a workaround for RobotC's
// inability to pass an array to a function.
typedef int tIntArray[MAX_ARR_SIZE];


void clearI2CError(tSensors link, ubyte address);
void clearI2Cbus(tSensors link);
bool waitForI2CBus(tSensors link);
bool writeI2C(tSensors link, tByteArray &request, tByteArray &reply, int replylen);
bool writeI2C(tSensors link, tByteArray &request);


/**
* Clear out the error state on I2C bus by sending a bunch of dummy
* packets.
* @param link the port number
* @param address the I2C address we're sending to
*/
void clearI2CError(tSensors link, ubyte address) {
	ubyte error_array[2];
	error_array[0] = 1; 		// Message size
	error_array[1] = address;	// I2C Address

	#ifdef __COMMON_H_DEBUG__
	eraseDisplay();
	nxtDisplayTextLine(3, "rxmit: %d", ubyteToInt(error_array[1]));
	wait1Msec(2000);
	#endif // __COMMON_H_DEBUG__

	for (int i = 0; i < 5; i++) {
		sendI2CMsg(link, &error_array[0], 0);
		wait1Msec(5);
	}
}


/**
* Wait for the I2C bus to be ready for the next message
* @param link the port number
* @return true if no error occured, false if it did
*/
bool waitForI2CBus(tSensors link)
{
	while (true)
	{
		switch (nI2CStatus[link])
		{
		case NO_ERR:
			return true;
		case STAT_COMM_PENDING:
			break;
		case ERR_COMM_CHAN_NOT_READY:
			break;
		case ERR_COMM_BUS_ERR:
			#ifdef __COMMON_H_DEBUG__
			PlaySound(soundLowBuzz);
			while (bSoundActive) {}
			#endif // __COMMON_H_DEBUG__
			return false;
		}
		EndTimeSlice();
	}
}


/**
* Write to the I2C bus. This function will clear the bus and wait for it be ready
* before any bytes are sent.
* @param link the port number
* @param request the data to be sent
* @return true if no error occured, false if it did
*/
bool writeI2C(tSensors link, tByteArray &request) {

	#if (__COMMON_H_SENSOR_CHECK__ == 1)
	//TSensorTypes type = SensorType[link];

	switch (SensorType[link])
	{
	case sensorI2CCustom:					break;
	case sensorI2CCustom9V:					break;
	case sensorI2CCustomFast:				break;
	case sensorI2CCustomFast9V:				break;
	case sensorI2CCustomFastSkipStates9V:	break;
	case sensorI2CCustomFastSkipStates:		break;
	default:
		hogCPU();
		PlaySound(soundException);
		eraseDisplay();
		nxtDisplayCenteredTextLine(0, "3rd Party Driver");
		nxtDisplayCenteredTextLine(1, "ERROR");
		nxtDisplayCenteredTextLine(2, "You have not");
		nxtDisplayCenteredTextLine(3, "setup the sensor");
		nxtDisplayCenteredTextLine(4, "port correctly. ");
		nxtDisplayCenteredTextLine(5, "Please refer to");
		nxtDisplayCenteredTextLine(6, "one of the");
		nxtDisplayCenteredTextLine(7, "examples.");
		wait1Msec(10000);
		StopAllTasks();
	}
	#endif

	if (!waitForI2CBus(link)) {
		clearI2CError(link, request[1]);

		// Let's try the bus again, see if the above packets flushed it out
		// clearI2CBus(link);
		if (!waitForI2CBus(link))
			return false;
	}

	sendI2CMsg(link, &request[0], 0);

	if (!waitForI2CBus(link)) {
		clearI2CError(link, request[1]);
		sendI2CMsg(link, &request[0], 0);
		if (!waitForI2CBus(link))
			return false;
	}
	return true;
}


/**
* Write to the I2C bus. This function will clear the bus and wait for it be ready
* before any bytes are sent.
* @param link the port number
* @param request the data to be sent
* @param reply array to hold received data
* @param replylen the number of bytes (if any) expected in reply to this command
* @return true if no error occured, false if it did
*/
bool writeI2C(tSensors link, tByteArray &request, tByteArray &reply, int replylen) {
	// clear the input data buffer

	#if (__COMMON_H_SENSOR_CHECK__ == 1)
	//TSensorTypes type = SensorType[link];

	switch (SensorType[link])
	{
	case sensorI2CCustom:                 break;
	case sensorI2CCustom9V:               break;
	case sensorI2CCustomFast:             break;
	case sensorI2CCustomFast9V:           break;
	case sensorI2CCustomFastSkipStates9V: break;
	case sensorI2CCustomFastSkipStates:   break;
	default:
		hogCPU();
		PlaySound(soundException);
		eraseDisplay();
		nxtDisplayCenteredTextLine(0, "3rd Party Driver");
		nxtDisplayCenteredTextLine(1, "ERROR");
		nxtDisplayCenteredTextLine(2, "You have not");
		nxtDisplayCenteredTextLine(3, "setup the sensor");
		nxtDisplayCenteredTextLine(4, "port correctly. ");
		nxtDisplayCenteredTextLine(5, "Please refer to");
		nxtDisplayCenteredTextLine(6, "one of the");
		nxtDisplayCenteredTextLine(7, "examples.");
		wait1Msec(10000);
		StopAllTasks();
	}
	#endif

	if (!waitForI2CBus(link)) {
		clearI2CError(link, request[1]);

		// Let's try the bus again, see if the above packets flushed it out
		// clearI2CBus(link);
		if (!waitForI2CBus(link))
			return false;
	}

	sendI2CMsg(link, &request[0], replylen);

	if (!waitForI2CBus(link)) {
		clearI2CError(link, request[1]);
		sendI2CMsg(link, &request[0], replylen);
		if (!waitForI2CBus(link))
			return false;
	}

	// ask for the input to put into the data array
	readI2CReply(link, &reply[0], replylen);

	return true;
}


/**
* Create a unique ID (UID) for an NXT.  This based on the last 3 bytes
* of the Bluetooth address.  The first 3 bytes are manufacturer
* specific and identical for all NXTs and are therefore not used.
* @return a unique ID for the NXT.
*/
long getUID() {
	TBTAddress btAddr;
	getBTAddress(btAddr);

	// Only last 3 bytes are unique in the BT address, the other three are for the
	// manufacturer (LEGO):  http://www.coffer.com/mac_find/?string=lego
	return (long)btAddr[5] + ((long)btAddr[4] << 8) + ((long)btAddr[3] << 16);
}


#define STRTOK_MAX_TOKEN_SIZE 20
#define STRTOK_MAX_BUFFER_SIZE 50
/**
* Tokenise an array of chars, using a seperator
* @param buffer pointer to buffer we're parsing
* @param token pointer to buffer to hold the tokens as we find them
* @param seperator the seperator used between tokens
* @return true if there are still tokens left, false if we're done
*/
bool strtok(char *buffer, char *token, char *seperator)
{
	int pos = StringFind(buffer, seperator);
	char t_buff[STRTOK_MAX_BUFFER_SIZE];

	// Make sure we zero out the buffer and token
	memset(token, 0, STRTOK_MAX_TOKEN_SIZE);
	memset(&t_buff[0], 0, STRTOK_MAX_BUFFER_SIZE);

	// Looks like we found a seperator
	if (pos >= 0)
	{
		// Copy the first token into the token buffer,
		// only if the token isn't an empty one
		if (pos > 0)
			memcpy(token, buffer, pos);
		// Now copy characters -after- the seperator into the temp buffer
		memcpy(&t_buff[0], buffer + (pos + 1), strlen(buffer) - pos);
		// Zero out the real buffer
		memset(buffer, 0, strlen(buffer) + 1);
		// Copy the temp buffer, which now only contains everything after the previous
		// token into the buffer for the next round.
		memcpy(buffer, &t_buff[0], strlen(&t_buff[0]));
		return true;
	}
	// We found no seperator but the buffer still contains a string
	// This can happen when there is no trailing seperator
	else if(strlen(buffer) > 0)
	{
		// Copy the token into the token buffer
		memcpy(token, buffer, strlen(buffer));
		// Zero out the remainder of the buffer
		memset(buffer, 0, strlen(buffer) + 1);
		return true;
	}
	return false;
}



#endif // __COMMON_H__
