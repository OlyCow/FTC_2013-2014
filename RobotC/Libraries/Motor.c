#ifndef MOTOR_C
#define MOTOR_C
#pragma systemFile
#include "..\Headers\Motor.h"
// For default values, see above header file.



void Motor_SetPower(int power, tMotor motorName)
{
	motor[motorName] = power;
}
int Motor_GetPower(tMotor motorName)
{
	return motor[motorName];
}
void Motor_SetEncoder(long encoderValue, tMotor motorName)
{
	nMotorEncoder[motorName] = encoderValue;
}
long Motor_GetEncoder(tMotor motorName)
{
	return nMotorEncoder[motorName];
}
void Motor_SetBrakes(bool isOn)
{
	bFloatDuringInactiveMotorPWM = !isOn;
}
bool Motor_GetBrakes()
{
	return !bFloatDuringInactiveMotorPWM;
}
void Motor_SetMaxSpeed(int speed, MotorType motorType)
{
	switch (motorType)
	{
		case MOTORTYPE_NXT:
			nMaxRegulatedSpeedNxt = speed;
			break;
		case MOTORTYPE_12V:
			nMaxRegulatedSpeed12V = speed;
			break;
		case MOTORTYPE_ALL:
			nMaxRegulatedSpeedNxt = speed;
			nMaxRegulatedSpeed12V = speed;
			break;
	}
}
void Motor_SetMaxSpeed(int speed, tMotor motorName) //warning: for all motors of the same type!
{
	// TODO: First find the MotorType of the specified motor, then use
	// the same logic as the other (overloaded) Motor_SetMaxSpeed().
}
int Motor_GetMaxSpeed(MotorType motorType)
{
	// The "default" value is being set to a negative number to make errors obvious.
	int maxSpeed = -1024;
	switch (motorType)
	{
		case MOTORTYPE_NXT:
			maxSpeed = nMaxRegulatedSpeedNxt;
			break;
		case MOTORTYPE_12V:
			maxSpeed = nMaxRegulatedSpeed12V;
			break;
	}
	return maxSpeed;
}
int Motor_GetMaxSpeed(tMotor motorName)
{
	// TODO: First find the MotorType of the specified motor, then use
	// the same logic as the other (overloaded) Motor_GetMaxSpeed().
}
void Motor_SetEncoderTarget(long encoderValue, tMotor motorName)
{
	nMotorEncoderTarget[motorName] = encoderValue;
}
int  Motor_GetEncoderTarget(tMotor motorName)
{
	return nMotorEncoderTarget[motorName];
}
void Motor_SetPIDInterval(int interval, MotorType motorType)
{
	switch (motorType)
	{
		case MOTORTYPE_NXT:
			// nPidUpdateIntervalNxt is internally #defined as nPidUpdateInterval
			nPidUpdateInterval = interval;
			break;
		case MOTORTYPE_12V:
			nPidUpdateInterval12V = interval;
			break;
		case MOTORTYPE_ALL:
			nPidUpdateInterval = interval;
			nPidUpdateInterval12V = interval;
			break;
	}
}
void Motor_SetPIDInterval(int interval, tMotor motorName) //warning: for all motors of the same type!
{
	// TODO: First find the MotorType of the specified motor, then use
	// the same logic as the other (overloaded) Motor_SetPIDInterval().
}
int Motor_GetPIDInterval(MotorType motorType)
{
	// RobotC requires an explicit `return` statement not nested inside a function.
	// The "default" value is being set to a negative number to make errors obvious.
	int returnValue = -1024;
	switch (motorType)
	{
		case MOTORTYPE_NXT:
			// nPidUpdateIntervalNxt is internally #defined as nPidUpdateInterval
			returnValue = nPidUpdateInterval;
			break;
		case MOTORTYPE_12V:
			returnValue = nPidUpdateInterval12V;
			break;
	}
	return returnValue;
}
int Motor_GetPIDInterval(tMotor motorName)
{
	// TODO: First find the MotorType of the specified motor, then use
	// the same logic as the other (overloaded) Motor_SetPIDInterval().
}
int Motor_GetAssignedPower(tMotor motorName)
{
	return motorPWMLevel[motorName];
}
void Motor_SetState(TNxtRunState state, tMotor motorName)
{
	nMotorRunState[motorName] = state;
}
TNxtRunState Motor_GetState(tMotor motorName)
{
	return nMotorRunState[motorName];
}



#endif // MOTOR_C
