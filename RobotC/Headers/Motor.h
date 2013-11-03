#ifndef MOTOR_H
#define MOTOR_H
#pragma systemFile
#include "..\Headers\enums.h"



#define	Motor_SetPower(power, motorName)			(motor[motorName]=(power))
#define	Motor_GetPower(motorName)					(motor[motorName])
#define	Motor_ResetEncoder(motorName)				(nMotorEncoder[motorName]=0) //can only set to 0
#define	Motor_GetEncoder(motorName)					(nMotorEncoder[motorName])
void	Motor_SetBrakes(bool isOn=true);
#define	Motor_GetBrakes()							(!bFloatDuringInactiveMotorPWM)
void	Motor_SetMaxSpeed(int speed, MotorType motorType=MOTORTYPE_ALL);
void	Motor_SetMaxSpeed(int speed, tMotor motorName); //warning: for all motors of the same type!
int		Motor_GetMaxSpeed(MotorType motorType=MOTORTYPE_ALL);
int		Motor_GetMaxSpeed(tMotor motorName);
#define	Motor_SetEncoderTarget(value, motorName)	(nMotorEncoderTarget[motorName]=(value))
#define	Motor_GetEncoderTarget(motorName)			(nMotorEncoderTarget[motorName])
void	Motor_SetPIDInterval(int interval, MotorType motorType=MOTORTYPE_ALL);
void	Motor_SetPIDInterval(int interval, tMotor motorName); //warning: for all motors of the same type!
int		Motor_GetPIDInterval(MotorType motorType=MOTORTYPE_ALL);
int		Motor_GetPIDInterval(tMotor motorName);
#define	Motor_GetAssignedPower(motorName)			(motorPWMLevel[motorName]) //motorPWMLevel[] is read-only, apparently
#define	Motor_SetState(state, motorName)			(nMotorRunState[motorName]=(state)) //state is type TNxtRunState
#define	Motor_GetState(motorName)					(nMotorRunState[motorName]) //returns type TNxtRunState



//void Motor_SetPower(int power, tMotor motorName);
//int  Motor_GetPower(tMotor motorName);
//void Motor_SetEncoder(long encoderValue, tMotor motorName);
//long Motor_GetEncoder(tMotor motorName);
//bool Motor_GetBrakes();
//void Motor_SetEncoderTarget(long encoderValue, tMotor motorName);
//int  Motor_GetEncoderTarget(tMotor motorName);
//int  Motor_GetAssignedPower(tMotor motorName); //motorPWMLevel[] is read-only, apparently
//void Motor_SetState(TNxtRunState state, tMotor motorName); //DANGER! this function shouldn't even be defined :P
//TNxtRunState Motor_GetState(tMotor motorName);

//Types of functions:
// Low-level; used for setting vars; don't use; etc.



#include "..\Libraries\Motor.c"
#endif // MOTOR_H
