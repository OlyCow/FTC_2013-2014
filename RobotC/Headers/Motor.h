#ifndef MOTOR_H
#define MOTOR_H
#pragma systemFile
#include "..\Libraries\Motor.c"



//void Motor_Forward(tMotor motorName, int power);
//void Motor_Reverse(tMotor motorName, int power);
//void Motor_Stop(tMotor motorName, bool brake=true);

//// This function does NOT reset the encoder, in case that is being
//// used elsewhere. Reset the encoder periodically to prevent overflow.
//void Motor_Target(tMotor motorName, int angle);
//void Motor_SetPower(tMotor motorName, int power);
//int Motor_GetEncoder(tMotor motorName);
//void Motor_ResetEncoder(tMotor motorName);
//void Motor_SetBrakes(bool isOn=true);
//void Motor_SetMaxSpeed(int speed=750);s
//void Motor_SetPIDInterval(int interval=20);



//DONE!!!	=== bFloatDuringInactiveMotorPWM
//DONE!!!	===(Not doing this) bMotorReflected[]
//DONE!!!	=== motor[]
//DONE!!!	=== motorPWMLevel[]
//DONE!!!	=== nMaxRegulatedSpeed12V
//DONE!!!	=== nMaxRegulatedSpeedNxt
//DONE!!!	=== nMotorEncoder
//DONE!!!	=== nMotorEncoderTarget[]
//DONE!!!	===(Not doing this) nMotorPIDSpeedCtrl[]
//DONE!!!	===(Not doing this) nMotorRunState[]
//DONE!!!	=== nPidUpdateInterval
//DONE!!!	=== nPidUpdateInterval12V
//DONE!!!	===(Not doing this) nSyncedMotors
//DONE!!!	===(Not doing this) nSyncedTurnRatio

void Motor_SetPower(int power, tMotor motorName);
int  Motor_GetPower(tMotor motorName);
void Motor_SetEncoder(long encoderValue, tMotor motorName);
long Motor_GetEncoder(tMotor motorName);
void Motor_SetBrakes(bool isOn=true);
bool Motor_GetBrakes();
void Motor_SetMaxSpeed(int speed, MotorType motorType=MOTORTYPE_ALL);
void Motor_SetMaxSpeed(int speed, tMotor motorName); //warning: for all motors of the same type!
int  Motor_GetMaxSpeed(MotorType motorType=MOTORTYPE_ALL);
int  Motor_GetMaxSpeed(tMotor motorName);
void Motor_SetEncoderTarget(long encoderValue, tMotor motorName);
int  Motor_GetEncoderTarget(tMotor motorName);
void Motor_SetPIDInterval(int interval, MotorType motorType=MOTORTYPE_ALL);
void Motor_SetPIDInterval(int interval, tMotor motorName); //warning: for all motors of the same type!
int  Motor_GetPIDInterval(MotorType motorType=MOTORTYPE_ALL);
int  Motor_GetPIDInterval(tMotor motorName);
int  Motor_GetAssignedPower(tMotor motorName);

//Types of functions:
//
// Low-level; used for setting vars



#endif // MOTOR_H
