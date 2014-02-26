#ifndef SERVO_H
#define SERVO_H
#pragma systemFile



void Servo_SetWinch(TServoIndex servoName, int degrees);
void Servo_SetPosition(TServoIndex servoName, short position);
int  Servo_GetPosition(TServoIndex servoName);
void Servo_SetPower(TServoIndex servoName, short power); //for continuous rotation servos
int  Servo_GetPower(TServoIndex servoName); //for continuous rotation servos
void Servo_SetSpeed(TServoIndex servoName, int rate);
int  Servo_GetSpeed(TServoIndex servoName);
void Servo_LockPosition(bool isLocked=true);
void Servo_LockPosition(TServoIndex servoName, bool isLocked=true);
bool Servo_IsLocked();
bool Servo_IsLocked(TServoIndex servoName);
void Servo_UpdateInterval(int interval);
void Servo_UpdateInterval(TServoIndex servoName, int interval);
int  Servo_UpdateInterval();
int  Servo_UpdateInterval(TServoIndex servoName);



#include "..\Libraries\Servo.c"
#endif // SERVO_H
