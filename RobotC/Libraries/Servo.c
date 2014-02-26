#ifndef SERVO_C
#define SERVO_C
#pragma systemFile
#include "..\Headers\Servo.h"
// For default values, see above header file.



void Servo_SetWinch(TServoIndex servoName, int degrees) {
	degrees = round(Math_Limit((float)degrees, (float)575)); // MAGIC_NUM: Leaves us about 10.2 units on either side of insta-death.
	servo[servoName] = round((degrees+625)/1250.0*255.0);
}
void Servo_SetPosition(TServoIndex servoName, short position) {
	servo[servoName] = position;
}
int  Servo_GetPosition(TServoIndex servoName) {
	return ServoValue[servoName];
}
void Servo_SetPower(TServoIndex servoName, short power) {
	servo[servoName] = power+128;
}
int  Servo_GetPower(TServoIndex servoName) {
	return ServoValue[servoName];
}
void Servo_SetSpeed(TServoIndex servoName, int rate) {
	servoChangeRate[servoName] = rate; //apparently 0 is max speed
}
int  Servo_GetSpeed(TServoIndex servoName) {
	return servoChangeRate[servoName];
}
void Servo_LockPosition(bool isLocked) {
	bSystemLeaveServosEnabledOnProgramStop = isLocked;
}
void Servo_LockPosition(TServoIndex servoName, bool isLocked) {
	bSystemLeaveServosEnabledOnProgramStop = isLocked;
}
bool Servo_IsLocked() {
	return bSystemLeaveServosEnabledOnProgramStop;
}
bool Servo_IsLocked(TServoIndex servoName) {
	return bSystemLeaveServosEnabledOnProgramStop;
}
void Servo_UpdateInterval(int interval) {
	muxUpdateInterval = interval;
}
void Servo_UpdateInterval(TServoIndex servoName, int interval) {
	muxUpdateInterval = interval;
}
int  Servo_UpdateInterval() {
	return muxUpdateInterval;
}
int  Servo_UpdateInterval(TServoIndex servoName) {
	return muxUpdateInterval;
}



#endif // SERVO_C
