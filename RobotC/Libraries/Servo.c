#ifndef SERVO_C
#define SERVO_C
#pragma systemFile
#include "..\Headers\Servo.h"
// For default values, see above header file.



void Servo_SetWinch(TServoIndex servoName, int degrees) {
	// MAGIC_NUM and TODO: These numbers are experimentally determined (measured
	// with paper, not even a protractor). RMSE ~= 10.237 deg; slope +/- 0.03 deg
	degrees = Math_Limit(degrees, 1260); // MAGIC_NUM: Corresponds to about 100 bits.
	degrees /= 11.82; // This was experimentally determined.
	degrees = round(degrees) + 128; // Not 127!
	servo[servoName] = degrees;
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
