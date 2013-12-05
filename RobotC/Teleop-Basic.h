#include "Headers\enums.h"
#include "Headers\Math.h"



tMotor	Motor_Convert(Motor motorName);
Motor	Motor_Convert(tMotor motorName);
TServoIndex Servo_Convert(Servo servoName);
Servo	Servo_Convert(TServoIndex servoName);
float	Joystick_GetTranslationX();
float	Joystick_GetTranslationY();
float	Joystick_GetRotationMagnitude();



tMotor	Motor_Convert(Motor motorName) {
	tMotor conversion;
	switch (motorName) {
		case MOTOR_FL:
			conversion = motor_FL;
			break;
		case MOTOR_FR:
			conversion = motor_FR;
			break;
		case MOTOR_BR:
			conversion = motor_BR;
			break;
		case MOTOR_BL:
			conversion = motor_BL;
			break;
	}
	return conversion;
}
Motor	Motor_Convert(tMotor motorName) {
	Motor conversion;
	switch (motorName) {
		case motor_FR:
			conversion = MOTOR_FR;
			break;
		case motor_FL:
			conversion = MOTOR_FL;
			break;
		case motor_BL:
			conversion = MOTOR_BL;
			break;
		case motor_BR:
			conversion = MOTOR_BR;
			break;
	}
	return conversion;
}
TServoIndex Servo_Convert(Servo servoName) {
	TServoIndex conversion;
	switch (servoName) {
		case servo_FR:
			conversion = SERVO_FR;
			break;
		case servo_FL:
			conversion = SERVO_FL;
			break;
		case servo_BL:
			conversion = SERVO_BL;
			break;
		case servo_BR:
			conversion = SERVO_BR;
			break;
	}
	return conversion;
}
Servo	Servo_Convert(TServoIndex servoName) {
	Servo conversion;
	switch (servoName) {
		case SERVO_FR:
			conversion = servo_FR;
			break;
		case SERVO_FL:
			conversion = servo_FL;
			break;
		case SERVO_BL:
			conversion = servo_BL;
			break;
		case SERVO_BR:
			conversion = servo_BR;
			break;
	}
	return conversion;
}
float	Joystick_GetTranslationX() {
	return Math_Limit(
			Math_TrimDeadband((float)Joystick_Joystick(JOYSTICK_R, AXIS_X)), g_FullPower);
}
float	Joystick_GetTranslationY() {
	return Math_Limit(
			Math_TrimDeadband((float)Joystick_Joystick(JOYSTICK_R, AXIS_Y)), g_FullPower);
}
float	Joystick_GetRotationMagnitude() {
	return -Math_Limit(
			Math_TrimDeadband((float)Joystick_Joystick(JOYSTICK_L, AXIS_X)), g_FullPower);
}
