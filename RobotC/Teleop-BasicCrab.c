#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     motor_FR,      tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motor_FL,      tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motor_BL,      tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motor_BR,      tmotorTetrix, openLoop, reversed, encoder)

#include "Headers\includes.h"

task main() {
	int motorPower = 0;
	int crabPower = 0;
	while (true) {
		motorPower = Math_TrimDeadband(Joystick_Joystick(JOYSTICK_R, AXIS_Y), g_JoystickDeadband);
		crabPower = Math_TrimDeadband(Joystick_Joystick(JOYSTICK_L, AXIS_Y), g_JoystickDeadband);
		Motor_SetPower(motorPower, motor_FR);
		Motor_SetPower(motorPower, motor_FL);
		Motor_SetPower(motorPower, motor_BR);
		Motor_SetPower(crabPower, motor_BL);
	}
}
