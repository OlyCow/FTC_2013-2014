#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  motorA,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorB,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C1_1,     motor_R,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     motor_L,       tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     motor_flag,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motor_cube,    tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "includes.h"



task main() {
	bDisplayDiagnostics = false;
	float power_R = 0.0;
	float power_L = 0.0;
	float power_flag = 0.0;
	float power_cube = 0.0;
	Joystick_WaitForStart();

	while (true) {
		Joystick_UpdateData();
		power_R = Joystick_Joystick(JOYSTICK_R, AXIS_Y);
		power_L = Joystick_Joystick(JOYSTICK_L, AXIS_Y);
		power_cube = Joystick_Joystick(JOYSTICK_R, AXIS_Y, CONTROLLER_2);

		if (Joystick_Button(BUTTON_LB)==true) {
			power_flag = -100;
		} else if (Joystick_Button(BUTTON_RB)==true) {
			power_flag = 100;
		} else {
			power_flag = 0;
		}
		nxtDisplayTextLine(1, "L:%d", power_L);
		nxtDisplayTextLine(2, "R:%d", power_R);
		nxtDisplayTextLine(3, "flag:%d", power_flag);
		Motor_SetPower(power_cube, motor_cube);
		Motor_SetPower(power_R, motor_R);
		Motor_SetPower(power_L, motor_L);
		Motor_SetPower(power_flag, motor_flag);
	}
}
