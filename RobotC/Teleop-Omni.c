#pragma config(Hubs,	S1,	HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor,	S1,     ,               sensorI2CMuxController)
#pragma config(Motor,	mtr_S1_C1_1,	motor_FR,	tmotorTetrix,	openLoop)
#pragma config(Motor,	mtr_S1_C1_2,	motor_FL,	tmotorTetrix,	openLoop)
#pragma config(Motor,	mtr_S1_C2_1,	motor_BL,	tmotorTetrix,	openLoop)
#pragma config(Motor,	mtr_S1_C2_2,	motor_BR,	tmotorTetrix,	openLoop)

#include "includes.h"



task main()
{
	float translation_x = 0.0;
	float translation_y = 0.0;
	float rotation = 0.0;
	float power_FR = 0.0;
	float power_FL = 0.0;
	float power_BL = 0.0;
	float power_BR = 0.0;
	float power_cap = 0.0;
	bool doNormalizePower = false;
	Joystick_WaitForStart();

	while (true)
	{
		Joystick_UpdateData();
		translation_x = Math_Normalize(Math_TrimDeadband(Joystick_Joystick(JOYSTICK_R, AXIS_X), 10), 128, 100);
		translation_y = Math_Normalize(Math_TrimDeadband(Joystick_Joystick(JOYSTICK_R, AXIS_Y), 10), 128, 100);
		rotation = Math_Normalize(Math_TrimDeadband(Joystick_Joystick(JOYSTICK_L, AXIS_X), 10), 128, 100);

		power_FR = translation_y-translation_x-rotation;
		power_FL = translation_y+translation_x+rotation;
		power_BL = translation_y-translation_x+rotation;
		power_BR = translation_y+translation_x-rotation;

		if (power_FR>g_FullPower) {
			power_cap = power_FR;
			doNormalizePower = true;
		}
		if ((power_FL>g_FullPower)&&(power_FL>power_cap)) {
			power_cap = power_FL;
			doNormalizePower = true;
		}
		if ((power_BL>g_FullPower)&&(power_BL>power_cap)) {
			power_cap = power_BL;
			doNormalizePower = true;
		}
		if ((power_BR>g_FullPower)&&(power_BR>power_cap)) {
			power_cap = power_BR;
			doNormalizePower = true;
		}
		if (doNormalizePower==true) {
			Math_Normalize(power_FR, power_cap, g_FullPower);
			Math_Normalize(power_FR, power_cap, g_FullPower);
			Math_Normalize(power_FR, power_cap, g_FullPower);
			Math_Normalize(power_FR, power_cap, g_FullPower);
			doNormalizePower = false;
		}

		Motor_SetPower(power_FR, motor_FR);
		Motor_SetPower(power_FL, motor_FL);
		Motor_SetPower(power_BL, motor_BL);
		Motor_SetPower(power_BR, motor_BR);
	}
}
