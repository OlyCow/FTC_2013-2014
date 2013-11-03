#pragma config(Hubs,	S1,	HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor,	S1,     ,               sensorI2CMuxController)
#pragma config(Motor,	mtr_S1_C1_1,	motor_FR,	tmotorTetrix,	openLoop)
#pragma config(Motor,	mtr_S1_C1_2,	motor_FL,	tmotorTetrix,	openLoop)
#pragma config(Motor,	mtr_S1_C2_1,	motor_BL,	tmotorTetrix,	openLoop)
#pragma config(Motor,	mtr_S1_C2_2,	motor_BR,	tmotorTetrix,	openLoop)

#include "includes.h"



task main()
{
	Joystick_WaitForStart();

	while (true)
	{

	}
}
