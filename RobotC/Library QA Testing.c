#pragma config(Hubs,  S1, HTMotor,  none,     none,     none)
#pragma config(Hubs,  S2, HTServo,  none,     none,     none)
#pragma config(Motor,  motorA,          motor_A,       tmotorNXT, PIDControl)
#pragma config(Motor,  motorB,          motor_B,       tmotorNXT, PIDControl)
#pragma config(Motor,  motorC,          motor_C,       tmotorNXT, PIDControl)
#pragma config(Motor,  mtr_S1_C1_1,     motor_D,       tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motor_E,       tmotorTetrix, PIDControl, encoder)
#pragma config(Servo,  srvo_S2_C1_1,    servo_A,              tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    servo_B,              tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    servo_C,              tServoStandard)
#pragma config(Servo,  srvo_S2_C1_4,    servo_D,              tServoStandard)
#pragma config(Servo,  srvo_S2_C1_5,    servo_E,              tServoStandard)
#pragma config(Servo,  srvo_S2_C1_6,    servo_F,              tServoStandard)

#include "Headers\includes.h"



task main()
{
	initializeVariables();

	waitForStart();

	while (true)
	{
		//CODE!!!
	}
}
