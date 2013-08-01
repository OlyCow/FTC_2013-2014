#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S2, HTServo,  HTServo,  none,     none)
#pragma config(Motor,  mtr_S1_C1_1,     motor_FL,      tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motor_FR,      tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motor_BL,      tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motor_BR,      tmotorTetrix, PIDControl, encoder)
#pragma config(Servo,  srvo_S2_C1_1,    servo_FR,             tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_2,    servo_FL,             tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_3,    servo_BL,             tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_4,    servo_BR,             tServoContinuousRotation)

#include "Headers\includes.h"



task main()
{
	initializeGlobalVariables();
	float gyro_angle = 0;
	float rotation_magnitude = 0;
	float rotation_angle[4] = {0,0,0,0}; //4=# of motors (FR/FL/BL/BR)
	float rotation_x[4] = {0,0,0,0}; //4=# of motors (FR/FL/BL/BR)
	float rotation_y[4] = {0,0,0,0}; //4=# of motors (FR/FL/BL/BR)
	float translation_magnitude = 0;
	float translation_angle = 0;
	float translation_x = 0;
	float translation_y = 0;
	float combined_angle[4] = {0,0,0,0}; //4=# of motors (FR/FL/BL/BR)
	float combined_x[4] = {0,0,0,0}; //4=# of motors (FR/FL/BL/BR)
	float combined_y[4] = {0,0,0,0}; //4=# of motors (FR/FL/BL/BR)
	float motor_power[4] = {0,0,0,0}; //4=# of motors (FR/FL/BL/BR)
	float servo_angle[4] = {0,0,0,0}; //4=# of motors (FR/FL/BL/BR)

	while (true)
	{
		Joystick_UpdateData();

		rotation_magnitude = Joystick_Joystick(JOYSTICK_L, AXIS_X, CONTROLLER_1);
		translation_x = Joystick_Joystick(JOYSTICK_R, AXIS_X, CONTROLLER_1);
		translation_y = Joystick_Joystick(JOYSTICK_R, AXIS_Y, CONTROLLER_1);
		translation_magnitude = sqrt(pow(translation_x,2)+pow(translation_y,2)); //Pythagoras
		for (int i=(int)MOTOR_FR; i<=(int)MOTOR_BR; i++) {
			rotation_angle[i] = g_motorData[i].angleOffset+gyro_angle;
			rotation_x[i] = rotation_magnitude*sinDegrees(rotation_angle[i]);
			rotation_y[i] = rotation_magnitude*cosDegrees(rotation_angle[i]);
			combined_x[i] = translation_x+rotation_x;
			combined_y[i] = translation_y+rotation_y;
			combined_angle[i] = radiansToDegrees(atan2(combined_y[i],combined_x[i]));
			motor_power[i] = sqrt(pow(combined_x[i],2)+pow(combined_y[i],2))*sinDegrees(combined_angle[i]-rotation_angle[i]);
			if (abs(rotation_magnitude)>0) {
				servo_angle[i] = g_motorData[i].angleOffset;
			} else {
				translation_angle = radiansToDegrees(atan2(translation_y,translation_x))-gyro_angle-90; //degrees
				for (int j=(int)MOTOR_FR; j<=(int)MOTOR_BR; j++) {
					servo_angle[i] = translation_angle;
				}
			}
		}
	}
}
