#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S2, HTServo,  HTServo,  none,     none)
#pragma config(Sensor, S3,     ,               sensorI2CCustom)
#pragma config(Sensor, S4,     ,               sensorI2CCustom9V)
#pragma config(Motor,  mtr_S1_C1_1,     motor_FL,      tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motor_FR,      tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motor_BL,      tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motor_BR,      tmotorTetrix, PIDControl, encoder)
#pragma config(Servo,  srvo_S2_C1_1,    servo_FR,             tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_2,    servo_FL,             tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_3,    servo_BL,             tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_4,    servo_BR,             tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_1,    servo7,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_2,    servo8,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_3,    servo9,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo12,              tServoNone)

#include "Headers\includes.h"
#include "Teleop-Basic.h"

#pragma autoStartTasks



task PID();

task main() {
	initializeGlobalVariables();
	g_task_main = Task_GetCurrentIndex();
	bool  shouldNormalize = false;
	float gyro_angle = 0;
	float rotation_magnitude = 0;
	float rotation_angle[POD_NUM] = {0,0,0,0};
	float rotation_x[POD_NUM] = {0,0,0,0};
	float rotation_y[POD_NUM] = {0,0,0,0};
	float translation_magnitude = 0;
	float translation_angle = 0;
	float translation_x = 0;
	float translation_y = 0;
	float combined_magnitude[POD_NUM] = {0,0,0,0};
	float combined_angle[POD_NUM] = {0,0,0,0};
	float combined_x[POD_NUM] = {0,0,0,0};
	float combined_y[POD_NUM] = {0,0,0,0};
	Joystick_WaitForStart();

	while (true) {
		Joystick_UpdateData();

		// This part is temporary. We are assigning a gyro angle from a joystick.
		// An operator moves the joystick to correspond with a flag mounted on the
		// frame of the drive base, to simulate the input from an actual joystick.
		// No word as to when we will be able to purchase a prototype board and
		// the MPU-6050 breakout board.
		gyro_angle = Math_RadToDeg(atan2(
							Joystick_Joystick(JOYSTICK_R, AXIS_Y, CONTROLLER_2),
							Joystick_Joystick(JOYSTICK_R, AXIS_X, CONTROLLER_2) ));

		// Actual code starts here. It is ridiculously simple, but oddly counter-
		// intuitive. The rotation vector and the translation vector are combined,
		// then the final vector is normalized. That is it. This algorithm has the
		// property of seeming like the direction of the resultant vector is the
		// average of the input vectors' directions, which it is. It might seem like
		// there would be inevitable slipping involved in this scheme, but a simple
		// derivative analysis of a generalized cycloid shows the above algorithm
		// to describe the direction of the wheel pods perfectly.
		rotation_magnitude = Joystick_GetRotationMagnitude();
		translation_x = Joystick_GetTranslationX();
		translation_y = Joystick_GetTranslationY();
		translation_magnitude = sqrt(pow(translation_x,2)+pow(translation_y,2)); //Pythagoras
		translation_angle = Math_RadToDeg(atan2(translation_y, translation_x));
		for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
			rotation_angle[i] = g_MotorData[i].angleOffset+gyro_angle+90; //tangent to circle, +90
			rotation_x[i] = rotation_magnitude*cosDegrees(rotation_angle[i]);
			rotation_y[i] = rotation_magnitude*sinDegrees(rotation_angle[i]);
			combined_x[i] = translation_x+rotation_x;
			combined_y[i] = translation_y+rotation_y;
			combined_magnitude[i] = sqrt(pow(combined_x[i],2)+pow(combined_y[i],2));
			if (combined_magnitude[i]>g_FullPower) {
				shouldNormalize = true;
			}
			combined_angle[i] = Math_RadToDeg(atan2(combined_y[i], combined_x[i]));
			g_ServoData[i].angle = combined_angle[i];
		}
		if (shouldNormalize==true) {
			float originalMaxPower =	combined_magnitude[0] +
										combined_magnitude[1] +
										combined_magnitude[2] +
										combined_magnitude[3];
			for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
				Math_Normalize(combined_magnitude[i], originalMaxPower, g_FullPower);
			}
		}
		for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
			g_MotorData[i].power = combined_magnitude[i];
		}

		Task_EndTimeslice();
	}
}



task PID() {
	g_task_PID = Task_GetCurrentIndex();
	float kP = 25; //might need to make these terms arrays, if inverse-ness is an issue
	float kI = 0; //ditto^
	float kD = 0; //ditto^
	float error[4] = {0,0,0,0};
	float term_P[4] = {0,0,0,0};
	float term_I[4] = {0,0,0,0};
	float term_D[4] = {0,0,0,0};
	float total_correction[4] = {0,0,0,0};
	for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
		Motor_SetEncoder(Math_Normalize(90, 360, 1440), Motor_Convert((Motor)i)); // Might not be 90, could be some other number :P
	}
	Joystick_WaitForStart();

	while (true) {
		Task_HogCPU(); //This may be unnecessary/too greedy. Remember to remove `Task_ReleaseCPU()` if not needed.

		// Assigns power to servos in same loop as power is calculated (kinda is rolling assignment)
		for (int i=POD_FR; i<(int)POD_NUM; i++) {
			error[i] = g_ServoData[i].angle - Math_Normalize(Motor_GetEncoder(Motor_Convert((Motor)i)), 1440, 360);
			term_P[i] = kP*error[i]; //kP might become an array; see declaration
			term_I[i] = 0; //TODO! Has timers :P
			term_D[i] = 0; //TODO! Has timers :P
			total_correction[i] = term_P[i]+term_I[i]+term_D[i];
			Servo_SetPower(Servo_Convert((Servo)i), total_correction[i]);
		}

		// Parse the motor settings and assign them to the motors (in same loop).
		for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
			if (g_MotorData[i].isReversed==true) {
				g_MotorData[i].power *= -1;
			}
			Motor_SetPower(g_MotorData[i].power, Motor_Convert((Motor)i));
		}

		Task_ReleaseCPU();
		Task_EndTimeslice(); //Unless `Task_ReleaseCPU()` automatically switches tasks?
	}
}
