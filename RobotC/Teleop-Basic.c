#pragma config(Hubs,  S3, HTServo,  none,     none,     none)
#pragma config(Hubs,  S4, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S3,     ,               sensorI2CMuxController)
#pragma config(Sensor, S4,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S4_C1_1,     motor_FR,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S4_C1_2,     motor_FL,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S4_C2_1,     motor_BL,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S4_C2_2,     motor_BR,      tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S3_C1_1,    servo_FR,             tServoStandard)
#pragma config(Servo,  srvo_S3_C1_2,    servo_FL,             tServoStandard)
#pragma config(Servo,  srvo_S3_C1_3,    servo_BL,             tServoStandard)
#pragma config(Servo,  srvo_S3_C1_4,    servo_BR,             tServoStandard)
#pragma config(Servo,  srvo_S3_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S3_C1_6,    servo6,               tServoNone)

#include "Headers\includes.h"
#include "Teleop-Basic.h"

// Automatically starts all tasks defined in this file. (May not be necessary,
// now that we've merged `task PID()` into `task main()`. This change can be
// reversed if our code gets too complicated/causes too much lag.)
#pragma autoStartTasks
#warning "I can declare warnings at will!"
#define WILL_EXPLODE // Comment out this line (Ctrl-Q) to prevent this from compiling!
#ifndef WILL_EXPLODE
#error "This code will explode!"
#endif

//---------------- README!!! ------------------------------------------------>>
//   The version of the code in the commit "WORKING VERSION!!!" is "working".
// If you somehow mess up the code, you can always revert. So feel free to play
// around.
//   Set the robot up as follows: with the front (where the NXT is mounted) facing
// towards you, the side of the wheel pods with ABS plastic should be on the
// left. As defined in "enums.h", the wheel pods are "numbered": `FR`, `FL`,
// `BL`, `BR`. As of this writing, `POD_BL` is missing a bevel gear and encoder,
// and thus has an omni-wheel attached to it. (This may or may not affect moving
// forward and rotating at the same time - I cannot test the code because the
// robot is out of batteries.)
//   The code is split into 2 chunks: 1. Find the target angle, velocity, etc.
// 2. Adjust the current angle, velocity, etc. through a PID control. The PID
// loop doesn't have an "I" or "D" term yet, so it either overshoots (and then
// starts oscillating), or it never reaches the target ("steady-state error").
// Go ahead and implement that if you wish!



task main() {
	initializeGlobalVariables(); // Defined in "global vars.h", this intializes all struct members.
	disableDiagnosticsDisplay(); // This disables the "samostat.rxe"-like diagnostics screen.

	// For finding target values:
	//g_task_main = Task_GetCurrentIndex(); // This was used when we had multiple tasks.
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

	// For PID control:
	float kP = 0.9;
	float kI = 0; // These terms aren't implemented yet.
	float kD = 0; // These terms aren't implemented yet.
	float error[POD_NUM] = {0,0,0,0}; // Difference between set-point and measured value.
	float term_P[POD_NUM] = {0,0,0,0};
	float term_I[POD_NUM] = {0,0,0,0}; // Not implemented. ^
	float term_D[POD_NUM] = {0,0,0,0}; // Not implemented. ^
	float total_correction[POD_NUM] = {0,0,0,0}; // Simply "term_P + term_I + term_D".

	// For troubleshooting:
	float ENCODER_VALUE[POD_NUM] = {0,0,0,0};

	Joystick_WaitForStart();

	while (true) {
		Joystick_UpdateData();

		// This part is temporary. We are assigning a gyro angle from a joystick.
		// An operator moves the joystick to correspond with a flag mounted on the
		// frame of the drive base, to simulate the input from an actual joystick.
		// No word as to when we will be able to purchase a prototype board and
		// the MPU-6050 breakout board.
		gyro_angle = 0;
		//gyro_angle = Math_RadToDeg(atan2(
		//					Joystick_Joystick(JOYSTICK_R, AXIS_Y, CONTROLLER_2),
		//					Joystick_Joystick(JOYSTICK_R, AXIS_X, CONTROLLER_2) ));

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
			combined_x[i] = translation_x+rotation_x[i];
			combined_y[i] = translation_y+rotation_y[i];
			combined_magnitude[i] = sqrt(pow(combined_x[i],2)+pow(combined_y[i],2));
			if (combined_magnitude[i]>g_FullPower) {
				shouldNormalize = true;
			}
			combined_angle[i] = (Math_RadToDeg(atan2(combined_y[i], combined_x[i]))+360)%360;
			g_ServoData[i].angle = 2*combined_angle[i];
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

		if (Joystick_Button(BUTTON_RT)==true) {
			for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
				g_MotorData[i].fineTuneFactor = 4;
			}
		} else if (Joystick_Button(BUTTON_LT)==true) {
			for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
				g_MotorData[i].fineTuneFactor = g_FullPower;
			}
		} else {
			for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
				g_MotorData[i].fineTuneFactor = 1;
			}
		}



		////PID control loop:
		//for (int i=POD_FR; i<(int)POD_NUM; i++) {
		//	error[i] = g_ServoData[i].angle - (Math_Normalize(Motor_GetEncoder(Motor_Convert((Motor)i)), 1440, 360)%360);
		//	term_P[i] = kP*error[i]; //kP might become an array
		//	term_I[i] = 0; //TODO! Has timers :P
		//	term_D[i] = 0; //TODO! Has timers :P
		//	total_correction[i] = Math_Limit((term_P[i]+term_I[i]+term_D[i]), 128);
		//}



		error[POD_FR] = g_ServoData[POD_FR].angle-(Math_Normalize(Motor_GetEncoder(motor_BL), (float)1440, 360)%360);
		error[POD_FL] = g_ServoData[POD_FL].angle-(Math_Normalize(Motor_GetEncoder(motor_FL), (float)1440, 360)%360);
		error[POD_BL] = g_ServoData[POD_BL].angle-(Math_Normalize(Motor_GetEncoder(motor_BL), (float)1440, 360)%360);
		error[POD_BR] = g_ServoData[POD_BR].angle-(Math_Normalize(Motor_GetEncoder(motor_BR), (float)1440, 360)%360);
		for (int i=POD_FR; i<(int)POD_NUM; i++) {
			if (error[i]>180) {
				error[i] = error[i]-360;
			}
		}
		term_P[POD_FL] = kP*error[POD_FL];
		term_P[POD_FR] = kP*error[POD_FR];
		term_P[POD_BL] = kP*error[POD_BL];
		term_P[POD_BR] = kP*error[POD_BR];
		total_correction[POD_FR] = Math_Limit((term_P[POD_FR]), 128);
		total_correction[POD_FL] = Math_Limit((term_P[POD_FL]), 128);
		total_correction[POD_BL] = Math_Limit((term_P[POD_BL]), 128);
		total_correction[POD_BR] = Math_Limit((term_P[POD_BR]), 128);



		// Troubleshooting:
		float ANGLE_FR = g_ServoData[MOTOR_FR].angle;
		float ANGLE_FL = g_ServoData[MOTOR_FL].angle;
		float ANGLE_BL = g_ServoData[MOTOR_BL].angle;
		float ANGLE_BR = g_ServoData[MOTOR_BR].angle;
		float CURRENT_FR = Math_Normalize(Motor_GetEncoder(motor_BL), (float)1440, 360)%360;
		float CURRENT_FL = Math_Normalize(Motor_GetEncoder(motor_FL), (float)1440, 360)%360;
		float CURRENT_BL = Math_Normalize(Motor_GetEncoder(motor_BL), (float)1440, 360)%360;
		float CURRENT_BR = Math_Normalize(Motor_GetEncoder(motor_BR), (float)1440, 360)%360;
		nxtDisplayTextLine(0, "FR deg%d pow%d", ANGLE_FR, total_correction[POD_FR]);
		nxtDisplayTextLine(1, "FL deg%d pow%d", ANGLE_FL, total_correction[POD_FL]);
		nxtDisplayTextLine(2, "BL deg%d pow%d", ANGLE_BL, total_correction[POD_BL]);
		nxtDisplayTextLine(3, "BR deg%d pow%d", ANGLE_BR, total_correction[POD_BR]);
		nxtDisplayTextLine(4, "FR current%d", CURRENT_FR);
		nxtDisplayTextLine(5, "FL current%d", CURRENT_FL);
		nxtDisplayTextLine(6, "BL current%d", CURRENT_BL);
		nxtDisplayTextLine(7, "BR current%d", CURRENT_BR);



		servo[servo_FR] = total_correction[POD_FR]+128;
		servo[servo_FL] = total_correction[POD_FL]+128;
		servo[servo_BL] = 128; // We don't have an encoder mounted for this servo... derp.
		//servo[servo_BL] = total_correction[POD_BL]+128;
		servo[servo_BR] = total_correction[POD_BR]+128;

		// Parse the motor settings and assign them to the motors (in same loop).
		for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
			if (g_MotorData[i].isReversed==true) {
				g_MotorData[i].power *= -1;
			}
			Motor_SetPower(g_MotorData[i].power/g_MotorData[i].fineTuneFactor, Motor_Convert((Motor)i));
		}



		//Task_EndTimeslice();
		if (Joystick_ButtonReleased(BUTTON_X)==true){
			Sound_Moo();
		}
	}
}
