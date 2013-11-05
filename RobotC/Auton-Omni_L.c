#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTServo,  HTServo,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     sensor_IR,      sensorI2CCustomFastSkipStates9V)
#pragma config(Sensor, S4,     sensor_gyro,    sensorI2CCustomFastSkipStates9V)
#pragma config(Motor,  mtr_S1_C1_1,     motor_FR,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     motor_FL,      tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     motor_BL,      tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     motor_BR,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     motor_sweeper, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     motor_lift,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     motor_flag,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     motorK,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S2_C1_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo_funnel_L,       tServoStandard)
#pragma config(Servo,  srvo_S2_C1_6,    servo_funnel_R,       tServoStandard)
#pragma config(Servo,  srvo_S2_C2_1,    servo_dump,           tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    servo_flag,           tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    servo9,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo12,              tServoNone)

#include "includes.h"

// Automatically starts all tasks defined in this file.
#pragma autoStartTasks

float g_translation_x = 0.0;
float g_translation_y = 0.0;
float g_rotation = 0.0;



task main() {
	Joystick_WaitForStart();
}



// Most of the following logic comes straight from our teleop code. :P
task drive() {
	float power_FR = 0.0;
	float power_FL = 0.0;
	float power_BL = 0.0;
	float power_BR = 0.0;
	float power_cap = 0.0; // The max value to normalize to.
	bool doNormalizePower = false;
	Joystick_WaitForStart();

	while (true) {
		// For the derivation of this dark wizardry algorithm, refer to our
		// engineering notebook, or talk to Ernest, Kieran, or Nathan.
		power_FR = g_translation_y-g_translation_x-g_rotation;
		power_FL = g_translation_y+g_translation_x+g_rotation;
		power_BL = g_translation_y-g_translation_x+g_rotation;
		power_BR = g_translation_y+g_translation_x-g_rotation;

		// Iterate through all four power levels, tripping the "normalize" flag
		// and updating `power_cap` if the power level exceeds full power AND
		// the current `power_cap`. `power_cap` doesn't need to be reset because
		// it doesn't matter if the "normalize" flag isn't tripped, and if the
		// flag is tripped `power_cap` will be overwritten.
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
			doNormalizePower = false; // Reset this flag for the next iteration.
		}

		// Finally get to set the power levels! Now for another iteration...
		Motor_SetPower(power_FR, motor_FR);
		Motor_SetPower(power_FL, motor_FL);
		Motor_SetPower(power_BL, motor_BL);
		Motor_SetPower(power_BR, motor_BR);
	}
}
