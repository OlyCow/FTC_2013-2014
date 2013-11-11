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
#pragma config(Motor,  mtr_S1_C3_2,     motor_lift,    tmotorTetrix, openLoop, encoder)
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

task drive();
task setLift();
task waveFlag();

// 1 = L, -1 = R; this should only affect horizontal movements.
const int AUTON_L_R = -1;

float g_translation_x = 0.0;
float g_translation_y = 0.0;
float g_rotation = 0.0;
float g_lift_target = 0.0;
bool g_isWavingFlag = false;



task main() {
	Task_Spawn(drive);
	Task_Spawn(setLift);
	HTIRS2setDSPMode(sensor_IR, g_IRsensorMode);

	typedef enum Crate {
		CRATE_UNKNOWN	= -1,
		CRATE_OUTER_A	= 0,
		CRATE_INNER_A	= 1,
		CRATE_INNER_B	= 2,
		CRATE_OUTER_B	= 3,
		CRATE_NUM,
	};

	// TODO: Determine all the following values.
	const int fine_tune_power = 100;
	const float LIFT_LOW_POS = 0.0;
	const float LIFT_MED_POS = 0.0;
	const float LIFT_HIGH_POS = 0.0;
	const short IR_threshold = 500;
	const int servo_dump_open = 0;
	const int servo_dump_closed = 0;
	const int servo_dump_delay = 0;
	const int drive_time_low[CRATE_NUM] = {0.0, 0.0, 0.0, 0.0};
	const int drive_time_high[CRATE_NUM] = {0.0, 0.0, 0.0, 0.0};
	const int start_to_first_turn_time = 1000; // milliseconds?
	const int first_turn_to_second_turn_time = 1000; // milliseconds?
	const int second_turn_to_ramp_time = 1000; // milliseconds?
	const int iteration_delay = 0; // For flag waving.
	Crate crate_IR = CRATE_UNKNOWN;

	Joystick_WaitForStart();

	Time_ClearTimer(T1); // We will use this to guage which crate we're putting cubes into.

	// Raise the lift and move sideways until in front of the IR beacon.
	g_translation_x = AUTON_L_R*(-fine_tune_power);
	g_lift_target = LIFT_MED_POS;

	// These will later trigger breaking out of the next loop to dump the
	// cubes into the crate. Only `IR_value_C` is needed for detection,
	// but Xander's drivers only has a function which needs buffers for
	// each direction. Apparently they're also of type `short`.
	short IR_value_A = 0;
	short IR_value_B = 0;
	short IR_value_C = 0;
	short IR_value_D = 0;
	short IR_value_E = 0;
	float drive_time = 0.0;
	while (true) {
		HTIRS2readAllACStrength(sensor_IR, IR_value_A, IR_value_B, IR_value_C, IR_value_D, IR_value_E);
		if (IR_value_C>IR_threshold) {
			g_translation_x = 0;
			drive_time = Time_GetTime(T1);
			Task_Spawn(waveFlag);
			if ((drive_time>drive_time_low[CRATE_OUTER_A])&&(drive_time<drive_time_high[CRATE_OUTER_A])) {
				crate_IR = CRATE_OUTER_A;
			} else if ((drive_time>drive_time_low[CRATE_INNER_A])&&(drive_time<drive_time_high[CRATE_INNER_A])) {
				crate_IR = CRATE_INNER_A;
			} else if ((drive_time>drive_time_low[CRATE_INNER_B])&&(drive_time<drive_time_high[CRATE_INNER_B])) {
				crate_IR = CRATE_INNER_B;
			} else if ((drive_time>drive_time_low[CRATE_OUTER_B])&&(drive_time<drive_time_high[CRATE_OUTER_B])) {
				crate_IR = CRATE_OUTER_B;
			} // else it defaults to the initial "CRATE_UNKNOWN" value.
			break;
		}
		Time_Wait(5); // Magic number! (Do we even need this line?)
	}

	// Now we dump the IR cube...
	Servo_SetPosition(servo_dump, servo_dump_open);
	Time_Wait(servo_dump_delay);

	// And move onto the ramp.
	g_lift_target = LIFT_LOW_POS;
	g_translation_x = AUTON_L_R*(-fine_tune_power);
	Time_Wait(start_to_first_turn_time-drive_time);
	g_translation_x = 0;
	g_translation_y = fine_tune_power;
	Time_Wait(first_turn_to_second_turn_time);
	g_translation_y = 0;
	g_translation_x = AUTON_L_R*fine_tune_power;
	Time_Wait(second_turn_to_ramp_time);
	g_translation_x = 0;

	// Celebrate!
	while (true) {
		if (g_isWavingFlag==false) {
			Task_Spawn(waveFlag);
		}
		Time_Wait(iteration_delay);
	}
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



task setLift() {
	const float kP = 1.0;
	float current_position = 0.0;
	// `error` would be `g_lift_target-current_position`, but that crashes RobotC :/
	float error = 0.0;
	float power_lift = 0;
	Motor_ResetEncoder(motor_lift);
	Joystick_WaitForStart();

	while (true) {
		error = g_lift_target-current_position;
		power_lift = kP*error;
		Motor_SetPower(power_lift, motor_lift);
	}
}



task waveFlag() {
	g_isWavingFlag = true;

	// TODO: Determine these values.
	const int flag_L_pos = 0;
	const int flag_M_pos = 0;
	const int flag_R_pos = 0;
	const int delay = 0;
	const int wave_num = 3;

	for (int i=0; i<wave_num; i++) {
		Servo_SetPosition(servo_flag, flag_L_pos);
		Time_Wait(delay);
		Servo_SetPosition(servo_flag, flag_R_pos);
		Time_Wait(delay);
	}
	Servo_SetPosition(servo_flag, flag_M_pos);
	Time_Wait(delay);

	g_isWavingFlag = false;
	Task_Kill(nCurrentTask);
}
