#ifndef SWERVE_DRIVE
#define SWERVE_DRIVE
#include "includes.h"



typedef enum WheelPod {
	POD_FR = 0,
	POD_FL = 1,
	POD_BL = 2,
	POD_BR = 3,
	POD_NUM,
} WheelPod;

typedef struct motorData {
	bool	isReversed;
	int		angleOffset;
	float	fineTuneFactor;
	int		power;
} motorData;

typedef struct servoData {
	int angle;
	int power;
} servoData;

// The mode the IR sensor is set at.
// The competition IR beam/emmiter pulses at 1200 hertz.
const tHTIRS2DSPMode g_IRsensorMode = DSP_1200;

// The threshold for IR values to count as detected.
const int g_IRthreshold = 60; // arbitrary units from 0~1024.

// TODO: This number is just a guess. Not verified at all.
const int g_EncoderDeadband = 1; // degrees.

// Transfers data between tasks. These have physical significance,
// such as the positioning of each pod (relative to the center of
// the robot. Both are initialized in `initializeRobotVariables()`.
motorData g_MotorData[POD_NUM];
servoData g_ServoData[POD_NUM];

// Various servo/encoder (motor) positions.
// MAGIC_NUM: TODO (all).
const int lift_pos_pickup		= 0;
const int lift_pos_dump			= 2600; // TODO
const int lift_pos_top			= 3500;	// TODO
const int lift_max_height		= 3600;
const int lift_sweeper_guard	= 150;	// TODO
const int lift_buffer_top		= 2500;
const int lift_buffer_bottom	= 1200;
const int lift_tube_guard		= 1000;

const int servo_climb_L_open	= 255;	// TODO
const int servo_climb_L_closed	= 0;	// TODO
const int servo_climb_R_open	= 0;	// TODO
const int servo_climb_R_closed	= 255;	// TODO
const int servo_dump_open		= 31;
const int servo_dump_closed		= 0;
const int servo_flip_L_up		= 213;
const int servo_flip_L_down		= 31;
const int servo_flip_R_up		= 42;
const int servo_flip_R_down		= 224;
const int servo_auton_up		= 255;
const int servo_auton_down		= 0;
const int servo_auton_hold 		= 128;

// These two are how far the wheel pod servos can be off (it's how
// wheel pod alignment is classified).
const int align_far_limit		= 30;
const int align_medium_limit	= 15;

// Transfers the cubes to dump to `dumpCubesTask` (can't pass to tasks).
int f_cubeDumpNum = 4; // MAGIC_NUM: by default, dump all cubes.

tMotor		Motor_Convert(WheelPod motorName);
WheelPod	Motor_Convert(tMotor motorName);
TServoIndex Servo_Convert(WheelPod servoName); // TODO: Doesn't work.
WheelPod	Servo_Convert(TServoIndex servoName); // TODO: Doesn't work.

void initializeRobotVariables();
void resetMotorsServos();

void dumpCubes(int num=4); // MAGIC_NUM.
task dumpCubesTask();



tMotor		Motor_Convert(WheelPod motorName) {
	tMotor conversion;
	switch (motorName) {
		case POD_FL:
			conversion = motor_FL;
			break;
		case POD_FR:
			conversion = motor_FR;
			break;
		case POD_BR:
			conversion = motor_BR;
			break;
		case POD_BL:
			conversion = motor_BL;
			break;
	}
	return conversion;
}
WheelPod	Motor_Convert(tMotor motorName) {
	WheelPod conversion;
	switch (motorName) {
		case motor_FR:
			conversion = POD_FR;
			break;
		case motor_FL:
			conversion = POD_FL;
			break;
		case motor_BL:
			conversion = POD_BL;
			break;
		case motor_BR:
			conversion = POD_BR;
			break;
	}
	return conversion;
}
TServoIndex Servo_Convert(WheelPod servoName) {
	TServoIndex conversion;
	switch (servoName) {
		case POD_FR:
			conversion = servo_FR;
			break;
		case POD_FL:
			conversion = servo_FL;
			break;
		case POD_BL:
			conversion = servo_BL;
			break;
		case POD_BR:
			conversion = servo_BR;
			break;
	}
	return conversion;
}
WheelPod	Servo_Convert(TServoIndex servoName) {
	WheelPod conversion;
	switch (servoName) {
		case servo_FR:
			conversion = POD_FR;
			break;
		case servo_FL:
			conversion = POD_FL;
			break;
		case servo_BL:
			conversion = POD_BL;
			break;
		case servo_BR:
			conversion = POD_BR;
			break;
	}
	return conversion;
}

void initializeRobotVariables()
{
	Motor_ResetEncoder(motor_lift_front);

	// MAGIC_NUM. These can't be set in a loop.
	g_MotorData[POD_FR].angleOffset = 45;
	g_MotorData[POD_FL].angleOffset = 135;
	g_MotorData[POD_BL].angleOffset = -135;
	g_MotorData[POD_BR].angleOffset = -45;

	for (int i=POD_FR; i<(int)POD_NUM; i++) {
		Motor_ResetEncoder(Motor_Convert((WheelPod)i));
		Servo_SetWinch(Servo_Convert((WheelPod)i), 0); // This is legal because the servos aren't CR.

		g_MotorData[i].isReversed = false;
		g_MotorData[i].fineTuneFactor = 1;
		g_MotorData[i].power = 0;
		g_ServoData[i].angle = 0;
		g_ServoData[i].power = 0;
	}

	Servo_SetPosition(servo_dump, servo_dump_closed);
	Servo_SetPosition(servo_flip_L, servo_flip_L_up);
	Servo_SetPosition(servo_flip_R, servo_flip_R_up);
	Servo_SetPosition(servo_climb_L, servo_climb_L_closed);
	Servo_SetPosition(servo_climb_R, servo_climb_R_closed);
	Servo_SetPosition(servo_auton, servo_auton_hold);

	HTIRS2setDSPMode(sensor_IR, g_IRsensorMode);

	//for (int i=0; i<5; i++) {
	//	Time_Wait(1000); // Because the pods might need to move.
	//}
	// TODO: Re-enable the above when swerve drive works.
	for (int i=POD_FR; i<(int)POD_NUM; i++) {
		Motor_ResetEncoder(Motor_Convert((WheelPod)i));
	}

	// MAGIC_NUM: 13V.
	if (externalBatteryAvg<13600) {
		PlaySound(soundDownwardTones);
	}
	// MAGIC_NUM: 8V.
	if (nAvgBatteryLevel<8000) {
		PlaySound(soundDownwardTones);
	}
}
void resetMotorsServos()
{
	Motor_SetPower(0, motor_FR);
	Motor_SetPower(0, motor_FL);
	Motor_SetPower(0, motor_BL);
	Motor_SetPower(0, motor_BR);
	Motor_SetPower(0, motor_lift_front);
	Motor_SetPower(0, motor_lift_back);
	Motor_SetPower(0, motor_sweeper);
	Motor_SetPower(0, motor_flag);
	Motor_SetPower(0, motor_assist_L);
	Motor_SetPower(0, motor_assist_R);
	Servo_SetPower(servo_FR, 0);
	Servo_SetPower(servo_FL, 0);
	Servo_SetPower(servo_BL, 0);
	Servo_SetPower(servo_BR, 0);
	Servo_SetPosition(servo_auton, servo_auton_hold);
	Servo_SetPosition(servo_dump, servo_dump_open);
	Servo_SetPosition(servo_flip_L, servo_flip_L_up);
	Servo_SetPosition(servo_flip_R, servo_flip_R_up);
	Servo_SetPosition(servo_climb_L, servo_climb_L_closed);
	Servo_SetPosition(servo_climb_R, servo_climb_R_closed);
	Servo_SetWinch(servo_FR, 0);
	Servo_SetWinch(servo_FL, 0);
	Servo_SetWinch(servo_BL, 0);
	Servo_SetWinch(servo_BR, 0);
}

void dumpCubes(int num)
{
	f_cubeDumpNum = num;
	Task_Spawn(dumpCubesTask);
}
task dumpCubesTask()
{
	const int short_delay = 160; // MAGIC_NUM. (milliseconds)
	const int long_delay = 1400; // MAGIC_NUM: TODO. (milliseconds)
	Servo_SetPosition(servo_dump, servo_dump_open);
	if (f_cubeDumpNum<4) {
		Time_Wait(short_delay);
	} else {
		Time_Wait(long_delay);
	}
	Servo_SetPosition(servo_dump, servo_dump_closed);
}



#endif // SWERVE_DRIVE
