#ifndef SWERVE_DRIVE
#define SWERVE_DRIVE
#include "includes.h"



typedef enum WheelPod {
	POD_FR = 0,
	POD_FL = 1,
	POD_BL = 2,
	POD_BR = 3,
	POD_NUM, // Should this be hard-coded to be 4?
};

typedef enum Servo {
	SERVO_FR = 0,
	SERVO_FL = 1,
	SERVO_BL = 2,
	SERVO_BR = 3,
	SERVO_NUM,
};

typedef struct motorData {
	bool isReversed;
	int angleOffset;
	float fineTuneFactor;
	int power;
} motorData;

typedef struct servoData {
	int angle;
	int power;
} servoData;

// The mode the IR sensor is set at.
// The competition IR beam/emmiter pulses at 1200 hertz.
const tHTIRS2DSPMode g_IRsensorMode = DSP_1200;

// The threshold for IR values to count as detected.
const int g_IRthreshold = 10; //units?

// This number is just a working guess. Not verified at all.
const float g_EncoderDeadband = 1.0;

// Various structs, initialized in `initializeGlobalVariables()`.
motorData g_MotorData[4]; //4 drive base motors.
servoData g_ServoData[4]; //4 continuous rotation servos.

// Various servo positions.
// MAGIC_NUM: TODO (all).
const int lift_pos_pickup = 0;
const int lift_pos_dump = 6000;
const int lift_pos_top = 6200;
const int servo_funnel_L_open = 40;
const int servo_funnel_L_closed = 180;
const int servo_funnel_R_open = -127;
const int servo_funnel_R_closed = 128;
const int servo_dump_open = 30;
const int servo_dump_closed = 0;
const int servo_flag_L = -127;
const int servo_flag_R = 128;
const int servo_flag_M = 0;

bool f_isWavingFlag = false;
int f_waveNum = 3;
int f_cubeDumpNum = 4;

tMotor	Motor_Convert(Motor motorName);
Motor	Motor_Convert(tMotor motorName);
TServoIndex Servo_Convert(Servo servoName); // TODO: Doesn't work.
Servo	Servo_Convert(TServoIndex servoName); // TODO: Doesn't work.

void initializeRobotVariables();

void dumpCubes(int num=4);
task dumpCubesTask();
void waveFlag(int waveNum=4);
task waveFlagTask();



tMotor	Motor_Convert(Motor motorName) {
	tMotor conversion;
	switch (motorName) {
		case MOTOR_FL:
			conversion = motor_FL;
			break;
		case MOTOR_FR:
			conversion = motor_FR;
			break;
		case MOTOR_BR:
			conversion = motor_BR;
			break;
		case MOTOR_BL:
			conversion = motor_BL;
			break;
	}
	return conversion;
}
Motor	Motor_Convert(tMotor motorName) {
	Motor conversion;
	switch (motorName) {
		case motor_FR:
			conversion = MOTOR_FR;
			break;
		case motor_FL:
			conversion = MOTOR_FL;
			break;
		case motor_BL:
			conversion = MOTOR_BL;
			break;
		case motor_BR:
			conversion = MOTOR_BR;
			break;
	}
	return conversion;
}
TServoIndex Servo_Convert(Servo servoName) {
	TServoIndex conversion;
	switch (servoName) {
		case servo_FR:
			conversion = SERVO_FR;
			break;
		case servo_FL:
			conversion = SERVO_FL;
			break;
		case servo_BL:
			conversion = SERVO_BL;
			break;
		case servo_BR:
			conversion = SERVO_BR;
			break;
	}
	return conversion;
}
Servo	Servo_Convert(TServoIndex servoName) {
	Servo conversion;
	switch (servoName) {
		case SERVO_FR:
			conversion = servo_FR;
			break;
		case SERVO_FL:
			conversion = servo_FL;
			break;
		case SERVO_BL:
			conversion = servo_BL;
			break;
		case SERVO_BR:
			conversion = servo_BR;
			break;
	}
	return conversion;
}
float	Joystick_GetTranslationX() {
	return Math_Limit(
			Math_TrimDeadband((float)Joystick_Joystick(JOYSTICK_R, AXIS_X)), g_FullPower);
}
float	Joystick_GetTranslationY() {
	return Math_Limit(
			Math_TrimDeadband((float)Joystick_Joystick(JOYSTICK_R, AXIS_Y)), g_FullPower);
}
float	Joystick_GetRotationMagnitude() {
	return -Math_Limit(
			Math_TrimDeadband((float)Joystick_Joystick(JOYSTICK_L, AXIS_X)), g_FullPower);
}

void initializeRobotVariables() {
	nMotorEncoder[motor_FR] = 0;
	nMotorEncoder[motor_FL] = 0;
	nMotorEncoder[motor_BL] = 0;
	nMotorEncoder[motor_BR] = 0;

	g_MotorData[MOTOR_FR].angleOffset = 45;
	g_MotorData[MOTOR_FL].angleOffset = 135;
	g_MotorData[MOTOR_BL].angleOffset = -135;
	g_MotorData[MOTOR_BR].angleOffset = -45;

	for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
		g_MotorData[i].isReversed = false;
		g_MotorData[i].fineTuneFactor = 1;
	}

	for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
		g_ServoData[i].angle = 90;	// These should start out facing forward.
		g_ServoData[i].power = 0;
	}
}

void dumpCubes(int num) {
	f_cubeDumpNum = num;
	Task_Spawn(dumpCubesTask);
}

task dumpCubesTask() {
	const int short_delay = 160; // MAGIC_NUM: TODO. (milliseconds)
	const int long_delay = 1600; // MAGIC_NUM: TODO. (milliseconds)
	Servo_SetPosition(servo_dump, servo_dump_open);
	if (f_cubeDumpNum<4) {
		Time_Wait(short_delay);
	} else {
		Time_Wait(long_delay);
	}
	Servo_SetPosition(servo_dump, servo_dump_closed);
}

void waveFlag(int waveNum) {
	f_waveNum = waveNum;
	Task_Spawn(waveFlagTask);
}

task waveFlagTask() {
	f_isWavingFlag = true;

	// MAGIC_NUM: TODO.
	const int delay = 0;

	for (int i=0; i<f_waveNum; i++) {
		Servo_SetPosition(servo_flag, servo_flag_L);
		Time_Wait(delay);
		Servo_SetPosition(servo_flag, servo_flag_R);
		Time_Wait(delay);
	}
	Servo_SetPosition(servo_flag, servo_flag_M);
	Time_Wait(delay);

	f_isWavingFlag = false;
	Task_Kill(waveFlagTask);
}



#endif // SWERVE_DRIVE
