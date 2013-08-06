#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H
#pragma systemFile
#include "..\Headers\enums.h"
#include "..\Headers\structs.h"



// The number of internal timers available to use.
const int g_TimerNumber = 4; //defined internally as macro `kNumbOfTimers`
// The mode the IR sensor is set at.
// The competition IR beam/emmiter pulses at 1200 hertz.
const tHTIRS2DSPMode g_IRsensorMode = DSP_1200;

// The threshold for IR values to count as detected.
const int g_IRthreshold = 10; //units?

// The deadzone for joysticks (eliminates humming).
// The highest we've ever recorded is +/-8.
const int g_JoystickDeadZone = 10;

// The factor to divide by when fine-tuning motors.
const int g_FineTuneFactor = 4;

// The various motor-regulation speeds.
const int g_FullPower = 100;
const int g_FullRegulatedPower = 80;

motorData g_MotorData[4]; //4 drive base motors.
joystickData g_JoystickData[2]; //2 controllers.
joystickData g_PrevJoystickData[2]; //2 controllers.



void initializeGlobalVariables() {
	g_MotorData[MOTOR_FR].angleOffset = 45;
	g_MotorData[MOTOR_FL].angleOffset = 135;
	g_MotorData[MOTOR_BL].angleOffset = 225;
	g_MotorData[MOTOR_BR].angleOffset = 315;

	for (int i=MOTOR_FR; i<=MOTOR_BR; i++) {
		g_MotorData[i].isReversed = false;
	}

	for (int i=CONTROLLER_1; i<=(int)CONTROLLER_2; i++) {
		g_JoystickData[i].buttonMap = 0;
		g_JoystickData[i].direction = DIRECTION_NONE;
	}
	for (int i=CONTROLLER_1; i<=(int)CONTROLLER_2; i++) {
		g_PrevJoystickData[i].buttonMap = 0;
		g_PrevJoystickData[i].direction = DIRECTION_NONE;
	}
}



#endif // GLOBAL_VARS_H
