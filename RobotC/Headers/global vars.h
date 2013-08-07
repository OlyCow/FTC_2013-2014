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

// The highest value a joystick can go to. Not set to 128 we prefer to
// assign too much power than not assign enough.
const int g_JoystickMax = 127;

// The various motor-regulation speeds.
const int g_FullPower = 100;

// Each task will define a const variable with the same name as the task,
// equal to the value of a call to `nCurrentTask`. The corresponding int
// here should be assigned that value immediately once the tasks starts.
int g_task_main	= -1;
int g_task_PID	= -1;

// Various structs, initialized in `initializeGlobalVariables()`.
motorData g_MotorData[4]; //4 drive base motors.
servoData g_ServoData[4]; //4 continuous rotation servos.
joystickData g_JoystickData[2]; //2 controllers.
joystickData g_PrevJoystickData[2]; //2 controllers.



void initializeGlobalVariables() {
	g_MotorData[MOTOR_FR].angleOffset = 45;
	g_MotorData[MOTOR_FL].angleOffset = 135;
	g_MotorData[MOTOR_BL].angleOffset = 225;
	g_MotorData[MOTOR_BR].angleOffset = 315;

	for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
		g_MotorData[i].isReversed = false;
	}

	for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
		g_ServoData[i].angle = 90;	// These should start out facing forward.
		g_ServoData[i].power = 0;
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
