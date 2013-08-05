#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H
#pragma systemFile
#include "..\Headers\enums.h"



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

// The various motor-regulation speeds.
const int g_FullPower = 100;

// Each task will define a const variable with the same name as the task,
// equal to the value of a call to `nCurrentTask`, to refer to an element
// of this struct. `Task_Spawn()` and other "Task.h" functions will then
// use `g_taskIndices[TASK_FOO_CONST]` to refer to that task.
const int g_taskIndices[20] =
	{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19}; // There can only be 20 tasks, including `main()`.
	// Declared here instead of with a for loop because these should be const.

// Various structs, initialized in `initializeGlobalVariables()`.
motorData g_MotorData[4]; //4 drive base motors.
joystickData g_JoystickData[2]; //2 controllers.
joystickData g_PrevJoystickData[2]; //2 controllers.



void initializeGlobalVariables() {
	g_MotorData[MOTOR_FR].angleOffset = 45;
	g_MotorData[MOTOR_FL].angleOffset = 135;
	g_MotorData[MOTOR_BL].angleOffset = 225;
	g_MotorData[MOTOR_BR].angleOffset = 315;

	//`<=` used here because we want to evaluate CONTROLLER_2 as well.
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
