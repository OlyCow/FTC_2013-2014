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

// Highest "noise" we've ever recorded is +/-8.
const int g_JoystickDeadband = 10;

// This number is just a working guess. Not verified at all.
const float g_EncoderDeadband = 1.0;

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



#endif // GLOBAL_VARS_H
