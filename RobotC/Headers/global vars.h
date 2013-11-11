#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H
#pragma systemFile
#include "..\Headers\enums.h"
#include "..\Headers\structs.h"



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
// MAGIC_NUM: TODO.
const int servo_dump_open = 0;
const int servo_dump_closed = 0;
const int servo_flag_L = 0;
const int servo_flag_R = 0;
const int servo_flag_M = 0;



#endif // GLOBAL_VARS_H
