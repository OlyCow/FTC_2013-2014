#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H
#pragma systemFile
#include "..\Headers\includes.h"



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

motorData g_motorData[4];



void initializeVariables()
{
	g_motorData[MOTOR_FR].angleOffset = 45;
	g_motorData[MOTOR_FR].power = 0;

	g_motorData[MOTOR_FL].angleOffset = 135;
	g_motorData[MOTOR_FL].power = 0;

	g_motorData[MOTOR_BL].angleOffset = 225;
	g_motorData[MOTOR_BL].power = 0;

	g_motorData[MOTOR_BR].angleOffset = 315;
	g_motorData[MOTOR_BR].power = 0;
}



#endif // GLOBAL_VARS_H
