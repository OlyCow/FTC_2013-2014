#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

#pragma systemFile



// The mode the IR sensor is set at.
const tHTIRS2DSPMode g_IRsensorMode = DSP_1200;

// The threshold for IR values to count as detected.
const int g_IRthreshold = 10;

// The deadzone for joysticks (eliminates humming).
// The highest we've ever recorded is +/-8.
const int g_JoystickDeadZone = 10;

// The factor to divide by when fine-tuning motors.
const int g_FineTuneFactor = 4;

// The various motor-regulation speeds.
const int g_FullPower = 100;
const int g_FullRegulatedPower = 80;




//// We might not use masking at all to increase spped--we're not sure.
//
//// (CONTROLLER_1):
//// Mask for the "bitmap" values from the controller for everything
//// we don't need (other than A/B/X/Y/LT/RT/JOYL/JOYR):
//	//   0000 1100 1100 1111
//	// & ???? ???? ???? ????
//	// ---------------------
//	//   0000 ??00 ??00 ????
//// This information was gleaned from the definition of joy1Btn().
//	//   2^0 + 2^1 + 2^2 + 2^3 + 2^6 + 2^7 + 2^10 + 2^11
//	// = 1 + 2 + 4 + 8 + 64 + 128 + 1024 + 2048
//	// = 3279
//// If it doesn't work, use the commented-out line:
////const short g_ControllerMask = 6559;
//const short g_ControllerMaskA = 3279;	//CONTROLLER_1
//
//// (CONTROLLER_2):
//// Masks for the "bitmap" values from the controller for everything
//// we don't need (other than A/B/X/Y):
//	//   0000 0000 0000 1111
//	// & ???? ???? ???? ????
//	// ---------------------
//	//   0000 0000 0000 ????
//// This information was gleaned from the definition of joy1Btn().
//	//   2^0 + 2^1 + 2^2 + 2^3
//	// = 1 + 2 + 4 + 8
//	// = 15
//// If it doesn't work, use the commented-out line:
////const short g_ControllerMask = 31;
//const short g_ControllerMaskB = 15;	//CONTROLLER_2



#endif
