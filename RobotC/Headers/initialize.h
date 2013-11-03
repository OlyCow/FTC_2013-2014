#ifndef INITIALIZE_H
#define INITIALIZE_H
#pragma systemFile
#include "..\includes.h"
#include "..\Headers\enums.h"
#include "..\Headers\structs.h"



void initializeGlobalVariables() {
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

	// Initialize extra copies of joystick data for use in "Joystick.h" (and "Joystick.c").
	for (int i=CONTROLLER_1; i<=(int)CONTROLLER_2; i++) {
		g_JoystickData[i].buttonMap = 0;
		g_JoystickData[i].direction = DIRECTION_NONE;
	}
	for (int i=CONTROLLER_1; i<=(int)CONTROLLER_2; i++) {
		g_PrevJoystickData[i].buttonMap = 0;
		g_PrevJoystickData[i].direction = DIRECTION_NONE;
	}
}



#endif // INITIALIZE_H
