#ifndef INITIALIZE_H
#define INITIALIZE_H
#pragma systemFile
#include "Joystick.h"
#include "Buttons.h"



void initializeGlobalVariables() {
	// Initialize extra copies of joystick data for use in "Joystick.h" (and "Joystick.c").
	for (int i=CONTROLLER_1; i<=(int)CONTROLLER_2; i++) {
		g_JoystickData[i].buttonMap = 0;
		g_JoystickData[i].direction = DIRECTION_NONE;
		g_PrevJoystickData[i].buttonMap = 0;
		g_PrevJoystickData[i].direction = DIRECTION_NONE;
	}

	// Initialize NXT buttons' data for use in "Buttons.h" (and "Buttons.c").
	for (int i=NXT_BUTTON_ESC; i<=(int)NXT_BUTTON_YES; i++) {
		g_NXTButtonsData[i] = false;
		g_NXTButtonsData_prev[i] = false;
	}
}



#endif // INITIALIZE_H
