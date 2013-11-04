#ifndef BUTTONS_C
#define BUTTONS_C
#pragma systemFile
#include "..\Headers\Buttons.h"
// For default values, see above header file.



void Buttons_UpdateData() {
	for (int i=NXT_BUTTON_YES; i<=(int)NXT_BUTTON_ESC; i++) {
		g_NXTButtonsData_prev[i] = g_NXTButtonsData[i];
		g_NXTButtonsData[i] = (nNxtButtonPressed==i);
	}
}
bool Buttons_Pressed(NXTButtons button) {
	bool isTriggered = ((g_NXTButtonsData_prev[button]==false)&&(g_NXTButtonsData[button]==true));
	return isTriggered;
}
bool Buttons_Released(NXTButtons button) {
	bool isTriggered = ((g_NXTButtonsData_prev[button]==true)&&(g_NXTButtonsData[button]==false));
	return isTriggered;
}



#endif // BUTTONS_C
