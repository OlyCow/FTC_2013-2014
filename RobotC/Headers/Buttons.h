#ifndef BUTTONS_H
#define BUTTONS_H
#pragma systemFile



// NOT TESTED YET
typedef enum NXTButtons {
	NXT_BUTTON_YES	= 0,
	NXT_BUTTON_L	= 1,
	NXT_BUTTON_R	= 2,
	NXT_BUTTON_ESC	= 3,
	NXT_BUTTON_NUM,
};



// For detecting changes in the state of the NXT buttons.
// Initialized in the initialization routine ("initialize.h").
bool g_NXTButtonsData[NXT_BUTTON_NUM];
bool g_NXTButtonsData_prev[NXT_BUTTON_NUM];



void	Buttons_UpdateData();
#define	Buttons_GetState(button)	(g_NXTButtonsData[button]) //button is type NXTButtons; reads from the gNXTButtonsData array.
bool	Buttons_Pressed(NXTButtons button);
bool	Buttons_Released(NXTButtons button);



#include "..\Libraries\Buttons.c"
#endif // BUTTONS_H
