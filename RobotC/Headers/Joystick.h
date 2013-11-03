#ifndef JOYSTICK_H
#define JOYSTICK_H
#pragma systemFile
#include "..\Headers\enums.h"
#include "..\Headers\global vars.h"



typedef struct joystickData {
	short buttonMap;
	Direction direction;
} joystickData;



joystickData g_JoystickData[2]; //2 controllers.
joystickData g_PrevJoystickData[2]; //2 controllers.



#define Joystick_WaitForStart()	waitForStart()
void	Joystick_UpdateData(); //Actually does a lot of stuff :P
bool	Joystick_ButtonPressed(JoystickButton button, Controller controller=CONTROLLER_1);
bool	Joystick_ButtonReleased(JoystickButton button, Controller controller=CONTROLLER_1);
bool	Joystick_DirectionPressed(Direction direction, Controller controller=CONTROLLER_1);
bool	Joystick_DirectionReleased(Direction direction, Controller controller=CONTROLLER_1);
bool	Joystick_Direction(Direction direction, Controller controller=CONTROLLER_1);
bool	Joystick_Button(JoystickButton button, Controller controller=CONTROLLER_1);
Direction Joystick_Direction(Controller controller=CONTROLLER_1);
int		Joystick_Joystick(	JoystickLR Joystick,	//Still a relatively good line of code
							JoystickAxis axis,
							Controller controller=CONTROLLER_1);



#include "..\Libraries\Joystick.c"
#endif // JOYSTICK_H
