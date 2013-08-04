#ifndef JOYSTICK_H
#define JOYSTICK_H
#pragma systemFile
#include "..\Headers\enums.h"
#include "..\Headers\structs.h"
#include "..\Headers\global vars.h"



void Joystick_UpdateData(); //Actually does a lot of stuff :P
bool Joystick_ButtonPressed(JoystickButton button, Controller controller=CONTROLLER_1);
bool Joystick_ButtonReleased(JoystickButton button, Controller controller=CONTROLLER_1);
bool Joystick_DirectionPressed(Direction direction, Controller controller=CONTROLLER_1);
bool Joystick_DirectionReleased(Direction direction, Controller controller=CONTROLLER_1);
bool Joystick_Direction(Direction direction, Controller controller=CONTROLLER_1);
bool Joystick_Button(JoystickButton button, Controller controller=CONTROLLER_1);
Direction Joystick_Direction(Controller controller=CONTROLLER_1);
int  Joystick_Joystick(	JoystickLR Joystick,	//Still a relatively good line of code
						JoystickAxis axis,
						Controller controller=CONTROLLER_1);



// Joystick_UpdateData
// Joystick_ButtonPressed
// Joystick_ButtonReleased
// Joystick_DirectionPressed
// Joystick_DirectionReleased
// Joystick_Direction
// Joystick_Button
// TESTED:PASS>>	Joystick_Direction
// TESTED:PASS>>	Joystick_Joystick
#include "..\Libraries\Joystick.c"
#endif // JOYSTICK_H
