#ifndef JOYSTICK_H
#define JOYSTICK_H
#pragma systemFile
#include "..\Libraries\Joystick.c"



void Joystick_UpdateData();
bool Joystick_Button(JoystickButton button, Controller controller = CONTROLLER_1);
Direction Joystick_Direction(Controller controller = CONTROLLER_1);
// Takes an input of "Joystick" instead of "joystick" to
// avoid conflict with name of built-in struct "joystick";
int  Joystick_Joystick(	JoystickLR Joystick,	//Still a relatively good line of code
						JoystickAxis axis,
						Controller controller = CONTROLLER_1);



#endif // JOYSTICK_H
