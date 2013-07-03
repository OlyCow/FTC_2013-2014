#ifndef JOYSTICK_C
#define JOYSTICK_C
#pragma systemFile
#include "..\Headers\Joystick.h"



void Joystick_UpdateData()
{
	//"joystick" is a magic number (struct?) that RobotC forces upon us.
	getJoystickSettings(joystick);
}
bool Joystick_Button(JoystickButton button, Controller controller = CONTROLLER_1)
{
	//"false" is the default return value.
	bool isPressed = false;
	switch (controller)
	{
		case CONTROLLER_1:
			isPressed = (bool)(joy1Btn(button));
			break;
		case CONTROLLER_2:
			isPressed = (bool)(joy2Btn(button));
			break;
	}
	return isPressed;
}
Direction Joystick_Direction(Controller controller = CONTROLLER_1)
{
	Direction direction = DIRECTION_NONE;
	switch (controller)
	{
		case CONTROLLER_1:
			direction = (Direction)joystick.joy1_TopHat;
			break;
		case CONTROLLER_2:
			direction = (Direction)joystick.joy2_TopHat;
			break;
	}
	return direction;
}
// Takes an input of "Joystick" instead of "joystick" to
// avoid conflict with name of built-in struct "joystick";
int Joystick_Joystick(	JoystickLR Joystick,	//Still a relatively good line of code
						JoystickAxis axis,
						Controller controller = CONTROLLER_1)
{
	int axisValue = 0;
	switch (controller)
	{
		case CONTROLLER_1:
			switch (Joystick)
			{
				case JOYSTICK_L:
					switch (axis)
					{
						case AXIS_X:	//controller 1, joystick L, X-axis
							axisValue = joystick.joy1_x1;
							break;
						case AXIS_Y:	//controller 1, joystick L, Y-axis
							axisValue = joystick.joy1_y1;
							break;
					}
					break;
				case JOYSTICK_R:
					switch (axis)
					{
						case AXIS_X:	//controller 1, joystick L, X-axis
							axisValue = joystick.joy1_x2;
							break;
						case AXIS_Y:	//controller 1, joystick L, Y-axis
							axisValue = joystick.joy1_y2;
							break;
					}
					break;
			}
			break;
		case CONTROLLER_2:
			switch (Joystick)
			{
				case JOYSTICK_L:
					switch (axis)
					{
						case AXIS_X:	//controller 2, joystick L, X-axis
							axisValue = joystick.joy2_x1;
							break;
						case AXIS_Y:	//controller 2, joystick L, Y-axis
							axisValue = joystick.joy2_y1;
							break;
					}
					break;
				case JOYSTICK_R:
					switch (axis)
					{
						case AXIS_X:	//controller 2, joystick L, X-axis
							axisValue = joystick.joy2_x2;
							break;
						case AXIS_Y:	//controller 2, joystick L, Y-axis
							axisValue = joystick.joy2_y2;
							break;
					}
					break;
			}
			break;
	}
	return axisValue;
}



#endif // JOYSTICK_C
