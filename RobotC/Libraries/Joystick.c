#ifndef JOYSTICK_C
#define JOYSTICK_C
#pragma systemFile
#include "..\Headers\Joystick.h"
// For default values, see above header file.



void Joystick_UpdateData() {
	//"joystick" is a magic number (struct?) that RobotC forces upon us.
	getJoystickSettings(joystick);
	for (int i=CONTROLLER_1; i<(int)CONTROLLER_NUM; i++) {
		g_PrevJoystickData[i].buttonMap = g_JoystickData[i].buttonMap;
		g_PrevJoystickData[i].direction = g_JoystickData[i].direction;
	}
	g_JoystickData[CONTROLLER_1].buttonMap = joystick.joy1_Buttons;
	g_JoystickData[CONTROLLER_2].buttonMap = joystick.joy2_Buttons;
	g_JoystickData[CONTROLLER_1].direction = (Direction)joystick.joy1_TopHat;
	g_JoystickData[CONTROLLER_2].direction = (Direction)joystick.joy2_TopHat;
}
bool Joystick_Button(JoystickButton button, Controller controller) {
	//"false" is the default return value.
	bool isPressed = false;
	switch (controller) {
		case CONTROLLER_1 :
			isPressed = (bool)(joy1Btn(button));
			break;
		case CONTROLLER_2 :
			isPressed = (bool)(joy2Btn(button));
			break;
	}
	return isPressed;
}
bool Joystick_ButtonPressed(JoystickButton button, Controller controller) {
	bool wasPressed = (((g_PrevJoystickData[controller].buttonMap)&(1<<(button-1)))!=0);
	bool isPressed = (((g_JoystickData[controller].buttonMap)&(1<<(button-1)))!=0);
	bool isTriggered = (wasPressed==false)&&(isPressed==true);
	return isTriggered;
}
bool Joystick_ButtonReleased(JoystickButton button, Controller controller) {
	bool wasPressed = (((g_PrevJoystickData[controller].buttonMap)&(1<<(button-1)))!=0);
	bool isPressed = (((g_JoystickData[controller].buttonMap)&(1<<(button-1)))!=0);
	bool isTriggered = (wasPressed==true)&&(isPressed==false);
	return isTriggered;
}
bool Joystick_Direction(Direction direction, Controller controller) {
	bool isPressed = false;
	switch (controller) {
		case CONTROLLER_1 :
			if (joystick.joy1_TopHat==direction) {
				isPressed = true;
			}
			break;
		case CONTROLLER_2 :
			if (joystick.joy2_TopHat==direction) {
				isPressed = true;
			}
			break;
	}
	return isPressed;
}
Direction Joystick_Direction(Controller controller) {
	Direction direction = DIRECTION_NONE;
	switch (controller) {
		case CONTROLLER_1 :
			direction = (Direction)joystick.joy1_TopHat;
			break;
		case CONTROLLER_2 :
			direction = (Direction)joystick.joy2_TopHat;
			break;
	}
	return direction;
}
bool Joystick_DirectionPressed(Direction direction, Controller controller) {
	bool wasPressed = (direction == g_PrevJoystickData[controller].direction);
	bool isPressed = (direction == g_JoystickData[controller].direction);
	bool isTriggered = (wasPressed==false)&&(isPressed==true);
	return isTriggered;
}
bool Joystick_DirectionReleased(Direction direction, Controller controller) {
	bool wasPressed = (direction == g_PrevJoystickData[controller].direction);
	bool isPressed = (direction == g_JoystickData[controller].direction);
	bool isTriggered = (wasPressed==true)&&(isPressed==false);
	return isTriggered;
}
// Takes an input of `Joystick` instead of `joystick` to
// avoid conflict with name of built-in struct `joystick`.
int  Joystick_Joystick(	JoystickLR Joystick,
						JoystickAxis axis,
						Controller controller)
{
	int axisValue = 0;
	switch (controller) {
		case CONTROLLER_1 :
			switch (Joystick) {
				case JOYSTICK_L :
					switch (axis) {
						case AXIS_X :	//controller 1, joystick L, X-axis
							axisValue = joystick.joy1_x1;
							break;
						case AXIS_Y :	//controller 1, joystick L, Y-axis
							axisValue = joystick.joy1_y1;
							break;
					}
					break;
				case JOYSTICK_R :
					switch (axis) {
						case AXIS_X :	//controller 1, joystick R, X-axis
							axisValue = joystick.joy1_x2;
							break;
						case AXIS_Y :	//controller 1, joystick R, Y-axis
							axisValue = joystick.joy1_y2;
							break;
					}
					break;
			}
			break;
		case CONTROLLER_2 :
			switch (Joystick) {
				case JOYSTICK_L :
					switch (axis) {
						case AXIS_X :	//controller 2, joystick L, X-axis
							axisValue = joystick.joy2_x1;
							break;
						case AXIS_Y :	//controller 2, joystick L, Y-axis
							axisValue = joystick.joy2_y1;
							break;
					}
					break;
				case JOYSTICK_R :
					switch (axis) {
						case AXIS_X :	//controller 2, joystick R, X-axis
							axisValue = joystick.joy2_x2;
							break;
						case AXIS_Y :	//controller 2, joystick R, Y-axis
							axisValue = joystick.joy2_y2;
							break;
					}
					break;
			}
			break;
	}
	return axisValue;
}
// Takes an input of `Joystick` instead of `joystick` to
// avoid conflict with name of built-in struct `joystick`.
float	Joystick_GenericInput(	JoystickLR Joystick,
								JoystickAxis axis,
								Controller controller)
{
	float return_value;
	return_value = Joystick_Joystick(Joystick, axis, controller);
	return_value = Math_TrimDeadband(return_value);
	return_value = Math_Normalize(return_value, g_JoystickMax, g_FullPower);
	return_value = Math_ResponseCurve(return_value, g_FullPower);
	return return_value;
}



#endif // JOYSTICK_C
