#ifndef JOYSTICK_H
#define JOYSTICK_H
#pragma systemFile



typedef enum JoystickButton {
	BUTTON_A		= 2,
	BUTTON_B		= 3,
	BUTTON_X		= 1,
	BUTTON_Y		= 4,
	BUTTON_LB		= 5,
	BUTTON_RB		= 6,
	BUTTON_LT		= 7,
	BUTTON_RT		= 8,
	BUTTON_BACK		= 9,
	BUTTON_START	= 10,
	BUTTON_JOYL		= 11,
	BUTTON_JOYR		= 12,
	BUTTON_NUM,
} JoystickButton;

typedef enum Controller {
	CONTROLLER_1 = 0,
	CONTROLLER_2 = 1,
	CONTROLLER_NUM,
} Controller;

typedef enum JoystickAxis {
	AXIS_X = 0,
	AXIS_Y = 1,
	AXIS_NUM,
} JoystickAxis;

typedef enum JoystickLR {
	JOYSTICK_L = 0,
	JOYSTICK_R = 1,
	JOYSTICK_NUM,
} JoystickLR;

typedef enum Direction {
	DIRECTION_NONE	= -1,
	DIRECTION_F		= 0,
	DIRECTION_FR	= 1,
	DIRECTION_R		= 2,
	DIRECTION_BR	= 3,
	DIRECTION_B		= 4,
	DIRECTION_BL	= 5,
	DIRECTION_L		= 6,
	DIRECTION_FL	= 7,
	DIRECTION_NUM,
} Direction;

typedef struct joystickData {
	short buttonMap;
	Direction direction;
} joystickData;



// Highest "noise" we've ever recorded is +/-8.
const int g_JoystickDeadband = 10;

// The highest value one axis of a joystick can go to.
const int g_JoystickMax = 128;

// For detecting changes in state of the controllers.
// Initialized in the initialization routine in "initialize.h".
joystickData g_JoystickData[CONTROLLER_NUM]; // 2 controllers.
joystickData g_PrevJoystickData[CONTROLLER_NUM]; // 2 controllers.



#define Joystick_WaitForStart()	(waitForStart())
void	Joystick_UpdateData(); //Actually does a lot of stuff :P
bool	Joystick_Button(JoystickButton button, Controller controller=CONTROLLER_1);
bool	Joystick_ButtonPressed(JoystickButton button, Controller controller=CONTROLLER_1);
bool	Joystick_ButtonReleased(JoystickButton button, Controller controller=CONTROLLER_1);
bool	Joystick_Direction(Direction direction, Controller controller=CONTROLLER_1);
Direction Joystick_Direction(Controller controller=CONTROLLER_1);
bool	Joystick_DirectionPressed(Direction direction, Controller controller=CONTROLLER_1);
bool	Joystick_DirectionReleased(Direction direction, Controller controller=CONTROLLER_1);
int		Joystick_Joystick(	JoystickLR Joystick,	// Capitalized to avoid naming conflict.
							JoystickAxis axis,
							Controller controller=CONTROLLER_1);
float	Joystick_GenericInput(	JoystickLR Joystick,	// Capitalized to avoid naming conflict.
								JoystickAxis axis,
								Controller controller=CONTROLLER_1);



#include "..\Libraries\Joystick.c"
#endif // JOYSTICK_H
