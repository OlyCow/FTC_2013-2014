#ifndef ENUMS_H
#define ENUMS_H
#pragma systemFile



typedef enum Motor {
	MOTOR_FR = 0,
	MOTOR_FL = 1,
	MOTOR_BL = 2,
	MOTOR_BR = 3,
};

typedef enum MotorType {
	MOTORTYPE_NXT	= 0,
	MOTORTYPE_12V	= 1,
	MOTORTYPE_ALL	= 2,
};

//NOT final! (not sure if using joy1Btn() or not)
typedef enum JoystickButton {
	BUTTON_A = 2,
	BUTTON_B = 3,
	BUTTON_X = 1,
	BUTTON_Y = 4,
	BUTTON_LB = 5,
	BUTTON_RB = 6,
	BUTTON_LT = 7,
	BUTTON_RT = 8,
	BUTTON_BACK = 9,
	BUTTON_START = 10,
	BUTTON_JOYL = 11,	//button on the joystick
	BUTTON_JOYR = 12,	//button on the joystick
};

typedef enum Controller {
	CONTROLLER_1 = 1,
	CONTROLLER_2 = 2,
};

typedef enum JoystickAxis {
	AXIS_X = 0,
	AXIS_Y = 1,
};

typedef enum JoystickLR {
	JOYSTICK_L = 0,
	JOYSTICK_R = 1,
};

// This still needs a definitive test.
typedef enum Direction {
	DIRECTION_NONE = -1,
	DIRECTION_F = 0,
	DIRECTION_FR = 1,
	DIRECTION_R = 2,
	DIRECTION_BR = 3,
	DIRECTION_B = 4,
	DIRECTION_BL = 5,
	DIRECTION_L = 6,
	DIRECTION_FL = 7,
};

// These frequencies are all in hertz, rounded to the nearest int.
typedef enum SoundNotes {
	NOTE_REST = 0,

	NOTE_C3 = 131,	NOTE_C4 = 262,	NOTE_C5 = 523,
	NOTE_D3 = 147,	NOTE_D4 = 294,	NOTE_D5 = 587,
	NOTE_E3 = 165,	NOTE_E4 = 330,	NOTE_E5 = 659,
	NOTE_F3 = 175,	NOTE_F4 = 349,	NOTE_F5 = 698,
	NOTE_G3 = 196,	NOTE_G4 = 392,	NOTE_G5 = 784,
	NOTE_A3 = 220,	NOTE_A4 = 440,	NOTE_A5 = 880,
	NOTE_B3 = 247,	NOTE_B4 = 494,	NOTE_B5 = 988,

	NOTE_C3s = 139,	NOTE_C4s = 277,	NOTE_C5s = 554,
	NOTE_D3s = 156,	NOTE_D4s = 311,	NOTE_D5s = 622,
	NOTE_E3s = 175,	NOTE_E4s = 349,	NOTE_E5s = 698,
	NOTE_F3s = 185,	NOTE_F4s = 370,	NOTE_F5s = 740,
	NOTE_G3s = 208,	NOTE_G4s = 415,	NOTE_G5s = 831,
	NOTE_A3s = 233,	NOTE_A4s = 466,	NOTE_A5s = 932,
	NOTE_B3s = 262,	NOTE_B4s = 523,	NOTE_B5s = 1047,

	NOTE_C3f = 123,	NOTE_C4f = 247,	NOTE_C5f = 494,
	NOTE_D3f = 139,	NOTE_D4f = 277,	NOTE_D5f = 554,
	NOTE_E3f = 156,	NOTE_E4f = 311,	NOTE_E5f = 622,
	NOTE_F3f = 165,	NOTE_F4f = 330,	NOTE_F5f = 659,
	NOTE_G3f = 185,	NOTE_G4f = 370,	NOTE_G5f = 740,
	NOTE_A3f = 208,	NOTE_A4f = 415,	NOTE_A5f = 831,
	NOTE_B3f = 233,	NOTE_B4f = 466,	NOTE_B5f = 932,
};



#endif // ENUMS_H
