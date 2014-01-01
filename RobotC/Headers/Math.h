#ifndef MATH_H
#define MATH_H
#pragma systemFile



typedef enum Quadrant {
	QUADRANT_I		= 0,
	QUADRANT_II		= 1,
	QUADRANT_III	= 2,
	QUADRANT_IV		= 3,
	QUADRANT_NUM,
} Quadrant;

typedef enum AngleUnit {
	UNIT_DEG		= 0,
	UNIT_DEGREE		= 0,
	UNIT_RAD		= 1,
	UNIT_RADIAN		= 1,
	UNIT_GRAD		= 2,
	UNIT_GRADIAN	= 2,
} AngleUnit;



// Mathematical facts. (Can a float even hold this many sig figs?)
const float g_pi	= 3.1415926535897932384626433832795; // 32 sig figs.
const float g_e		= 2.7182818284590452353602874713527; // 32 sig figs.
const float g_phi	= 1.6180339887498948482045868343656; // 32 sig figs.

// Highest "noise" we've ever recorded is +/-8.
const int g_JoystickDeadband = 10;



#define Math_Sign(input)  (sgn(input))
int	  Math_Min(int a, int b);
float Math_Min(float a, float b);
int	  Math_Max(int a, int b);
float Math_Max(float a, float b);
int	  Math_Min(int a, int b, int c);
float Math_Min(float a, float b, float c);
int	  Math_Max(int a, int b, int c);
float Math_Max(float a, float b, float c);
int   Math_TrimDeadband(int input, int deadband=g_JoystickDeadband);
float Math_TrimDeadband(float input, float deadband=g_JoystickDeadband);
int   Math_Limit(int input, int max);
float Math_Limit(float input, float max);
float Math_Normalize(float input, float originalMax, float newMax);
float Math_ResponseCurve(float input, float newMax);
float Math_Sin(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Cos(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Tan(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Cot(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Sec(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Csc(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Arcsin(float input, AngleUnit units=UNIT_DEG);
float Math_Arccos(float input, AngleUnit units=UNIT_DEG);
float Math_Arctan(float input, AngleUnit units=UNIT_DEG);
float Math_Arcsin2(float x, float y, AngleUnit units=UNIT_DEG); // TODO: Is this possible?
float Math_Arccos2(float x, float y, AngleUnit units=UNIT_DEG); // TODO: Is this possible?
float Math_Arctan2(float x, float y, AngleUnit units=UNIT_DEG);



#include "..\Libraries\Math.c"
#endif // MATH_H
