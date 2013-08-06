#ifndef MATH_H
#define MATH_H
#pragma systemFile
#include "..\Headers\enums.h"



const float pi	= 3.141592653589793238462643383279502884197169399375105820974944592; // 64 sig figs
const float e	= 2.718281828459045235360287471352662497757247093699959574966967628; // 64 sig figs
const float phi	= 1.618033988749894848204586834365638117720309179805762862135448623; // 64 sig figs
int   Math_TrimDeadzone(int input, int deadzone=g_JoystickDeadZone);
float Math_TrimDeadzone(float input, float deadzone=g_JoystickDeadZone);
float Math_ConvertAngle(float input, AngleUnit inputUnit, AngleUnit outputUnit);
int   Math_Normalize(int input, int originalMax, int newMax);
float Math_Normalize(float input, float originalMax, float newMax);
float Math_Sin(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Cos(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Tan(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Cot(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Sec(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Csc(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Arcsin(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Arccos(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Arctan(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Arccot(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Arcsec(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Arccsc(float input, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Arcsin2(float x, float y, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG); //Is this possible?
float Math_Arccos2(float x, float y, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG); //Is this possible?
float Math_Arctan2(float x, float y, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Arccot2(float x, float y, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG);
float Math_Arcsec2(float x, float y, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG); //Is this possible?
float Math_Arccsc2(float x, float y, AngleUnit inputUnit=UNIT_DEG, AngleUnit outputUnit=UNIT_DEG); //Is this possible?
float Math_DegToRad(float input);
float Math_DegToGrad(float input);
float Math_RadToDeg(float input);
float Math_RadToGrad(float input);
float Math_GradToDeg(float input);
float Math_GradToRad(float input);



#include "..\Libraries\Math.c"
#endif // MATH_H
