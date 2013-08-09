#ifndef MATH_C
#define MATH_C
#pragma systemFile
#include "..\Headers\Math.h"
// For default values, see above header file.



int   Math_TrimDeadzone(int input, int deadzone) {
	int output = 0;
	if (abs(input)>deadzone==true) {
		output = input;
	}
	return output;
}
float Math_TrimDeadzone(float input, float deadzone) {
	float output = 0;
	if (abs(input)>deadzone==true) {
		output = input;
	}
	return output;
}
int   Math_Limit(int input, int max) {
	int output = input;
	if (input>max) {
		output = max;
	} else if (input<(-max)) {
		output = (-max);
	}
	return output;
}
float Math_Limit(float input, float max) {
	float output = input;
	if (input>max) {
		output = max;
	} else if (input<(-max)) {
		output = (-max);
	}
	return output;
}
float Math_ConvertAngle(float input, AngleUnit inputUnit, AngleUnit outputUnit) {
	float output = 0;
	switch (inputUnit) {
		case UNIT_DEG: // Not considering UNIT_DEGREE because it is the same "case"
			switch (inputUnit) {
				case UNIT_RAD:
					output = input*pi/180;
					break;
				case UNIT_GRAD:
					output = input*10/9;
					break;
				default:
					break;
			}
			break;
		case UNIT_RAD:
			switch (inputUnit) {
				case UNIT_DEG:
					output = input*180/pi;
					break;
				case UNIT_GRAD:
					output = input*200/pi;
					break;
				default:
					break;
			}
			break;
		case UNIT_GRAD:
			switch (inputUnit) {
				case UNIT_DEG:
					output = input*9/10;
					break;
				case UNIT_RAD:
					output = input*pi/200;
					break;
				default:
					break;
			}
			break;
	}
	return output;
}
int   Math_Normalize(int input, int originalMax, int newMax) {
	return input*newMax/originalMax; // intentional int division
}
float Math_Normalize(float input, float originalMax, float newMax) {
	return input*newMax/originalMax;
}
float Math_Sin(float input, AngleUnit inputUnit, AngleUnit outputUnit) {
}
float Math_Cos(float input, AngleUnit inputUnit, AngleUnit outputUnit) {
}
float Math_Tan(float input, AngleUnit inputUnit, AngleUnit outputUnit) {
}
float Math_Cot(float input, AngleUnit inputUnit, AngleUnit outputUnit) {
}
float Math_Sec(float input, AngleUnit inputUnit, AngleUnit outputUnit) {
}
float Math_Csc(float input, AngleUnit inputUnit, AngleUnit outputUnit) {
}
float Math_Arcsin(float input, AngleUnit units) {
}
float Math_Arccos(float input, AngleUnit units) {
}
float Math_Arctan(float input, AngleUnit units) {
}
float Math_Arccot(float input, AngleUnit units) {
}
float Math_Arcsec(float input, AngleUnit units) {
}
float Math_Arccsc(float input, AngleUnit units) {
}
float Math_Arcsin2(float x, float y, AngleUnit units) {
}
float Math_Arccos2(float x, float y, AngleUnit units) {
}
float Math_Arctan2(float x, float y, AngleUnit units) {
}
float Math_Arccot2(float x, float y, AngleUnit units) {
}
float Math_Arcsec2(float x, float y, AngleUnit units) {
}
float Math_Arccsc2(float x, float y, AngleUnit units) {
}
float Math_DegToRad(float input) {
	return input*pi/180;
}
float Math_DegToGrad(float input) {
	return input*10/9;
}
float Math_RadToDeg(float input) {
	return input*180/pi;
}
float Math_RadToGrad(float input) {
	return input*200/pi;
}
float Math_GradToDeg(float input) {
	return input*9/10;
}
float Math_GradToRad(float input) {
	return input*pi/200;
}



#endif // MATH_C
