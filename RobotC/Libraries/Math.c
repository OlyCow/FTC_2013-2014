#ifndef MATH_C
#define MATH_C
#pragma systemFile
#include "..\Headers\Math.h"
// For default values, see above header file.



int	  Math_Min(int a, int b) {
	int return_value = a;
	if (a>b) {
		return_value = b;
	}
	return return_value;
}
float Math_Min(float a, float b) {
	float return_value = a;
	if (a>b) {
		return_value = b;
	}
	return return_value;
}
int	  Math_Max(int a, int b) {
	int return_value = a;
	if (a<b) {
		return_value = b;
	}
	return return_value;
}
float Math_Max(float a, float b) {
	float return_value = a;
	if (a<b) {
		return_value = b;
	}
	return return_value;
}
int	  Math_Min(int a, int b, int c) {
	int return_value = a;
	if (a>b) {
		return_value = b;
	}
	if (return_value>c) {
		return_value = c;
	}
	return return_value;
}
float Math_Min(float a, float b, float c) {
	float return_value = a;
	if (a>b) {
		return_value = b;
	}
	if (return_value>c) {
		return_value = c;
	}
	return return_value;
}
int	  Math_Max(int a, int b, int c) {
	int return_value = a;
	if (a<b) {
		return_value = b;
	}
	if (return_value<c) {
		return_value = c;
	}
	return return_value;
}
float Math_Max(float a, float b, float c) {
	float return_value = a;
	if (a<b) {
		return_value = b;
	}
	if (return_value<c) {
		return_value = c;
	}
	return return_value;
}
int   Math_TrimDeadband(int input, int deadband) {
	int output = 0;
	if (abs(input)>deadband==true) {
		output = input;
	}
	return output;
}
float Math_TrimDeadband(float input, float deadband) {
	float output = 0;
	if (abs(input)>deadband==true) {
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
float Math_Normalize(float input, float originalMax, float newMax) {
	return input*newMax/originalMax;
}
float Math_ResponseCurve(float input, float newMax) {
	float return_value = input*input;
	return_value /= newMax; // Because math.
	return_value *= Math_Sign(input);
	return return_value;
}
float Math_Sin(float input, AngleUnit inputUnit, AngleUnit outputUnit) {
	float return_value = 0.0;
	if ((inputUnit==UNIT_DEG)&&(outputUnit==UNIT_DEG)) {
		return_value = sinDegrees(input);
	} else {
		float temp = input;
		switch (inputUnit) {
			// In case of UNIT_DEG, we've already set `temp` to `input`.
			case UNIT_RAD :
				temp = Math_Normalize(input, g_pi, 180.0);
				break;
			case UNIT_GRAD :
				// 100 grad = 90 deg. Simplified for more acuracy.
				temp = Math_Normalize(input, 10.0, 9.0);
				break;
		}
		temp = sinDegrees(temp);
		switch (outputUnit) {
			// In case of UNIT_DEG, `temp` is already converted.
			case UNIT_RAD :
				temp = Math_Normalize(input, 180.0, g_pi);
				break;
			case UNIT_GRAD :
				// 90 deg = 100 grad. Simplified for more acuracy.
				temp = Math_Normalize(input, 9.0, 10.0);
				break;
		}
		return_value = temp;
	}
	return return_value;
}
float Math_Cos(float input, AngleUnit inputUnit, AngleUnit outputUnit) {
	float return_value = 0.0;
	if ((inputUnit==UNIT_DEG)&&(outputUnit==UNIT_DEG)) {
		return_value = cosDegrees(input);
	} else {
		float temp = input;
		switch (inputUnit) {
			// In case of UNIT_DEG, we've already set `temp` to `input`.
			case UNIT_RAD :
				temp = Math_Normalize(input, g_pi, 180.0);
				break;
			case UNIT_GRAD :
				// 100 grad = 90 deg. Simplified for more acuracy.
				temp = Math_Normalize(input, 10.0, 9.0);
				break;
		}
		temp = cosDegrees(temp);
		switch (outputUnit) {
			// In case of UNIT_DEG, `temp` is already converted.
			case UNIT_RAD :
				temp = Math_Normalize(input, 180.0, g_pi);
				break;
			case UNIT_GRAD :
				// 90 deg = 100 grad. Simplified for more acuracy.
				temp = Math_Normalize(input, 9.0, 10.0);
				break;
		}
		return_value = temp;
	}
	return return_value;
}
float Math_Tan(float input, AngleUnit inputUnit, AngleUnit outputUnit) {
	// TODO: Implement this more efficiently (approximation? CORDIC?).
	float sin_value = 0.0;
	float cos_value = 0.0;
	sin_value = Math_Sin(input, inputUnit, outputUnit);
	cos_value = Math_Cos(input, inputUnit, outputUnit);
	return (sin_value/cos_value);
}
float Math_Cot(float input, AngleUnit inputUnit, AngleUnit outputUnit) {
	// TODO: Implement this more efficiently (approximation? CORDIC?).
	float sin_value = 0.0;
	float cos_value = 0.0;
	sin_value = Math_Sin(input, inputUnit, outputUnit);
	cos_value = Math_Cos(input, inputUnit, outputUnit);
	return (cos_value/sin_value);
}
float Math_Sec(float input, AngleUnit inputUnit, AngleUnit outputUnit) {
	// TODO: Implement this more efficiently (approximation? CORDIC?).
	float cos_value = Math_Cos(input, inputUnit, outputUnit);
	return (1.0/cos_value);
}
float Math_Csc(float input, AngleUnit inputUnit, AngleUnit outputUnit) {
	// TODO: Implement this more efficiently (approximation? CORDIC?).
	float sin_value = Math_Sin(input, inputUnit, outputUnit);
	return (1.0/sin_value);
}
float Math_Arcsin(float input, AngleUnit units) {
	float return_value = asin(input);
	switch (units) {
		case UNIT_DEG :
			return_value = Math_Normalize(return_value, g_pi, 180.0);
			break;
		// If `units` is radians, we're already done.
		case UNIT_GRAD :
			return_value = Math_Normalize(return_value, g_pi, 200.0);
			break;
	}
	return return_value;
}
float Math_Arccos(float input, AngleUnit units) {
	float return_value = acos(input);
	switch (units) {
		case UNIT_DEG :
			return_value = Math_Normalize(return_value, g_pi, 180.0);
			break;
		// If `units` is radians, we're already done.
		case UNIT_GRAD :
			return_value = Math_Normalize(return_value, g_pi, 200.0);
			break;
	}
	return return_value;
}
float Math_Arctan(float input, AngleUnit units) {
	// TODO: Implement this more efficiently (approximation?) if possible.
	float return_value = atan2(input, 1);
	switch (units) {
		case UNIT_DEG :
			return_value = Math_Normalize(return_value, g_pi, 180.0);
			break;
		// If `units` is radians, we're already done.
		case UNIT_GRAD :
			return_value = Math_Normalize(return_value, g_pi, 200.0);
			break;
	}
	return return_value;
}
float Math_Arcsin2(float x, float y, AngleUnit units) {
	// TODO: Implement this. If it's even possible.
}
float Math_Arccos2(float x, float y, AngleUnit units) {
	// TODO: Implement this. If it's even possible.
}
float Math_Arctan2(float x, float y, AngleUnit units) {
	float return_value = atan2(y, x);
	switch (units) {
		case UNIT_DEG :
			return_value = Math_Normalize(return_value, g_pi, 180.0);
			break;
		// If `units` is radians, we're already done.
		case UNIT_GRAD :
			return_value = Math_Normalize(return_value, g_pi, 200.0);
			break;
	}
	return return_value;
}



#endif // MATH_C
