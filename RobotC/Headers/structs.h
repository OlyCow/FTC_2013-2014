#ifndef STRUCTS_H
#define STRUCTS_H
#pragma systemFile
#include "..\Headers\enums.h"



typedef struct motorData {
	bool isReversed;
	int angleOffset;
	int power;
} motorData;

typedef struct joystickData {
	short buttonMap;
	Direction direction;
} joystickData;



#endif // STRUCTS_H
