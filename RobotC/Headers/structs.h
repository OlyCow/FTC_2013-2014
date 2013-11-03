#ifndef STRUCTS_H
#define STRUCTS_H
#pragma systemFile
#include "..\Headers\enums.h"



typedef struct motorData {
	bool isReversed;
	int angleOffset;
	float fineTuneFactor;
	int power;
} motorData;

typedef struct servoData {
	int angle;
	int power;
} servoData;



#endif // STRUCTS_H
