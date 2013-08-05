#ifndef STRUCTS_H
#define STRUCTS_H
#pragma systemFile



typedef struct motorData {
	int angleOffset;
	int power;
} motorData;

typedef struct joystickData {
	short buttonMap;
	Direction direction;
} joystickData;



#endif // STRUCTS_H
