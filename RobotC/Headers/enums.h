#ifndef ENUMS_H
#define ENUMS_H
#pragma systemFile



typedef enum WheelPod {
	POD_FR = 0,
	POD_FL = 1,
	POD_BL = 2,
	POD_BR = 3,
	POD_NUM, // Should this be hard-coded to be 4?
};

typedef enum Servo {
	SERVO_FR = 0,
	SERVO_FL = 1,
	SERVO_BL = 2,
	SERVO_BR = 3,
	SERVO_NUM,
};

//typedef enum Aligned {
//	ALIGNED_FAR		= 0,
//	ALIGNED_MEDIUM	= 1,
//	ALIGNED_CLOSE	= 2,
//};



#endif // ENUMS_H
