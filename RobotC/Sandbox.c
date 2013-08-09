#pragma config(Hubs,  S3, HTServo,  none,     none,     none)
#pragma config(Hubs,  S4, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S3,     ,               sensorI2CMuxController)
#pragma config(Sensor, S4,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S4_C1_1,     motor_FR,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S4_C1_2,     motor_FL,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S4_C2_1,     motor_BL,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S4_C2_2,     motor_BR,      tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S3_C1_1,    servo_FR,             tServoStandard)
#pragma config(Servo,  srvo_S3_C1_2,    servo_FL,             tServoStandard)
#pragma config(Servo,  srvo_S3_C1_3,    servo_BL,             tServoStandard)
#pragma config(Servo,  srvo_S3_C1_4,    servo_BR,             tServoStandard)
#pragma config(Servo,  srvo_S3_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S3_C1_6,    servo6,               tServoNone)

#include "Headers\includes.h"
#include "Teleop-Basic.h"



task main() {
	nMotorEncoder[motor_FR] = 0;
	nMotorEncoder[motor_FL] = 0;
	nMotorEncoder[motor_BL] = 0;
	nMotorEncoder[motor_BR] = 0;
	float NUMBER_FR = -288;
	float NUMBER_FL = -388;
	float NUMBER_BL = -488;
	float NUMBER_BR = -588;
	Joystick_WaitForStart();
	while (true) {
		NUMBER_FR = nMotorEncoder[motor_BL];
		NUMBER_FL = nMotorEncoder[motor_FL];
		NUMBER_BR = nMotorEncoder[motor_BR];
		nxtDisplayCenteredTextLine(2, "FR: %f", NUMBER_FR);
		nxtDisplayCenteredTextLine(3, "FL: %f", NUMBER_FL);
		nxtDisplayCenteredTextLine(4, "BL: %f", NUMBER_BL);
		nxtDisplayCenteredTextLine(5, "BR: %f", NUMBER_BR);
	}
}
