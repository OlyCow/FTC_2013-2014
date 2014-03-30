#pragma config(Hubs,  S1, HTServo,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTMotor,  HTServo,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     sensor_IR,      sensorI2CCustomFastSkipStates9V)
#pragma config(Sensor, S4,     sensor_protoboard, sensorI2CCustomFastSkipStates9V)
#pragma config(Motor,  mtr_S1_C2_1,     motor_flag,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motor_sweeper, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     motor_climb,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     motor_lift,    tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C4_1,     motor_BL,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C4_2,     motor_FL,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C1_1,     motor_BR,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C1_2,     motor_FR,      tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S1_C1_1,    servo_BL,             tServoStandard)
#pragma config(Servo,  srvo_S1_C1_2,    servo_FL,             tServoStandard)
#pragma config(Servo,  srvo_S1_C1_3,    servo_dump,           tServoStandard)
#pragma config(Servo,  srvo_S1_C1_4,    servo_flag,           tServoStandard)
#pragma config(Servo,  srvo_S1_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_6,    servo_climb_L,        tServoStandard)
#pragma config(Servo,  srvo_S2_C2_1,    servo_BR,             tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    servo_FR,             tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    servo9,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo_climb_R,        tServoStandard)

#include "includes.h"
#include "swerve-drive.h"

#define TEST_MOTOR motor_BL

task main()
{
	bDisplayDiagnostics = false;
	float angle = 0.0;
	Motor_ResetEncoder(TEST_MOTOR);
	while (true) {
		angle = Motor_GetEncoder(TEST_MOTOR);
		angle /= 4.0;
		nxtDisplayCenteredBigTextLine(3, "%+5d", angle);
		Time_Wait(100);
	}
}
