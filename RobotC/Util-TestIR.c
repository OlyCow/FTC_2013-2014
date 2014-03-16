#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTServo)
#pragma config(Hubs,  S2, HTServo,  HTServo,  HTMotor,  HTMotor)
#pragma config(Sensor, S3,     sensor_IR,      sensorI2CCustomFastSkipStates9V)
#pragma config(Sensor, S4,     sensor_protoboard, sensorI2CCustom9V)
#pragma config(Motor,  motorA,          motor_assist_R,   tmotorNXT, PIDControl, reversed, encoder)
#pragma config(Motor,  motorB,          motor_assist_L,   tmotorNXT, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     motor_BR,         tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motor_lift_back,  tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motor_lift_front, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motor_FR,         tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C3_1,     motor_FL,         tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C3_2,     motor_sweeper,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C4_1,     motor_flag,       tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S2_C4_2,     motor_BL,         tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S1_C3_1,    servo_BR,             tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    servo_climb_R,        tServoStandard)
#pragma config(Servo,  srvo_S1_C3_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_1,    servo_flip_R,         tServoStandard)
#pragma config(Servo,  srvo_S1_C4_2,    servo8,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_3,    servo9,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S1_C4_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo_FR,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_1,    servo13,              tServoNone)
#pragma config(Servo,  srvo_S2_C1_2,    servo14,              tServoNone)
#pragma config(Servo,  srvo_S2_C1_3,    servo15,              tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    servo16,              tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo_flip_L,         tServoStandard)
#pragma config(Servo,  srvo_S2_C1_6,    servo_FL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C2_1,    servo_BL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    servo_dump,           tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    servo21,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_4,    servo22,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo23,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo_climb_L,        tServoStandard)

#include "includes.h"
#include "swerve-drive.h"

task main()
{
	initializeGlobalVariables();
	initializeRobotVariables();
	bDisplayDiagnostics = false;
	Display_Clear();

	int	IR_A = 0,
		IR_B = 0,
		IR_C = 0,
		IR_D = 0,
		IR_E = 0;

	while (true) {
		HTIRS2readAllACStrength(sensor_IR, IR_A, IR_B, IR_C, IR_D, IR_E);
		nxtDisplayCenteredBigTextLine(3, "%4d", IR_C);
		Time_Wait(50);
	}
}
