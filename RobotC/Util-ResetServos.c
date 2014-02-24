#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTServo)
#pragma config(Hubs,  S2, HTServo,  HTServo,  HTMotor,  HTMotor)
#pragma config(Sensor, S3,     sensor_IR,      sensorI2CCustomFastSkipStates9V)
#pragma config(Sensor, S4,     sensor_protoboard, sensorI2CCustom9V)
#pragma config(Motor,  motorA,          motor_assist_R, tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          motor_assist_L, tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     motor_lift_front, tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motor_lift_back, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     motor_BR,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motor_FR,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C3_1,     motor_FL,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C3_2,     motor_sweeper, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C4_1,     motor_flag,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C4_2,     motor_BL,      tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S1_C3_1,    servo_BR,             tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    servo_FR,             tServoStandard)
#pragma config(Servo,  srvo_S1_C3_3,    servo_flip_R,         tServoStandard)
#pragma config(Servo,  srvo_S1_C3_4,    servo_climb_R,        tServoStandard)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoStandard)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoStandard)
#pragma config(Servo,  srvo_S1_C4_1,    servo_dump,           tServoStandard)
#pragma config(Servo,  srvo_S1_C4_2,    servo8,               tServoStandard)
#pragma config(Servo,  srvo_S1_C4_3,    servo9,               tServoStandard)
#pragma config(Servo,  srvo_S1_C4_4,    servo10,              tServoStandard)
#pragma config(Servo,  srvo_S1_C4_5,    servo_flip_L,         tServoStandard)
#pragma config(Servo,  srvo_S1_C4_6,    servo_FL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_1,    servo_BL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    servo_climb_L,        tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    servo15,              tServoStandard)
#pragma config(Servo,  srvo_S2_C1_4,    servo16,              tServoStandard)
#pragma config(Servo,  srvo_S2_C1_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo12,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_1,    servo_FL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    servo_BL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    servo_flip_L,         tServoStandard)
#pragma config(Servo,  srvo_S2_C2_4,    servo_climb_L,        tServoStandard)
#pragma config(Servo,  srvo_S2_C2_5,    servo17,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo18,              tServoNone)

#include "includes.h"
#include "swerve-drive.h"

//#define WILL_EXPLODE // Uncomment this line to prevent development code from compiling.
#ifdef WILL_EXPLODE
#warn "This code will explode!"
#endif

task main()
{
	initializeGlobalVariables(); // Defined in "initialize.h", this intializes all struct members.
	initializeRobotVariables();
	Task_Kill(displayDiagnostics); // This is set separately in the "Display" task.
	Display_Clear();

	while (true) {
		servo[servo_FR] = 127;
		servo[servo_FL] = 127;
		servo[servo_BL] = 127;
		servo[servo_BR] = 0;
	}
}
