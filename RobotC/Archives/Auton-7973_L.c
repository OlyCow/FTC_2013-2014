#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     motor_R,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     motor_L,       tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     motor_flag,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motorG,        tmotorTetrix, openLoop)

#include "includes.h"



void driveForward(int individual_power, int seconds);
void driveBackward(int individual_power, int seconds);
void turnLeft(int individual_power, int seconds);
void turnRight(int individual_power, int seconds);
void stopAllMotors();

task main() {
	bDisplayDiagnostics = false;
	int forward_time	= 1500;
	int turn_time		= 500;
	int ramp_time		= 2000;
	Joystick_WaitForStart();

	// Fix numbers

	driveForward(100, forward_time);
	turnLeft(50, turn_time);
	driveForward(100, ramp_time);
	stopAllMotors();
}



void driveForward(int individual_power, int seconds) {
	Motor_SetPower(individual_power, motor_L);
	Motor_SetPower(individual_power, motor_R);
	Time_Wait(seconds);
	stopAllMotors();
}
void driveBackward(int individual_power, int seconds) {
	Motor_SetPower(-individual_power, motor_L);
	Motor_SetPower(-individual_power, motor_R);
	Time_Wait(seconds);
	stopAllMotors();
}
void turnLeft(int individual_power, int seconds) {
	Motor_SetPower(-individual_power, motor_L);
	Motor_SetPower(individual_power, motor_R);
	Time_Wait(seconds);
	stopAllMotors();
}
void turnRight(int individual_power, int seconds) {
	Motor_SetPower(individual_power, motor_L);
	Motor_SetPower(-individual_power, motor_R);
	Time_Wait(seconds);
	stopAllMotors();
}
void stopAllMotors() {
	Motor_SetPower(0, motor_L);
	Motor_SetPower(0, motor_R);
}
