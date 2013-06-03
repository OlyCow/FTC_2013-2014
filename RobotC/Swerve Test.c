#pragma config(Hubs,  S1, HTMotor,  none,     none,     none)
#pragma config(Hubs,  S2, HTServo,  none,     none,     none)
#pragma config(Motor,  motorA,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorB,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C1_1,     motor_wheel,   tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motorE,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S2_C1_1,    servo_wheel,          tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo6,               tServoNone)

#include "Headers\includes.h"



task main()
{
	initializeVariables();

	servoChangeRate[servo_wheel]=5;

	float servoPosition=127; //servo value ranges from 0 to 255
	float motorPower=0; //safe initialization

	waitForStart();

	while(true)
	{
		wait10Msec(1);
		getJoystickSettings(joystick);
		motorPower=0; //as we have learned, zero-ing is important


		//-------------------------MOTORS----------
		if( (joystick.joy1_y2>g_JoystickDeadZone) ||
			(joystick.joy1_y2<-g_JoystickDeadZone) )
				motorPower=-(joystick.joy1_y2);
		if( (joy1Btn(BUTTON_LT)||joy1Btn(BUTTON_RT)) ==true)
			motorPower /= 4;
		motor[motor_wheel]=motorPower;


		//-------------------------SERVOS----------
		if(joystick.joy1_TopHat==(short)DIRECTION_R)
			servoPosition+=4;
		if(joystick.joy1_TopHat==(short)DIRECTION_L)
			servoPosition-=4;
		if( (joy1Btn(BUTTON_LB)||joy1Btn(BUTTON_RB)) ==true )
		{
			if(joystick.joy1_TopHat==(short)DIRECTION_R)
				servoPosition-=3;
			if(joystick.joy1_TopHat==(short)DIRECTION_L)
				servoPosition+=3;
		}
		servo[servo_wheel]=servoPosition;
		servoChangeRate[servo_wheel]=5;
		if(servoPosition>255)
			servoPosition=255;
		if(servoPosition<0)
			servoPosition=0;
	}
}
