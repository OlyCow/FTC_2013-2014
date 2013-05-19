#pragma config(Hubs,  S1, HTMotor,  none,     none,     none)
#pragma config(Hubs,  S2, HTServo,  none,     none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Motor,  motorA,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorB,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C1_1,     motor_wheel,   tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motorE,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S2_C1_1,    servo_cow,            tServoNone)
#pragma config(Servo,  srvo_S2_C1_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo6,               tServoNone)

#include "JoystickDriver.c"

task main()
{
	servoChangeRate[servo_cow]=5;

	float servoPosition=127; //servo value ranges from 0 to 255
	float motorPower=0; //safe initialization

	waitForStart();

	while(true)
	{
		wait10Msec(1);
		getJoystickSettings(joystick);
		motorPower=0;


		//-------------------------MOTORS-----
		if( (joystick.joy1_y2>10) || (joystick.joy1_y2<-10) )
			motorPower=-(joystick.joy1_y2);
		if( (joy1Btn(6)||joy1Btn(7)) ==true)
			motorPower /= 4;
		motor[motor_wheel]=motorPower;


		//-------------------------SERVOS-----
		if(joystick.joy1_TopHat==2)
			servoPosition++;
		if(joystick.joy1_TopHat==6)
			servoPosition--;
		if( (joy1Btn(4)||joy1Btn(5)) ==true )
		{
			if(joystick.joy1_TopHat==2)
				servoPosition+=4;
			if(joystick.joy1_TopHat==6)
				servoPosition-=4;
		}
		servo[servo_cow]=servoPosition;
		servoChangeRate[servo_cow]=5;
		if(servoPosition>255)
			servoPosition=255;
		if(servoPosition<0)
			servoPosition=0;
	}
}
