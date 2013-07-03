#pragma config(Sensor, S1,     startButton,    sensorTouch)
#pragma config(Motor,  motorA,          rotationSensor, tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          motorL,        tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorC,          motorR,        tmotorNXT, PIDControl, encoder)

//#include "JoystickDriver.c"  // Handles Bluetooth messages?



const int g_touchThreshold = 512;
 float kP=12;
 float kI=0;
 float kD=1;

task main()
{
	//int startButtonPressed=0;
	//while (startButtonPressed<g_touchThreshold) {
	//	startButtonPressed=SensorRaw[startButton];
	//}

	nMotorEncoder[rotationSensor]=0;
	time10[T1]=0;
	float termP=0, termI=0, termD=0;
	float totalChange=0;
	int prevValue=0, currentValue=0, deltaValue=0;
	int prevTime=0, currentTime=0, deltaTime=0;
	int accumulatedError=0;
	while (true){
		prevValue=currentValue;
		currentValue=nMotorEncoder[rotationSensor];
		deltaValue=currentValue-prevValue;
		prevTime=currentTime;
		currentTime=time10[T1];
		deltaTime=currentTime-prevTime;
		accumulatedError=(deltaValue/2+prevValue)*deltaTime;
		termP=kP*currentValue;
		termI=kI*accumulatedError;
		//termD=kD*((float)deltaValue/(float)deltaTime);
		totalChange=termP+termI+termD;
		motor[motorL]=totalChange;
		motor[motorR]=totalChange;
	}
}
