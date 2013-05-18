#ifndef LOW_LEVEL_FUNCTIONS_H
#define LOW_LEVEL_FUNCTIONS_H


/////////////////////////////
//    "namespace" Motor    //
/////////////////////////////

void Motor_Forward(tMotor motorName, int power=75)
{
	motor[motorName] = power;
}

void Motor_Reverse(tMotor motorName, int power=75)
{
	motor[motorName] = -1 * power;
}

void Motor_Stop(tMotor motorName, bool brake=true)
{
	motor[motorName] = 0;
	// Using this flag instead of `Motor_SetBrakes` since this is low-level.
	bFloatDuringInactiveMotorPWM = !(brake);
}

// This function does NOT reset the encoder, in case that is being
// used elsewhere. Reset the encoder periodically to prevent overflow.
void Motor_Target(tMotor motorName, int angle)
{
	// Using some variables directly since this code is low-level.
	int originalAngle = 0;
	originalAngle = nMotorEncoder[motorName];
	nMotorEncoderTarget[motorName] = angle + originalAngle;
}

void Motor_SetPower(tMotor motorName, int power)
{
	motor[motorName] = power;
}

int Motor_GetEncoder(tMotor motorName)
{
	int encoder = 0;
	encoder = nMotorEncoder[motorName];
	return encoder;
}

void Motor_ResetEncoder(tMotor motorName)
{
	nMotorEncoder[motorName] = 0;
}

void Motor_SetBrakes(bool isOn=true)
{
	bFloatDuringInactiveMotorPWM = !isOn;
}

void Motor_SetMaxSpeed(int speed=750)
{
	nMaxRegulatedSpeedNxt = speed;
}

void Motor_SetPIDInterval(int interval=20)
{
	nPidUpdateInterval = interval;
}


/////////////////////////////
//    "namespace" Servo    //
/////////////////////////////

void Servo_Rotate(TServoIndex servoName, short position)
{
	servo[servoName] = position;
}

short Servo_GetPosition(TServoIndex servoName)
{
	return ServoValue[servoName];
}

void Servo_SetSpeed(TServoIndex servoName, int rate)
{
	servoChangeRate[servoName] = rate;
}

void Servo_LockPosition(TServoIndex servoName, bool isLocked=true)
{
	bSystemLeaveServosEnabledOnProgramStop = isLocked;
}


/////////////////////////////
//    "namespace" Sensor   //
/////////////////////////////


/////////////////////////////
//   "namespace" Joystick  //
/////////////////////////////

void Joystick_UpdateData()
{
	getJoystickSettings(joystick);
}

bool Joystick_Button(	JoystickButton button,
						Controller controller = CONTROLLER_1)
{
	bool isPressed = false;

	switch (controller)
	{
		case CONTROLLER_1:
			isPressed = (bool)(joy1Btn(button));
			break;
		case CONTROLLER_2:
			isPressed = (bool)(joy2Btn(button));
			break;
	}

	return isPressed;
}

// Takes an input of "Joystick" instead of "joystick" to
// avoid conflict with name of built-in struct "joystick";
int Joystick_Joystick(	Joystick Joystick,	//Still a relatively good line of code
						Axis axis,
						Controller controller = CONTROLLER_1)
{
	int axisValue = 0;


	switch (controller)
	{
		case CONTROLLER_1:
			switch (Joystick)
			{
				case JOYSTICK_L:
					switch (axis)
					{
						case AXIS_X:	//controller 1, joystick L, X-axis
							axisValue = joystick.joy1_x1;
							break;
						case AXIS_Y:	//controller 1, joystick L, Y-axis
							axisValue = joystick.joy1_y1;
							break;
					}
					break;
				case JOYSTICK_R:
					switch (axis)
					{
						case AXIS_X:	//controller 1, joystick L, X-axis
							axisValue = joystick.joy1_x2;
							break;
						case AXIS_Y:	//controller 1, joystick L, Y-axis
							axisValue = joystick.joy1_y2;
							break;
					}
					break;
			}
			break;

		case CONTROLLER_2:
			switch (Joystick)
			{
				case JOYSTICK_L:
					switch (axis)
					{
						case AXIS_X:	//controller 2, joystick L, X-axis
							axisValue = joystick.joy2_x1;
							break;
						case AXIS_Y:	//controller 2, joystick L, Y-axis
							axisValue = joystick.joy2_y1;
							break;
					}
					break;
				case JOYSTICK_R:
					switch (axis)
					{
						case AXIS_X:	//controller 2, joystick L, X-axis
							axisValue = joystick.joy2_x2;
							break;
						case AXIS_Y:	//controller 2, joystick L, Y-axis
							axisValue = joystick.joy2_y2;
							break;
					}
					break;
			}
			break;
	}


	return axisValue;
}

JoystickDirection Joystick_Direction(Controller controller =
															CONTROLLER_1)
{
	Direction direction = DIRECTION_NONE;

	switch (controller)
	{
		case CONTROLLER_1:
			direction = (Direction)joystick.joy1_TopHat;
			break;
		case CONTROLLER_2:
			direction = (Direction)joystick.joy2_TopHat;
			break;
	}

	return direction;
}


/////////////////////////////
//   "namespace" Buttons   //
/////////////////////////////


/////////////////////////////
//     "namespace" File    //
/////////////////////////////


/////////////////////////////
//     "namespace" Time    //
/////////////////////////////

// breaks down time to wait into 10ms and 1ms chunks
void Time_Wait(int time)
{
	wait10Msec(time);
}


/////////////////////////////
//    "namespace" Sound    //
/////////////////////////////


/////////////////////////////
//   "namespace" Display   //
/////////////////////////////


/////////////////////////////
//  "namespace" Bluetooth  //
/////////////////////////////


/////////////////////////////
//  "namespace" Semaphore  //
/////////////////////////////

void Semaphore_Initialize(TSemaphore semaphore)
{
	SemaphoreInitialize(semaphore);
}

void Semaphore_Lock(TSemaphore semaphore, int wait)
{
	SemaphoreLock(semaphore, wait);
}

void Semaphore_Unlock(TSemaphore semaphore)
{
	SemaphoreUnlock(semaphore);
}

bool Semaphore_IsCurrentlyOwned(TSemaphore semaphore)
{
	return (bool)bDoesTaskOwnSemaphore(semaphore);
}

ubyte Semaphore_GetOwner(TSemaphore semaphore)
{
	return getSemaphoreTaskOwner(semaphore);
}


/////////////////////////////
//  "namespace" Multitask  //
/////////////////////////////

void Task_ReleaseTimeslice()
{
	abortTimeslice();
}

void Task_StartTask(void taskID, short priority=7)
{
	StartTask(taskID, priority);
}

void Task_StopTask(void taskID)
{
	StopTask(taskID);
}

void Task_HogCPU()
{
	hogCPU();
}

void Task_ReleaseCPU()
{
	releaseCPU();
}

void Task_AbortAll()
{
	StopAllTasks();
}


/////////////////////////////
//     "namespace" Math    //
/////////////////////////////

// TODO: make it actually convert to logarithmic values.
// For converting joystick input to logarithmic values.
int Math_ToLogarithmic(int input)
{
	int convertedInput = 0;
	if (input >= 0)
	{
		convertedInput = input;
	}
	else if (input < 0)
	{
		convertedInput = input;
	}
	return convertedInput;
}


/////////////////////////////
//     "namespace" Misc    //
/////////////////////////////


#endif
