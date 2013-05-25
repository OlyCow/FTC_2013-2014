#ifndef ROBOTC_ENCAPSULATION_H
#define ROBOTC_ENCAPSULATION_H
#pragma systemFile
#include "..\Headers\includes.h"
#include "..\Libraries\RobotC encapsulation.c"



/////////////////////////////
//    "namespace" Motor    //
/////////////////////////////
void Motor_Forward(tMotor motorName, int power=75);
void Motor_Reverse(tMotor motorName, int power=75);
void Motor_Stop(tMotor motorName, bool brake=true);

// This function does NOT reset the encoder, in case that is being
// used elsewhere. Reset the encoder periodically to prevent overflow.
void Motor_Target(tMotor motorName, int angle);
void Motor_SetPower(tMotor motorName, int power);
int Motor_GetEncoder(tMotor motorName);
void Motor_ResetEncoder(tMotor motorName);
void Motor_SetBrakes(bool isOn=true);
void Motor_SetMaxSpeed(int speed=750);
void Motor_SetPIDInterval(int interval=20);


/////////////////////////////
//    "namespace" Servo    //
/////////////////////////////
void Servo_Rotate(TServoIndex servoName, short position);
short Servo_GetPosition(TServoIndex servoName);
void Servo_SetSpeed(TServoIndex servoName, int rate);
void Servo_LockPosition(TServoIndex servoName, bool isLocked=true);


/////////////////////////////
//    "namespace" Sensor   //
/////////////////////////////


/////////////////////////////
//   "namespace" Joystick  //
/////////////////////////////
void Joystick_UpdateData();
bool Joystick_Button(	JoystickButton button,
						Controller controller = CONTROLLER_1);

// Takes an input of "Joystick" instead of "joystick" to
// avoid conflict with name of built-in struct "joystick";
int Joystick_Joystick(	JoystickLR Joystick,	//Still a relatively good line of code
						JoystickAxis axis,
						Controller controller = CONTROLLER_1);
Direction Joystick_Direction(Controller controller = CONTROLLER_1);


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
void Time_Wait(int time);


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
void Semaphore_Initialize(TSemaphore semaphore);
void Semaphore_Lock(TSemaphore semaphore, int wait);
void Semaphore_Unlock(TSemaphore semaphore);
bool Semaphore_IsCurrentlyOwned(TSemaphore semaphore);
ubyte Semaphore_GetOwner(TSemaphore semaphore);


/////////////////////////////
//  "namespace" Multitask  //
/////////////////////////////
void Task_ReleaseTimeslice();
void Task_StartTask(void taskID, short priority=7);
void Task_StopTask(void taskID);
void Task_HogCPU();
void Task_ReleaseCPU();
void Task_AbortAll();


/////////////////////////////
//     "namespace" Math    //
/////////////////////////////


/////////////////////////////
//     "namespace" Misc    //
/////////////////////////////



#endif // ROBOTC_ENCAPSULATION_H
