#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTServo,  HTServo,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     sensor_IR,      sensorI2CCustomFastSkipStates9V)
#pragma config(Sensor, S4,     sensor_gyro,    sensorI2CCustomFastSkipStates9V)
#pragma config(Motor,  mtr_S1_C1_1,     motor_FR,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     motor_FL,      tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     motor_BL,      tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     motor_BR,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     motor_climb,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     motor_flag,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     motor_lift,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     motor_sweeper, tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S2_C1_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo_funnel_L,       tServoStandard)
#pragma config(Servo,  srvo_S2_C1_6,    servo_funnel_R,       tServoStandard)
#pragma config(Servo,  srvo_S2_C2_1,    servo_dump,           tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    servo_flag,           tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    servo9,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo12,              tServoNone)

#include "includes.h"



task main()
{
	bDisplayDiagnostics = false;
	float loopDelay = 0.0;		// Milliseconds.
	bool isSweeping = false;	// Currently not reversible--should it be?
	float heading = 0.0;		// Degrees.
	float translation_x = 0.0;
	float translation_y = 0.0;
	float rotation = 0.0;
	float power_FR = 0.0;
	float power_FL = 0.0;
	float power_BL = 0.0;
	float power_BR = 0.0;
	float power_cap = 0.0;		// The max value to normalize to.
	bool doNormalizePower = false;
	float power_lift = 0.0;
	float power_climb = 0.0;
	bool isRaisingFlag = false;
	typedef enum ClimbMode {
		CLIMB_NEUTRAL	= 1,
		CLIMB_RELEASE	= 2,
		CLIMB_PULL		= 3,
	} ClimbMode;
	ClimbMode climbingMode = CLIMB_NEUTRAL;

	// The gyro needs to be calibrated before the match because it actually
	// takes a significant amount of time to do so (approx. 250 milliseconds).
	// The wait-time for this is included when the function is called.
	HTGYROstartCal(sensor_gyro);
	Joystick_WaitForStart();

	// If the timer isn't cleared here, the first gyro reading will be off
	// by a lot, and consequently all the other readings that come later.
	Time_ClearTimer(T1);
	while (true)
	{
		// Read from the gyro (integrate another delta t) and clear timer T1 for
		// the next loop. This way we don't need separate variables to keep track
		// of the current time, previous time, and delta t for this iteration.
		heading += (-1)*((float)HTGYROreadRot(sensor_gyro))*((float)Time_GetTime(T1))/((float)1000); // 1000 milliseconds per second.
		loopDelay = Time_GetTime(T1);
		Time_ClearTimer(T1);

		Joystick_UpdateData();
		translation_x = Math_Normalize(Math_TrimDeadband(Joystick_Joystick(JOYSTICK_L, AXIS_X), g_JoystickDeadband), g_JoystickMax, g_FullPower);
		translation_y = Math_Normalize(Math_TrimDeadband(Joystick_Joystick(JOYSTICK_L, AXIS_Y), g_JoystickDeadband), g_JoystickMax, g_FullPower);
		rotation = Math_Normalize(Math_TrimDeadband(Joystick_Joystick(JOYSTICK_R, AXIS_X), g_JoystickDeadband), g_JoystickMax, g_FullPower);

		// Pressing `BUTTON_RT` slows down translation; `BUTTON_LT` slows rotation.
		// Both can be pressed at once (so we can't use an if...else statement).
		if (Joystick_Button(BUTTON_RT)==true) {
			translation_x /= (float)g_FineTuneFactor;
			translation_y /= (float)g_FineTuneFactor;
		}
		if (Joystick_Button(BUTTON_LT)==true) {
			rotation /= (float)g_FineTuneFactor;
		}

		// TEMP(?): debugging stuff.
		nxtDisplayTextLine(2, "x0:%f", translation_x);
		nxtDisplayTextLine(3, "y0:%f", translation_y);

		Math_RotateVector(translation_x, translation_y, -(heading)); // MAGIC_NUM: 4 is a magic constant. :P

		// TEMP(?): debugging stuff.
		nxtDisplayTextLine(0, "rot:%f", heading);
		nxtDisplayTextLine(4, "x':%f", translation_x);
		nxtDisplayTextLine(5, "y':%f", translation_y);
		nxtDisplayTextLine(7, "dt:%d", loopDelay);

		// For the derivation of this dark wizardry algorithm, refer to our
		// engineering notebook, or talk to Ernest, Kieran, or Nathan.
		power_FR = translation_y-translation_x-rotation;
		power_FL = translation_y+translation_x+rotation;
		power_BL = translation_y-translation_x+rotation;
		power_BR = translation_y+translation_x-rotation;

		// Iterate through all four power levels, tripping the "normalize" flag
		// and updating `power_cap` if the power level exceeds full power AND
		// the current `power_cap`. `power_cap` doesn't need to be reset because
		// it doesn't matter if the "normalize" flag isn't tripped, and if the
		// flag is tripped `power_cap` will be overwritten.
		if (power_FR>g_FullPower) {
			power_cap = power_FR;
			doNormalizePower = true;
		}
		if ((power_FL>g_FullPower)&&(power_FL>power_cap)) {
			power_cap = power_FL;
			doNormalizePower = true;
		}
		if ((power_BL>g_FullPower)&&(power_BL>power_cap)) {
			power_cap = power_BL;
			doNormalizePower = true;
		}
		if ((power_BR>g_FullPower)&&(power_BR>power_cap)) {
			power_cap = power_BR;
			doNormalizePower = true;
		}
		if (doNormalizePower==true) {
			Math_Normalize(power_FR, power_cap, g_FullPower);
			Math_Normalize(power_FR, power_cap, g_FullPower);
			Math_Normalize(power_FR, power_cap, g_FullPower);
			Math_Normalize(power_FR, power_cap, g_FullPower);
			doNormalizePower = false; // Reset this flag for the next iteration.
		}

		// Toggle the streeper whenever `BUTTON_A` is *released*.
		if (Joystick_ButtonReleased(BUTTON_A)==true) {
			isSweeping = (!isSweeping); // TODO: see if this can be replaced with `^=`.
		}

		// The lift controls of the second driver are overriden by the first's.
		power_lift = Math_Normalize(Math_TrimDeadband(Joystick_Joystick(JOYSTICK_R, AXIS_Y, CONTROLLER_2), g_JoystickDeadband), g_JoystickMax, g_FullPower);
		if (Joystick_Direction(DIRECTION_NONE)==false) {
			power_lift = 0;
			if (Joystick_Direction(DIRECTION_F)==true) {
				power_lift = g_FullPower;
			} else if (Joystick_Direction(DIRECTION_B)==true) {
				power_lift = -g_FullPower;
			}
		}

		// Only the second driver can control climbing. (This should be an agreed
		// upon decision anyway; plus, climbing isn't time-critical either.)
		if (Joystick_ButtonReleased(BUTTON_A)==true) {
			climbingMode = CLIMB_RELEASE;
		} else if (Joystick_ButtonReleased(BUTTON_B)==true) {
			climbingMode = CLIMB_PULL;
		} else if (Joystick_ButtonReleased(BUTTON_X)==true) {
			climbingMode = CLIMB_NEUTRAL;
		}
		switch (climbingMode) {
			case CLIMB_NEUTRAL :
				power_climb = 0;
				break;
			case CLIMB_RELEASE :
				power_climb = -100;
				break;
			case CLIMB_PULL :
				power_climb = 100;
		}

		// `BUTTON_Y` has to be held down to raise the flag; this is safer since a
		// toggle actually requires more work (given the speed we raise the flag at).
		if (Joystick_Button(BUTTON_Y)==true) {
			isRaisingFlag = true;
		} else {
			isRaisingFlag = false;
		}

		// `BUTTON_B` will reset the heading (to compensate for gyro drift).
		// TODO: Limit function to work only when button is pressed quickly in
		// succession, to prevent accidental presses from confusing the driver.
		if (Joystick_ButtonReleased(BUTTON_B)==true) {
			heading = 0.0;
		}

		// Set power levels to all motors. The motors that have their power levels
		// determined by booleans are either on full power or at zero power.
		Motor_SetPower(power_FR, motor_FR);
		Motor_SetPower(power_FL, motor_FL);
		Motor_SetPower(power_BL, motor_BL);
		Motor_SetPower(power_BR, motor_BR);
		if (isSweeping==true) {
			Motor_SetPower(g_FullPower, motor_sweeper);
		} else {
			Motor_SetPower(0, motor_sweeper);
		}
		Motor_SetPower(power_lift, motor_lift);
		if (isRaisingFlag==true) {
			Motor_SetPower(g_FullPower, motor_flag);
		} else {
			Motor_SetPower(0, motor_flag);
		}
		Motor_SetPower(power_climb, motor_climb);
	}
}
