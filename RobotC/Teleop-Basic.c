#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S2, HTServo,  none,     none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     motor_FR,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motor_FL,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motor_BL,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motor_BR,      tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S2_C1_1,    servo_FR,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    servo_FL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    servo_BL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_4,    servo_BR,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_5,    servo_transmission,   tServoStandard)
#pragma config(Servo,  srvo_S2_C1_6,    servo_lock,           tServoStandard)

#include "Headers\includes.h"
#include "Teleop-Basic.h"

// Automatically starts all tasks defined in this file.
#pragma autoStartTasks

//#define WILL_EXPLODE // Uncomment this line (Ctrl-Q) to prevent development code from compiling.
#ifdef WILL_EXPLODE
#error "This code will explode!"
#endif

//---------------- README!!! ------------------------------------------------>>
//
//     Set the robot up as follows: with the front (where the NXT is mounted)
// facing towards you, the side of the wheel pods with 3D-printed gears should
// face forwards. As defined in "enums.h", the wheel pods are "numbered": `FR`,
// `FL`, `BL`, and `BR` (going counterclockwise starting with `FR`). As of this
// writing, `POD_BL` is missing a bevel gear and motor axle (for an encoder),
// and thus has an omni-wheel attached to it. This causes the robot not to be
// able to go in straight lines at some orientations, since the motor power on
// one side is greater than that on the other.
//
//     The code is currently split into 3 loops, which will be split into their
// own tasks once they are completed. #1 Find the target angle, velocity, etc.
// #2 Adjust the current angle, velocity, etc. through a PID control. The PID
// loop isn't completely implemented yet; go ahead and do that! :) #3 Display
// data on the NXT screen for debugging purposes. Believe me, this is useful :)
//
// CONTROLS:	Controller_1, Joystick_R:	Translational movement.
//				Controller_1, Button_LB/RB:	Rotational movement.
//				Controller_1, Button_A:		Toggles servo locking.
//				Controller_1, Button_X:		Toggles transmission gearing.
//				Controller_1, Button_LT:	Stop motors (adjust pod direction).
//				Controller_1, Button_RT:	Fine-tune motors.
//				Controller_2, Joystick_R:	Simulates gyro input.
//				Controller_1, Button_Y:		Moo.
//
//     To troubleshoot, simply download the code, give the robot some input, and
// look at the screen. "FR/FL/BL/BR" refers to the wheel pod. "set" refers to
// the target angle for the servo, while "chg" refers to the correction force
// applied by the servo. "encdr" prints the reading of the encoder (normalized,
// to compensate for gearing), and "pow" is the power applied to the motor.
//
//--------------------------------------------------------------------------->>



task main() {
	initializeGlobalVariables(); // Defined in "global vars.h", this intializes all struct members.
	disableDiagnosticsDisplay(); // Disables the "samostat.rxe"-like diagnostics screen which
	// comes with "JoystickDriver.c".

	// For finding target values:
	//g_task_main = Task_GetCurrentIndex(); // This was used when we had multiple tasks.
	float gyro_x = 0.0; // These two will be unnecessary once we get an actual gyro.
	float gyro_y = 0.0;
	float gyro_angle = 0.0;
	float rotation_magnitude = 0.0; // Components of the vector of rotation.
	float rotation_angle[POD_NUM] = {0,0,0,0};
	float rotation_x[POD_NUM] = {0,0,0,0};
	float rotation_y[POD_NUM] = {0,0,0,0};
	float translation_magnitude = 0.0; // Components of the vector of translation.
	float translation_angle = 0.0;
	float translation_x = 0.0;
	float translation_y = 0.0;
	float combined_magnitude[POD_NUM] = {0,0,0,0}; // Components of the final (assigned) vector.
	float combined_angle[POD_NUM] = {0,0,0,0};
	float combined_angle_prev[POD_NUM] = {0,0,0,0}; // Prevents atan2(0,0)=0 from resetting the wheel pods to 0.
	float combined_x[POD_NUM] = {0,0,0,0};
	float combined_y[POD_NUM] = {0,0,0,0};
	bool shouldNormalize = false; // This flag is set if motor values go over 100. All motor values will be scaled down.

	// For PID control:
	float time_current = Time_GetTime(TIMER_PROGRAM);
	float time_previous = time_current;
	float time_difference = time_current-time_previous;
	const int kI_delay = 10;
	float error_accumulated[POD_NUM][kI_delay];
	for (int i=0; i<(int)POD_NUM; i++) {
		for (int j=0; j<kI_delay; j++) {
			error_accumulated[i][j] = 0;
		}
	}
	float error_accumulated_total[POD_NUM] = {0,0,0,0};
	float kP[POD_NUM] = {2.4, 2.4, 2.4, 14.4}; // {FR, FL, BL, BR} // BR scrapes. What it really needs is a larger kI.
	float kI[POD_NUM] = {0.0, 0.0, 0.0, 0.0}; // {FR, FL, BL, BR}
	float kD[POD_NUM] = {0.0, 0.0, 0.0, 0.0}; // {FR, FL, BL, BR}
	float current_encoder[POD_NUM] = {0,0,0,0};
	float error[POD_NUM] = {0,0,0,0}; // Difference between set-point and measured value.
	float term_P[POD_NUM] = {0,0,0,0};
	float term_I[POD_NUM] = {0,0,0,0};
	float term_D[POD_NUM] = {0,0,0,0};
	float total_correction[POD_NUM] = {0,0,0,0}; // Simply "term_P + term_I + term_D".
	Aligned isAligned = ALIGNED_CLOSE; // If false, cut motor power so that wheel pod can get aligned.

	// Miscellaneous variables:
	bool isLocked = false;
	const int lockedPosition = 100; // I just made up these numbers.
	const int unlockedPosition = 20; // I just made up these numbers.
	bool isLowGear = false;
	const int lowGearPosition = 0; // I just made up these numbers.
	const int highGearPosition = 255; // I just made up these numbers.
	bool isPlaying = false;

	Joystick_WaitForStart();

	time_previous = 0;
	time_current = Time_GetTime(TIMER_PROGRAM);
	time_difference = time_current-time_previous;

	while (true) {
		Joystick_UpdateData();

		// This part is temporary. We are assigning a gyro angle from a joystick.
		// An operator moves the joystick to correspond with the front of the drive
		// base, to simulate the input from an actual gyro. When we get a gyro, fix
		// this so it returns the actual value of the gyro. If you just ignore the
		// second controller, then the movement will be absolute (not relative).
		gyro_x = Math_TrimDeadband(Joystick_Joystick(JOYSTICK_R, AXIS_X, CONTROLLER_2));
		gyro_y = Math_TrimDeadband(Joystick_Joystick(JOYSTICK_R, AXIS_Y, CONTROLLER_2));
		gyro_angle = Math_RadToDeg(atan2(gyro_y, gyro_x)); //atan2(0,0)=0

		// Actual code starts here. It is ridiculously simple, but oddly counter-
		// intuitive. The rotation vector and the translation vector are combined,
		// then the final vector is normalized. That is it. This algorithm has the
		// property of seeming like the direction of the resultant vector is the
		// average of the input vectors' directions, which it is. It might seem like
		// there would be inevitable slipping involved in this scheme, but a simple
		// derivative analysis of a generalized cycloid shows the above algorithm
		// to describe the direction of the wheel pods perfectly.
		rotation_magnitude = Joystick_GetRotationMagnitude(); // Either LB/RB or X-axis of other joystick.
		translation_x = Joystick_GetTranslationX();
		translation_y = Joystick_GetTranslationY();
		translation_magnitude = sqrt(pow(translation_x,2)+pow(translation_y,2)); // Pythagorean Theorem.
		translation_angle = Math_RadToDeg(atan2(translation_y, translation_x)); // -180deg ~ 180deg
		for (int i=POD_FR; i<=(int)POD_BR; i++) {
			rotation_angle[i] = g_MotorData[i].angleOffset+90; // Tangent to circle, so +90deg.
			rotation_x[i] = rotation_magnitude*cosDegrees(rotation_angle[i]); // Simple algebra.
			rotation_y[i] = rotation_magnitude*sinDegrees(rotation_angle[i]);
			combined_x[i] = translation_x+rotation_x[i]; // Combining the vectors' components.
			combined_x[i] = combined_x[i];
			combined_y[i] = translation_y+rotation_y[i];
			combined_magnitude[i] = sqrt(pow(combined_x[i],2)+pow(combined_y[i],2));
			if (combined_magnitude[i]>g_FullPower) {
				shouldNormalize = true;
			}
			combined_angle[i] = (Math_RadToDeg(atan2(combined_y[i], combined_x[i]))+gyro_angle+360)%360;
			if ((combined_angle[i]==0)&&(combined_magnitude[i]==0)) {
				combined_angle[i] = combined_angle_prev[i];
			} else {
				combined_angle_prev[i] = combined_angle[i];
			}
			g_ServoData[i].angle = combined_angle[i];
		}

		// Normalize our motors' power values if a motor's power went above g_FullPower.
		if (shouldNormalize==true) {
			float originalMaxPower = combined_magnitude[0];
			for (int i=POD_FR; i<=(int)POD_BR; i++) {
				if (combined_magnitude[i]>originalMaxPower) {
					originalMaxPower = combined_magnitude[i];
				}
			}
			for (int i=POD_FR; i<=(int)POD_BR; i++) {
				combined_magnitude[i] = Math_Normalize(combined_magnitude[i], originalMaxPower, g_FullPower);
			}
			shouldNormalize = false; // Reset this for the next iteration.
		}
		for (int i=POD_FR; i<=(int)POD_BR; i++) {
			g_MotorData[i].power = combined_magnitude[i];
		}

		// Ideally, this should be made more intuitive. Maybe a single trigger = slow,
		// while holding both triggers stops movement? The `if... else if...` structure
		// is also a problem (BUTTON_LT takes precedence over BUTTON_RT).
		// Set our "fine-tune" factor (amount motor power is divided by).
		if (Joystick_Button(BUTTON_LT)==true) {
			for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
				g_MotorData[i].fineTuneFactor = 0; // Equivalent to zeroing motor power.
			}
		} else if (Joystick_Button(BUTTON_RT)==true) {
			for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
				g_MotorData[i].fineTuneFactor = 0.2;
			}
		} else {
			for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
				g_MotorData[i].fineTuneFactor = 1; // Equivalent to not fine-tuning at all.
			}
		}



		// PID control loop:
		for (int i=POD_FR; i<(int)POD_NUM; i++) {
			time_previous = time_current;
			time_current = Time_GetTime(TIMER_PROGRAM);
			time_difference = time_current-time_previous;
			current_encoder[i] = Motor_GetEncoder(Motor_Convert((Motor)i))/(float)(-2); // Encoders are geared up by 2 (and "backwards").
			current_encoder[i] = Math_Normalize(current_encoder[i], (float)1440, 360);
			current_encoder[i] = (float)(round(current_encoder[i])%360); // Value is now between -360 ~ 360.
			current_encoder[i] += 360; // Value is now >= 0.
			current_encoder[i] = (float)(round(current_encoder[i])%360); // Value is now between 0 ~ 360.
			error[i] = g_ServoData[i].angle-current_encoder[i];
			if (error[i]>180) {
				error[i] = error[i]-360;
			} else if (error[i]<-180) {
				error[i] = error[i]+360;
			} // TODO: Can the above chain be simplified to something having to do with modulo 180?
			//// This works, but doesn't eliminate the 180 deg turns. :?
			//if (error[i]>90) {
			//	error[i] = error[i]-180;
			//	g_MotorData[i].isReversed = true;
			//} else if (error[i]<90) {
			//	error[i] = error[i]+180;
			//	g_MotorData[i].isReversed = true;
			//} else {
			//	g_MotorData[i].isReversed = false;
			//} // TODO: Can the above chain be simplified to something having to do with modulo 90?
			Math_TrimDeadband(error[i], g_EncoderDeadband);
			for (int j=0; j<kI_delay; j++) {
				error_accumulated_total[i] += error_accumulated[i][j];
			}
			if (abs(error[i])>36) { //36 is an arbitrary number :P
				isAligned = ALIGNED_FAR;
			} else if (abs(error[i])>12) {
				isAligned = ALIGNED_MEDIUM;
			} else {
				isAligned = ALIGNED_CLOSE;
			}
			term_P[i] = kP[i]*error[i]; // kP might become an array
			term_I[i] = kI[i]*error_accumulated_total[i]; // TODO! Has timers :P
			term_D[i] = 0; // TODO! Has timers :P
			total_correction[i] = Math_Limit((term_P[i]+term_I[i]+term_D[i]), 128);
		}
		for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
			switch (isAligned) {
				case ALIGNED_FAR:
					g_MotorData[i].fineTuneFactor *= 0; // Zeroes motor power.
					break;
				case ALIGNED_MEDIUM:
					g_MotorData[i].fineTuneFactor *= 1/abs(error[i])*10; // Ranges from 28~83%.
					break;
				// Not checking the "ALIGNED_CLOSE" condition may increase performance.
				// If we were to check it, the assignment would look like this:
				// g_MotorData[i].fineTuneFactor *= 1; // Keeps it the same.
			}
		}



		// Assign the power settings to the motors (already parsed).
		for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
			if (g_MotorData[i].isReversed==true) {
				g_MotorData[i].power *= -1;
			}
			g_MotorData[i].power *= g_MotorData[i].fineTuneFactor;
			Motor_SetPower(g_MotorData[i].power, Motor_Convert((Motor)i));
		}

		// Assign the power settings to the servos. Can't make a loop yet, since one
		// of the assignments is different from the rest. >:(
		servo[servo_FR] = -total_correction[POD_FR]+128; // +128, because that's how continuous rotation servos work.
		servo[servo_FL] = -total_correction[POD_FL]+128;
		servo[servo_BL] = 128; // We don't have an encoder mounted for this servo... derp.
		//servo[servo_BL] = -total_correction[POD_BL]+128; // Uncomment this when we do mount one.
		servo[servo_BR] = -total_correction[POD_BR]+128;

		// Check if we should lock/release wheelpods.
		if (Joystick_ButtonPressed(BUTTON_A)==true) {
			isLocked = !isLocked;
		}
		if (isLocked==true) {
			Servo_SetPosition(servo_lock, lockedPosition);
		} else {
			Servo_SetPosition(servo_lock, unlockedPosition);
		}

		// Check if we want to gear motors down.
		if (Joystick_ButtonPressed(BUTTON_X)==true) {
			isLowGear = !isLowGear;
		}
		if (isLowGear==true) {
			Servo_SetPosition(servo_transmission, lowGearPosition);
		} else {
			Servo_SetPosition(servo_transmission, highGearPosition);
		}



		// Troubleshooting display:
		nxtDisplayTextLine(0, "FR set%d chg%d", g_ServoData[POD_FR].angle, total_correction[POD_FR]);
		nxtDisplayTextLine(1, "FL set%d chg%d", g_ServoData[POD_FL].angle, total_correction[POD_FL]);
		nxtDisplayTextLine(2, "BL set%d chg%d", g_ServoData[POD_BL].angle, total_correction[POD_BL]);
		nxtDisplayTextLine(3, "BR set%d chg%d", g_ServoData[POD_BR].angle, total_correction[POD_BR]);
		nxtDisplayTextLine(4, "FR encdr%d pow%d", current_encoder[POD_FR], g_MotorData[POD_FR].power);
		nxtDisplayTextLine(5, "FL encdr%d pow%d", current_encoder[POD_FL], g_MotorData[POD_FL].power);
		nxtDisplayTextLine(6, "BL encdr%d pow%d", current_encoder[POD_BL], g_MotorData[POD_BL].power);
		nxtDisplayTextLine(7, "BR encdr%d pow%d", current_encoder[POD_BR], g_MotorData[POD_BR].power);
		//// The following will display the "raw" encoder value of POD_FL, and
		//// you can see for yourself that it drifts. Ungood. Very ungood.
		//int ENCODER_RAW = Motor_GetEncoder(Motor_Convert((Motor)MOTOR_FL));
		//nxtDisplayCenteredBigTextLine(3, "RAW: %d", ENCODER_RAW);

		//Task_EndTimeslice(); // This was used when we had multiple tasks.



		// Check if "moo.rso" should be playing. (EASTER EGG!)
		if (Joystick_ButtonPressed(BUTTON_Y)==true) {
			isPlaying = !isPlaying;
		}
		if (isPlaying==true) {
			if (Sound_IsPlaying()==false) {
				Sound_Moo();
			}
		} else {
			if (Sound_IsPlaying()==true) {
				Sound_ClearQueue();
			}
		}
	}
}
