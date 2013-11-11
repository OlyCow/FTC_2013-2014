#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTServo,  HTServo,  none,     none)
#pragma config(Sensor, S3,     sensor_IR,      sensorI2CCustomFastSkipStates9V)
#pragma config(Sensor, S4,     sensor_protoboard, sensorI2CCustomFastSkipStates9V)
#pragma config(Motor,  mtr_S1_C1_1,     motor_FR,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motor_FL,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motor_BL,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motor_BR,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     motor_sweeper, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     motor_lift,    tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C4_1,     motor_flag_L,  tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C4_2,     motor_flag_R,  tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S2_C1_1,    servo_FR,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    servo_FL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    servo_BL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_4,    servo_BR,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_5,    servo_funnel_L,       tServoStandard)
#pragma config(Servo,  srvo_S2_C1_6,    servo_funnel_R,       tServoStandard)
#pragma config(Servo,  srvo_S2_C2_1,    servo_dump,           tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    servo_flag,           tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    servo9,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo12,              tServoNone)

#include "includes.h"
#include "Teleop-Basic.h"
#include "subroutines.h"

//#define WILL_EXPLODE // Uncomment this line (Ctrl-Q) to prevent development code from compiling.
#ifdef WILL_EXPLODE
#warn "This code will explode!"
#endif

task PID(); // Sets CR-servos' power, wheel pod motors' power, and lift motor's power. Others set in main.
task CommLink(); // Reads/writes to the protoboard as tightly as possible.
task Display(); // A separate task for updating the NXT's LCD display.
task Autonomous(); // Ooooh.

//---------------- README!!! ------------------------------------------------>>
//     Set the robot up as follows: with the front (where the NXT is mounted)
// facing towards you, the side of the wheel pods with 3D-printed gears should
// face forwards. As defined in "enums.h", the wheel pods are "numbered": `FR`,
// `FL`, `BL`, and `BR` (going counterclockwise starting with `FR`).
//
//     The code is currently split into 3 loops, which will be split into their
// own tasks once they are completed. #1 Find the target angle, velocity, etc.
// #2 Adjust the current angle, velocity, etc. through a PID control. The PID
// loop isn't completely implemented yet; go ahead and do that! :) #3 Display
// data on the NXT screen for debugging purposes. Believe me, this is useful :)
//
// CONTROLS:	Controller_1, Joystick_R:	Translational movement.
//				Controller_1, Button_LB/RB:	Rotational movement.
//				Controller_1, Button_LT:	Stop motors (adjust pod direction).
//				Controller_1, Button_RT:	Fine-tune motors.
//				Controller_1, Button_Y:		Moo.
//
//     Troubleshooting: "FR/FL/BL/BR" refers to the wheel pod. "set" refers to
// the target angle for the servo, while "chg" refers to the correction force
// applied by the servo. "encdr" prints the reading of the encoder (normalized,
// to compensate for gearing), and "pow" is the power applied to the motor.
//--------------------------------------------------------------------------->>

bool isAutonomous = false;
bool isSweeping = false;
int f_angle_z = 0;
int f_angle_y = 0;
int f_angle_x = 0;
int f_pos_z = 0;
int f_pos_y = 0;
int f_pos_x = 0;
float power_FR = 0.0;
float power_FL = 0.0;
float power_BL = 0.0;
float power_BR = 0.0;
float power_sweeper = 0.0;
float power_lift = 0.0;
float power_flag = 0.0;
int lift_pos = 0;
int servo_funnel_L_pos = servo_funnel_L_open;
int servo_funnel_R_pos = servo_funnel_R_open;

task main() {
	initializeGlobalVariables(); // Defined in "initialize.h", this intializes all struct members.
	Task_Spawn(PID);
	Task_Spawn(CommLink);
	Task_Spawn(Display);

	// Not initializing these structs for now: once data starts coming in
	// from the controllers, all the members of these will get updated.
	vector2D rotation[POD_NUM];
	vector2D translation; // Not a struct because all wheel pods share the same values.
	vector2D combined[POD_NUM]; // The averaged values: angle is pod direction, magnitude is power.
	float combined_angle_prev[POD_NUM] = {90.0,90.0,90.0,90.0}; // Prevents atan2(0,0)=0 from resetting the wheel pods to 0. `90` starts facing forward.
	bool shouldNormalize = false; // Set if motor values go over 100. All wheel pod power will be scaled down.

	Joystick_WaitForStart();



	while (true) {
		Joystick_UpdateData();

		// A rotation vector is added to translation vector, and the resultant vector
		// is normalized. A differential analysis of the parametric equations of
		// each wheel pod confirms that the above algorithm works perfectly, despite
		// its apparent simplicity. Use of the Vector2D library makes some of this
		// slightly less efficient (there are some unnecessary update calculations)
		// but the benefit of increased readability is well worth it.
		translation.x = Joystick_GetTranslationX();
		translation.y = Joystick_GetTranslationY();
		Vector2D_UpdateRot(translation);
		for (int i=POD_FR; i<(int)POD_NUM; i++) {
			rotation[i].r = Joystick_GetRotationMagnitude();
			rotation[i].theta = g_MotorData[i].angleOffset+90; // The vector is tangent to the circle (+90 deg).
			Vector2D_UpdatePos(rotation[i]);
			Vector2D_Add(rotation[i], translation, combined[i]);
			//combined[i].x = -combined[i].x; // Flipping a sign can work wonders (sometimes).
			if (combined[i].r>g_FullPower) {
				shouldNormalize = true;
			}
			if ((combined[i].theta==0)&&(combined[i].r==0)) { // AND encoder is within however many turns
				combined[i].theta = combined_angle_prev[i];
				// No need to update `combined_angle_prev[i]` because it stays the same.
				Vector2D_UpdatePos(combined[i]); // This might be unnecessary.
			} else {
				combined_angle_prev[i] = combined[i].theta;
			}
			g_ServoData[i].angle = combined[i].theta;
		}

		// Normalize our motors' power values if a motor's power went above g_FullPower.
		if (shouldNormalize==true) {
			float originalMaxPower = g_FullPower; // If there was a false positive, this ensures nothing changes.
			for (int i=POD_FR; i<(int)POD_NUM; i++) {
				if (combined[i].r>originalMaxPower) {
					originalMaxPower = combined[i].r;
				}
			}
			for (int i=POD_FR; i<(int)POD_NUM; i++) {
				combined[i].r = Math_Normalize(combined[i].r, originalMaxPower, g_FullPower);
				Vector2D_UpdatePos(combined[i]); // This might be unnecessary.
			}
			shouldNormalize = false; // Reset this for the next iteration.
		}
		for (int i=POD_FR; i<(int)POD_NUM; i++) {
			g_MotorData[i].power = combined[i].r;
		}

		// Set our "fine-tune" factor (amount motor power is divided by).
		// Ideally, this should be made more intuitive. Maybe a single trigger = slow,
		// while holding both triggers stops movement? The `if... else if...` structure
		// is also a problem, since BUTTON_LT will take precedence over BUTTON_RT.
		if (Joystick_Button(BUTTON_LT)==true) {
			for (int i=POD_FR; i<(int)POD_NUM; i++) {
				g_MotorData[i].fineTuneFactor = 0; // Equivalent to zeroing motor power.
			}
		} else if (Joystick_Button(BUTTON_RT)==true) {
			for (int i=POD_FR; i<(int)POD_NUM; i++) {
				g_MotorData[i].fineTuneFactor = 0.2; // MAGIC_NUM.
			}
		} else {
			for (int i=POD_FR; i<(int)POD_NUM; i++) {
				g_MotorData[i].fineTuneFactor = 1; // Equivalent to not fine-tuning at all.
			}
		}

		// Second driver's lift controls are overridden by the first's. The first
		// driver can also lock the lift position by pressing the D-pad (L or R).
		if (Joystick_Direction(DIRECTION_F)==true) {
			lift_pos += 100;
		} else if (Joystick_Direction(DIRECTION_B)==true) {
			lift_pos -= 100;
		} else if ((Joystick_Direction(DIRECTION_L))||(Joystick_Direction(DIRECTION_R))!=true) {
			lift_pos += Math_Normalize(Math_TrimDeadband(Joystick_Joystick(JOYSTICK_R, AXIS_Y, CONTROLLER_2), g_JoystickDeadband), g_JoystickMax, g_FullPower);
			if (Joystick_DirectionPressed(DIRECTION_F, CONTROLLER_2)==true) {
				lift_pos = lift_pos_dump;
			}
			if (Joystick_DirectionPressed(DIRECTION_B, CONTROLLER_2)==true) {
				lift_pos = lift_pos_pickup;
			}
		}

		// On conflicting input, 2 cubes are dumped instead of 4.
		if ((Joystick_ButtonReleased(BUTTON_A))||(Joystick_ButtonReleased(BUTTON_A, CONTROLLER_2))==true) {
			dumpCubes(2); // MAGIC_NUM.
		} else if ((Joystick_ButtonReleased(BUTTON_B))||(Joystick_ButtonReleased(BUTTON_B, CONTROLLER_2))==true) {
			dumpCubes(4); // MAGIC_NUM.
		}

		// Only `CONTROLLER_2` can funnel cubes in.
		if (Joystick_ButtonPressed(BUTTON_LB, CONTROLLER_2)==true) {
			switch (servo_funnel_L_pos) {
				case servo_funnel_L_closed :
					servo_funnel_L_pos = servo_funnel_L_open;
					break;
				case servo_funnel_L_open :
					servo_funnel_L_pos = servo_funnel_L_closed;
					break;
				default:
					servo_funnel_L_pos = servo_funnel_L_closed;
					break;
			}
		}
		if (Joystick_ButtonPressed(BUTTON_RB, CONTROLLER_2)==true) {
			switch (servo_funnel_R_pos) {
				case servo_funnel_R_closed :
					servo_funnel_R_pos = servo_funnel_R_open;
					break;
				case servo_funnel_L_open :
					servo_funnel_R_pos = servo_funnel_R_closed;
					break;
				default:
					servo_funnel_R_pos = servo_funnel_R_closed;
					break;
			}
		}

		// Toggle autonomous mode when `BUTTON_START` is pressed on both controllers.
		if ((Joystick_ButtonReleased(BUTTON_START))&&(Joystick_ButtonReleased(BUTTON_START, CONTROLLER_2))==true) {
			switch (isAutonomous) {
				case true :
					isAutonomous = false;
					Task_Kill(Autonomous);
					break;
				case false :
					isAutonomous = true;
					Task_Spawn(Autonomous);
					break;
			}
		}

		// If the flag is already waving, add 3 more waves.
		if (Joystick_ButtonPressed(BUTTON_X, CONTROLLER_2)==true) {
			switch (f_isWavingFlag) {
				case true :
					f_waveNum += 3; // MAGIC_NUM.
					break;
				case false :
					waveFlag();
					break;
			}
		}
		if ((Joystick_ButtonPressed(BUTTON_Y))||(Joystick_ButtonPressed(BUTTON_Y, CONTROLLER_2))==true) {
			isSweeping = !isSweeping; // TODO: see if `= !isSweeping` can be replaced with `^=`.
		}

		// Set motor and servo values (lift motor is set in PID()):
		if (isSweeping==true) {
			power_sweeper = 100;
		} else {
			power_sweeper = 0;
		}
		Motor_SetPower(power_sweeper, motor_sweeper);
		Motor_SetPower(power_flag, motor_flag_L);
		Motor_SetPower(power_flag, motor_flag_R);
		Servo_SetPosition(servo_funnel_L, servo_funnel_L_pos);
		Servo_SetPosition(servo_funnel_R, servo_funnel_R_pos);
	}
}



task PID() {

	typedef enum Aligned {
		ALIGNED_FAR		= 0,
		ALIGNED_MEDIUM	= 1,
		ALIGNED_CLOSE	= 2,
	};

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
	float error_accumulated_total[POD_NUM] = {0,0,0,0}; // {FR, FL, BL, BR}
	float kP[POD_NUM] = {0.6,	0.6,	0.6,	0.6}; // TODO: PID tuning.
	float kI[POD_NUM] = {0.0,	0.0,	0.0,	0.0};
	float kD[POD_NUM] = {0.0,	0.0,	0.0,	0.0};
	float current_encoder[POD_NUM] = {0,0,0,0};
	float error[POD_NUM] = {0,0,0,0}; // Difference between set-point and measured value.
	float error_prev[POD_NUM] = {0,0,0,0}; // Easier than using the `error_accumulated` array, and prevents the case where that array is size <=1.
	float error_rate[POD_NUM] = {0,0,0,0};
	float term_P[POD_NUM] = {0,0,0,0};
	float term_I[POD_NUM] = {0,0,0,0};
	float term_D[POD_NUM] = {0,0,0,0};
	float total_correction[POD_NUM] = {0,0,0,0}; // Equals "term_P + term_I + term_D".
	Aligned isAligned = ALIGNED_CLOSE; // If false, cut motor power so that wheel pod can get aligned.

	Joystick_WaitForStart();

	while (true) {
		time_previous = time_current;
		time_current = Time_GetTime(TIMER_PROGRAM);
		time_difference = time_current-time_previous;
		// TEMPORARY!!!
		//gyro_angle += time_difference*SensorValue[S3];
		for (int i=POD_FR; i<(int)POD_NUM; i++) {
			current_encoder[i] = Motor_GetEncoder(Motor_Convert((Motor)i))/(float)(-2); // Encoders are geared up by 2 (and "backwards").
			current_encoder[i] = Math_Normalize(current_encoder[i], (float)1440, 360);
			current_encoder[i] = (float)(round(current_encoder[i])%360); // Value is now between -360 ~ 360.
			current_encoder[i] += 360; // Value is now >= 0.
			current_encoder[i] = (float)(round(current_encoder[i])%360); // Value is now between 0 ~ 360.
			error_prev[i] = error[i];
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
			error_accumulated_total[i] -= error_accumulated[i][kI_delay-1]; // Array indices.
			error_accumulated_total[i] += error_accumulated[i][0];
			for (int j=kI_delay-1; j>0; j--) { //`j=kI_delay-1` because we are dealing with array indices.
				error_accumulated[i][j] = error_accumulated[i][j-1];
			}
			error_accumulated[i][0] = error[i]*time_difference;
			error_rate[i] = (error[i]-error_prev[i])/time_difference;
			if (abs(error[i])>36) { //36 is an arbitrary number :P
				isAligned = ALIGNED_FAR;
			} else if (abs(error[i])>12) {
				isAligned = ALIGNED_MEDIUM;
			} else {
				isAligned = ALIGNED_CLOSE;
			}
			term_P[i] = kP[i]*error[i];
			term_I[i] = kI[i]*error_accumulated_total[i];
			term_D[i] = kD[i]*error_rate[i];
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
			}
		}
	}


		// Assign the power settings to the motors (already parsed).
		for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
			// The following line requires a PID loop on velocity, it seems.
			//g_MotorData[i].power += total_correction[i]/(float)(10); // Correcting for servo rotation (doesn't work yet).
			g_MotorData[i].power = Math_Limit(g_MotorData[i].power, 100);
			if (g_MotorData[i].isReversed==true) {
				g_MotorData[i].power *= -1;
			}
			g_MotorData[i].power *= g_MotorData[i].fineTuneFactor;
			Motor_SetPower(g_MotorData[i].power, Motor_Convert((Motor)i));
		}
		Motor_SetPower(power_lift, motor_lift);

		// Assign the power settings to the servos.
		Servo_SetPower(servo_FR, -total_correction[POD_FR]);
		Servo_SetPower(servo_FL, -total_correction[POD_FL]);
		Servo_SetPower(servo_BL, -total_correction[POD_BL]);
		Servo_SetPower(servo_BR, -total_correction[POD_BR]);
}



task CommLink() {
	Joystick_WaitForStart();

	while (true) {
		// Continuously update stuff.
	}
}



// Task for displaying stuff on the LCD screen.
task Display() {

	typedef enum DisplayMode {
		DISP_FCS,				// Default FCS screen.
		DISP_SWERVE_DEBUG,		// Encoders, target values, PID output, power levels.
		DISP_PID_DEBUG,			// Error, P-term, I-term, D-term.
		DISP_COMM_STATUS,		// Each line of each frame.
		DISP_ENCODERS,			// Raw encoder values (7? 8?).
		DISP_SENSORS,			// Might need to split this into two screens.
		DISP_SERVOS,			// Show each servo's position.
		DISP_TASKS,				// Which tasks are running.
		DISP_AUTONOMOUS_INFO,	// Misc. status info.
	};

	DisplayMode isMode = DISP_FCS;
	Joystick_WaitForStart();

	while (true) {
		switch (isMode) {
			case DISP_FCS :
				while (true) {
					bDisplayDiagnostics = true;
					if (Buttons_Released(NXT_BUTTON_L)==true) {
						isMode = DISP_COMM_STATUS;
						break;
					}
					if (Buttons_Released(NXT_BUTTON_R)==true) {
						isMode = DISP_SWERVE_DEBUG;
						break;
					}
				}
				break;
			case DISP_SWERVE_DEBUG :
				while (true) {
					bDisplayDiagnostics = false;
					if (Buttons_Released(NXT_BUTTON_L)==true) {
						isMode = DISP_FCS;
						break;
					}
					if (Buttons_Released(NXT_BUTTON_R)==true) {
						isMode = DISP_PID_DEBUG;
						break;
					}
					//// TODO: FOLLOWING CODE BLOCK NOT IMPLEMENTED ANYWHERE (COMPILE ERRORS)
					//// Troubleshooting display:
					//nxtDisplayTextLine(0, "FR set%d chg%d", g_ServoData[POD_FR].angle, total_correction[POD_FR]);
					//nxtDisplayTextLine(1, "FL set%d chg%d", g_ServoData[POD_FL].angle, total_correction[POD_FL]);
					//nxtDisplayTextLine(2, "BL set%d chg%d", g_ServoData[POD_BL].angle, total_correction[POD_BL]);
					//nxtDisplayTextLine(3, "BR set%d chg%d", g_ServoData[POD_BR].angle, total_correction[POD_BR]);
					//nxtDisplayTextLine(4, "FR encdr%d pow%d", current_encoder[POD_FR], g_MotorData[POD_FR].power);
					//nxtDisplayTextLine(5, "FL encdr%d pow%d", current_encoder[POD_FL], g_MotorData[POD_FL].power);
					//nxtDisplayTextLine(6, "BL encdr%d pow%d", current_encoder[POD_BL], g_MotorData[POD_BL].power);
					//nxtDisplayTextLine(7, "BR encdr%d pow%d", current_encoder[POD_BR], g_MotorData[POD_BR].power);
				}
				break;
			case DISP_PID_DEBUG :
				while (true) {
					bDisplayDiagnostics = false;
					if (Buttons_Released(NXT_BUTTON_L)==true) {
						isMode = DISP_SWERVE_DEBUG;
						break;
					}
					if (Buttons_Released(NXT_BUTTON_R)==true) {
						isMode = DISP_COMM_STATUS;
						break;
					}
				}
				break;
			case DISP_COMM_STATUS :
				while (true) {
					bDisplayDiagnostics = false;
					if (Buttons_Released(NXT_BUTTON_L)==true) {
						isMode = DISP_PID_DEBUG;
						break;
					}
					if (Buttons_Released(NXT_BUTTON_R)==true) {
						isMode = DISP_FCS;
						break;
					}
				}
				break;
		}
	}
}



task Autonomous() {

	isAutonomous = true;

	while (true) {
		if (isAutonomous==false) {
			break;
		}
		// TODO: Do stuff. I have no idea how.
	}

	isAutonomous = false;
	Task_Kill(Autonomous);
}
