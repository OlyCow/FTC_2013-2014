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
#include "subroutines.h"

task PID(); // Sets CR-servos' power, wheel pod motors' power, and lift motor's power. Others set in main.
task Display(); // A separate task for updating the NXT's LCD display.

// For PID:
float term_P_pod[POD_NUM] = {0,0,0,0};
float pod_current[POD_NUM] = {0,0,0,0};
float pod_raw[POD_NUM] = {0,0,0,0};
float error_pod[POD_NUM] = {0,0,0,0}; // Difference between set-point and measured value.
float correction_pod[POD_NUM] = {0,0,0,0}; // Equals "term_P + term_I + term_D".



task main()
{
	initializeGlobalVariables(); // Defined in "initialize.h", this intializes all struct members.
	Task_Spawn(PID);
	Task_Spawn(Display);

	const int maxTurns = 2; // On each side. To prevent the wires from getting too twisted.

	// Read data from file.

	// Set `g_ServoData[i].angle` to that number.

	for (int i=POD_FR; i<(int)POD_NUM; i++) {
		g_ServoData[i].angle = 0; // TODO: Assign values here.
	}
}



task PID()
{
	// Variables for wheel pod PID calculations.
	float kP[POD_NUM] = {1.0,	1.0,	1.0,	1.0}; // MAGIC_NUM: TODO: PID tuning.
	const int turnLimit = 3; // On each side. To prevent the wires from getting too twisted.

	Joystick_WaitForStart();

	while (true) {
		// Update timer first, in case something happens later during the loop.
		t_prev = t_current;
		t_current = Time_GetTime(TIMER_PROGRAM);
		t_delta = t_current-t_prev;

		// Calculate the targets and error for each wheel pod.
		for (int i=POD_FR; i<(int)POD_NUM; i++) {
			pod_raw[i] = Motor_GetEncoder(Motor_Convert((Motor)i))/(float)(-2); // Encoders are geared up by 2 (and "backwards").
			pod_raw[i] = Math_Normalize(pod_raw[i], (float)1440, 360); // Encoders are 1440 CPR.
			pod_current[i] = (float)(round(pod_raw[i])%360); // Value is now between -360 ~ 360.
			pod_current[i] += 360; // Value is now >= 0 (between 0 ~ 720).
			pod_current[i] = (float)(round(pod_current[i])%360); // Value is now between 0 ~ 360.
			error_prev_pod[i] = error_pod[i];
			error_pod[i] = g_ServoData[i].angle-pod_current[i];

			// Make sure we don't hit the maximum turning limit:
			if (error_pod[i]+pod_raw[i]>turnLimit*360) {
				//TODO: Add even more limits so if the pods get off >90deg, bad things don't happen.
				error_pod[i] = error_pod[i]-180;
				g_MotorData[i].isReversed = (!g_MotorData[i].isReversed);
			} else if (error_pod[i]+pod_raw[i]<turnLimit*(-360)) {
				//TODO: Add even more limits so if the pods get off >90deg, bad things don't happen.
				error_pod[i] = error_pod[i]+180;
				g_MotorData[i].isReversed = (!g_MotorData[i].isReversed);
			}

			term_P_pod[i] = kP[i]*error_pod[i];
			correction_pod[i] = Math_Limit(term_P_pod[i], 128); // Because servos, not motors.
		}

		// Assign the power settings to the servos.
		Servo_SetPower(servo_FR, -correction_pod[POD_FR]);
		Servo_SetPower(servo_FL, -correction_pod[POD_FL]);
		Servo_SetPower(servo_BL, -correction_pod[POD_BL]);
		Servo_SetPower(servo_BR, -correction_pod[POD_BR]);
	}
}



// Task for displaying data on the NXT's LCD screen.
// TODO: Put a lot of the display stuff into loops. Do we want to?
task Display()
{
	typedef enum DisplayMode {
		DISP_FCS,				// Default FCS screen.
		DISP_SWERVE_DEBUG,		// Encoders, target values, PID output, power levels.
		DISP_SWERVE_PID,		// Error, P-term, I-term, D-term.
		DISP_ENCODERS,			// Raw encoder values (7? 8?).
		DISP_NUM,
	};

	DisplayMode isMode = DISP_FCS;

	Joystick_WaitForStart();

	while (true) {
		Buttons_UpdateData();

		switch (isMode) {
			case DISP_FCS :
				break;
			case DISP_SWERVE_DEBUG :
				// The value of `pod_current[i]` is (should be?) between 0~360.
				nxtDisplayTextLine(0, "FR rot%3d tgt%3d", pod_current[POD_FR], g_ServoData[POD_FR].angle);
				nxtDisplayTextLine(1, "FL rot%3d tgt%3d", pod_current[POD_FL], g_ServoData[POD_FL].angle);
				nxtDisplayTextLine(2, "BL rot%3d tgt%3d", pod_current[POD_BL], g_ServoData[POD_BL].angle);
				nxtDisplayTextLine(3, "BR rot%3d tgt%3d", pod_current[POD_BR], g_ServoData[POD_BR].angle);
				nxtDisplayTextLine(4, " chg%+4d pow%+4d", correction_pod[POD_FR], g_MotorData[POD_FR].power);
				nxtDisplayTextLine(5, " chg%+4d pow%+4d", correction_pod[POD_FL], g_MotorData[POD_FL].power);
				nxtDisplayTextLine(6, " chg%+4d pow%+4d", correction_pod[POD_BL], g_MotorData[POD_BL].power);
				nxtDisplayTextLine(7, " chg%+4d pow%+4d", correction_pod[POD_BR], g_MotorData[POD_BR].power);
				break;
			case DISP_SWERVE_PID :
				nxtDisplayTextLine(0, "FR err%d P:%d", error_pod[POD_FR], term_P_pod[POD_FR]);
				nxtDisplayTextLine(1, "FL err%d P:%d", error_pod[POD_FL], term_P_pod[POD_FL]);
				nxtDisplayTextLine(2, "BL err%d P:%d", error_pod[POD_BL], term_P_pod[POD_BL]);
				nxtDisplayTextLine(3, "BR err%d P:%d", error_pod[POD_BR], term_P_pod[POD_BR]);
				nxtDisplayTextLine(4, "FR I:%d D:%d", term_I_pod[POD_FR], term_D_pod[POD_FR]);
				nxtDisplayTextLine(5, "FL I:%d D:%d", term_I_pod[POD_FL], term_D_pod[POD_FL]);
				nxtDisplayTextLine(6, "BL I:%d D:%d", term_I_pod[POD_BL], term_D_pod[POD_BL]);
				nxtDisplayTextLine(7, "BR I:%d D:%d", term_I_pod[POD_BR], term_D_pod[POD_BR]);
				break;
			default :
				nxtDisplayCenteredTextLine(3, "This debug screen");
				nxtDisplayCenteredTextLine(4, "does not exist.");
				break;
		}

		if (Buttons_Released(NXT_BUTTON_L)==true) {
			Display_Clear();
			isMode = (DisplayMode)((isMode+DISP_NUM-1)%DISP_NUM);
			if (isMode==DISP_FCS) {
				bDisplayDiagnostics = true;
			} else {
				bDisplayDiagnostics = false;
			}
		}
		if (Buttons_Released(NXT_BUTTON_R)==true) {
			Display_Clear();
			isMode = (DisplayMode)((isMode+DISP_NUM+1)%DISP_NUM);
			if (isMode==DISP_FCS) {
				bDisplayDiagnostics = true;
			} else {
				bDisplayDiagnostics = false;
			}
		}
		Time_Wait(100); // MAGIC_NUM: Prevents the LCD from updating itself to death. (Okay, maybe not that dramatic.)
	}
}
