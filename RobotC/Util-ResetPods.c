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



float term_P[POD_NUM]		= {0,0,0,0}; // (P-controller.)
float rot_raw[POD_NUM]		= {0,0,0,0}; // Encoder value (it's geared down by 2!).
float rot_error[POD_NUM]	= {0,0,0,0}; // Difference between set-point and measured value.

task Display(); // A separate task for updating the NXT's LCD display.



task main()
{
	initializeGlobalVariables(); // Defined in "initialize.h", this intializes all struct members.
	Task_Spawn(Display);

	// Variables for file I/O operations.
	TFileHandle IO_handle;
	TFileIOResult IO_result;
	const string file_name = "_reset_data.txt";
	int file_size = 0;

	// Variables for PID calculations.
	float kP[POD_NUM] = {1.0,	1.0,	1.0,	1.0}; // MAGIC_NUM: TODO: PID tuning.

	OpenRead(IO_handle, IO_result, file_name, file_size);
	for (int i=POD_FR; i<(int)POD_NUM; i++) {
		ReadShort(IO_handle, IO_result, g_ServoData[i].angle); // TODO: Assign values here.
	}
	Close(IO_handle, IO_result);

	// Press orange button to start resetting.

	while (true) {
		// Calculate the targets and error for each wheel pod.
		for (int i=POD_FR; i<(int)POD_NUM; i++) {
			rot_raw[i] = Motor_GetEncoder(Motor_Convert((Motor)i))/(float)(-2); // Encoders are geared up by 2 (and "backwards").
			rot_raw[i] = Math_Normalize(rot_raw[i], (float)1440, 360); // Encoders are 1440 CPR.
			rot_error[i] = g_ServoData[i].angle-rot_raw[i];
			term_P[i] = Math_Limit(kP[i]*rot_error[i], 128); // Because servos, not motors.
		}

		// Assign the power settings to the servos.
		Servo_SetPower(servo_FR, -term_P[POD_FR]);
		Servo_SetPower(servo_FL, -term_P[POD_FL]);
		Servo_SetPower(servo_BL, -term_P[POD_BL]);
		Servo_SetPower(servo_BR, -term_P[POD_BR]);
	}

	// Press orange button to confirm jig has been applied.

	// Delete old pod position file.

	// Write new pod position file. (All 0's).
}



// Task for displaying data on the NXT's LCD screen.
// TODO: Put a lot of the display stuff into loops. Do we want to?
task Display()
{
	typedef enum DisplayMode {
		DISP_FCS,				// Default FCS screen.
		DISP_PID,
		DISP_NUM,
	};

	DisplayMode isMode = DISP_FCS;

	Joystick_WaitForStart();

	while (true) {
		Buttons_UpdateData();

		switch (isMode) {
			case DISP_FCS :
				break;
			case DISP_PID :
				// The value of `pod_current[i]` is (should be?) between 0~360.
				nxtDisplayTextLine(0, "FR encdr%3d P%3d", pod_current[POD_FR], g_ServoData[POD_FR].angle);
				nxtDisplayTextLine(1, "FL encdr%3d P%3d", pod_current[POD_FL], g_ServoData[POD_FL].angle);
				nxtDisplayTextLine(2, "BL encdr%3d P%3d", pod_current[POD_BL], g_ServoData[POD_BL].angle);
				nxtDisplayTextLine(3, "BR encdr%3d P%3d", pod_current[POD_BR], g_ServoData[POD_BR].angle);
				nxtDisplayCenteredTextLine(4, "err%+4d", correction_pod[POD_FR], g_MotorData[POD_FR].power);
				nxtDisplayCenteredTextLine(5, "err%+4d", correction_pod[POD_FL], g_MotorData[POD_FL].power);
				nxtDisplayCenteredTextLine(6, "err%+4d", correction_pod[POD_BL], g_MotorData[POD_BL].power);
				nxtDisplayCenteredTextLine(7, "err%+4d", correction_pod[POD_BR], g_MotorData[POD_BR].power);
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
