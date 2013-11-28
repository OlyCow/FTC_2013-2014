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

task main()
{
	Task_Kill(displayDiagnostics);
	Display_Clear();
	nxtDisplayTextLine(0, "Initializing vars...");
	initializeGlobalVariables(); // Defined in "initialize.h", this intializes all struct members.

	const int finish_delay = 10*1000; // MAGIC_NUM: milliseconds to delay when program ends.
	TFileHandle IO_handle;
	TFileIOResult IO_result;
	const string file_name = "_reset_data.txt";
	int file_size = 0;
	const int resetRange = 10; // MAGIC_NUM: How close the program resets each pod (deg).
	bool isResetting = true;
	// kP is always negative because the servos are geared (and reversed).
	float kP[POD_NUM] = {-1.0, -1.0, -1.0, -1.0}; // MAGIC_NUM: TODO: PID tuning.
	float term_P[POD_NUM]		= {0,0,0,0}; // (P-controller.)
	float rot_raw[POD_NUM]		= {0,0,0,0}; // Encoder value (it's geared down by 2!).
	float rot_error[POD_NUM]	= {0,0,0,0}; // Difference between set-point and measured value.

	// Read in all the values of the pods from a text file.
	nxtDisplayTextLine(1, "Recovering data...");
	OpenRead(IO_handle, IO_result, file_name, file_size); // TODO: Add more error handling.
	if (	(IO_result==ioRsltFileNotFound) ||
			(IO_result==ioRsltNoSpace) ||
			(IO_result==ioRsltNoMoreFiles) ) {
		nxtDisplayTextLine(2, "Data not found. :(");
		nxtDisplayTextLine(3, "Terminating.");
		Time_Wait(finish_delay);
		return; // The only way to break out of this function?
	} else if (IO_result!=ioRsltSuccess) {
		nxtDisplayCenteredTextLine(2, "Unknown error!");
		nxtDisplayTextLine(3, "Debugging needed.");
		nxtDisplayTextLine(4, "Terminating.");
		Time_Wait(finish_delay);
		return; // The only way to break out of this function?
	}
	for (int i=POD_FR; i<(int)POD_NUM; i++) {
		ReadShort(IO_handle, IO_result, g_ServoData[i].angle);
		g_ServoData[i].angle = -g_ServoData[i].angle; // We're going "backwards" to 0 (centering principle!).
	}
	Close(IO_handle, IO_result);

	// Press orange button to start resetting.
	nxtDisplayCenteredTextLine(2, "Press [ENTER] to");
	nxtDisplayCenteredTextLine(3, "continue.");
	while (Buttons_Released(NXT_BUTTON_YES)==true) {
		Buttons_UpdateData();
		Time_Wait(100); // MAGIC_NUM: Arbitrary LCD refresh delay.
	}

	// Start pod reset (basic P-controller).
	Display_Clear();
	nxtDisplayTextLine(0, "Recovering data...");
	nxtDisplayCenteredTextLine(1, "Press [ENTER] to");
	nxtDisplayCenteredTextLine(2, "continue.");
	nxtDisplayTextLine(3, "Resetting pods...");
	nxtDisplayCenteredTextLine(5, "|     |");
	nxtDisplayCenteredTextLine(7, "KEEP PROGRAM OPEN.");

	while (isResetting==true) {

		// If this isn't zero'd here, the loop will never break,
		// since `isResetting` is calculated with AND statements.
		isResetting = true;

		// Calculate the targets and error for each wheel pod.
		for (int i=POD_FR; i<(int)POD_NUM; i++) {
			rot_raw[i] = Motor_GetEncoder(Motor_Convert((Motor)i))/(float)(-2); // Encoders are geared up by 2 (and "backwards").
			rot_raw[i] = Math_Normalize(rot_raw[i], (float)1440, 360); // Encoders are 1440 CPR.
			rot_error[i] = g_ServoData[i].angle-rot_raw[i];
			term_P[i] = Math_Limit(kP[i]*rot_error[i], 128); // Because servos, not motors.
			isResetting = isResetting&&(abs(rot_error[i])<resetRange);
		}

		// Assign the power settings to the servos.
		Servo_SetPower(servo_FR, term_P[POD_FR]);
		Servo_SetPower(servo_FL, term_P[POD_FL]);
		Servo_SetPower(servo_BL, term_P[POD_BL]);
		Servo_SetPower(servo_BR, term_P[POD_BR]);

		// Update display.
		nxtDisplayCenteredTextLine(4, "%5d-%5d", -rot_error[POD_FL], -rot_error[POD_FR]);
		nxtDisplayCenteredTextLine(6, "%5d-%5d", -rot_error[POD_BL], -rot_error[POD_BR]);
	}

	// Press orange button to confirm jig has been applied.
	Display_Clear();
	nxtDisplayTextLine(0, "Resetting pods...");
	nxtDisplayCenteredTextLine(1, "%5d-%5d", -rot_error[POD_FL], -rot_error[POD_FR]);
	nxtDisplayCenteredTextLine(2, "|     |");
	nxtDisplayCenteredTextLine(3, "%5d-%5d", -rot_error[POD_BL], -rot_error[POD_BR]);
	nxtDisplayCenteredTextLine(4, "KEEP PROGRAM OPEN.");
	nxtDisplayCenteredTextLine(5, "Apply jig, then");
	nxtDisplayCenteredTextLine(6, "press [ENTER] to");
	nxtDisplayCenteredTextLine(7, "continue.");
	while (Buttons_Released(NXT_BUTTON_YES)==true) {
		Buttons_UpdateData();
		Time_Wait(100); // MAGIC_NUM: Arbitrary LCD refresh delay.
	}

	// Delete old pod position file, create a new one, and write 0's to it.
	Display_Clear();
	nxtDisplayCenteredTextLine(0, "%5d-%5d", -rot_error[POD_FL], -rot_error[POD_FR]);
	nxtDisplayCenteredTextLine(1, "|     |");
	nxtDisplayCenteredTextLine(2, "%5d-%5d", -rot_error[POD_BL], -rot_error[POD_BR]);
	nxtDisplayCenteredTextLine(3, "KEEP PROGRAM OPEN.");
	nxtDisplayCenteredTextLine(4, "Apply jig, then");
	nxtDisplayCenteredTextLine(5, "press [ENTER] to");
	nxtDisplayCenteredTextLine(6, "continue.");
	nxtDisplayTextLine(7, "Rewriting position...");
	Delete(file_name, IO_result);
	OpenWrite(IO_handle, IO_result, file_name, file_size);
	for (int i=POD_FR; i<(int)POD_NUM; i++) {
		WriteShort(IO_handle, IO_result, 0);
	}
	Close(IO_handle, IO_result);

	// Alert user that program has terminated, and delay before ending.
	Display_Clear();
	nxtDisplayCenteredTextLine(0, "|     |");
	nxtDisplayCenteredTextLine(1, "%5d-%5d", -rot_error[POD_BL], -rot_error[POD_BR]);
	nxtDisplayCenteredTextLine(2, "KEEP PROGRAM OPEN.");
	nxtDisplayCenteredTextLine(3, "Apply jig, then");
	nxtDisplayCenteredTextLine(4, "press [ENTER] to");
	nxtDisplayCenteredTextLine(5, "continue.");
	nxtDisplayTextLine(6, "Rewriting position...");
	nxtDisplayTextLine(7, "Finished.");
	Time_Wait(finish_delay);
}
