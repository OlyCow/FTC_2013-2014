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
#pragma config(Motor,  mtr_S1_C4_1,     motor_flag_L,  tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     motor_flag_R,  tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S2_C1_1,    servo_FR,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    servo_FL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    servo_BL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_4,    servo_BR,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_5,    servo_dump,           tServoStandard)
#pragma config(Servo,  srvo_S2_C1_6,    servo_flag,           tServoStandard)
#pragma config(Servo,  srvo_S2_C2_1,    servo_funnel_L,       tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    servo_funnel_R,       tServoStandard)
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
	initializeGlobalVariables(); // Defined in "initialize.h", this intializes all struct members.

	const int finish_delay = 4*1000; // MAGIC_NUM: milliseconds to delay when program ends.
	TFileHandle IO_handle;
	TFileIOResult IO_result;
	const string filename = "_reset_pods.txt";
	const string filename_temp = "_reset_pods_tmp.txt"; // _temp seems to be too long of a file name??
	string filename_found = "";
	int file_size = 0;
	bool isConfirmed = false;

	// Confirm that we want to proceed (slightly dangerous; could lose data).
	nxtDisplayTextLine(0, "This writes a");
	nxtDisplayTextLine(1, "blank wheel pod");
	nxtDisplayTextLine(2, "position file.");
	nxtDisplayCenteredTextLine(3, "Press [ENTER] to");
	nxtDisplayCenteredTextLine(4, "continue.");
	while (Buttons_Released(NXT_BUTTON_YES)==false) {
		Buttons_UpdateData();
		Time_Wait(100); // MAGIC_NUM: Arbitrary LCD refresh delay.
	}

	// Not looping since we only have two files to deal with.
	// TODO: Reconsider looping? ^
	FindFirstFile(IO_handle, IO_result, filename, filename_found, file_size);
	if (IO_result==ioRsltSuccess) {
		nxtDisplayTextLine(5, "File exists.");
		nxtDisplayTextLine(6, "Press [ENTER] to");
		nxtDisplayTextLine(7, "overwrite.");
		do {
			Buttons_UpdateData();
			Time_Wait(100); // MAGIC_NUM: Arbitrary LCD refresh delay.
		} while (Buttons_Released(NXT_BUTTON_YES)==false);
		isConfirmed = true;
		Delete(filename, IO_result);
	} else if (IO_result==ioRsltFileNotFound) {
		FindFirstFile(IO_handle, IO_result, filename_temp, filename_found, file_size);
		if (IO_result==ioRsltSuccess) {
			nxtDisplayTextLine(5, "File exists.");
			nxtDisplayCenteredTextLine(6, "Press [ENTER] to");
			nxtDisplayCenteredTextLine(7, "overwrite.");
			do {
				Buttons_UpdateData();
				Time_Wait(100); // MAGIC_NUM: Arbitrary LCD refresh delay.
			} while (Buttons_Released(NXT_BUTTON_YES)==false);
			isConfirmed = true;
			Delete(filename_temp, IO_result);
		} else if (IO_result==ioRsltFileNotFound) {
			isConfirmed = true;
		} else {
			// TODO: More (better?) error handling.
			nxtDisplayTextLine(5, "Unknown err.!");
			nxtDisplayTextLine(6, "Terminating.");
			isConfirmed = false; // Already initialized this; just making sure.
		}
	} else {
		// TODO: More (better?) error handling.
		nxtDisplayTextLine(5, "Unknown err.!");
		nxtDisplayTextLine(6, "Terminating.");
		isConfirmed = false; // Already initialized this; just making sure.
	}
	if (isConfirmed==true) {
		file_size = 72; // 4 shorts, ea. 16-bits. I think? There's some buffer just in case.
		OpenWrite(IO_handle, IO_result, filename, file_size);
		for (int i=POD_FR; i<(int)POD_NUM; i++) {
			WriteShort(IO_handle, IO_result, 0);
		}
		Close(IO_handle, IO_result);
		Display_Clear();
		nxtDisplayTextLine(1, "File written to");
		nxtDisplayTextLine(2, "\"_reset_pods.txt\"");
		nxtDisplayCenteredTextLine(4, "Press [ENTER] to");
		nxtDisplayCenteredTextLine(5, "end program.");
		do {
			Buttons_UpdateData();
			Time_Wait(100); // MAGIC_NUM: Arbitrary LCD refresh delay.
		} while (Buttons_Released(NXT_BUTTON_YES)==false);
		nxtDisplayTextLine(7, "Terminating.");
	}
	Time_Wait(finish_delay);
}
