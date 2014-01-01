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

	const int finish_delay = 4*1000; // MAGIC_NUM: milliseconds to delay when program ends. // TODO: Figure out why the delay is nowhere near this.
	TFileHandle IO_handle;
	TFileIOResult IO_result;
	const string filename = "_reset_pods.txt";
	const string filename_temp = "_reset_pods_tmp.txt"; // temp makes the filename too long??
	int file_size = 0;
	short rotation[POD_NUM] = {0,0,0,0};


	// Read in all the values of the pods from a text file.
	OpenRead(IO_handle, IO_result, filename, file_size); // TODO: Add more error handling.
	if (IO_result==ioRsltSuccess) {
		nxtDisplayTextLine(0, "Read from file");
		nxtDisplayTextLine(1, "\"_reset_pods.txt\"");
	} else if (IO_result==ioRsltFileNotFound) {
		OpenRead(IO_handle, IO_result, filename_temp, file_size); // TODO: Add more error handling.
		if (IO_result==ioRsltSuccess) {
			nxtDisplayTextLine(0, "Read from file");
			nxtDisplayTextLine(1, "\"_pods_tmp.txt\"");
		} else if (	(IO_result==ioRsltFileNotFound)	||
					(IO_result==ioRsltNoSpace)		||
					(IO_result==ioRsltNoMoreFiles)	) {
			nxtDisplayTextLine(2, "No data file.");
			nxtDisplayTextLine(3, "Terminating.");
			Time_Wait(finish_delay);
			return; // The only way to break out of this function?
		}
	} else if ((IO_result==ioRsltNoSpace)||(IO_result==ioRsltNoMoreFiles)) {
		nxtDisplayTextLine(2, "No data file.");
		nxtDisplayTextLine(3, "Terminating.");
		Time_Wait(finish_delay);
		return; // The only way to break out of this function?
	}
	for (int i=POD_FR; i<(int)POD_NUM; i++) {
		ReadShort(IO_handle, IO_result, rotation[i]);
	}
	Close(IO_handle, IO_result);

	nxtDisplayTextLine(3, "FL:%5d FR:%5d", rotation[POD_FL], rotation[POD_FR]);
	nxtDisplayTextLine(4, "BL:%5d BR:%5d", rotation[POD_BL], rotation[POD_BR]);

	nxtDisplayCenteredTextLine(6, "Press [ENTER] to");
	nxtDisplayCenteredTextLine(7, "end program.");
	do {
		Buttons_UpdateData();
		Time_Wait(100); // MAGIC_NUM: Arbitrary LCD refresh delay.
	} while (Buttons_Released(NXT_BUTTON_YES)==false);
}
