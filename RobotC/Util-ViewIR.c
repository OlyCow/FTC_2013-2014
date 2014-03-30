#pragma config(Sensor, S3,     sensor_IR,      sensorI2CCustomFastSkipStates9V)

#include "includes.h"

task main()
{
	bDisplayDiagnostics = false;
	Display_Clear();

	int	IR_A = 0,
		IR_B = 0,
		IR_C = 0,
		IR_D = 0,
		IR_E = 0;

	while (true) {
		HTIRS2readAllACStrength(sensor_IR, IR_A, IR_B, IR_C, IR_D, IR_E);
		nxtDisplayCenteredBigTextLine(3, "%4d", IR_C);
		Time_Wait(50);
	}
}
