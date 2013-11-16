#pragma config(Sensor, S3,     sensor_IR,      sensorI2CCustomFastSkipStates9V)
#pragma config(Sensor, S4,     sensor_gyro,    sensorI2CCustomFastSkipStates9V)

#include "includes.h"



task main()
{
	const float loopDelay = 10.0; // milliseconds.

	bDisplayDiagnostics = false;
	float rotation = 0.0;
	float heading = 0.0;
	float IR_dirA = 0.0;
	float IR_dirB = 0.0;
	float IR_dirC = 0.0;
	float IR_dirD = 0.0;
	float IR_dirE = 0.0;
	HTGYROstartCal(sensor_gyro);
	Time_Wait(50);
	HTIRS2setDSPMode(sensor_IR, DSP_1200);
	Display_Clear();
	Time_ClearTimer(T1);

	while (true)
	{
		rotation = HTGYROreadRot(sensor_gyro);
		heading += rotation*loopDelay/1000.0; // 1000 seconds per millisecond.
		nxtDisplayTextLine(1, "Rot: %f", heading*4);
		HTIRS2readAllACStrength(sensor_IR, IR_dirA, IR_dirB, IR_dirC, IR_dirD, IR_dirE);
		nxtDisplayTextLine(3, "IR-A: %f", IR_dirA);
		nxtDisplayTextLine(4, "IR-B: %f", IR_dirB);
		nxtDisplayTextLine(5, "IR-C: %f", IR_dirC);
		nxtDisplayTextLine(6, "IR-D: %f", IR_dirD);
		nxtDisplayTextLine(7, "IR-E: %f", IR_dirE);
		Time_Wait(loopDelay);
	}
}
