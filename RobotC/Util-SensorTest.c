#pragma config(Sensor, S3,     sensor_IR,      sensorI2CCustomFastSkipStates9V)
#pragma config(Sensor, S4,     sensor_gyro,    sensorI2CCustomFastSkipStates9V)

#include "includes.h"



task main()
{
	bDisplayDiagnostics = false;
	float rotation = 0.0;
	float heading = 0.0;
	int IR_dirA = 0.0;
	int IR_dirB = 0.0;
	int IR_dirC = 0.0;
	int IR_dirD = 0.0;
	int IR_dirE = 0.0;
	HTGYROstartCal(sensor_gyro); // Does this function actually work? ...
	Time_Wait(50); // <-unnecessary?
	HTIRS2setDSPMode(sensor_IR, DSP_1200);
	Display_Clear();
	Time_ClearTimer(T1);

	while (true)
	{
		heading += ((float)Time_GetTime(T1))*((float)HTGYROreadRot(sensor_gyro))/((float)1000.0); // 1000 milliseconds per second.
		Time_ClearTimer(T1);
		nxtDisplayTextLine(1, "Rot: %f", heading);
		HTIRS2readAllACStrength(sensor_IR, IR_dirA, IR_dirB, IR_dirC, IR_dirD, IR_dirE);
		nxtDisplayTextLine(3, "IR-A: %d", IR_dirA);
		nxtDisplayTextLine(4, "IR-B: %d", IR_dirB);
		nxtDisplayTextLine(5, "IR-C: %d", IR_dirC);
		nxtDisplayTextLine(6, "IR-D: %d", IR_dirD);
		nxtDisplayTextLine(7, "IR-E: %d", IR_dirE);
		Time_Wait(10);
	}
}
