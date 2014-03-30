#pragma config(Sensor, S3,     sensor_IR,      sensorI2CCustomFastSkipStates9V)
#pragma config(Sensor, S4,     sensor_gyro,    sensorI2CCustomFastSkipStates9V)

#include "includes.h"



task main()
{
	bDisplayDiagnostics = false;
	float rotation = 0.0;
	float heading = 0.0;
	HTGYROstartCal(sensor_gyro); // Does this function actually work? ...
	Time_Wait(50); // <-unnecessary?
	HTIRS2setDSPMode(sensor_IR, DSP_1200);
	Display_Clear();
	Time_ClearTimer(T1);
	nxtDisplayTextLine(7, "Actual drift less than shown.");

	while (true)
	{
		heading += ((float)Time_GetTime(T1))*((float)HTGYROreadRot(sensor_gyro))/((float)1000.0); // 1000 milliseconds per second.
		Time_ClearTimer(T1);
		nxtDisplayCenteredTextLine(3, "Rot: %f", heading);
		Time_Wait(10);
	}
}
