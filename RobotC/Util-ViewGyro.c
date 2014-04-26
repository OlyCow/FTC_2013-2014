#pragma config(Sensor, S4,     sensor_gyro,    sensorI2CCustomFastSkipStates9V)

#include "includes.h"

task main()
{
	bDisplayDiagnostics = false;
	float rotation = 0.0;
	float heading = 0.0;
	HTGYROstartCal(sensor_gyro); // Does this function actually work? ...
	Display_Clear();
	nxtDisplayTextLine(6, "Actual drift");
	nxtDisplayTextLine(7, "less than shown.");
	Time_ClearTimer(T1);

	while (true)
	{
		heading += ((float)Time_GetTime(T1))*((float)HTGYROreadRot(sensor_gyro))/((float)1000.0); // 1000 milliseconds per second.
		Time_ClearTimer(T1);
		nxtDisplayCenteredBigTextLine(3, "%+.2f", heading);
		Time_Wait(10);
	}
}
