#pragma config(Sensor, S1,     sensor_SuperPro, sensorI2CCustomFastSkipStates9V)

#include "includes.h"

float x = 0;
float time_delta = 0;

task display() {
	nxtDisplayCenteredTextLine(3, "----");
	bDisplayDiagnostics = false;

	while (true) {
		nxtDisplayCenteredTextLine(2, "ADC:%f", x);
		nxtDisplayCenteredTextLine(4, "Poll:%f", time_delta);
		Time_Wait(200);
	}
}

task main() {
	HTSPBsetupIO(sensor_SuperPro, 0x1);
	StartTask(display);

	while (true) {
		time_delta = time1[T1];
		ClearTimer(T1);
		x = HTSPBreadADC(sensor_SuperPro, 0, 8);
	}
}
