#ifndef SUBROUTINES_H
#define SUBROUTINES_H
#include "includes.h"
#include "global vars.h"



bool g_isWavingFlag = false;
int g_waveNum = 3;

void dumpCubes(int num=4);
void waveFlag(int waveNum=4);
task waveFlag();



void dumpCubes(int num) {
	const int short_delay = 0; // MAGIC_NUM: TODO. (milliseconds)
	const int long_delay = 0; // MAGIC_NUM: TODO. (milliseconds)
	Servo_SetPosition(servo_dump, servo_dump_open);
	if (num<4) {
		Time_Wait(short_delay);
	} else {
		Time_Wait(long_delay);
	}
	Servo_SetPosition(servo_dump, servo_dump_closed);
}

void waveFlag(int waveNum) {
	g_waveNum = waveNum;
	Task_Spawn(waveFlag);
}

task waveFlag() {
	g_isWavingFlag = true;

	// MAGIC_NUM: TODO.
	const int delay = 0;

	for (int i=0; i<g_waveNum; i++) {
		Servo_SetPosition(servo_flag, servo_flag_L);
		Time_Wait(delay);
		Servo_SetPosition(servo_flag, servo_flag_R);
		Time_Wait(delay);
	}
	Servo_SetPosition(servo_flag, servo_flag_M);
	Time_Wait(delay);

	g_isWavingFlag = false;
	Task_Kill(waveFlag);
}



#endif // SUBROUTINES_H
