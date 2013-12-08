#ifndef SUBROUTINES_H
#define SUBROUTINES_H
#include "includes.h"
#include "global vars.h"



bool f_isWavingFlag = false;
int f_waveNum = 3;
int f_cubeDumpNum = 4;

void dumpCubes(int num=4);
task dumpCubesTask();
void waveFlag(int waveNum=4);
task waveFlagTask();



void dumpCubes(int num) {
	f_cubeDumpNum = num;
	Task_Spawn(dumpCubesTask);
}

task dumpCubesTask() {
	const int short_delay = 160; // MAGIC_NUM: TODO. (milliseconds)
	const int long_delay = 1600; // MAGIC_NUM: TODO. (milliseconds)
	Servo_SetPosition(servo_dump, servo_dump_open);
	if (f_cubeDumpNum<4) {
		Time_Wait(short_delay);
	} else {
		Time_Wait(long_delay);
	}
	Servo_SetPosition(servo_dump, servo_dump_closed);
}

void waveFlag(int waveNum) {
	f_waveNum = waveNum;
	Task_Spawn(waveFlagTask);
}

task waveFlagTask() {
	f_isWavingFlag = true;

	// MAGIC_NUM: TODO.
	const int delay = 0;

	for (int i=0; i<f_waveNum; i++) {
		Servo_SetPosition(servo_flag, servo_flag_L);
		Time_Wait(delay);
		Servo_SetPosition(servo_flag, servo_flag_R);
		Time_Wait(delay);
	}
	Servo_SetPosition(servo_flag, servo_flag_M);
	Time_Wait(delay);

	f_isWavingFlag = false;
	Task_Kill(waveFlagTask);
}



#endif // SUBROUTINES_H
