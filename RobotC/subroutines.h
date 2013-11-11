#ifndef SUBROUTINES_H
#define SUBROUTINES_H
#include "includes.h"
#include "global vars.h"



void dumpCubes(int num=4);



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



#endif // SUBROUTINES_H
