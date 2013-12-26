#ifndef TIME_C
#define TIME_C
#pragma systemFile
#include "..\Headers\Time.h"
// For default values, see above header file.



void Time_ClearTimer(int &timer) {
	timer = nPgmTime;
}
void Time_ClearTimer(TTimers timer) {
	ClearTimer(timer);
}
void Time_ClearAllTimers() {
	for (int i=T1; i<g_TimerNumber; i++) {
		ClearTimer(i);
	}
}
void Time_SetTime(int &timer, int milliseconds) {
	timer = nPgmTime-milliseconds;
}
int  Time_GetTime(int timer) {
	return (nPgmTime - timer);
}
int  Time_GetTime(TTimers timer) {
	return time1[timer];
}
int  Time_GetTime(TimerType type) {
	int time=-1;
	switch (type) {
		case TIMER_CLOCK:
			time = nClockMinutes; //TODO: will this overflow an int if converted to milliseconds?
			break;
		case TIMER_SYSTEM:
			time = nSysTime;
			break;
		case TIMER_PROGRAM:
			time = nPgmTime;
			break;
	}
	return time;
}
void Time_Freeze(int opcodes) {
	for (int i=0; i<opcodes; i++) {
		noOp();
	}
}



//---ClearTimer(TTimers timer)
//---nClockMinutes
//---nPgmTime
//---nSysTime
//---time1[], time10[], time100[]
//---wait1Msec()
//---wait10Msec()
//---sleep
//---noOp()
#endif // TIME_C
