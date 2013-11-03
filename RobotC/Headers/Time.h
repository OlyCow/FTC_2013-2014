#ifndef TIME_H
#define TIME_H
#pragma systemFile



typedef enum TimerType {
	TIMER_CLOCK		= 0,
	TIMER_SYSTEM	= 1,
	TIMER_PROGRAM	= 2,
};



void Time_Wait(int milliseconds);
void Time_Sleep(int milliseconds);
void Time_Freeze(int opcodes=1);
void Time_ClearTimer(TTimers timer);
void Time_ClearAllTimers();
int  Time_GetTime(TTimers timer); //returns milliseconds
int  Time_GetTime(TimerType type=TIMER_PROGRAM); //TIMER_CLOCK is in minutes.



#include "..\Libraries\Time.c"
#endif // TIME_H
