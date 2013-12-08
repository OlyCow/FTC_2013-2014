#ifndef TIME_H
#define TIME_H
#pragma systemFile



typedef enum TimerType {
	TIMER_CLOCK		= 0,
	TIMER_SYSTEM	= 1,
	TIMER_PROGRAM	= 2,
};



// The number of internal timers available to use.
const int g_TimerNumber = 4; // Defined internally as macro `kNumbOfTimers`.



#define Time_Wait(milliseconds) (wait1Msec(milliseconds))
void Time_ClearTimer(int &timer);
void Time_ClearTimer(TTimers timer);
void Time_ClearAllTimers();
void Time_SetTime(int &timer, int milliseconds);
int  Time_GetTime(int timer); // Returns milliseconds.
int  Time_GetTime(TTimers timer); // Returns milliseconds.
int  Time_GetTime(TimerType type=TIMER_PROGRAM); // `TIMER_CLOCK` is in minutes.
#define Time_Sleep(milliseconds) (Sleep(milliseconds))
void Time_Freeze(int opcodes=1);



#include "..\Libraries\Time.c"
#endif // TIME_H
