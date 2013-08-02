#ifndef TASK_H
#define TASK_H
#pragma systemFile



// Very unsure about this `void` thing.
void Task_Spawn(void taskID, int priority=kDefaultTaskPriority);
void Task_Suspend(); //If I can cast an `int` to a `void` this function can be combined with the next.
void Task_Suspend(void taskID);
void Task_Stop(); //If I can cast an `int` to a `void` this function can be combined with the next.
void Task_Stop(void taskID);
void Task_StopAll();
void Task_HogCPU();
void Task_ReleaseCPU();
void Task_EndTimeslice();
void Task_SetPriority(int priority);
void Task_SetPriority(void taskID, int priority);
int  Task_GetPriority(); //If I can cast an `int` to a `void` this function can be combined with the next.
int  Task_GetPriority(void taskID);
void Task_SetTimesliceSize(int size); //Source claims this is read-only...
int  Task_GetTimesliceSize();
void Task_NewSemaphore(TSemaphore semaphore);
void Task_LockSemaphore(TSemaphore semaphore, int waitTime);
void Task_UnlockSemaphore(TSemaphore semaphore);
ubyte Task_SemaphoreOwner(TSemaphore semaphore); //Not sure what the return type for this should be...




#include "..\Libraries\Task.c"
#endif // TASK_H
