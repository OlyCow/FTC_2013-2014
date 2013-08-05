#ifndef TASK_H
#define TASK_H
#pragma systemFile



// Very unsure about this `void` thing.
void Task_Spawn(short taskID, int priority=kDefaultTaskPriority);
void Task_Suspend(short taskID=nCurrentTask);
void Task_Stop(short taskID=nCurrentTask);
void Task_StopAll();
void Task_HogCPU();
void Task_ReleaseCPU();
void Task_EndTimeslice();
void Task_SetPriority(int priority);
void Task_SetPriority(short taskID, int priority);
int  Task_GetPriority(short taskID=nCurrentTask);
void Task_SetTimesliceSize(int size); //Source claims this is read-only...
int  Task_GetTimesliceSize();
void Task_NewSemaphore(TSemaphore semaphore);
void Task_LockSemaphore(TSemaphore semaphore, int waitTime);
void Task_UnlockSemaphore(TSemaphore semaphore);
ubyte Task_SemaphoreOwner(TSemaphore semaphore); //Not sure what the return type for this should be...




#include "..\Libraries\Task.c"
#endif // TASK_H
