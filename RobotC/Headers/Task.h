#ifndef TASK_H
#define TASK_H
#pragma systemFile



// Very unsure about this `void` thing.
void Task_StartTask(void taskID, int priority=kDefaultTaskPriority);
void Task_SuspendTask(); //If I can cast an `int` to a `void` this function can be combined with the next.
void Task_SuspendTask(void taskID);
void Task_StopTask(); //If I can cast an `int` to a `void` this function can be combined with the next.
void Task_StopTask(void taskID);
void Task_StopAllTasks();
void Task_HogCPU();
void Task_ReleaseCPU();
void Task_EndTimeslice();
void Task_SetPriority(int priority);
void Task_SetPriority(void taskID, int priority);
int  Task_GetPriority(); //If I can cast an `int` to a `void` this function can be combined with the next.
int  Task_GetPriority(void taskID);
void Task_SetTimesliceSize(int size); //Source claims this is read-only...
int  Task_GetTimesliceSize();



#include "..\Libraries\Task.c"
#endif // TASK_H
