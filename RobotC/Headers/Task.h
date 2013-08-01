#ifndef TASK_H
#define TASK_H
#pragma systemFile



// Very unsure about this `void` thing.
void Task_StartTask(void taskID, int priority=kDefaultTaskPriority);
void Task_SuspendTask();
void Task_SuspendTask(void taskID);
void Task_StopTask();
void Task_StopTask(void taskID);
void Task_StopAllTasks();
void Task_HogCPU();
void Task_ReleaseCPU();
void Task_EndTimeslice();
void Task_SetPriority(int priority);
void Task_SetPriority(void taskID, int priority);
int  Task_GetPriority();
int  Task_GetPriority(void taskID);
void Task_SetTimesliceSize();
int  Task_GetTimesliceSize();



#include "..\Libraries\Task.c"
#endif // TASK_H
