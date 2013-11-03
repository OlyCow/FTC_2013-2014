#ifndef TASK_H
#define TASK_H
#pragma systemFile



#define	Task_Spawn(taskID)				(StartTask(taskID))
#define	Task_PrioritySpawn(taskID, priority)	(StartTask(taskID, (priority))) //should rarely need this
#define	Task_Suspend(taskID)			(suspendTask(taskID))
#define	Task_Kill(taskID)				(StopTask(taskID))
#define	Task_KillAll()					(StopAllTasks())
#define	Task_HogCPU()					(hogCPU())
#define	Task_ReleaseCPU()				(releaseCPU())
#define	Task_EndTimeslice()				(abortTimeslice()) //equivalent to EndTimeSlice
#define	Task_GetCurrentIndex()			(nCurrentTask)
#define	Task_SetPriority(taskID, priority)	(setTaskPriority(taskID, priority))
#define	Task_SetCurrentPriority(priority)	(setTaskPriority(nCurrentTask, priority))
#define	Task_GetPriority(taskID)		(getTaskPriority(taskID))
#define	Task_GetCurrentPriority()		(getTaskPriority(nCurrentTask))
#define	Task_SetTimesliceSize(opcodes)	(nOpcodesPerTimeslice = opcodes) //Source claims this is read-only...
#define	Task_GetTimesliceSize()			(nOpcodesPerTimeslice)
#define	Task_NewSemaphore(semaphore)	(SemaphoreInitialize(semaphore)) //semaphore is of type TSemaphore
#define Task_LockSemaphore(semaphore, delay)	(SemaphoreLock(semaphore, delay)) //semaphore is of type TSemaphore
#define Task_UnlockSemaphore(semaphore)	(SemaphoreUnlock(semaphore)) //semaphore is of type TSemaphore
#define Task_SemaphoreOwner(semaphore)	(getSemaphoreTaskOwner(semaphore)) //semaphore is of type TSemaphore. Not sure what the return type for this should be... ubyte?




//void Task_Spawn(short taskID, int priority=skDefaultTaskPriority);
//void Task_Suspend(short taskID=nCurrentTask);
//void Task_Kill(short taskID=nCurrentTask);
//void Task_StopAll();
//void Task_HogCPU();
//void Task_ReleaseCPU();
//void Task_EndTimeslice();
//int  Task_GetCurrentIndex();
//void Task_SetPriority(int priority);
//void Task_SetPriority(short taskID, int priority);
//int  Task_GetPriority(short taskID=nCurrentTask);
//void Task_SetTimesliceSize(int size); //Source claims this is read-only...
//int  Task_GetTimesliceSize();
//void Task_NewSemaphore(TSemaphore semaphore);
//void Task_LockSemaphore(TSemaphore semaphore, int waitTime);
//void Task_UnlockSemaphore(TSemaphore semaphore);
//ubyte Task_SemaphoreOwner(TSemaphore semaphore); //Not sure what the return type for this should be...




#include "..\Libraries\Task.c"
#endif // TASK_H
