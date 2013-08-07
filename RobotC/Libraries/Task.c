#ifndef TASK_C
#define TASK_C
#pragma systemFile
#include "..\Headers\Task.h"
// For default values, see above header file.



void Task_Spawn(short taskID, int priority) {
	StartTask(taskID, priority);
}
void Task_Suspend(short taskID) {
	suspendTask(taskID);
}
void Task_Stop(short taskID) {
	StopTask(taskID);
}
void Task_StopAll() {
	StopAllTasks();
}
void Task_HogCPU() {
	hogCPU();
}
void Task_ReleaseCPU() {
	releaseCPU();
}
void Task_EndTimeslice() {
	abortTimeslice();
}
int  Task_GetCurrentIndex() {
	return nCurrentTask;
}
void Task_SetPriority(int priority) {
	setTaskPriority(nCurrentTask, priority);
}
void Task_SetPriority(short taskID, int priority) {
	setTaskPriority(taskID, priority);
}
int  Task_GetPriority(short taskID) {
	return getTaskPriority(taskID);
}
void Task_SetTimesliceSize(int size) {
	nOpcodesPerTimeslice = size; //may not be legal since nOpcodesPerTimeslice might be read-only :P
}
int  Task_GetTimesliceSize() {
	return nOpcodesPerTimeslice;
}
void Task_NewSemaphore(TSemaphore semaphore) {
	SemaphoreInitialize(semaphore);
}
void Task_LockSemaphore(TSemaphore semaphore, int waitTime) {
	SemaphoreLock(semaphore, waitTime);
}
void Task_UnlockSemaphore(TSemaphore semaphore) {
	SemaphoreUnlock(semaphore);
}
ubyte Task_SemaphoreOwner(TSemaphore semaphore) { //return type is really sketchy... Like, sketch, man.
	return getSemaphoreTaskOwner(semaphore);
}



//---SemaphoreInitialize(nSemaphore)
//---SemaphoreLock(nSemaphore, waitTime)
//---SemaphoreUnlock(nSemaphore)
//---getSemaphoreTaskOwner(nSemaphore)
//---bDoesTaskOwnSemaphore(nSemaphore)
//---abortTimeslice()
//---EndTimeSlice()
//---getTaskPriority(taskID)
//---hogCPU()
//---releaseCPU()
//---StartTask(taskID)
//---StartTask(taskID, priority)
//---StopAllTasks()
//---StopTask(taskID)
//---suspendTask(taskID) //internal use only?
//---bClearVariablesOnProgramStart //Not touching :P
//---nOpcodesPerTimeslice //YEAH
//---nSchedulePriority
//---kDefaultTaskPriority (7)
//---kLowPriority (0)
//---kHighPriority (255)
#endif // TASK_C
