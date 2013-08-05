#ifndef TASK_C
#define TASK_C
#pragma systemFile
#include "..\Headers\Task.h"
// For default values, see above header file.



// Very unsure about this `void` thing.
void Task_Spawn(void taskID, int priority) {
	StartTask(taskID, priority);
}
void Task_Suspend() {
	suspendTask(nCurrentTask); //doing this may not be legal, since suspendTask expects a `void`
}
void Task_Suspend(void taskID) {
	suspendTask(taskID);
}
void Task_Stop() {
	StopTask(nCurrentTask); //doing this may not be legal, since StopTask expects a `void`
}
void Task_Stop(void taskID) {
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
void Task_SetPriority(int priority) {
	setTaskPriority(nCurrentTask, priority); //doing this may not be legal, since setTaskPriority expects a `void`
}
void Task_SetPriority(void taskID, int priority) {
	setTaskPriority(taskID, priority);
}
int  Task_GetPriority() {
	return getTaskPriority(nCurrentTask); //doing this may not be legal, since getTaskPriority expects a `void`
}
int  Task_GetPriority(void taskID) {
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
