#ifndef TASK_C
#define TASK_C
#pragma systemFile
#include "..\Headers\Task.h"
// For default values, see above header file.



// Very unsure about this `void` thing.
void Task_StartTask(void taskID, int priority) {
	StartTask(taskID, priority);
}
void Task_SuspendTask() {
	suspendTask(nCurrentTask); //doing this may not be legal, since suspendTask expects a `void`
}
void Task_SuspendTask(void taskID) {
	suspendTask(taskID);
}
void Task_StopTask() {
	StopTask(nCurrentTask); //doing this may not be legal, since StopTask expects a `void`
}
void Task_StopTask(void taskID) {
	StopTask(taskID);
}
void Task_StopAllTasks() {
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
