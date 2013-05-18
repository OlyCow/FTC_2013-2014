# Encapsulation Scheme (RobotC)

NOTE: The "natural language" advertised by RobotC is actually very limited.  
Its only useful function is the `void wait(float waitTime)` function.

## Original Functions/Variables
- __Motors__
    - `bool bFloatDuringInactiveMotorPWM` _(var)_
		- true is coast, false is brake
	- `bool bMotorReflected[tMotor motor]` _(var)_
		- whether that motor is reflected or not
	- `int motor[tMotor motor]` _(var)_
		- sets motor power level, from -100 to 100
	- `int nMaxRegulatedSpeedNXT` _(var)_
		- max speed for PID regulation (encoder counts/sec) 
		- default is 1000
	- `long nMotorEncoder[tMotor motor]` _(var)_
		- current value of encoder
		- wraps after about 90 turns
		- reset as often as possible to avoid overflow
	- `long nMotorEncoderTarget[tMotor motor]` _(var)_
		- after target is reached motor will stop, 0 means never stop
	- `TMotorRegulation nMotorPIDSpeedCtrl[tMotor motor]` _(var)_
		- whether PID is on; on by default
		- can be set in Motor & Sensor Setup
	- `TNxtRunState nMotorRunState[tMotor motor]` _(var)_
		- internal state of the motor:
			* 0x00 = `runStateIdle`
			* 0x10 = `runStateRampUp`
			* 0x20 = `runStateRunning`
			* 0x40 = `runStateRampDown`
			* 0x01 = `runStateHoldPosition`
	- `int nPidUpdateInterval` _(var)_
		- interval in ms to use for PID updates; default is 25
		- too short results in jerky motion
		- too high will seem unregulated (lag)
	- `TSynchedMotors nSyncedMotors` _(var)_
		- `=synchXY`, `X` is master, `Y` is slave
	- `int nSyncedTurnRatio` _(var)_
		- turn ratio for slave, from -100 to 100
- __Servos__
	- `short servoChangeRate[TServoIndex servo]` _(var)_
		- rate at which servo is changed, 0 is max speed
		- default is 10 positions/update
		- 20 milliseconds/update
	- `short servo[TServoIndex servo]` _(var)_
		- target position from 0 to 255
		- might take a couple updates to see effect
	- `short ServoValue[TServoIndex servo]` _(var)_
		- reads last position told to move to, not actual position
- __Sensors__
	- `word SensorRaw[tSensors sensor]` _(var)_
		- returns the un-normalized value, from 0 to 1023
	- `TSensorTypes SensorType[tSensors sensor]` _(var)_
		- sets the sensor type; try to use Sensor Setup instead
	- `word SensorValue[tSensors sensor]` _(var)_
		- returns the normalized/interpreted value
- __Joystick__
	- `getJoystickSettings(joystick)` _(fn)_
		- gets newest data from joysticks
		- joysticks send data every 50 to 100 ms
		- argument is always `joystick`
	- `TPCJoystick joy1_Buttons` _(var)_
		- returns a "bitmap" of which buttons are pressed
		- COMPLICATED mechanism I haven't figured out yet
		- involves two undocumented functions
	- `TPCJoystick joy1_TopHat` _(var)_
		- returns octant "direction pad" is in
		- -1 when nothing pressed, 0 to 7 for the octants
	- `TPCJoystick joy1_x1` _(var)_
		- value of x-axis on left joystick, from -128 to 127
	- `TPCJoystick joy1_y1` _(var)_
		- value of y-axis on left joystick, from -128 to 127
	- `TPCJoystick joy1_x2` _(var)_
		- value of x-axis on right joystick, from -128 to 127
	- `TPCJoystick joy1_y2` _(var)_
		- value of y-axis on right joystick, from -128 to 127
	- `TPCJoystick joy2_Buttons` _(var)_
		- returns a "bitmap" of which buttons are pressed
		- COMPLICATED mechanism I haven't figured out yet
		- involves two undocumented functions
	- `TPCJoystick joy2_TopHat` _(var)_
		- returns octant "direction pad" is in
		- -1 when nothing pressed, 0 to 7 for the octants
	- `TPCJoystick joy2_x1` _(var)_
		- value of x-axis on left joystick, from -128 to 127
	- `TPCJoystick joy2_y1` _(var)_
		- value of y-axis on left joystick, from -128 to 127
	- `TPCJoystick joy2_x2` _(var)_
		- value of x-axis on right joystick, from -128 to 127
	- `TPCJoystick joy2_y2` _(var)_
		- value of y-axis on right joystick, from -128 to 127
- __NXT Buttons__
	- `TButtons nNxtButtonPressed` _(var)_
		- which button is currently pressed, only one at a time
			* -1 = `kNoButton`
			* 0 = `kExitButton`
			* 1 = `kRightButton`
			* 2 = `kLeftButton`
			* 3 = `kEnterButton`
	- `word nNxtButtonTask` _(var)_
		- has something to do with the program processing buttons
		- (no idea what this actually does)
	- `word nNxtExitClicks` _(var)_
		- how many "Exit Button" clicks until abort program
- __File Access__
	- According to what I see on the API, this is pretty unnecessary.
- __Timing__
	- `void ClearTimer(TTimers theTimer)` _(fn)_
		- timers auto-start, always reset them; timer can be T1 to T4
	- `word nClockMinutes` _(var)_
		- a read/write var from 0 to 1439, wraps at 24 hours
	- `const long nPgmTime` _(var)_
		- program time; starts once program starts running
	- `const long nSysTime` _(var)_
		- system time; starts once NXT is turned on
	- `long time1[TTimers timer]` _(var)_
		- value of timer (T1 to T4) in units of 1 millisecond
	- `long time10[TTimers timer]` _(var)_
		- value of timer (T1 to T4) in units of 10 milliseconds
	- `long time100[TTimers timer]` _(var)_
		- value of timer (T1 to T4) in units of 100 milliseconds
	- `void wait1Msec(const long nMSec)` _(fn)_
		- does not consume CPU cycles
		- can be up to 32 milliseconds
		- nMSec is how many MSec to wait
	- `void wait10Msec(const long nTenMSec)` _(fn)_
		- does not consume CPU cycles
		- can be up to 320 milliseconds
		- nTenMSec is how many TenMSec to wait
- __Sound__
	- `word bPlaySounds` _(var)_
		- bool flag indicating whether new sounds should be accepted
		- `true` or `false` value
	- `const bool bSoundActive` _(var)_
		- flag indicating if system is playing a sound
	- `const bool bSoundQueueAvailable` _(var)_
		- flag indicating whether sound queue space is available
	- `void ClearSounds()` _(fn)_
		- clears existing and buffered sound commands
	- `void MuteSound()` _(fn)_
		- mutes all subsequent sound commands
	- `word nVolume` _(var)_
		- sets volume, from 0 to 4 (loudest)
	- `void PlayImmediateTone(const int x, const int y)` _(fn)_
		- _parameter clarification_:
			* "x" = `frequency`
			* "y" = `durationIn10MsecTicks`
		- plays tone ahead of other queued requests
	- `void PlaySound(TSounds sound)` _(fn)_
		- plays a system predefined sound
			* 0 = `soundBlip`
			* 1 = `soundBeepBeep`
			* 2 = `soundDownwardTones`
			* 3 = `soundUpwardTones`
			* 4 = `soundLowBuzz`
			* 5 = `soundFastUpwardTones`
			* 6 = `soundShortBlip`
			* 7 = `soundException`
			* 8 = `soundLowBuzzShort`
	- `void PlaySoundFile(const string &sFileName)` _(fn)_
		- plays a sound file present on the NXT file system
	- `void PlayTone(const int x, const int y)` _(fn)_
		- _parameter clarification_:
			* "x" = `frequency`
			* "y" = `durationIn10MsecTicks`
		- plays a constant tone
	- `void UnmuteSound()` _(fn)_
		- restores sound playback to volume before mute
- __Display__
- __Bluetooth__
- __Semaphores__
	- `void SemaphoreInitialize(TSemaphore semaphore)` _(fn)_
	- `void SemaphoreLock(TSemaphore semaphore, int wait)` _(fn)_
		- time to wait is (by default) about 32 ms (6FFF in hex)
	- `void SemaphoreUnlock(TSemaphore semaphore)` _(fn)_
	- `ubyte getSemaphoreTaskOwner(TSemaphore semaphore)` _(fn)_
		- returns the task that owns the semaphore
	- `bool bDoesTaskOwnSemaphore(TSemaphore semaphore)` _(fn)_
		- returns whether current task owns the semaphore
- __Multitasking__
	- `void abortTimeslice()` _(fn)_
		- immediately ends the current timeslice of the current task
		- timeslices are only used if multiple tasks have same priority
	- `void hogCPU()` _(fn)_
		- ignores priorities and gives current task 100% CPU time
	- `void releaseCPU()` _(fn)_
		- reverses effects of `void hogCPU()`
	- `void StartTask(void TaskID, const short nTaskPriority)` _(fn)_
		- starts a pre-defined task
		- default task priority is 7
	- `void StopTask(void TaskID)` _(fn)_
		- stops a previously started task
	- `void StopAllTasks()` _(fn)_
		- ends _all_ tasks, including `task main()`
	- `word nSchedulePriority` _(var)_
		- priority of a task from 0 to 255; 0 is lowest & 255 is highest
		- default for an assigned task is 7
- __Miscellaneous__
	- `bool bSystemLeaveServosEnabledOnProgramStop` _(var)_
		- leave servos locked in position until poweroff = `true`
	- `word muxUpdateInterval` _(var)_
		- amount of time (ms) between updates on TETRIX controller values
		- default is 25 ms
	- `short externalBattery` _(var)_
		- battery level of TETRIX battery in millivolts
	- `short externalBatteryAvg` _(var)_
		- battery level of TETRIX battery in millivolts
		- taken over an averge of 20 samplings

## Custom Functions/Variables
_These are all functions. Duh._
- __Motors__
	- `void Motor_Forward(tMotor motor_name, int power=75)`
	- `void Motor_Reverse(tMotor motor_name, int power=75)`
	- `void Motor_Stop(tMotor motor_name, bool brake=true)`
	- `void Motor_Target(tMotor motor_name, int angle)`
	- `int Motor_GetEncoder(tMotor motor_name)`
	- `void Motor_ResetEncoder(tMotor motor_name)`
		- reset is always absolute (since beginning of program)
	- `void Motor_SetBrakes(bool isOn=true)`
		- RobotC doesn't allow individual brakes to be set
	- `void Motor_SetMaxSpeed(int speed=750)`
	- `void Motor_SetPIDInterval(int interval=20)`
		- in milliseconds; RobotC's default value is 25.
- __Servos__
	- `void Servo_Rotate(TServoIndex servoName, short position)`
		- turns the servo to a position (0 to 255)
	- `short Servo_GetPosition(TServoIndex servoName)`
		- returns a value from 0 - 255
	- `void Servo_SetSpeed(TServoIndex servoName, int rate)`
		- rate is in positions updated per update
		- the brick sends 1 update per 20 milliseconds
		- 0 is top speed
	- `void Servo_LockPosition(TServoIndex s, bool isLocked=true)`
		- _parameter clarification_:
			* `TServoIndex s` is `TServoIndex servoName`
		- set whether servo holds position after poweroff
- __Sensors__
- __Joystick__
	- `void Joystick_UpdateData()`
	- `bool Joystick_Button(JoystickButton b, JoystickController c = C)`
	    - _parameter clarification_:
			* `JoystickButton b` is `JoystickButton button`
			* `JoystickController c` is `JoystickController controller`
			* `C` is `CONTROLLER_1`
	- `int JoyStick_Joystick(J...J... j, J...A... a, J...C... c = C)`
		- _parameter clarification_:
			* `J...J... j` is `JoystickJoystick joystick`
			* `J...A... a` is `JoystickAxis axis`
			* `J...C... c` is `JoystickController controller`
			* `C` is `CONTROLLER_1`
	- `JoystickDirection Joystick_Direction(JoystickController c = C)`
		- _parameter clarification_:
			* `c` is `controller`
			* `C` is `CONTROLLER_1`
- __NXT Buttons__
- __File Access__
- __Timing__
	- `void Time_Wait(int milliseconds)`
- __Sound__
- __Display__
- __Bluetooth__
- __Semaphores__
	- `void Semaphore_Initialize(TSemaphore semaphore)`
	- `void Semaphore_Lock(TSemaphore semaphore, int wait)`
		- wait time defaults to 6FFF ms (about 32 ms)
	- `void Semaphore_Unlock(TSemaphore semaphore)`
	- `bool Semaphore_IsCurrentlyOwned(TSemaphore semaphore)`
	- `ubyte Semaphore_GetOwner(TSemaphore semaphore)`
		- returns the "taskID" of the task owning the semaphore
- __Multitasking__
	- `void Task_ReleaseTimeslice()`
		- timeslices are only used if tasks have same priorities
	- `void Task_StartTask(void taskID, short priority)`
	- `void Task_StopTask(void taskID)`
	- `void Task_HogCPU()`
	- `void Task_ReleaseCPU()`
	- `void Task_AbortAll()`
		- also aborts `task main()`
- __Math__
	- `int Math_ToLogarithmic(int input)`
		- for converting joystick values to logarithmic values
- __Miscellaneous__