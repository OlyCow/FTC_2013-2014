#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTServo)
#pragma config(Hubs,  S2, HTServo,  HTServo,  HTMotor,  HTMotor)
#pragma config(Sensor, S3,     sensor_IR,      sensorI2CCustomFastSkipStates9V)
#pragma config(Sensor, S4,     sensor_protoboard, sensorI2CCustom9V)
#pragma config(Motor,  motorA,          motor_assist_R, tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          motor_assist_L, tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     motor_lift_front, tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motor_lift_back, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     motor_BR,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motor_FR,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C3_1,     motor_FL,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C3_2,     motor_sweeper, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C4_1,     motor_flag,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C4_2,     motor_BL,      tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S1_C3_1,    servo_BR,             tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    servo_FR,             tServoStandard)
#pragma config(Servo,  srvo_S1_C3_3,    servo_flip_R,         tServoStandard)
#pragma config(Servo,  srvo_S1_C3_4,    servo_climb_R,        tServoStandard)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoStandard)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoStandard)
#pragma config(Servo,  srvo_S1_C4_1,    servo_dump,           tServoStandard)
#pragma config(Servo,  srvo_S1_C4_2,    servo_auton,          tServoStandard)
#pragma config(Servo,  srvo_S1_C4_3,    servo9,               tServoStandard)
#pragma config(Servo,  srvo_S1_C4_4,    servo10,              tServoStandard)
#pragma config(Servo,  srvo_S1_C4_5,    servo_flip_L,         tServoStandard)
#pragma config(Servo,  srvo_S1_C4_6,    servo_FL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_1,    servo_BL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    servo_climb_L,        tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    servo15,              tServoStandard)
#pragma config(Servo,  srvo_S2_C1_4,    servo16,              tServoStandard)
#pragma config(Servo,  srvo_S2_C1_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo12,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_1,    servo_FL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    servo_BL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    servo_flip_L,         tServoStandard)
#pragma config(Servo,  srvo_S2_C2_4,    servo_climb_L,        tServoStandard)
#pragma config(Servo,  srvo_S2_C2_5,    servo17,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo18,              tServoNone)

#include "includes.h"
#include "swerve-drive.h"

#define WILL_EXPLODE // Uncomment this line to prevent development code from compiling.
#ifdef WILL_EXPLODE
#warn "This code will explode!"
#endif

task PID(); // Sets winch servos' position, wheel pod motors' power, and lift motor's power. Others set in main.
task CommLink(); // R/W to the prototype board as tightly as possible. TODO: Separate into its own library.
task Display(); // Updates the NXT's LCD display with useful info.
task TimedOperations(); // Anything depending on match time (release climbing, etc.).

//---------------- README!!! ------------------------------------------------>>
//     As defined in "enums.h", the wheel pods are "numbered": `FR`, `FL`,
// `BL`, and `BR` (going counterclockwise starting with `FR`).
//
//     The code is split into a couple tasks. I.) `main` does most of the high-
// level logic processing (mostly controller input), and sets targets for the
// PID task (motors and continuous rotation servos). If the power assignment is
// trivial (e.g. the sweeper motor) it is done directly in the `main` loop.
// II.) The `PID` loop currently runs a very simple PID loop monitoring lift
// position, and is hard-coded to never allow the lift to go below 0. It also
// runs a more complex PID loop for the wheel pods' continuous rotation servos,
// which limits them to a certain amount of turns in each direction so that the
// motor wires don't get all twisted up. III.) `CommLink` is how data is trans-
// ferred between the SuperPro prototype board and the AVR(s?) we have. This is
// BLACK MAGIC, DO NOT TOUCH. In the future we will want to optimize it, and
// possibly move it into its own library. IV.) `Display` is a cyclical display
// that provides valuable debugging information. Press the arrow buttons to go
// to a different screen. V.) This is an easter egg I'll probably never get to
// implement. :P It would basically be an autonomous teleop period.
//
// CONTROLS:	Controller_1, Joystick_R:	Translational movement.
//				Controller_1, Joystick_L:	Rotational movement.
//				Controller_1, Button_Joy_R:	Reset gyro.
//				Controller_1, Button_LT*:	Cut motor power (adjust pods).
//				Controller_1, Button_RT*:	Fine-tune motors.
//				Controller_1, Button_LB:	Dump 4 cubes.
//				Controller_1, Button_RB:	Dump 1 cube.
//				Controller_1, Button_A:		Toggle sweeper state.
//				Controller_1, Button_B:		Reset gyro (eventually flag).
//				Controller_1, Button_X:		Flag (eventually climb down).
//				Controller_1, Button_Y:		Climb (eventually climb up).
//				Controller_1, Direction_F:	Raise lift.
//				Controller_1, Direction_B:	Lower lift.
//				Controller_1, Direction_L:	Flag CCW fine-tune.
//				Controller_1, Direction_R:	Flag CW fine-tune.
//				Controller_1, Button_Start:	Toggle auton mode.
//				Controller_1, Button_Back:	[UNUSED]
//
//				Controller_2, Joystick_L:	Lift height.
//				Controller_2, Joystick_R:	Climbing.
//				Controller_2, Button_LB:	Dump 4 cubes.
//				Controller_2, Button_RB:	Dump 1 cube.
//				Controller_2, Button_LT:	Release climbing.
//				Controller_2, Button_RT:	Release climbing.
//				Controller_2, Button_A:		Dump auton cube.
//				Controller_2, Button_B:		Lift modifier key.
//				Controller_2, Button_Joy_L:	[UNUSED]
//				Controller_2, Button_Joy_R:	[UNUSED]
//				Controller_2, Button_X:		[UNUSED]
//				Controller_2, Button_Y:		Turns sweeper off.
//				Controller_2, Direction_L:	Flag CCW.
//				Controller_2, Direction_R:	Flag CW.
//				Controller_2, Direction_F:	Sweep outwards/off.
//				Controller_2, Direction_B:	Sweep inwards.
//				Controller_2, Button_Start:	Toggle auton mode.
//				Controller_2, Button_Back:	Emergency shutdown.
//
// *: Button_LT overrides Button_RT.
//-------------------------------------------------------------------------->>

// For main task:
float power_lift = 0.0;
int lift_target = 0;
bool isResettingLift = false;
float power_flag = 0.0;

// For PID:
float lift_pos = 0.0; // Really should be an int; using a float so I don't have to cast all the time.

// For comms link:
// TODO: Make more efficient by putting vars completely inside bytes, etc.
const int NXT_LINE_NUM = 6;
typedef enum CardinalDirection {
	CARDINAL_DIR_N	= 0,
	CARDINAL_DIR_E	= 1,
	CARDINAL_DIR_S	= 2,
	CARDINAL_DIR_W	= 3,
	CARDINAL_DIR_NUM,
} CardinalDirection;
typedef enum CommLinkMode {
	COMM_LINK_POS_XY	= 0,
	COMM_LINK_ROT_LIGHT	= 1,
	COMM_LINK_RANGE_AB	= 2,
	COMM_LINK_RANGE_CD	= 3,
	COMM_LINK_TELEOP	= 4,
	COMM_LINK_BUMPERS	= 5,
} CommLinkMode;
CommLinkMode f_commLinkMode[6] = {	COMM_LINK_POS_XY,
									COMM_LINK_ROT_LIGHT,
									COMM_LINK_RANGE_AB,
									COMM_LINK_RANGE_CD,
									COMM_LINK_TELEOP,
									COMM_LINK_BUMPERS	};
int f_angle_x = 0; // RobotC doesn't support unsigned types (other than ubyte).
int f_angle_y = 0;
int f_angle_z = 0;
int f_pos_x = 0;
int f_pos_y = 0;
int f_pos_z = 0;
ubyte f_closeRange[CARDINAL_DIR_NUM] = {0,0,0,0};
int f_longRange[CARDINAL_DIR_NUM] = {0,0,0,0};
ubyte f_lineSensor[2][4] = {0,0,0,0,0,0,0,0}; // 0 = front, 1 = back; 0 = left, 3 = right.
ubyte f_cubeNum = 0; // Type is `ubyte` to avoid complications.
bool f_cubeDetected[8] = {0,0,0,0,0,0,0,0}; // 0 = leftmost, 7 = rightmost.
bool f_isFlagBumped = false;
bool f_isHangBumped = false;
bool f_isBumped[CARDINAL_DIR_NUM] = {false, false, false, false};
bool f_isRedAlliance = true; // Changing this var is helpful for testing the MCU connection.

// Comm link debugging vars:
const ubyte mask_read = 0b00111111; // We read from the last 6 bits.
const ubyte mask_write = 0b11000000; // We write to the first 2 bits. TODO: Not actually needed to write?
ubyte f_byte_write = 0;
ubyte f_byte_read = 0;
bool isClockHigh = true;
bool isResync = true; // We start off with a resync.
int error_num = 0; // Incremented every time there's a consecutive error we can't correct.
bool header_write = false;
bool header_read[6] = {false, false, false, false, false, false};
ubyte frame_write[4] = {0x55,0x6F,0xE5,0x7A};
ubyte frame_read[6][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};



task main()
{
	typedef enum SweepMode {
		SWEEP_IN		= 0,
		SWEEP_EJECT		= 1,
		SWEEP_SUSPENDED	= 2,
		SWEEP_OFF		= 3,
	} SweepMode;

	initializeGlobalVariables(); // Defined in "initialize.h", this intializes all struct members.
	initializeRobotVariables();
	Task_Kill(displayDiagnostics); // This is set separately in the "Display" task.
	Task_Spawn(PID);
	Task_Spawn(CommLink);
	Task_Spawn(Display);
	Task_Spawn(TimedOperations); // Immediately start this once the match starts.

	float power_L = 0.0;
	float power_R = 0.0;

	// Misc. variables.
	SweepMode sweepMode = SWEEP_OFF;
	float power_climb = 0.0;
	const int eject_delay = 1200;
	int timer_eject = 0;
	Time_ClearTimer(timer_eject);

	Joystick_WaitForStart();

	while (true) {
		Joystick_UpdateData();

		power_L = -Joystick_GenericInput(JOYSTICK_L, AXIS_Y);
		power_R = -Joystick_GenericInput(JOYSTICK_R, AXIS_Y);
		Motor_SetPower(power_L, motor_FL);
		Motor_SetPower(power_L, motor_BL);
		Motor_SetPower(power_R, motor_FR);
		Motor_SetPower(power_R, motor_BR);

		// Set our "fine-tune" factor (amount motor power is divided by).
		// Ideally, this should be made more intuitive. Maybe a single trigger = slow,
		// while holding both triggers stops movement? The `if... else if...` structure
		// is also a problem, since BUTTON_LT will take precedence over BUTTON_RT.
		if (Joystick_Button(BUTTON_RT)==true) {
			for (int i=POD_FR; i<(int)POD_NUM; i++) {
				g_MotorData[i].fineTuneFactor = 0.25;
			}
		}
		if (Joystick_Button(BUTTON_LT)==true) {
			for (int i=POD_FR; i<(int)POD_NUM; i++) {
				g_MotorData[i].fineTuneFactor = 0; // Equivalent to zeroing motor power.
			}
		} else {
			for (int i=POD_FR; i<(int)POD_NUM; i++) {
				g_MotorData[i].fineTuneFactor = 1; // Equivalent to not fine-tuning at all.
			}
		}

		// Second driver's lift controls are overridden by the first's. The first
		// driver can also lock the lift position by pressing the D-pad (L or R).
		// Proper procedure for resetting lift would be to press Button_B and then
		// go to press the Joystick_L button. Resetting of the lift is registered
		// when the joystick button is released.
		//// TODO: Figure this out. Semaphores? Is it even necessary?
		//Task_HogCPU();
		if (Joystick_Direction(DIRECTION_F)==true) {
			lift_target += 160; // MAGIC_NUM
		} else if (Joystick_Direction(DIRECTION_B)==true) {
			lift_target -= 100; // MAGIC_NUM
		} else if (((Joystick_Direction(DIRECTION_FL))||(Joystick_Direction(DIRECTION_FR)))==true) {
			lift_target += 80; // MAGIC_NUM
		} else if (((Joystick_Direction(DIRECTION_BL))||(Joystick_Direction(DIRECTION_BR)))==true) {
			lift_target -= 50; // MAGIC_NUM
		} else if ((Joystick_Direction(DIRECTION_L))||(Joystick_Direction(DIRECTION_R))!=true) {
			// Nesting these is more efficient.
			if (Joystick_Button(BUTTON_B, CONTROLLER_2)==true) {
				if (Joystick_DirectionPressed(DIRECTION_F, CONTROLLER_2)==true) {
					lift_target = lift_pos_top;
					sweepMode = SWEEP_EJECT;
					Time_ClearTimer(timer_eject);
				} else if (Joystick_DirectionPressed(DIRECTION_B, CONTROLLER_2)==true) {
					lift_target = lift_pos_pickup;
					sweepMode = SWEEP_IN;
				}
			}
			if (Joystick_Button(BUTTON_JOYL, CONTROLLER_2)==true) {
				isResettingLift = true;
				power_lift = Joystick_GenericInput(JOYSTICK_L, AXIS_Y, CONTROLLER_2)/g_FineTuneFactor;
			} else {
				isResettingLift = false; // This is important! Or it never stops resetting.
				// MAGIC_NUM: to make this more realistic. Just a constant scale(-down?).
				lift_target += Joystick_GenericInput(JOYSTICK_L, AXIS_Y, CONTROLLER_2)*1.1;
			}
		}
		// Setting the lift too high or too low is handled in the PID loop.
		//// TODO: Figure this out. Semaphores? Is it even necessary?
		//Task_ReleaseCPU();

		// If both buttons are pressed, 1 cube is dumped instead of 4.
		if ((Joystick_ButtonReleased(BUTTON_RB))||(Joystick_ButtonReleased(BUTTON_RB, CONTROLLER_2))==true) {
			dumpCubes(4); // MAGIC_NUM.
		} else if ((Joystick_ButtonReleased(BUTTON_LB))||(Joystick_ButtonReleased(BUTTON_LB, CONTROLLER_2))==true) {
			dumpCubes(1); // MAGIC_NUM.
		}

		// TODO: Make sure driver 1 does indeed override driver 2. Not very urgent.
		//// TODO: Figure this out. Semaphores? Is it even necessary?
		//Task_HogCPU();
		if (Joystick_Button(BUTTON_B, CONTROLLER_2)==false) {
			if (Joystick_Direction(DIRECTION_F, CONTROLLER_2)==true) {
				sweepMode = SWEEP_EJECT;
				Time_ClearTimer(timer_eject);
			} else if (Joystick_Direction(DIRECTION_B, CONTROLLER_2)==true) {
				sweepMode = SWEEP_IN;
			} // No "else" here so that Button_B can do other stuff.
		}
		if (Joystick_ButtonPressed(BUTTON_Y, CONTROLLER_2)==true) {
			sweepMode = SWEEP_OFF;
		}
		if (Joystick_ButtonPressed(BUTTON_A)==true) {
			switch (sweepMode) {
				case SWEEP_IN :
					sweepMode = SWEEP_EJECT;
					Time_ClearTimer(timer_eject);
					break;
				case SWEEP_EJECT :
				case SWEEP_SUSPENDED :
					// Intentional fall-through.
					sweepMode = SWEEP_OFF;
					break;
				case SWEEP_OFF :
					sweepMode = SWEEP_IN;
					break;
			}
		}
		if ((sweepMode==SWEEP_EJECT)&&(Time_GetTime(timer_eject)>eject_delay)) {
			sweepMode = SWEEP_OFF;
		}
		if (lift_pos>lift_sweeper_guard) {
			if (sweepMode == SWEEP_IN) {
				sweepMode = SWEEP_SUSPENDED;
				// In all other cases the sweeping mode shouldn't change.
			}
		} else {
			if (sweepMode==SWEEP_SUSPENDED) {
				sweepMode = SWEEP_IN;
			}
		}
		//// TODO: Figure this out. Semaphores? Is it even necessary?
		//Task_ReleaseCPU();

		if (Joystick_Direction(DIRECTION_L, CONTROLLER_2)==true) {
			power_flag = -g_FullPower;
		} else if (Joystick_Direction(DIRECTION_R, CONTROLLER_2)==true) {
			power_flag = g_FullPower;
		} else {
			power_flag = 0;
		}

		// TODO: Make climbing work. When we get comms working, fix the controls.
		// As usual, driver 1's controls take precedence over driver 2.
		if (Joystick_Button(BUTTON_X)==true) {
			// TODO: Climb "down" instead.
			power_flag = g_FullPower;
		} else if (Joystick_Direction(DIRECTION_L)==true) {
			power_flag = g_FullPower/(g_FineTuneFactor*2);
		} else if (Joystick_Direction(DIRECTION_R)==true) {
			power_flag = -g_FullPower/(g_FineTuneFactor*2);
		} else if (Joystick_Direction(DIRECTION_L, CONTROLLER_2)==true) {
			power_flag = g_FullPower/(g_FineTuneFactor*2);
		} else if (Joystick_Direction(DIRECTION_R, CONTROLLER_2)==true) {
			power_flag = -g_FullPower/(g_FineTuneFactor*2);
		} else {
			power_flag = 0;
		}
		if (Joystick_Button(BUTTON_Y)==true) {
			power_climb = g_FullPower; // Climb "up".
		} else {
			// This gracefully handles the "else" condition (sets climbing power to 0.
			power_climb = Joystick_GenericInput(JOYSTICK_R, AXIS_Y, CONTROLLER_2);
		}

		if (Joystick_Button(BUTTON_LT, CONTROLLER_2)==true) {
			Servo_SetPosition(servo_climb_L, servo_climb_L_open);
			Servo_SetPosition(servo_climb_R, servo_climb_R_open);
		} else if (Joystick_Button(BUTTON_RT, CONTROLLER_2)==true) {
			Servo_SetPosition(servo_climb_L, servo_climb_L_closed);
			Servo_SetPosition(servo_climb_R, servo_climb_R_closed);
		}

		// Set motor and servo values (lift motor is set in PID()):
		switch (sweepMode) {
			case SWEEP_IN :
				Motor_SetPower(g_FullPower, motor_sweeper);
				Motor_SetPower(g_FullPower, motor_assist_L);
				Motor_SetPower(g_FullPower, motor_assist_R);
				Servo_SetPosition(servo_flip_L, servo_flip_L_down);
				Servo_SetPosition(servo_flip_R, servo_flip_R_down);
				break;
			case SWEEP_EJECT :
				Motor_SetPower(-g_FullPower, motor_sweeper);
				Motor_SetPower(-g_FullPower, motor_assist_L);
				Motor_SetPower(-g_FullPower, motor_assist_R);
				Servo_SetPosition(servo_flip_L, servo_flip_L_up);
				Servo_SetPosition(servo_flip_R, servo_flip_R_up);
				break;
			case SWEEP_SUSPENDED :
			case SWEEP_OFF :
				// Intentional fall-through.
				Motor_SetPower(0, motor_sweeper);
				Motor_SetPower(0, motor_assist_L);
				Motor_SetPower(0, motor_assist_R);
				Servo_SetPosition(servo_flip_L, servo_flip_L_up);
				Servo_SetPosition(servo_flip_R, servo_flip_R_up);
				break;
		}
		// TODO: make the flag and climbing stuff actually work according to how
		// our robot functions. This may take a while. :P
		Motor_SetPower(power_flag, motor_flag);
		//Motor_SetPower(power_climb, motor_climb);

		// While the "back" button is pressed on controller 2, go into shutdown mode.
		if (Joystick_Button(BUTTON_BACK, CONTROLLER_2)==true) {
			isResettingLift = true;
			power_lift = 0.0;
		} else {
			isResettingLift = false;
		}

		// If bDisconnected is true, go into an infinite loop and continually assign 0 to everything.
		if (bDisconnected==true) {
			int original_counter_limit = nNoMessageCounterLimit;
			nNoMessageCounterLimit = 250; // 250 * 4ms = 1000ms = 1sec
			Task_Suspend(PID);	// These two tasks won't get to execute anyway (CPU is hogged).
			Task_Suspend(CommLink);
			do {
				Task_HogCPU();
				resetMotorsServos();
				if (bSoundActive==false) {
					PlaySound(soundFastUpwardTones);
				}
				Task_ReleaseCPU();
				Task_EndTimeslice();
			} while (bDisconnected==true);
			Task_Resume(PID);
			Task_Resume(CommLink);
			nNoMessageCounterLimit = original_counter_limit;
		}
	}
}



float term_P_lift = 0.0;
float term_D_lift = 0.0;

task PID()
{
	// Timer variables.
	int timer_loop = 0;
	Time_ClearTimer(timer_loop);
	int t_delta = Time_GetTime(timer_loop);

	// Variables for lift PID calculations.
	// MAGIC_NUM: PID constants.
	float kP_lift_up	= 0.28;
	float kP_lift_down	= 0.11;
	float kD_lift_up	= 0.0;
	float kD_lift_down	= 0.0;
	float error_lift = 0.0;
	float error_prev_lift = 0.0;
	float error_rate_lift = 0.0;

	Joystick_WaitForStart();
	Time_ClearTimer(timer_loop);

	while (true) {
		Task_HogCPU();

		// We need to update the timers outside of any loops.
		t_delta = Time_GetTime(timer_loop);
		Time_ClearTimer(timer_loop);

		// Yes, this is a complete PID loop, despite it being so short. :)
		// TODO: Replace this :P
		if (isResettingLift==false) {
			lift_pos = Motor_GetEncoder(motor_lift_front); // TODO: Usage of `motor_lift_A` is not final.
			error_prev_lift = error_lift;
			if (lift_target<0) { // Because we're safe.
				lift_target = 0;
			} else if (lift_target>lift_max_height) {
				lift_target = lift_max_height;
			}
			error_lift = lift_target-lift_pos;
			error_rate_lift = (error_lift-error_prev_lift)/t_delta;
			if (error_lift>0) {
				term_P_lift = kP_lift_up*error_lift;
				term_D_lift = kD_lift_up*error_rate_lift;
			} else if (error_lift<=0) {
				term_P_lift = kP_lift_down*error_lift;
				term_D_lift = kD_lift_down*error_rate_lift;
			}
			power_lift=term_P_lift+term_D_lift;
			power_lift = Math_Limit(power_lift, g_FullPower);
		} else {
			lift_pos = 0;
			Motor_ResetEncoder(motor_lift_front); // TODO: Usage of `motor_lift_A` is not final.
		}
		Motor_SetPower(power_lift, motor_lift_front);
		Motor_SetPower(power_lift, motor_lift_back);

		Task_ReleaseCPU();
		Task_EndTimeslice(); // TODO: Is this command superfluous?
	}
}



void processCommTick()
{
	f_byte_write &= ~(1<<7); // Clear the clock bit.
	f_byte_write |= (isClockHigh<<7); // Set the clock bit to appropriate clock value.
	HTSPBwriteIO(sensor_protoboard, f_byte_write);
	f_byte_read = HTSPBreadIO(sensor_protoboard, mask_read);
	isClockHigh = !isClockHigh; // Cannot use XOR (bools are weird).
}
task CommLink()
{
	ubyte current_index_mask = 0; // Convenience variable. See specific uses. (DARK MAGIC; MIGHT NOT WORK)
	ubyte byte_temp = 0;// Convenience variable. See specific uses. (DARK MAGIC; MIGHT NOT WORK)
	const int max_error_num = 5; // If we get more corrupted packets, we should restart transmission.
	bool wasCorrupted = false;
	// Check bits DO NOT include header bits!
	bool check_write = 0; // TODO: Switch to Hamming codes! (Mebbe?) :D
	bool check_read[6] = {0,0,0,0,0,0}; // Value read.
	bool check_read_ack[6] = {0,0,0,0,0,0}; // Value computed.
	bool isBadData[6] = {false, false, false, false, false, false};

	HTSPBsetupIO(sensor_protoboard, mask_write); // `mask_write` happens to conform to the expected format.

	// We don't want to wait for start here (we need to establish
	// a communication link as soon as possible).

	while (true) {

		// Restart the communication link.
		while (isResync==true) {
			// First make sure we're in sync.
			short sync_count = 0;
			int fail_count = 0; // TODO: If this gets too high, alert the drivers.
			while (sync_count<6) { // 3 high and 3 low. Has to be more than a frame? TODO
				f_byte_write |= (1<<6); // Set the data bit high.
				processCommTick();
				f_byte_read |= 0b11000000; // Make sure the "write" bits aren't random.
				switch (isClockHigh) { // We want all the bits to be high (0b11111111). The MAGIC_NUM depends on the clock.
					case false : // These may seem flipped, but that's because the clock is ready for the next tick.
						f_byte_read = f_byte_read^0b00000000; // MAGIC_NUM, kinda
						break;
					case true : // These may seem flipped, but that's because the clock is ready for the next tick.
						f_byte_read = f_byte_read^0b00111111; // MAGIC_NUM, kinda
						break;
				}
				if (f_byte_read==0b11111111) {
					sync_count++;
				} else {
					sync_count = 0;
					fail_count++;
				}
			}
			if (isClockHigh==true) {
				// If so, let it go for another tick.
				processCommTick();
			}

			// Now bring the data line low for 2 clock ticks.
			f_byte_write &= ~(1<<6); // Clear the data bit low.
			processCommTick();
			f_byte_read &= 0b00111111; // Make sure the "write" bits aren't random.
			if (f_byte_read!=0b00000000) {
				isResync = true;
				continue;
			} else {
				isResync = false;
			}
			processCommTick(); // Wait another tick...
			f_byte_read &= 0b00111111; // Make sure the "write" bits aren't random.
			if (f_byte_read!=0b00000000) { // 0, DUH...
				isResync = true;
				continue;
			} else {
				isResync = false;
			}
			// If everything is still good at this point, go on.
		}

		// Write header.
		f_byte_write &= ~(1<<6); // Clear the data bit.

		// TODO: use ubyte instead of bool and just use last bit?
		// A bool can be true but not have the last bit be on.
		if (header_write==true) {
			f_byte_write |= (1<<6);
		} // No "else" needed (already set to 0).
		processCommTick();

		// Read in all 6 data lines.
		for (int line=0; line<NXT_LINE_NUM; line++) {
			current_index_mask = 1<<line;
			// No fancy shifting needed here (header_read is a bool):
			header_read[line] = Bit_FixBool((bool)(f_byte_read&current_index_mask)); // Theoretically, if >0 then true.
		}

		// Data:
		// Clear parity bits.
		for (int line=0; line<NXT_LINE_NUM; line++) {
			check_read_ack[line] = false;
			for (int i=0; i<4; i++) {
				frame_read[line][i] = 0; // TODO: MAKE ATOMIC! IMPORTANT! <<====
			}
		}
		for (int bit=0; bit<32; bit++) {
			int frame = bit/8; // Intentional int division.
			int sub_bit = bit%8;

			// Set MOSI.
			f_byte_write &= ~(1<<6); // Clear the data bit.
			current_index_mask = 1<<(sub_bit); // Set the data bit; `i%8` because data is in bytes.

			// Intentional int division (returns intended byte) (see next statement).
			// Using a temp var because `true!=1` (can be any positive int); statement
			// also clears byte_temp because the mask was cleared (and now AND'd).
			byte_temp = (frame_write[frame])&current_index_mask; // TODO: Use current_index_mask instead of temp var?

			// TODO: combine the two shifts below into one shift.
			byte_temp = byte_temp>>(sub_bit); // Shift data bit over to bit 0.
			f_byte_write |= (byte_temp<<6); // Set the data bit.

			check_write = (Bit_FixBool((bool)byte_temp) != check_write);
			// TODO: For optimization, delete the following line. Right now it's to be safe.
			check_write = Bit_FixBool(check_write);
			processCommTick();

			// Read in all 6 data lines (MISO).
			for (int line=0; line<NXT_LINE_NUM; line++) {
				current_index_mask = 1<<line;
				byte_temp = f_byte_read&current_index_mask; // Isolating the bit we want.
				// TODO: combine the two shifts below into one shift. Actually, we might not even need byte_temp here.
				byte_temp = byte_temp>>line; // Shift the bit into bit 0.
				frame_read[line][frame] |= (byte_temp<<sub_bit); // Shift bit into appropriate place in frame.

				// Because `byte_temp` only has one bit now.
				check_read_ack[line] = (Bit_FixBool((bool)byte_temp) != check_read_ack[line]);
				// TODO: For optimization, delete the following line. Right now it's to be safe.
				check_read_ack[line] = Bit_FixBool(check_read_ack[line]);
			}
		}

		// Check bits.
		f_byte_write &= ~(1<<6); // Clear the data bit.
		if (check_write==true) {
			f_byte_write |= 0b01000000;
		} // No need for "else" condition (byte already cleared).
		processCommTick();
		check_write = false; // Clear this now that we've sent it already.

		// Read check bits.
		for (int line=0; line<NXT_LINE_NUM; line++) {
			current_index_mask = 1<<line; // Select the bit we want to find.
			check_read[line] = Bit_FixBool((bool)(f_byte_read&current_index_mask));
			if (check_read[line] == check_read_ack[line]) {
				isBadData[line] = false;
			} else {
				isBadData[line] = true;
				error_num++;
				wasCorrupted = true;
			}
		}
		// `check_read_ack[]` is reset before use each loop.

		if (error_num>max_error_num) {
			isResync = true; // This happens at the beginning of the next iteration.
			error_num = 0;
			wasCorrupted = false;
		} else if ((error_num!=0)&&(wasCorrupted==false)) {
			error_num = 0; // Not a consecutive error.
		} else if (error_num==0) {
			wasCorrupted = false; // This executes even if first condition was true (it checks again?)
		}

		for (int line=0; line<NXT_LINE_NUM; line++) {
			if (isBadData[line]==true) {
				continue;
			}
			if (header_read[line]==false) {
				switch (f_commLinkMode[line]) {
					case COMM_LINK_POS_XY :
						// TODO: Figure out CPU hogging, you pig.
						//Task_HogCPU(); // So that the main program doesn't try to access these vars.
						f_pos_x = frame_read[line][3];
						f_pos_x = f_pos_x<<1; // There's one more bit of data we need to access.
						f_pos_x += ((frame_read[line][2]&0b10000000)>>7); // TODO: Optimize: masking unnecessary?
						f_angle_x = frame_read[line][2];
						f_angle_x &= 0b01111111;
						f_pos_y = frame_read[line][1];
						f_pos_y = f_pos_y<<1; // There's one more bit of data we need to access.
						f_pos_y += ((frame_read[line][0]&0b10000000)>>7); // TODO: Optimize: masking unnecessary?
						f_angle_y = frame_read[line][0];
						f_angle_y &= 0b01111111;
						//Task_ReleaseCPU();
						break;
					case COMM_LINK_ROT_LIGHT :
						//Task_HogCPU(); // So that the main program doesn't try to access these vars.
						f_angle_z = frame_read[line][2];
						f_angle_z += ((frame_read[line][3]&0b00000001)<<8);
						f_pos_z = ((frame_read[line][3]&0b01111110)>>1);
						f_isRedAlliance = Bit_FixBool((bool)(frame_read[line][3]&0b10000000)); // TODO: only assign this at the beginning of the match.
						for (int i=0; i<4; i++) {
							f_lineSensor[0][i] = Bit_FixBool((bool)(frame_read[line][1]&(1<<i)));
							f_lineSensor[1][i] = Bit_FixBool((bool)(frame_read[line][1]&(1<<(i+4))));
						}
						for (int i=0; i<8; i++) {
							f_cubeDetected[i] = Bit_FixBool((bool)(frame_read[line][0]&(1<<i)));
						}
						//Task_ReleaseCPU();
						break;
					case COMM_LINK_RANGE_AB :
						//Task_HogCPU(); // So that the main program doesn't try to access these vars.
						f_closeRange[CARDINAL_DIR_N] = (frame_read[line][3]>>1)&0b01111111;
						f_longRange[CARDINAL_DIR_N] = frame_read[line][2];
						f_longRange[CARDINAL_DIR_N] += ((frame_read[line][3]<<8)&0b00000001);
						f_closeRange[CARDINAL_DIR_E] = (frame_read[line][1]>>1)&0b01111111;
						f_longRange[CARDINAL_DIR_E] = frame_read[line][0];
						f_longRange[CARDINAL_DIR_E] += ((frame_read[line][1]<<8)&0b00000001);
						//Task_ReleaseCPU();
						break;
					case COMM_LINK_RANGE_CD :
						//Task_HogCPU(); // So that the main program doesn't try to access these vars.
						f_closeRange[CARDINAL_DIR_S] = (frame_read[line][3]>>1)&0b01111111;
						f_longRange[CARDINAL_DIR_S] = frame_read[line][2];
						f_longRange[CARDINAL_DIR_S] += ((frame_read[line][3]<<8)&0b00000001);
						f_closeRange[CARDINAL_DIR_W] = (frame_read[line][1]>>1)&0b01111111;
						f_longRange[CARDINAL_DIR_W] = frame_read[line][0];
						f_longRange[CARDINAL_DIR_W] += ((frame_read[line][1]<<8)&0b00000001);
						//Task_ReleaseCPU();
						break;
					case COMM_LINK_TELEOP :
						//Task_HogCPU(); // So that the main program doesn't try to access these vars.
						f_cubeNum = (frame_read[line][3]>>4)&0b00000111;
						//Task_ReleaseCPU();
						break;
					case COMM_LINK_BUMPERS :
						//Task_HogCPU(); // So that the main program doesn't try to access these vars.
						// TODO: Make these terrible masks better or something. IT HURTS
						f_isFlagBumped = Bit_FixBool((bool)((frame_read[line][3]>>7)&0b00000001));
						f_isHangBumped = Bit_FixBool((bool)((frame_read[line][3]>>6)&0b00000001));
						for (int i=CARDINAL_DIR_N; i<(int)CARDINAL_DIR_NUM; i++) {
							f_isBumped[i] = Bit_FixBool((bool)(frame_read[line][3]&(1<<i)));
						}
						//Task_ReleaseCPU();
						break;
					default :
						// Much to do about nothing. ;)
						break;
				}
			} else {
				// Handle special codes here.
			}
		}
	}
}



// Task for displaying data on the NXT's LCD screen.
// TODO: Put a lot of the display stuff into loops. Do we want to?
task Display()
{
	typedef enum DisplayMode {
		DISP_FCS,				// Default FCS screen.
		DISP_ENCODERS,			// Raw encoder values (7? 8?).
		DISP_COMM_STATUS,		// Each line of each frame.
		DISP_COMM_DEBUG,
		DISP_SENSORS,			// Might need to split this into two screens.
		DISP_JOYSTICKS,			// For convenience. TODO: Add buttons, D-pad, etc.?
		DISP_NUM
	};

	Task_Spawn(displayDiagnostics); // Explicit here: this is only spawned when buttons are pressed.
	DisplayMode isMode = DISP_FCS;

	// We don't need to wait for start. ;)

	while (true) {
		Buttons_UpdateData();

		switch (isMode) {
			case DISP_FCS :
				break;
			case DISP_ENCODERS :
				nxtDisplayTextLine(0, "Lift P: %f", term_P_lift);
				nxtDisplayTextLine(1, "pwr: %f", power_lift);
				nxtDisplayTextLine(2, "encdr: %5d", lift_pos);
				nxtDisplayTextLine(3, "Flag: %+4d", power_flag);
				nxtDisplayTextLine(7, "Gyro: %+6d", f_angle_z);
				break;
			case DISP_COMM_STATUS :
				switch (f_isRedAlliance) {
					case true :
						nxtDisplayCenteredTextLine(0, "RED ALLIANCE");
						break;
					case false :
						nxtDisplayCenteredTextLine(0, "BLUE ALLIANCE");
						break;
					default :
						nxtDisplayCenteredTextLine(0, "BOOL ERROR! DEBUG!");
						break;
				}
				switch (isResync) {
					case true :
						nxtDisplayTextLine(1, "Resyncing...");
						break;
					case false :
						nxtDisplayTextLine(1, "Transmitting...");
						break;
				}
				nxtDisplayTextLine(2, "lost pkts: %d", error_num);
				break;
			case DISP_COMM_DEBUG :
				nxtDisplayCenteredTextLine(0, "W %#4X  R %#4X", f_byte_write, f_byte_read);
				nxtDisplayTextLine(2, "F %2X-%2X-%2X-%2X", frame_read[5][3], frame_read[5][2], frame_read[5][1], frame_read[5][0]);
				nxtDisplayTextLine(3, "E %2X-%2X-%2X-%2X", frame_read[4][3], frame_read[4][2], frame_read[4][1], frame_read[4][0]);
				nxtDisplayTextLine(4, "D %2X-%2X-%2X-%2X", frame_read[3][3], frame_read[3][2], frame_read[3][1], frame_read[3][0]);
				nxtDisplayTextLine(5, "C %2X-%2X-%2X-%2X", frame_read[2][3], frame_read[2][2], frame_read[2][1], frame_read[2][0]);
				nxtDisplayTextLine(6, "B %2X-%2X-%2X-%2X", frame_read[1][3], frame_read[1][2], frame_read[1][1], frame_read[1][0]);
				nxtDisplayTextLine(7, "A %2X-%2X-%2X-%2X", frame_read[0][3], frame_read[0][2], frame_read[0][1], frame_read[0][0]);
				break;
			case DISP_SENSORS :
				nxtDisplayTextLine(0, "%1d cubes", f_cubeNum);
				nxtDisplayTextLine(1, "(%+5d,%+5d,%+3d)", f_pos_x, f_pos_y, f_pos_z);
				nxtDisplayTextLine(2, "(%3d,%3d,%3d)", f_angle_x, f_angle_y, f_angle_z);
				break;
			case DISP_JOYSTICKS :
				nxtDisplayCenteredTextLine(0, "--Driver I:--");
				nxtDisplayCenteredTextLine(1, "LX:%4d RX:%4d", joystick.joy1_x1, joystick.joy1_x2);
				nxtDisplayCenteredTextLine(2, "LY:%4d RY:%4d", joystick.joy1_y1, joystick.joy1_y2);
				nxtDisplayCenteredTextLine(4, "--Driver II:--");
				nxtDisplayCenteredTextLine(5, "LX:%4d RX:%4d", joystick.joy2_x1, joystick.joy2_x2);
				nxtDisplayCenteredTextLine(6, "LY:%4d RY:%4d", joystick.joy2_y1, joystick.joy2_y2);
				break;
			default :
				nxtDisplayCenteredTextLine(3, "Doesn't work...");
				nxtDisplayCenteredTextLine(4, "Yet. >:(");
				break;
		}

		if (Buttons_Released(NXT_BUTTON_L)==true) {
			Display_Clear();
			isMode = (DisplayMode)((isMode+DISP_NUM-1)%DISP_NUM);
			if (isMode==DISP_FCS) {
				Task_Spawn(displayDiagnostics);
			} else {
				Task_Kill(displayDiagnostics);
			}
		}
		if (Buttons_Released(NXT_BUTTON_R)==true) {
			Display_Clear();
			isMode = (DisplayMode)((isMode+DISP_NUM+1)%DISP_NUM);
			if (isMode==DISP_FCS) {
				Task_Spawn(displayDiagnostics);
			} else {
				Task_Kill(displayDiagnostics);
			}
		}
		Time_Wait(100); // MAGIC_NUM: Prevents the LCD from updating itself to death. (Okay, maybe not that dramatic.)
	}
}



task TimedOperations()
{
	Joystick_WaitForStart();
	for (int i=0; i<90; i++) { // MAGIC_NUM: 90=120-30
		Time_Wait(1000);
	}
	Servo_SetPosition(servo_climb_L, servo_climb_L_open);
	Servo_SetPosition(servo_climb_R, servo_climb_R_open);
}
