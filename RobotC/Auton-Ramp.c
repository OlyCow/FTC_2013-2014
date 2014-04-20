#pragma config(Hubs,  S1, HTServo,  HTMotor,  HTMotor,  none)
#pragma config(Hubs,  S2, HTServo,  HTServo,  HTMotor,  HTMotor)
#pragma config(Sensor, S3,     sensor_IR,      sensorI2CCustomFastSkipStates9V)
#pragma config(Sensor, S4,     sensor_protoboard, sensorI2CCustom9V)
#pragma config(Motor,  motorA,          motor_assist_L, 	tmotorNXT, PIDControl, reversed, encoder)
#pragma config(Motor,  motorB,          motor_assist_R, 	tmotorNXT, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motor_BR,       	tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motor_lift_back,	tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     motor_lift_front,   tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_2,     motor_FR,   	    tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C3_1,     motor_FL,      		tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C3_2,     motor_sweeper, 		tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C4_1,     motor_flag,    		tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S2_C4_2,     motor_BL,      		tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S1_C1_1,    servo_climb_R,        tServoStandard)
#pragma config(Servo,  srvo_S1_C1_2,    servo_omni_R,         tServoStandard)
#pragma config(Servo,  srvo_S1_C1_3,    servo_auton,          tServoStandard)
#pragma config(Servo,  srvo_S1_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_5,    servo5,       		  tServoNone)
#pragma config(Servo,  srvo_S1_C1_6,    servo_flip_R,         tServoStandard)
#pragma config(Servo,  srvo_S2_C1_1,    servo_dump,           tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    servo_FR,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    servo_BR,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_4,    servo16,              tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo_climb_L,        tServoStandard)
#pragma config(Servo,  srvo_S2_C1_6,    servo_18,             tServoNone)
#pragma config(Servo,  srvo_S2_C2_1,    servo_omni_L,         tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    servo20,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_3,    servo21,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_4,    servo_flip_L,         tServoStandard)
#pragma config(Servo,  srvo_S2_C2_5,    servo_FL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C2_6,    servo_BL,             tServoStandard)

#include "includes.h"
#include "swerve-drive.h"

//#define WILL_EXPLODE // Uncomment this line to prevent development code from compiling.
#ifdef WILL_EXPLODE
#warn "This code will explode!"
#endif

task Gyro(); // Constantly updates the heading of the robot.
task PID(); // Sets lift motor's power.
task Display(); // Updates the NXT's LCD display with useful info.

//---------------- README!!! ------------------------------------------------>>
//     The code is split into a couple tasks. I.) `main` contains the linear
// autonomous program logic, and calls other convenience functions to do things.
// If it needs to make a trivial power assignment (such as raising the flag),
// it is done directly in the `main` loop. II.) The `PID` loop runs a simple PD
// loop which monitors position, and has hard-coded limits/safeties. III.)
// `CommLink` is how data is trans ferred between the SuperPro prototype board
// and the AVRs we have. This is BLACK MAGIC, DO NOT TOUCH. Since it conflicts
// with our use of the gyro, currently the task isn't spawned at all for autono-
// mous or teleop. In the future it will probably be moved into its own library.
// IV.) `Display` is a cyclical display that provides valuable debugging infor-
// mation. Press the arrow buttons to go to different screens.
//-------------------------------------------------------------------------->>

// Program settings:
bool DO_DELAY_START		= false;
bool DO_START_ON_R		= true;
bool DO_DUMP_AUTON		= true;
bool DO_TURN_ON_RAMP	= false;
bool DO_DEFEND_RAMP		= false;

// Gyro readings:
float heading = 0.0; // Because f_angle_z is an int.

// For PID:
float power_lift = 0.0;
int lift_target = 0;
float lift_pos = 0.0; // Really should be an int; using a float so I don't have to cast all the time.



#include "auton.h"
task main()
{
	initializeGlobalVariables(); // Defined in "initialize.h", this intializes all struct members.
	initializeRobotVariables();
	Servo_SetPosition(servo_omni_L, servo_omni_L_down);
	Servo_SetPosition(servo_omni_R, servo_omni_R_down);
	Task_Kill(displayDiagnostics); // This is set separately in the "Display" task.
	Task_Spawn(Gyro);
	Task_Spawn(Display);

	// MAGIC_NUM: Distances in inches, turns in degrees, delays in seconds.
	const int delay_start					= 15;
	const int dist_dump_L					= 7;
	const int dist_dump_R					= 11;	// TODO
	const int dist_ramp_L					= 26;	// TODO
	const int dist_ramp_R					= 23;	// TODO
	const int delay_charge_L				= 1300;	// milliseconds TODO
	const int delay_charge_R				= 1400;	// milliseconds TODO

	string a="", b="", c="";
	a = "Delay start?";
	b = "YES";
	c = "NO";
	config_values(DO_DELAY_START, a, true, b, false, c);
	a = "Start on____side.";
	b = "RIGHT";
	c = "LEFT";
	config_values(DO_START_ON_R, a, true, b, false, c);
	a = "Move____after IR.";
	b = "STRAIGHT";
	c = "BACKWARD";
	config_values(DO_DUMP_AUTON, a, true, b, false, c);
	a = "Dump auton cube?";
	b = "YES";
	c = "NO";
	config_values(DO_TURN_ON_RAMP, a, true, b, false, c);
	a = "Hold ramp pos?";
	b = "YES";
	c = "NO";
	config_values(DO_DEFEND_RAMP, a, true, b, false, c);

	Joystick_WaitForStart();
	heading = 0.0;
	Motor_ResetEncoder(omniL);
	Motor_ResetEncoder(omniR);
	if (DO_DELAY_START) {
		for (int i=0; i<delay_start; ++i) {
			Time_Wait(1000);
		}
	}

	if (DO_START_ON_R) {
		MoveBackward(dist_dump_R);
	} else {
		MoveForward(dist_dump_L);
	}
	if (DO_DUMP_AUTON) {
		DumpAutonCube();
	}
	if (DO_START_ON_R) {
		TurnLeft(15);
	} else {
		TurnRight(15);
	}
	if (DO_DUMP_AUTON) {
		LowerAutonArm();
	}
	if (DO_START_ON_R) {
		MoveBackward(dist_ramp_R);
		TurnRight(75);
		ChargeForward(delay_charge_R, g_FullPower, true, false);
	} else {
		MoveForward(dist_ramp_L);
		TurnRight(105);
		ChargeForward(delay_charge_L, g_FullPower, true, false);
	}

	if (DO_TURN_ON_RAMP) {
		// TODO: Would it be better to do this with a timer?
		Settle();
		TurnLeft(90);
	}
	if (DO_DEFEND_RAMP) {
		DefendRamp();
	}
}



task Gyro()
{
	float vel_curr = 0.0;
	float vel_prev = 0.0;
	float dt = 0.0;
	int timer_gyro = 0.0;
	Time_ClearTimer(timer_gyro);
	while (true) {
		vel_prev = (float)vel_curr;
		dt = (float)Time_GetTime(timer_gyro)/(float)1000.0;
		Time_ClearTimer(timer_gyro);
		vel_curr = (float)HTGYROreadRot(sensor_protoboard);
		heading += ((float)vel_prev+(float)vel_curr)*(float)0.5*(float)dt;
		Time_Wait(1);
	}
}



task PID()
{
	// Timer variables.
	int timer_loop = 0;
	Time_ClearTimer(timer_loop);
	int t_delta = Time_GetTime(timer_loop);

	// TODO: PID tuning.
	// MAGIC_NUM: Variables for lift PID calculations.
	// Separate constants are needed for up vs. down motion of the lift because
	// gravity significantly affects how the lift behaves (lowering the lift is
	// almost twice as fast as raising the lift with the same amount of power).
	const float lift_guard_divisor	= 2.2;
	const float kP_lift_up			= 0.27;
	const float kP_lift_down		= 0.17;
	const float kD_lift_up			= 0.0;
	const float kD_lift_down		= 0.0;
	float error_lift		= 0.0;
	float error_prev_lift	= 0.0;	// For calculating `error_rate_lift`.
	float error_rate_lift	= 0.0;	// For the D-term of PID.
	float term_P_lift		= 0.0;
	float term_D_lift		= 0.0;

	Joystick_WaitForStart();
	Time_ClearTimer(timer_loop);

	while (true) {
		//Task_HogCPU();
		// We need to update the timers outside of any loops.
		t_delta = Time_GetTime(timer_loop);
		Time_ClearTimer(timer_loop);

		// Assign the power settings to the motors and servos.
		for (int i=POD_FR; i<(int)POD_NUM; i++) {
			g_MotorData[i].power = Math_Limit(g_MotorData[i].power, 100);
			g_MotorData[i].power *= g_MotorData[i].fineTuneFactor;
			Motor_SetPower(g_MotorData[i].power, Motor_Convert((WheelPod)i));
		}

		// TODO: Replace this hacked together lift resetter (or not?).
		// The following is a PID loop for setting the lift's power. Because the lift is so
		// fast, we are slowing it down intentionally to prevent the it from killing itself.
			lift_pos = Motor_GetEncoder(motor_lift_front);
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
			power_lift = term_P_lift+term_D_lift;
			power_lift = Math_Limit(power_lift, g_FullPower);

		// TODO: Fine tune this (maybe not make it a "hard"/abrupt condition?).
		// Our lift is so fast, we slow it down within a buffer zone to make sure it doesn't
		// kill itself when it hits the ends of its range (up and down).
		if (	(power_lift>0 && lift_pos>lift_buffer_top	) ||
				(power_lift<0 && lift_pos<lift_buffer_bottom) ) {
			power_lift /= lift_guard_divisor;
		}
		Motor_SetPower(power_lift, motor_lift_front);
		Motor_SetPower(power_lift, motor_lift_back); // The two motors should run the same direction.

		Time_Wait(2);

		//Task_ReleaseCPU();
		//Task_EndTimeslice(); // TODO: Is this command superfluous? (This needs a check on the forums.)
	}
}



// Task for displaying data on the NXT's LCD screen.
// TODO: Put a lot of the display stuff into loops. Do we want to?
task Display()
{
	typedef enum DisplayMode {
		DISP_FCS,				// Default FCS screen.
		DISP_SETTINGS,			// The current autonomous configuration.
		DISP_ENCODERS,			// Raw encoder values.
		DISP_JOYSTICKS,			// For convenience. TODO: Add buttons, D-pad, etc.?
		DISP_NUM
	};

	Task_Spawn(displayDiagnostics); // Explicit here: this is only spawned when buttons are pressed.
	DisplayMode isMode = DISP_SETTINGS;
	if (isMode != DISP_FCS) {
		Task_Kill(displayDiagnostics);
	}
	// We don't need to wait for start.

	while (true) {
		Buttons_UpdateData();

		switch (isMode) {
			case DISP_FCS :
				break;
			case DISP_SETTINGS :
				string text = "Auton Settings:";
				nxtDisplayCenteredTextLine(0, text);
				if (DO_DELAY_START) {
					text = "- delayed start";
				} else {
					text = "- normal start";
				}
				nxtDisplayTextLine(2, text);
				if (DO_START_ON_R) {
					text = "- start on R";
				} else {
					text = "- start on L";
				}
				nxtDisplayTextLine(3, text);
				if (DO_DUMP_AUTON) {
					text = "- dump auton";
				} else {
					text = "- do not dump";
				}
				nxtDisplayTextLine(4, text);
				if (DO_TURN_ON_RAMP) {
					text = "- turn on ramp";
				} else {
					text = "- stay still";
				}
				nxtDisplayTextLine(5, text);
				if (DO_DEFEND_RAMP) {
					text = "- defend pos";
				} else {
					text = "- passive ramp";
				}
				nxtDisplayTextLine(6, text);
			case DISP_ENCODERS :
				nxtDisplayTextLine(0, "Lift: %+6d", lift_pos);
				nxtDisplayTextLine(1, "Gyro: %+6d", heading);
				nxtDisplayTextLine(2, "FRpow %+4d", g_MotorData[POD_FR].power);
				nxtDisplayTextLine(3, "FLpow %+4d", g_MotorData[POD_FL].power);
				nxtDisplayTextLine(4, "Error: %i", error);
				nxtDisplayTextLine(5, "avg pos: %i", pos_avg);
				nxtDisplayTextLine(6, "encdr R: %i", pos_L);
				nxtDisplayTextLine(7, "encdr L: %i", pos_R);
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
		Time_Wait(50); // MAGIC_NUM: Prevents the LCD from updating itself to death.
	}
}
