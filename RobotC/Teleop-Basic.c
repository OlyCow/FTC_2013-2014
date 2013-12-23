#pragma config(Hubs,  S1, HTServo,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTMotor,  HTServo,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     sensor_IR,      sensorI2CCustomFastSkipStates9V)
#pragma config(Sensor, S4,     sensor_protoboard, sensorI2CCustomFastSkipStates9V)
#pragma config(Motor,  mtr_S1_C2_1,     motor_flag,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motor_sweeper, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     motor_F,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     motor_lift,    tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C4_1,     motor_BL,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C4_2,     motor_FL,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C1_1,     motor_BR,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C1_2,     motor_FR,      tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S1_C1_1,    servo_BL,             tServoStandard)
#pragma config(Servo,  srvo_S1_C1_2,    servo_FL,             tServoStandard)
#pragma config(Servo,  srvo_S1_C1_3,    servo_dump,           tServoStandard)
#pragma config(Servo,  srvo_S1_C1_4,    servo_flag,           tServoStandard)
#pragma config(Servo,  srvo_S1_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_6,    servo_climb_L,        tServoStandard)
#pragma config(Servo,  srvo_S2_C2_1,    servo_BR,             tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    servo_FR,             tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    servo9,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo_climb_R,        tServoStandard)

#include "includes.h"
#include "swerve-drive.h"

#define WILL_EXPLODE // Uncomment this line (Ctrl-Q) to prevent development code from compiling.
#ifdef WILL_EXPLODE
#warn "This code will explode!"
#endif

task PID(); // Sets CR-servos' power, wheel pod motors' power, and lift motor's power. Others set in main.
//task CommLink(); // Reads/writes to the protoboard as tightly as possible.
task Display(); // A separate task for updating the NXT's LCD display.
task TimedOperations();
task SaveData();
task Autonomous(); // Ooooh.

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
//				Controller_1, Button_RB:	Dump 2 cubes.
//				Controller_1, Button_A:		Toggle sweeper.
//				Controller_1, Button_B:		Reset gyro (eventually flag).
//				Controller_1, Button_X:		Flag (eventually climb down).
//				Controller_1, Button_Y:		Climb (eventually climb up).
//				Controller_1, Direction_F:	Raise lift.
//				Controller_1, Direction_B:	Lower lift.
//				Controller_1, Direction_L:	Stop lift (stops Driver 2).
//				Controller_1, Direction_R:	Stop lift (stops Driver 2).
//				Controller_1, Button_Start:	Start auton (&&).
//				Controller_1, Button_Back:	End auton (||).
//
//				Controller_2, Joystick_L:	Lift height.
//				Controller_2, Joystick_R:	Climbing.
//				Controller_2, Button_LB:	Dump 4 cubes.
//				Controller_2, Button_RB:	Dump 2 cubes.
//				Controller_2, Button_LT:	[UNUSED]
//				Controller_2, Button_RT:	[UNUSED]
//				Controller_2, Button_A:		Save pod reset data.
//				Controller_2, Button_B:		Reset lift (w/ Button_JL).
//				Controller_2, Button_Joy_L:	Reset lift (w/ Button_B).
//				Controller_2, Button_Joy_R:	[UNUSED]
//				Controller_2, Button_X:		Adds 3 flag waves.
//				Controller_2, Button_Y:		Morse code signaling.
//				Controller_2, Direction_L:	Flag CCW.
//				Controller_2, Direction_R:	Flag CW.
//				Controller_2, Direction_F:	Sweep outwards.
//				Controller_2, Direction_B:	Sweep inwards.
//				Controller_2, Button_Start:	Start auton (&&).
//				Controller_2, Button_Back:	End auton (||).
//
// *: Button_LT overrides Button_RT.
//-------------------------------------------------------------------------->>

// For control flow:
bool isAutonomous = false;

// For main task:
float power_lift = 0.0;
int lift_target = 0;

// For PID:
float term_P_pod[POD_NUM] = {0,0,0,0};
float term_I_pod[POD_NUM] = {0,0,0,0};
float term_D_pod[POD_NUM] = {0,0,0,0};
float encoder_pod[POD_NUM] = {0,0,0,0};
float pod_current[POD_NUM] = {0,0,0,0};
float pod_raw[POD_NUM] = {0,0,0,0};
float error_pod[POD_NUM] = {0,0,0,0}; // Difference between set-point and measured value.
float correction_pod[POD_NUM] = {0,0,0,0}; // Equals "term_P + term_I + term_D".
float lift_pos = 0.0; // Really should be an int; using a float so I don't have to cast all the time.
const int max_lift_height = 5200; // MAGIC_NUM. TODO: Find this value.

// For comms link:
typedef enum CardinalDirection {
	CARDINAL_DIR_N	= 0,
	CARDINAL_DIR_W	= 1,
	CARDINAL_DIR_S	= 2,
	CARDINAL_DIR_E	= 3,
	CARDINAL_DIR_NUM,
} CardinalDirection;
typedef enum CommLinkMode { // TODO: Make more efficient by putting vars completely inside bytes, etc.
	COMM_LINK_STD_A,
	COMM_LINK_STD_B,
	COMM_LINK_STD_C,
	COMM_LINK_STD_D,
	COMM_LINK_STD_E,
	COMM_LINK_STD_F,
} CommLinkMode; // TODO: Flesh this out.
// TODO: If there are too many variables here, start combining them, esp. the bitmaps.
const ubyte mask_read = 0b00111111; // We read from the last 6 bits.
const ubyte mask_write = 0b11000000; // We write to the first 2 bits. TODO: Not actually needed to write?
ubyte f_byte_write = 0;
ubyte f_byte_read = 0;
bool isClockHigh = true;
CommLinkMode f_commLinkMode[6] = {	COMM_LINK_STD_A,
									COMM_LINK_STD_B,
									COMM_LINK_STD_C,
									COMM_LINK_STD_D,
									COMM_LINK_STD_E,
									COMM_LINK_STD_F	};
int f_angle_x = 0; // RobotC doesn't support unsigned ints???
int f_angle_y = 0;
int f_angle_z = 0;
int f_pos_x = 0;
int f_pos_y = 0;
int f_pos_z = 0;
ubyte f_closeRange[CARDINAL_DIR_NUM] = {0,0,0,0};
int f_longRange[CARDINAL_DIR_NUM] = {0,0,0,0};
ubyte f_lineSensorCenter[CARDINAL_DIR_NUM] = {0,0,0,0};
ubyte f_lineSensor[CARDINAL_DIR_NUM][CARDINAL_DIR_NUM] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
byte f_cubeNum = 0;
bool f_liftReset = false;
bool f_podReset[POD_NUM] = {false, false, false, false};
bool f_cubeDetectedCenter = false;
bool f_cubeDetected[CARDINAL_DIR_NUM][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
bool f_flagBumperTriggered = false;
bool f_climbBumperTriggered = false;
bool f_bumperTriggered[CARDINAL_DIR_NUM] = {false, false, false, false};

bool isTank = false;



task main()
{
	typedef enum SweepDirection {
		SWEEP_IN	= 0,
		SWEEP_OUT	= 1,
		SWEEP_OFF	= 2,
	} SweepDirection;

	initializeGlobalVariables(); // Defined in "initialize.h", this intializes all struct members.
	initializeRobotVariables();
	Task_Kill(displayDiagnostics); // This is set separately in the "Display" task.
	Task_Spawn(PID);
	//Task_Spawn(CommLink);
	Task_Spawn(Display);
	Task_Spawn(TimedOperations); // Immediately start this once the match starts.

	// Not initializing these structs for now: once data starts coming in
	// from the controllers, all the members of these will get updated.
	vector2D rotation[POD_NUM];
	vector2D translation; // Not a struct because all wheel pods share the same values.
	vector2D combined[POD_NUM]; // The averaged values: angle is pod direction, magnitude is power.
	float rotation_temp = 0.0; // So we only fetch data from joysticks once.
	float combined_angle_prev[POD_NUM] = {0,0,0,0}; // Prevents atan2(0,0)=0 from resetting the wheel pods to 0.
	bool shouldNormalize = false; // Set if motor values go over 100. All wheel pod power will be scaled down.
	const int maxTurns = 1; // On each side. To prevent the wires from getting too twisted.

	float heading = 0.0; // Because f_angle_z is an int.
	float gyro_current = 0.0; // For trapezoidal approximation.
	float gyro_prev = 0.0; // For trapezoidal approximation.

	// Timers.
	int timer_gyro = 0.0;

	// Tank-mode power levels.
	float power_L = 0.0;
	float power_R = 0.0;

	// Misc. variables.
	SweepDirection sweepDirection = SWEEP_OFF;
	float power_flag = 0.0;
	float power_climb = 0.0;

	// TODO: Fix the eject and delete the corresponding code & vars.
	const int eject_delay = 250;

	Joystick_WaitForStart();
	Time_ClearTimer(timer_gyro);

	while (true) {
		Joystick_UpdateData();

		gyro_current = (float)HTGYROreadRot(sensor_protoboard);
		heading += (gyro_current+gyro_prev)*(float)Time_GetTime(timer_gyro)/(float)2000.0; // Trapezoid.
		Time_ClearTimer(timer_gyro);
		gyro_prev = gyro_current;
		f_angle_z = round(heading);

		if (Joystick_ButtonPressed(BUTTON_Y)==true) {
			isTank = !isTank;
		}
		switch (isTank) {
			case true :
				// MAGIC_NUM: Ian says these angles make the pods skid less.
				g_ServoData[POD_FR].angle = 102;
				g_ServoData[POD_FL].angle = 78;
				g_ServoData[POD_BL].angle = 78;
				g_ServoData[POD_BR].angle = 102;
				power_L = Joystick_GenericInput(JOYSTICK_L, AXIS_Y);
				power_R = Joystick_GenericInput(JOYSTICK_R, AXIS_Y);
				g_MotorData[POD_FR].power = power_R;
				g_MotorData[POD_FL].power = power_L;
				g_MotorData[POD_BL].power = power_L;
				g_MotorData[POD_BR].power = power_R;
				break;

			case false :
				// TODO: When the robot design is finalized and comms is working and all
				// that good stuff, take this out and only use the joystick button to reset
				// the gyro. Also update the climbing and lift controls when we finalize
				// those as well.
				if (Joystick_ButtonPressed(BUTTON_B)==true) {
					f_angle_z = 0;
					heading = 0;
				}
				if (Joystick_ButtonPressed(BUTTON_JOYR)==true) {
					f_angle_z = 0;
					heading = 0;
				}

				// A rotation vector is added to translation vector, and the resultant vector
				// is normalized. A differential analysis of the parametric equations of
				// each wheel pod confirms that the above algorithm works perfectly, despite
				// its apparent simplicity. Use of the Vector2D library makes some of this
				// slightly less efficient (there are some unnecessary update calculations)
				// but the benefit of increased readability is well worth it.
				// The following code is essentially wizardry (practically speaking).
				translation.x = Joystick_GenericInput(JOYSTICK_R, AXIS_X);
				translation.y = Joystick_GenericInput(JOYSTICK_R, AXIS_Y);
				Vector2D_UpdateRot(translation);
				Vector2D_Rotate(translation, -heading); // We want to correct it, not compound.
				rotation_temp = -Joystick_GenericInput(JOYSTICK_L, AXIS_X); // Intuitively, CCW = pos. rot.

				for (int i=POD_FR; i<(int)POD_NUM; i++) {
					rotation[i].r = rotation_temp;
					rotation[i].theta = g_MotorData[i].angleOffset+90; // The vector is tangent to the circle (+90 deg).
					Vector2D_UpdatePos(rotation[i]);
					Vector2D_Add(rotation[i], translation, combined[i]);
					if (combined[i].r>g_FullPower) {
						shouldNormalize = true;
					}
					if ((combined[i].theta==0)&&(combined[i].r==0)&&(pod_current[i]<maxTurns*360)&&(pod_current>-maxTurns*360)==true) { // AND encoder is within 2 turns.
						combined[i].theta = combined_angle_prev[i];
						// No need to update `combined_angle_prev[i]` because it stays the same.
						Vector2D_UpdatePos(combined[i]); // This might be unnecessary.
					} else {
						combined_angle_prev[i] = combined[i].theta;
					}
					g_ServoData[i].angle = combined[i].theta;
				}

				// Normalize our motors' power values if a motor's power went above g_FullPower.
				if (shouldNormalize==true) {
					float originalMaxPower = (float)g_FullPower; // If there was a false positive, this ensures nothing changes.
					for (int i=POD_FR; i<(int)POD_NUM; i++) {
						if (combined[i].r>originalMaxPower) {
							originalMaxPower = combined[i].r;
						}
					}
					for (int i=POD_FR; i<(int)POD_NUM; i++) {
						combined[i].r = Math_Normalize(combined[i].r, originalMaxPower, g_FullPower);
						Vector2D_UpdatePos(combined[i]); // This might be unnecessary.
					}
					shouldNormalize = false; // Reset this for the next iteration.
				}
				for (int i=POD_FR; i<(int)POD_NUM; i++) {
					g_MotorData[i].power = combined[i].r;
				}
				break; // case `isTank==false`
		}

		// Set our "fine-tune" factor (amount motor power is divided by).
		// Ideally, this should be made more intuitive. Maybe a single trigger = slow,
		// while holding both triggers stops movement? The `if... else if...` structure
		// is also a problem, since BUTTON_LT will take precedence over BUTTON_RT.
		if (Joystick_Button(BUTTON_LT)==true) {
			for (int i=POD_FR; i<(int)POD_NUM; i++) {
				g_MotorData[i].fineTuneFactor = 0; // Equivalent to zeroing motor power.
			}
		} else if (Joystick_Button(BUTTON_RT)==true) {
			for (int i=POD_FR; i<(int)POD_NUM; i++) {
				g_MotorData[i].fineTuneFactor = 0.25; // MAGIC_NUM.
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
		if (Joystick_Direction(DIRECTION_F)==true) {
			lift_target += 160; // MAGIC_NUM
		} else if (Joystick_Direction(DIRECTION_B)==true) {
			lift_target -= 100; // MAGIC_NUM
		} else if (((Joystick_Direction(DIRECTION_FL))||(Joystick_Direction(DIRECTION_FR)))==true) {
			lift_target += 80; // MAGIC_NUM
		} else if (((Joystick_Direction(DIRECTION_BL))||(Joystick_Direction(DIRECTION_BR)))==true) {
			lift_target -= 50; // MAGIC_NUM
		} else if ((Joystick_Direction(DIRECTION_L))||(Joystick_Direction(DIRECTION_R))!=true) {
			lift_target += Joystick_GenericInput(JOYSTICK_L, AXIS_Y)*0.2; // MAGIC_NUM: to make this more realistic. Just a constant scale(-down?).
			//Nesting these is more efficient.
			if (Joystick_Button(BUTTON_B, CONTROLLER_2)==true) {
				if (Joystick_DirectionPressed(DIRECTION_F, CONTROLLER_2)==true) {
					lift_target = lift_pos_dump;
				} else if (Joystick_DirectionPressed(DIRECTION_B, CONTROLLER_2)==true) {
					lift_target = lift_pos_pickup;
				}
				if (Joystick_ButtonReleased(BUTTON_JOYR, CONTROLLER_2)==true) {
					Motor_ResetEncoder(motor_lift);
					Motor_SetPower(motor_lift, 0); // TODO: This safety might be able to be improved.
				}
			}
		}
		// Setting the lift too high or too low is handled in the PID loop.

		// On conflicting input, 2 cubes are dumped instead of 4.
		if ((Joystick_ButtonReleased(BUTTON_RB))||(Joystick_ButtonReleased(BUTTON_RB, CONTROLLER_2))==true) {
			dumpCubes(2); // MAGIC_NUM.
		} else if ((Joystick_ButtonReleased(BUTTON_LB))||(Joystick_ButtonReleased(BUTTON_LB, CONTROLLER_2))==true) {
			dumpCubes(4); // MAGIC_NUM.
		}

		// Driver 1 overrides driver 2 because he assigns last.
		if (Joystick_Button(BUTTON_B, CONTROLLER_2)==false) {
			if (Joystick_Direction(DIRECTION_F, CONTROLLER_2)==true) {
				sweepDirection = SWEEP_OUT;
			} else if (Joystick_Direction(DIRECTION_B, CONTROLLER_2)==true) {
				sweepDirection = SWEEP_IN;
			} // No "else" here so that Button_B can do other stuff.
		}
		if (Joystick_ButtonPressed(BUTTON_Y, CONTROLLER_2)==true) {
			sweepDirection = SWEEP_OFF;
		}
		if (Joystick_ButtonPressed(BUTTON_A)==true) {
			switch (sweepDirection) {
				case SWEEP_IN :
					// TODO: Make this a task?
					sweepDirection = SWEEP_OFF;
					Motor_SetPower(-g_FullPower, motor_sweeper); // <- TODO: kludge, make better!
					Time_Wait(eject_delay);
					Motor_SetPower(0, motor_sweeper);
					break;
				case SWEEP_OUT :
					sweepDirection = SWEEP_OFF;
					break;
				case SWEEP_OFF :
					sweepDirection = SWEEP_IN;
					break;
			}
		}

		// TODO: All of the flag/climbing implementation depends on how Ian makes
		// the flag-raising and climbing work.
		if (Joystick_Button(BUTTON_B)==true) {
			//power_flag = g_FullPower; // TODO: When we get comms working, uncomment this.
		} else if (Joystick_Direction(DIRECTION_L, CONTROLLER_2)==true) {
			power_flag = -g_FullPower;
		} else if (Joystick_Direction(DIRECTION_R, CONTROLLER_2)==true) {
			power_flag = g_FullPower;
		} else {
			power_flag = 0;
		}

		// TODO: Make climbing work. When we get comms working, fix the controls.
		// Maybe even make the following two "if" statements into "if... else if".
		if (Joystick_Button(BUTTON_X)==true) {
			power_flag = g_FullPower; // TODO: get rid of this when we get comms working.
			// Climb "down" instead.
		} else {
			power_flag = 0;
		}
		if (Joystick_Button(BUTTON_Y)==true) {
			power_climb = g_FullPower;
			// Climb "up".
		} else if (Joystick_Joystick(JOYSTICK_R, AXIS_Y, CONTROLLER_2)<=-50) {
			power_climb = -g_FullPower;
		} else if (Joystick_Joystick(JOYSTICK_R, AXIS_Y, CONTROLLER_2)>=50) {
			power_climb = g_FullPower;
		} else {
			power_climb = 0;
		}
		// TODO: Depending on how climbing works, control with driver 2's joystick_R.

		if (Joystick_Button(BUTTON_LT, CONTROLLER_2)==true) {
			Servo_SetPosition(servo_climb_L, servo_climb_L_open);
			Servo_SetPosition(servo_climb_R, servo_climb_R_open);
		} else if (Joystick_Button(BUTTON_RT, CONTROLLER_2)==true) {
			Servo_SetPosition(servo_climb_L, servo_climb_L_closed);
			Servo_SetPosition(servo_climb_R, servo_climb_R_closed);
		}

		// Start autonomous mode when `BUTTON_START` is pressed on both controllers,
		// but only one controller's `BUTTON_BACK` needs to be pressed to end it.
		if ((Joystick_ButtonReleased(BUTTON_BACK))&&(Joystick_ButtonReleased(BUTTON_BACK, CONTROLLER_2))==true) {
			isAutonomous = false;
			Task_Kill(Autonomous);
		} else if ((Joystick_ButtonReleased(BUTTON_START))&&(Joystick_ButtonReleased(BUTTON_START, CONTROLLER_2))==true) {
			isAutonomous = true;
			Task_Spawn(Autonomous);
		}

		// If the flag is already waving, add 3 more waves.
		if (Joystick_ButtonPressed(BUTTON_X, CONTROLLER_2)==true) {
			switch (f_isWavingFlag) {
				case true :
					f_waveNum += 3; // MAGIC_NUM.
					break;
				case false :
					waveFlag();
					break;
			}
		}

		// Save the pod reset data.
		if (Joystick_ButtonPressed(BUTTON_A, CONTROLLER_2)==true) {
			Task_Spawn(SaveData);
		}

		// Set motor and servo values (lift motor is set in PID()):
		switch (sweepDirection) {
			case SWEEP_IN :
				Motor_SetPower(g_FullPower, motor_sweeper);
				break;
			case SWEEP_OUT :
				Motor_SetPower(-g_FullPower, motor_sweeper);
				break;
			case SWEEP_OFF :
				Motor_SetPower(0, motor_sweeper);
				break;
		}
		// TODO: make the flag and climbing stuff actually work according to how
		// our robot functions. This may take a while. :P
		Motor_SetPower(power_flag, motor_flag);
	}
}



task PID()
{
	typedef enum Aligned {
		ALIGNED_FAR		= 0,
		ALIGNED_MEDIUM	= 1,
		ALIGNED_CLOSE	= 2,
	};

	// Variables used in both wheel pod and lift PID calculations.
	float t_current = Time_GetTime(TIMER_PROGRAM);
	float t_prev = t_current;
	float t_delta = t_current-t_prev;

	// Variables for wheel pod PID calculations.
	const int kI_delay = 10; // Iterations.
	float error_sum_pod[POD_NUM][kI_delay];
	for (int i=0; i<(int)POD_NUM; i++) {
		for (int j=0; j<kI_delay; j++) {
			error_sum_pod[i][j] = 0;
		}
	}
	float error_sum_total_pod[POD_NUM] = {0,0,0,0}; // {FR, FL, BL, BR}
	float kP[POD_NUM] = {1.2,	1.1,	1.4,	1.6}; // MAGIC_NUM: TODO: PID tuning.
	float kI[POD_NUM] = {0.001,	0.001,	0.001,	0.001};
	float kD[POD_NUM] = {0.12,	0.12,	0.12,	0.12};
	float error_prev_pod[POD_NUM] = {0,0,0,0}; // Easier than using the `error_accumulated` array, and prevents the case where that array is size <=1.
	float error_rate_pod[POD_NUM] = {0,0,0,0};
	Aligned isAligned = ALIGNED_CLOSE; // If false, cut motor power so that wheel pod can get aligned.
	const int turnLimit = 3; // On each side. To prevent the wires from getting too twisted.
	int pod_pos_prev[POD_NUM] = {0,0,0,0};

	// Variables for lift PID calculations.
	float kP_lift_up	= 0.3; // TODO: PID tuning. MAGIC_NUM.
	float kP_lift_down	= 0.065;
	float kD_lift_up	= 0.0;
	float kD_lift_down	= 0.0;
	float error_lift = 0.0;
	float error_prev_lift = 0.0;
	float error_rate_lift = 0.0;
	float term_P_lift = 0.0;
	float term_D_lift = 0.0;

	// Variables for damping wheel pods.
	float max_pos = 0.0;
	float min_pos = 0.0;

	TFileHandle IO_handle;
	TFileIOResult IO_result;
	const string filename_pods = "_reset_pods.txt";
	const string filename_pods_temp = "_reset_pods_tmp.txt"; // _temp seems to be too long of a file name??
	int file_size = 0;

	// If we can't find the file, we go to the backup file.
	OpenRead(IO_handle, IO_result, filename_pods, file_size);
	if (IO_result==ioRsltSuccess) {
		for (int i=POD_FR; i<(int)POD_NUM; i++) {
			ReadShort(IO_handle, IO_result, pod_pos_prev[i]);
		}
		Close(IO_handle, IO_result);
	} else if (IO_result==ioRsltFileNotFound) {
		OpenRead(IO_handle, IO_result, filename_pods_temp, file_size);
		if (IO_result==ioRsltSuccess) {
			for (int i=POD_FR; i<(int)POD_NUM; i++) {
				ReadShort(IO_handle, IO_result, pod_pos_prev[i]);
			}
			Close(IO_handle, IO_result);
		} else if ((IO_result==ioRsltFileNotFound)||(IO_result==ioRsltNoMoreFiles)) {
			// TODO: (more) error handling, etc.
		}
	} else if (IO_result==ioRsltNoMoreFiles) {
		// TODO: (more) error handling, etc.
	}

	Joystick_WaitForStart();

	while (true) {
		// Update timer first, in case something happens later during the loop.
		t_prev = t_current;
		t_current = Time_GetTime(TIMER_PROGRAM);
		t_delta = t_current-t_prev;

		// Calculate the targets and error for each wheel pod.
		for (int i=POD_FR; i<(int)POD_NUM; i++) {
			encoder_pod[i] = Motor_GetEncoder(Motor_Convert((Motor)i));
			pod_raw[i] = encoder_pod[i]/(float)(-2); // Encoders are geared up by 2 (and "backwards").
			pod_raw[i] = Math_Normalize(pod_raw[i], (float)1440, 360); // Encoders are 1440 CPR.
			pod_raw[i] += pod_pos_prev[i];
			pod_current[i] = (float)(round(pod_raw[i])%360); // Value is now between -360 ~ 360.
			pod_current[i] += 360; // Value is now >= 0 (between 0 ~ 720).
			pod_current[i] = (float)(round(pod_current[i])%360); // Value is now between 0 ~ 360.

			error_prev_pod[i] = error_pod[i];
			error_pod[i] = g_ServoData[i].angle-pod_current[i];

			// TODO: Simplify the below to something having to do with modulo 180.
			// Make sure we turn at most 180 degrees:
			if (error_pod[i]>180) {
				error_pod[i] = error_pod[i]-360;
			} else if (error_pod[i]<-180) {
				error_pod[i] = error_pod[i]+360;
			}

			// TODO: Simplify the below to something having to do with modulo 90.
			// Motor reversals are being explicitly assigned (instead of XOR-ing)
			// because they aren't cleared each iteration and this is the first
			// time this iteration we access them. (Later we can XOR them.)
			// Make sure we turn at most 90 degrees:
			if (error_pod[i]>90) {
				error_pod[i] = error_pod[i]-180;
				g_MotorData[i].isReversed = true;
			} else if (error_pod[i]<-90) {
				error_pod[i] = error_pod[i]+180;
				g_MotorData[i].isReversed = true;
			} else {
				g_MotorData[i].isReversed = false;
			}

			// Make sure we don't hit the maximum turning limit:
			if (error_pod[i]+pod_raw[i]>turnLimit*360) {
				//TODO: Add even more limits so if the pods get off >90deg, bad things don't happen.
				error_pod[i] = error_pod[i]-180;
				g_MotorData[i].isReversed = (!g_MotorData[i].isReversed);
			} else if (error_pod[i]+pod_raw[i]<turnLimit*(-360)) {
				//TODO: Add even more limits so if the pods get off >90deg, bad things don't happen.
				error_pod[i] = error_pod[i]+180;
				g_MotorData[i].isReversed = (!g_MotorData[i].isReversed);
			}

			//// TODO: Encoders might have a tiny deadband (depends on backlash).
			//Math_TrimDeadband(error_pod[i], g_EncoderDeadband); // Unnecessary?

			// Calculate various aspects of the errors, for the I- and D- terms.
			error_sum_total_pod[i] -= error_sum_pod[i][kI_delay-1]; // -1: Array indices.
			error_sum_total_pod[i] += error_sum_pod[i][0];
			// TODO: Figure out whether this really needs to count down instead of up :P
			for (int j=kI_delay-1; j>0; j--) { //`j=kI_delay-1` because we are dealing with array indices.
				error_sum_pod[i][j] = error_sum_pod[i][j-1];
			}
			error_sum_pod[i][0] = error_pod[i]*t_delta;
			error_rate_pod[i] = (error_pod[i]-error_prev_pod[i])/t_delta;
			if (abs(error_pod[i])>12) { //12 is an arbitrary number :P
				isAligned = ALIGNED_FAR;
			} else if (abs(error_pod[i])>6) {
				isAligned = ALIGNED_MEDIUM;
			} else {
				isAligned = ALIGNED_CLOSE;
			}
			term_P_pod[i] = kP[i]*error_pod[i];
			term_I_pod[i] = kI[i]*error_sum_total_pod[i];
			term_D_pod[i] = kD[i]*error_rate_pod[i];
			correction_pod[i] = Math_Limit((term_P_pod[i]+term_I_pod[i]+term_D_pod[i]), 128); // Because servos, not motors.
		}

		// "Damp" motors depending on how far the wheel pods are from their targets.
		for (int i=MOTOR_FR; i<(int)MOTOR_NUM; i++) {
			switch (isAligned) {
				case ALIGNED_FAR:
					g_MotorData[i].fineTuneFactor *= 0; // Zeroes motor power.
					break;
				case ALIGNED_MEDIUM:
					g_MotorData[i].fineTuneFactor *= 1/abs(error_pod[i])*10; // Ranges from 28~83%.
					break;
				case ALIGNED_CLOSE :
					g_MotorData[i].fineTuneFactor *= 1;
					break;
				// Skipping the "ALIGNED_CLOSE" condition could increase performance.
			}
		}

		// Assign the power settings to the motors (already parsed).
		for (int i=MOTOR_FR; i<(int)MOTOR_NUM; i++) {
			// The following line requires a PID loop on velocity, it seems.
			//g_MotorData[i].power += total_correction[i]/(float)(10); // Correcting for servo rotation (doesn't work yet).
			g_MotorData[i].power = Math_Limit(g_MotorData[i].power, 100);
			if (g_MotorData[i].isReversed==true) {
				g_MotorData[i].power *= -1;
			}
			g_MotorData[i].power *= g_MotorData[i].fineTuneFactor;
			Motor_SetPower(g_MotorData[i].power, Motor_Convert((Motor)i));
		}

		// Assign the power settings to the servos.
		// Negative because the servo is powers the pod via a gear.
		Servo_SetPower(servo_FR, -correction_pod[POD_FR]);
		Servo_SetPower(servo_FL, -correction_pod[POD_FL]);
		Servo_SetPower(servo_BL, -correction_pod[POD_BL]);
		Servo_SetPower(servo_BR, -correction_pod[POD_BR]);

		// Another PID loop, this time for the lift.
		// Yes, it is a complete PID loop, despite being so much shorter. :)
		lift_pos = Motor_GetEncoder(motor_lift);
		error_prev_lift = error_lift;
		if (lift_target<0) { // Because we're safe.
			lift_target = 0;
		} else if (lift_target>max_lift_height) {
			lift_target = max_lift_height;
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
		Motor_SetPower(power_lift, motor_lift);
	}
}



//void processCommTick()
//{
//	f_byte_write &= ~(1<<7); // Clear the clock bit.
//	f_byte_write |= (isClockHigh<<7); // Set the clock bit to appropriate clock value.
//	HTSPBwriteIO(sensor_protoboard, f_byte_write);
//	f_byte_read = HTSPBreadIO(sensor_protoboard, mask_read);
//	isClockHigh = !isClockHigh; // TODO: Replace w/ XOR. (If possible.)
//}
//task CommLink()
//{
//	bool isResync = true; // We start off with a resync.
//	ubyte current_index_mask = 0; // Convenience variable. See specific uses. (DARK MAGIC; MIGHT NOT WORK)
//	ubyte byte_temp = 0;// Convenience variable. See specific uses. (DARK MAGIC; MIGHT NOT WORK)
//	const int max_error_num = 15; // If we get more corrupted packets, we should restart transmission.
//	int error_num = 0; // Incremented every time there's a consecutive error we can't correct.
//	bool wasCorrupted = false;
//	bool header_write = false;
//	bool header_read[6] = {false, false, false, false, false, false};
//	ubyte frame_write[4] = {0,0,0,0};
//	ubyte frame_read[6][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
//	ubyte check_write = 0; // TODO: Switch to Hamming codes! (Mebbe?) :D
//	ubyte check_read[6] = {0,0,0,0,0,0}; // Value read.
//	ubyte check_read_ack[6] = {0,0,0,0,0,0}; // Value computed.
//	bool isBadData[6] = {false, false, false, false, false, false};

//	HTSPBsetupIO(sensor_protoboard, mask_write); // `mask_write` happens to conform to the expected format.
//	Joystick_WaitForStart();

//	while (true) {

//		// Restart the communication link.
//		while (isResync==true) {
//			// First make sure we're in sync.
//			int sync_count = 0; // TODO: Use a byte if we want to save memory :P
//			int fail_count = 0; // TODO: If this gets too high, alert the drivers.
//			while (sync_count<6) { // 3 high and 3 low.
//				f_byte_write |= (1<<6); // Set the data bit high.
//				processCommTick();
//				f_byte_read |= 0b11000000; // Make sure the "write" bits aren't random.
//				switch (isClockHigh) { // We want all the bits to be high (0b11111111). The MAGIC_NUM depends on the clock.
//					case false : // These may seem flipped, but that's because the clock is ready for the next tick.
//						f_byte_read = f_byte_read^0b00000000; // MAGIC_NUM, kinda
//						break;
//					case true : // These may seem flipped, but that's because the clock is ready for the next tick.
//						f_byte_read = f_byte_read^0b00111111; // MAGIC_NUM, kinda
//						break;
//				}
//				if (f_byte_read==0b11111111) {
//					sync_count++;
//				} else {
//					sync_count = 0;
//					fail_count++;
//				}
//			}
//			if (isClockHigh==true) {
//				// If so, let it go for another tick.
//				processCommTick();
//			}

//			// Now bring the data line low for 2 clock ticks.
//			f_byte_write &= ~(1<<6); // Clear the data bit low.
//			processCommTick();
//			f_byte_read &= 0b00111111; // Make sure the "write" bits aren't random.
//			if (f_byte_read!=0b00000000) {
//				isResync = true;
//				continue;
//			} else {
//				isResync = false;
//			}
//			processCommTick(); // Wait another tick...
//			f_byte_read &= 0b00111111; // Make sure the "write" bits aren't random.
//			if (f_byte_read!=0b00000000) { // 0, DUH...
//				isResync = true;
//				continue;
//			} else {
//				isResync = false;
//			}
//			// If everything is still good at this point, go on.
//		}

//		// Write header.
//		f_byte_write &= ~(1<<6); // Clear the data bit.

//		// Originally this: f_byte_write |= (header_write<<6); // Set the data bit.
//		// TODO: use ubyte instead of bool and just use last bit.
//		// A bool can be true but not have the last bit be on.
//		if (header_write==true) {
//			f_byte_write |= (1<<6);
//		} else {
//			f_byte_write |= (0<<6);
//		}
//		processCommTick();

//		// Read in all 6 data lines.
//		for (int line=0; line<6; line++) {
//			current_index_mask = 0; // Clear mask.
//			current_index_mask |= (1<<line); // Shift a bit over to be the mask.

//			// No fancy shifting needed here (header_read is a bool).
//			header_read[line] = (bool)(f_byte_read&current_index_mask); // Theoretically, if >0 then true.
//		}

//		// Data:
//		for (int line=0; line<6; line++) {
//			check_read_ack[line] = 0; // Clear parity bits.
//		}
//		for (int bit=0; bit<32; bit++) {
//			// Set MOSI.
//			f_byte_write &= ~(1<<6); // Clear the data bit.
//			current_index_mask = 0; // Clear mask.
//			current_index_mask |= (1<<(bit%8)); // Set the data bit; `i%8` because data is in bytes.

//			// Intentional int division (returns intended byte) (see next statement).
//			// Using a temp var because `true!=1` (can be any positive int); statement
//			// also clears byte_temp because the mask was cleared (and now AND'd).
//			byte_temp = (frame_write[bit/8])&current_index_mask; // TODO: Use current_index_mask instead of temp var?

//			// TODO: combine the two shifts below into one shift.
//			byte_temp = byte_temp>>(bit%8); // Shift data bit over to bit 0.
//			f_byte_write |= (byte_temp<<6); // Set the data bit.

//			check_write = (byte_temp<<(bit/8))^check_write; // This is cleared when we send it.
//			processCommTick();

//			// Read in all 6 data lines (MISO).
//			for (int line=0; line<6; line++) {
//				// TODO: Optimize by (maybe?) making assigning this cyclically.
//				// Would only work for the inner-most loop, since this variable
//				// is reused outside of the loop (for every "for" statement).
//				// Also see note in check bit part about eliminating "for" loop.
//				current_index_mask = 0; // Clear mask.
//				current_index_mask |= (1<<(bit%8)); // Set mask. TODO: Assign this to mask directly (w/out clear)?
//				frame_read[line][bit/8] &= ~(1<<(bit%8)); // Clear bit to read. `bit/8`=current byte, `bit%8`=current bit.
//				byte_temp = f_byte_read&current_index_mask; // Isolating the bit we want. Clears byte_temp 'cause mask was.

//				// TODO: Are there other ways of doing this? Remember the ack is cleared previously.
//				check_read_ack[line] = check_read_ack[line]^((byte_temp>>(bit%8))<<(bit/8));

//				// TODO: combine the two shifts below into one shift. Actually, we might not even need byte_temp here.
//				byte_temp = byte_temp>>(bit%8); // Shift the bit into bit 0.
//				frame_read[line][bit/8] |= (byte_temp<<(bit%8)); // Shift bit into appropriate place in frame. `i/8`=current byte, `i%8`=current bit.
//			}
//		}

//		// Check bits. `bit`="current bit".
//		for (int bit=0; bit<4; bit++) {
//			// Write check bit.
//			f_byte_write &= ~(1<<6); // Clear the data bit.
//			current_index_mask = 0; // Clear mask.
//			current_index_mask |= (1<<bit); // Set the data bit we want to find.

//			// See same operation for data. This is essentially the same logic.
//			byte_temp = check_write&current_index_mask;

//			// TODO: combine the two shifts below into one shift.
//			byte_temp = byte_temp>>bit;
//			f_byte_write |= (byte_temp<<6); // Set the data bit in `f_byte_write`.
//			processCommTick();

//			// Read check bits. TODO: This can be further simplified (take out "for" loop?).
//			// TODO: `bit++` might be evaluated before this "for" loop; need to double-check that.
//			for (int line=0; line<6; line++) {
//				current_index_mask = 0; // Clear the mask.
//				current_index_mask |= (1<<bit); // Select the bit we want to find. TODO: This is already in the correct format! THESE TWO STEPS ARE UNNECESSARY?
//				check_read[line] &= ~(1<<bit); // Clear the bit.
//				check_read[line] |= (f_byte_read&current_index_mask); // Set the bit we read.

//				if (check_read[line]!=check_read_ack[line]) {
//					isBadData[line] = true;
//					error_num++;
//					wasCorrupted = true;
//				} else {
//					isBadData[line] = false;
//				}
//			}
//		}
//		check_write = 0; // Clear this now that we've sent it already.

//		if (error_num>max_error_num) {
//			isResync = true; // This happens at the beginning of the next iteration.
//			error_num = 0;
//			wasCorrupted = false;
//		} else if ((error_num!=0)&&(wasCorrupted==false)) {
//			error_num = 0; // Not a consecutive error.
//		} else if (error_num==0) {
//			wasCorrupted = false;
//		}

//		// TODO: Assign data to whatever the I/O lines are set to.
//		// TODO: Check this section of code for integrity.
//		for (int line=0; line<6; line++) {
//			if (isBadData[line]==true) {
//				continue;
//			}
//			if (header_read[line]==false) {
//				switch (f_commLinkMode) {
//					case COMM_LINK_STD_A :
//						Task_HogCPU(); // So that the main program doesn't try to access these vars.
//						f_pos_x = frame_read[line][0];
//						f_pos_x = f_pos_x<<1; // There's one more bit of data we need to access.
//						f_pos_x |= (frame_read[line][1]>>7); // TODO: This only works if right-shift leaves leading zeroes.
//						Task_ReleaseCPU();
//						Task_HogCPU();
//						f_angle_x = frame_read[line][1];
//						f_angle_x &= 0b01111111;
//						Task_ReleaseCPU();
//						break;
//					case COMM_LINK_STD_B :
//						Task_HogCPU(); // So that the main program doesn't try to access these vars.
//						f_pos_y = frame_read[line][0];
//						f_pos_y = f_pos_y<<1; // There's one more bit of data we need to access.
//						f_pos_y |= (frame_read[line][1]>>7); // TODO: This only works if right-shift leaves leading zeroes.
//						Task_ReleaseCPU();
//						Task_HogCPU();
//						f_angle_y = frame_read[line][1];
//						f_angle_y &= 0b01111111;
//						Task_ReleaseCPU();
//						break;
//					case COMM_LINK_STD_C :
//						//Task_HogCPU(); // So that the main program doesn't try to access these vars.
//						//f_pos_z = frame_read[line][0];
//						//f_pos_z = f_pos_z>>2; // We just read two extra bits.
//						//f_pos_z &= 0b00111111; // TODO: If right-shift results in leading zeroes, we can delete this line.
//						//Task_ReleaseCPU();
//						//Task_HogCPU();
//						//f_angle_z = frame_read[line][1]>>1; // TODO: Only works when shifting right gives leading zeroes.
//						//byte_temp = frame_read[line][0]&0b00000011;
//						//f_angle_z |= (byte_temp<<6); // TODO: See above note about right-shifting.
//						//Task_ReleaseCPU();
//						break;
//					case COMM_LINK_STD_D :
//						break;
//					case COMM_LINK_STD_E :
//						break;
//					case COMM_LINK_STD_F :
//						break;
//					default : // TODO: Purpose is to discard data?
//						break;
//				}
//			} else {
//				// Handle special codes here.
//			}
//		}
//	}
//}



// Task for displaying data on the NXT's LCD screen.
// TODO: Put a lot of the display stuff into loops. Do we want to?
task Display()
{
	typedef enum DisplayMode {
		DISP_FCS,				// Default FCS screen.
		DISP_SWERVE_DEBUG,		// Encoders, target values, PID output, power levels.
		DISP_SWERVE_PID,		// Error, P-term, I-term, D-term.
		DISP_ENCODERS,			// Raw encoder values (7? 8?).
		//DISP_COMM_STATUS,		// Each line of each frame.
		//DISP_SENSORS,			// Might need to split this into two screens.
		DISP_JOYSTICKS,			// For convenience. TODO: Add buttons, D-pad, etc.?
		//DISP_SERVOS,			// Show each servo's position.
		//DISP_TASKS,				// Which tasks are running.
		//DISP_AUTONOMOUS_INFO,	// Misc. status info.
		DISP_NUM,
	};

	DisplayMode isMode = DISP_FCS;
	Task_Spawn(displayDiagnostics); // Explicit here: this is only spawned when buttons are pressed.

	// We don't need to wait for start. ;)

	while (true) {
		Buttons_UpdateData();

		switch (isMode) {
			case DISP_FCS :
				break;
			case DISP_SWERVE_DEBUG :
				// The value of `pod_current[i]` is (should be?) between 0~360.
				nxtDisplayTextLine(0, "FR rot%3d tgt%3d", pod_current[POD_FR], g_ServoData[POD_FR].angle);
				nxtDisplayTextLine(1, "FL rot%3d tgt%3d", pod_current[POD_FL], g_ServoData[POD_FL].angle);
				nxtDisplayTextLine(2, "BL rot%3d tgt%3d", pod_current[POD_BL], g_ServoData[POD_BL].angle);
				nxtDisplayTextLine(3, "BR rot%3d tgt%3d", pod_current[POD_BR], g_ServoData[POD_BR].angle);
				nxtDisplayTextLine(4, " chg%+4d pow%+4d", correction_pod[POD_FR], g_MotorData[POD_FR].power);
				nxtDisplayTextLine(5, " chg%+4d pow%+4d", correction_pod[POD_FL], g_MotorData[POD_FL].power);
				nxtDisplayTextLine(6, " chg%+4d pow%+4d", correction_pod[POD_BL], g_MotorData[POD_BL].power);
				nxtDisplayTextLine(7, " chg%+4d pow%+4d", correction_pod[POD_BR], g_MotorData[POD_BR].power);
				break;
			case DISP_SWERVE_PID :
				nxtDisplayTextLine(0, "FR err%d P:%d", error_pod[POD_FR], term_P_pod[POD_FR]);
				nxtDisplayTextLine(1, "FL err%d P:%d", error_pod[POD_FL], term_P_pod[POD_FL]);
				nxtDisplayTextLine(2, "BL err%d P:%d", error_pod[POD_BL], term_P_pod[POD_BL]);
				nxtDisplayTextLine(3, "BR err%d P:%d", error_pod[POD_BR], term_P_pod[POD_BR]);
				nxtDisplayTextLine(4, " I:%+4d D:%+4d", term_I_pod[POD_FR], term_D_pod[POD_FR]);
				nxtDisplayTextLine(5, " I:%+4d D:%+4d", term_I_pod[POD_FL], term_D_pod[POD_FL]);
				nxtDisplayTextLine(6, " I:%+4d D:%+4d", term_I_pod[POD_BL], term_D_pod[POD_BL]);
				nxtDisplayTextLine(7, " I:%+4d D:%+4d", term_I_pod[POD_BR], term_D_pod[POD_BR]);
				break;
			case DISP_ENCODERS :
				nxtDisplayTextLine(0, "FR:   %+6d", encoder_pod[POD_FR]);
				nxtDisplayTextLine(1, "FL:   %+6d", encoder_pod[POD_FL]);
				nxtDisplayTextLine(2, "BL:   %+6d", encoder_pod[POD_BL]);
				nxtDisplayTextLine(3, "BR:   %+6d", encoder_pod[POD_BR]);
				nxtDisplayTextLine(4, "Lift: %+6d", lift_pos);
				nxtDisplayTextLine(5, "Gyro: %+6d", f_angle_z);
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
	for (int i=0; i<100; i++) { // MAGIC_NUM: 100=120-20
		Time_Wait(1000);
	}
	Servo_SetPosition(servo_climb_L, servo_climb_L_open);
	Servo_SetPosition(servo_climb_R, servo_climb_R_open);
}



task SaveData()
{
	TFileHandle IO_handle;
	TFileIOResult IO_result;
	const string filename_pods = "_reset_pods.txt";
	int file_size = 72; // Should be 64 (4 shorts).
	Task_HogCPU();
	Delete(filename_pods, IO_result); // TODO: Add error handling.
	OpenWrite(IO_handle, IO_result, filename_pods, file_size); // Size set (correctly?) earlier.
	for (int i=POD_FR; i<(int)POD_NUM; i++) {
		WriteShort(IO_handle, IO_result, (short)round(pod_current[i]));
	}
	Close(IO_handle, IO_result);
	Task_ReleaseCPU();
}



task Autonomous()
{
	isAutonomous = true;

	while (true) {
		if (isAutonomous==false) {
			break;
		}
		// TODO: Do stuff. I have no idea how.
	}

	isAutonomous = false;
	Task_Kill(Autonomous);
}
