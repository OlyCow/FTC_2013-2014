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
task CommLink(); // R/W to the prototype board as tightly as possible. TODO: Separate into its own library.
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

// Motor Assignments!
tMotor omniL = motor_FL;
tMotor omniR = motor_FR;

// Program settings:
bool DO_DELAY_AT_START	= false;
bool IS_FIRST_TURN_L	= true;
bool DO_DUMP_AUTON		= true;
bool DO_ATTEMPT_RAMP	= false;

// For debugging display.
float pos_L = 0.0;
float pos_R = 0.0;
float pos_avg = 0.0;
float error = 0.0;

// Gyro readings:
float heading = 0.0; // Because f_angle_z is an int.

// For PID:
float power_lift = 0.0;
int lift_target = 0;
float lift_pos = 0.0; // Really should be an int; using a float so I don't have to cast all the time.

// For comms link:
// TODO: Make more efficient by putting vars completely inside bytes, etc.
// If I had STL at my disposal much std::vector<bool> would happen here.
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

void config_values(	bool &key,	string txt_disp,
					bool val_L,	string txt_L,
					bool val_R,	string txt_R);
void DumpAutonCube();
void LowerAutonArm();
void MoveForward(float inches, bool doBrake=true);
void MoveBackward(float inches, bool doBrake=true);
void ChargeForward(int milliseconds, int power, bool doLowerOmni, bool doBrake=true);
void ChargeBackward(int milliseconds, int power, bool doLowerOmni, bool doBrake=true);
void TurnLeft(int degrees);
void TurnRight(int degrees);
void Settle();



task main()
{
	initializeGlobalVariables(); // Defined in "initialize.h", this intializes all struct members.
	initializeRobotVariables();
	Servo_SetPosition(servo_omni_L, servo_omni_L_down);
	Servo_SetPosition(servo_omni_R, servo_omni_R_down);
	Task_Kill(displayDiagnostics); // This is set separately in the "Display" task.
	Task_Spawn(Gyro);
	//Task_Spawn(PID);
	//Task_Spawn(CommLink);
	Task_Spawn(Display);

	string a = "Delayed start?";
	string b = "YES";
	string c = "NO";
	//config_values(DO_DELAY_AT_START, a, true, b, false, c);

	Joystick_WaitForStart();
	heading = 0.0;
	Motor_ResetEncoder(omniL);
	Motor_ResetEncoder(omniR);

	if (DO_DELAY_AT_START==true) {
		for (int i=0; i<10; i++) {
			Time_Wait(1000);
		}
	}

	//MoveForward(60);
	//TurnLeft(90);
	//MoveForward(50);

	Motor_ResetEncoder(omniL);
	Motor_ResetEncoder(omniR);
	float kp = -0.02;
	float power_L = 0.0;
	float power_R = 0.0;

	while (true) {
		pos_L = Motor_GetEncoder(omniL);
		pos_R = -Motor_GetEncoder(omniR);

		if (abs(pos_L) > 120) {
			power_L = kP * pos_L;
			if (power_L < 20) {
				power_L = 20;
			}
		} else {
			power_L = 0;
		}
		if (abs(pos_R) > 120) {
			power_R = kP * pos_R;
			if (power_R < 20) {
				power_R = 20;
			}
		} else {
			power_R = 0;
		}

		Motor_SetPower(power_L, motor_FL);
		Motor_SetPower(power_L, motor_BL);
		Motor_SetPower(power_R, motor_FR);
		Motor_SetPower(power_R, motor_BR);

		Time_Wait(10);
	}
}

void config_values(	bool &key,	string txt_disp,
					bool val_L,	string txt_L,
					bool val_R,	string txt_R)
{
	Task_Kill(Display);
	bDisplayDiagnostics = false;
	Display_Clear();

	bool isConfig = true;
	string arrow = "-> ";
	string blank = "   ";
	string disp_L = "";
	string disp_R = "";
	bool isL = true;
	bool isR = false;

	while (isConfig==true) {
		Buttons_UpdateData();
		if (Buttons_Released(NXT_BUTTON_YES)==true) {
			isConfig = false;
		} else if ((Buttons_Released(NXT_BUTTON_L)||Buttons_Released(NXT_BUTTON_R))==true) {
			isL = !isL;
			isR = !isR;
		}

		if (isL==true) {
			disp_L = arrow + txt_L;
		} else {
			disp_L = blank + txt_L;
		}
		if (isR==true) {
			disp_R = arrow + txt_R;
		} else {
			disp_R = blank + txt_R;
		}

		nxtDisplayString(0, txt_disp);
		nxtDisplayString(2, disp_L);
		nxtDisplayString(3, disp_R);

		Time_Wait(10);
	}
	if (isL==true) {
		isConfig = val_L;
	} else if (isR==true) {
		isConfig = val_R;
	}

	bDisplayDiagnostics = true;
	Task_Spawn(Display);
}
void DumpAutonCube()
{
	const int lower_delay = 480;
	const int drop_delay = 1000;
	const int raise_delay = 500;
	Servo_SetPosition(servo_auton, servo_auton_down);
	Time_Wait(lower_delay);
	Servo_SetPosition(servo_auton, servo_auton_hold);
	Time_Wait(drop_delay);
	Servo_SetPosition(servo_auton, servo_auton_up);
	Time_Wait(raise_delay);
	Servo_SetPosition(servo_auton, servo_auton_hold);
}
void LowerAutonArm()
{
	const int lower_delay = 1200;
	Servo_SetPosition(servo_auton, servo_auton_down);
	Time_Wait(lower_delay);
	Servo_SetPosition(servo_auton, servo_auton_hold);
}
void MoveForward(float inches, bool doBrake)
{
	float target = inches*152.789;
	float power = 0.0;
	float kP = 0.08;
	bool isMoving = true;

	Motor_ResetEncoder(omniL);
	Motor_ResetEncoder(omniR);

	while (isMoving==true){
		pos_L = Motor_GetEncoder(omniL);
		pos_R = -Motor_GetEncoder(omniR);
		pos_avg = (pos_L+pos_R)/2.0;
		error = target-pos_avg;
		if (error>3000) {
			power = g_FullPower;
		} else if (error<-3000) {
			power = -g_FullPower;
		} else {
			power = kP*error;
		}
		if (abs(power)<10) {
			if (power>0) {
				power = 10;
			} else if (power<0) {
				power = -10;
			}
		}
		power = Math_Limit(power, g_FullPower);
		Motor_SetPower(power, motor_FL);
		Motor_SetPower(power, motor_BL);
		Motor_SetPower(power, motor_FR);
		Motor_SetPower(power, motor_BR);
		if (abs(error)<50) {
			//if (abs(vel)<20) {
				isMoving = false;
			//}
		}
	}
	if (doBrake==true) {
		Motor_SetPower(0, motor_FL);
		Motor_SetPower(0, motor_BL);
		Motor_SetPower(0, motor_FR);
		Motor_SetPower(0, motor_BR);
	}
}
void MoveBackward(float inches, bool doBrake)
{
	MoveForward(-inches, doBrake);
}
void ChargeForward(int milliseconds, int power, bool doLowerOmni, bool doBrake)
{

	Servo_SetPosition(servo_omni_L, servo_omni_L_up);
	Servo_SetPosition(servo_omni_R, servo_omni_R_up);

	for (int i=0; i<1; i++)
{
	Motor_SetPower(power, motor_FL);
	Motor_SetPower(power, motor_BL);
	Motor_SetPower(power, motor_FR);
	Motor_SetPower(power, motor_BR);
	Time_Wait(1);
	}
	if (doBrake==true) {
		Motor_SetPower(0, motor_FL);
		Motor_SetPower(0, motor_BL);
		Motor_SetPower(0, motor_FR);
		Motor_SetPower(0, motor_BR);
	}
	if (doLowerOmni==true) {
		Servo_SetPosition(servo_omni_L, servo_omni_L_down);
		Servo_SetPosition(servo_omni_R, servo_omni_R_down);
	}
}
void ChargeBackward(int milliseconds, int power, bool doLowerOmni, bool doBrake)
{
	ChargeForward(milliseconds, power, doLowerOmni, doBrake);
}
void TurnLeft(int degrees)
{
	bool isTurning = true;
	float start_pos = heading;
	float current_pos = heading;
	float target = start_pos-(float)degrees;
	float power = 0.0;
	float power_neg = 0.0;
	float kP = 5.4;
	bool isFineTune = false;
	int finish_timer = 0;

	while (isTurning==true) {
		current_pos = heading;
		error = target-current_pos;
		if (error>60) {
			power = g_FullPower;
		} else if (error<-60) {
			power = -g_FullPower;
		} else {
			power = kP*error;
		}
		if (abs(power)<15) {
			if (power>0) {
				power = 15;
			} else if (power<0) {
				power = -15;
			}
		}
		power_neg = -power;
		Motor_SetPower(power, motor_FL);
		Motor_SetPower(power, motor_BL);
		Motor_SetPower(power_neg, motor_FR);
		Motor_SetPower(power_neg, motor_BR);
		if (abs(error)<1) {
			if (isFineTune==false) {
				Time_ClearTimer(finish_timer);
				isFineTune = true;
			} else if (Time_GetTime(finish_timer)>500) {
				isTurning = false;
			}
		}
	}
	Motor_SetPower(0, motor_FL);
	Motor_SetPower(0, motor_BL);
	Motor_SetPower(0, motor_FR);
	Motor_SetPower(0, motor_BR);
}
void TurnRight(int degrees)
{
	TurnLeft(-degrees);
}
void Settle()
{
	Time_Wait(500);
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

		Time_Wait(4);

		//Task_ReleaseCPU();
		//Task_EndTimeslice(); // TODO: Is this command superfluous? (This needs a check on the forums.)
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
		DISP_ENCODERS,			// Raw encoder values.
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
				nxtDisplayTextLine(0, "Lift: %+6d", lift_pos);
				nxtDisplayTextLine(1, "Gyro: %+6d", heading);
				nxtDisplayTextLine(2, "FRpow %+4d", g_MotorData[POD_FR].power);
				nxtDisplayTextLine(3, "FLpow %+4d", g_MotorData[POD_FL].power);
				nxtDisplayTextLine(4, "Error: %i", error);
				nxtDisplayTextLine(5, "avg pos: %i", pos_avg);
				nxtDisplayTextLine(6, "encdr R: %i", pos_L);
				nxtDisplayTextLine(7, "encdr L: %i", pos_R);
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
				int temp_x = f_angle_x - 30;	// TODO: Make this not ugly.
				int temp_y = f_angle_y - 30;
				nxtDisplayTextLine(2, "(%3d,%3d,%3d)", temp_x, temp_y, f_angle_z);
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
