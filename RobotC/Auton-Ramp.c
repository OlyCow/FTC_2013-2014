#pragma config(Hubs,  S1, HTServo,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTMotor,  HTServo,  none,     none)
#pragma config(Sensor, S3,     sensor_IR,      sensorI2CCustomFastSkipStates9V)
#pragma config(Sensor, S4,     sensor_protoboard, sensorI2CCustomFastSkipStates9V)
#pragma config(Motor,  mtr_S1_C2_1,     motor_D,       tmotorTetrix, openLoop)
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

task Drive();
task PID(); // Sets CR-servos' power, wheel pod motors' power, and lift motor's power. Others set in main.
task Display(); // A separate task for updating the NXT's LCD display.
task SaveData();

// 1 = L, -1 = R; this should only affect horizontal movements.
const int AUTON_L_R = 1;

// true = wait 15 seconds before starting, false = no delay.
const bool AUTON_WAIT = false;

// For main task:
float power_lift = 0.0;
int lift_target = 0;
float term_P_pod[POD_NUM] = {0,0,0,0};
float translation_x = 0.0;
float translation_y = 0.0;
float rotation_global = 0.0;
float heading = 0.0;
float fine_tune_factor = 1.0;

// For PID:
float term_I_pod[POD_NUM] = {0,0,0,0};
float term_D_pod[POD_NUM] = {0,0,0,0};
float encoder_pod[POD_NUM] = {0,0,0,0};
float pod_current[POD_NUM] = {0,0,0,0};
float pod_raw[POD_NUM] = {0,0,0,0};
float error_pod[POD_NUM] = {0,0,0,0}; // Difference between set-point and measured value.
float correction_pod[POD_NUM] = {0,0,0,0}; // Equals "term_P + term_I + term_D".

float lift_pos = 0.0; // Really should be an int; using a float so I don't have to cast all the time.
const int max_lift_height = 5800; // MAGIC_NUM. TODO: Find this value.



task main()
{
	initializeGlobalVariables(); // Defined in "initialize.h", this intializes all struct members.
	initializeRobotVariables();
	Task_Kill(displayDiagnostics); // This is set separately in the "Display" task.
	Task_Spawn(Drive);
	Task_Spawn(PID);
	Task_Spawn(Display);

	// Small little things for gyro correction.
	bool isRotating = true;
	float error = 0.0;

	const int initialize_delay	= 500;
	const int wait_delay		= 15*1000;
	const int pause_delay		= 500;
	const int align_delay		= 600;

	const int LIFT_LOW_POS		= 0;
	const int LIFT_MED_POS		= 1500;
	const int LIFT_HIGH_POS		= 2000;

	const int move_to_basket_time	= 700; // Wild guess. As are the following.
	const int approach_basket_time	= 600;
	const int forward_to_ramp_time	= 900;
	const int move_onto_ramp_time	= 1800;

	Joystick_WaitForStart();
	if (AUTON_WAIT==true) {
		Time_Wait(wait_delay);
	} else {
		Time_Wait(initialize_delay);
	}

	// Raise the lift and move forward as far as the baskets.
	fine_tune_factor = 0.0;
	translation_y = 40; // MAGIC_NUM
	Time_Wait(align_delay);
	fine_tune_factor = 1.0;
	Time_Wait(move_to_basket_time);
	translation_y = 0;
	Time_Wait(pause_delay);

	// Turn so robot is "0" degrees and raise the lift.
	fine_tune_factor = 0.0;
	rotation_global = 40;
	Time_Wait(align_delay);
	lift_target = LIFT_HIGH_POS;
	fine_tune_factor = 1.0;
	while (isRotating) {
		error = (-90)-heading; // MAGIC_NUM
		rotation_global = error*1.2; // MAGIC_NUM
		if (abs(error)<2.5) { // MAGIC_NUM
			isRotating = false;
		}
		Time_Wait(5); // MAGIC_NUM
	}
	rotation_global = 0;
	isRotating = true;
	Time_Wait(pause_delay);

	// Move forward a bit and dump the cubes, then lower the lift.
	fine_tune_factor = 0.0;
	translation_x = -40; // MAGIC_NUM
	Time_Wait(align_delay);
	fine_tune_factor = 1.0;
	Time_Wait(approach_basket_time);
	translation_x = 0;
	Time_Wait(pause_delay);
	dumpCubes(4);
	lift_target = LIFT_LOW_POS;

	// Back up a bit.
	fine_tune_factor = 0.0;
	translation_x = 40; // MAGIC_NUM
	Time_Wait(align_delay);
	fine_tune_factor = 1.0;
	Time_Wait(approach_basket_time);
	translation_x = 0;
	Time_Wait(pause_delay);

	// Get lined up with the ramp.
	fine_tune_factor = 0.0;
	translation_y = 40; // MAGIC_NUM
	Time_Wait(align_delay);
	fine_tune_factor = 1.0;
	Time_Wait(forward_to_ramp_time);
	translation_y = 0;
	Time_Wait(pause_delay);

	// Make sure the robot is "0" degrees again.
	fine_tune_factor = 0.0;
	rotation_global = 40;
	Time_Wait(align_delay);
	lift_target = LIFT_HIGH_POS;
	fine_tune_factor = 1.0;
	while (isRotating) {
		error = (-90)-heading; // MAGIC_NUM
		rotation_global = error*1.2; // MAGIC_NUM
		if (abs(error)<2.5) { // MAGIC_NUM
			isRotating = false;
		}
		Time_Wait(5); // MAGIC_NUM
	}
	isRotating = true;
	rotation_global = 0;
	Time_Wait(pause_delay);

	// Move onto the ramp.
	fine_tune_factor = 0.0;
	translation_x = -60; // MAGIC_NUM
	Time_Wait(align_delay);
	fine_tune_factor = 1.0;
	Time_Wait(move_onto_ramp_time);
	translation_x = 0;
	Time_Wait(pause_delay);

	// Done! Save pod data.
	Task_Spawn(SaveData);
	Task_Spawn(SaveData);
	Time_Wait(pause_delay);
	Task_Spawn(SaveData);
	Task_Spawn(SaveData);
}



task Drive()
{

	// Not initializing these structs for now: once data starts coming in
	// from the controllers, all the members of these will get updated.
	vector2D rotation[POD_NUM];
	vector2D translation; // Not a struct because all wheel pods share the same values.
	vector2D combined[POD_NUM]; // The averaged values: angle is pod direction, magnitude is power.
	float combined_angle_prev[POD_NUM] = {0,0,0,0}; // Prevents atan2(0,0)=0 from resetting the wheel pods to 0. `90` starts facing forward.
	bool shouldNormalize = false; // Set if motor values go over 100. All wheel pod power will be scaled down.
	const int maxTurns = 2; // On each side. To prevent the wires from getting too twisted.

	Joystick_WaitForStart();
	Time_ClearTimer(T3); // We will use this to guage the loop time for driving.
	heading = -45.0; // MAGIC_NUM

	while (true) {
		Joystick_UpdateData();

		heading += (float)HTGYROreadRot(sensor_protoboard)*(float)Time_GetTime(T3)/(float)1000.0;
		Time_ClearTimer(T3);

		// A rotation vector is added to translation vector, and the resultant vector
		// is normalized. A differential analysis of the parametric equations of
		// each wheel pod confirms that the above algorithm works perfectly, despite
		// its apparent simplicity. Use of the Vector2D library makes some of this
		// slightly less efficient (there are some unnecessary update calculations)
		// but the benefit of increased readability is well worth it.
		translation.x = translation_x;
		translation.y = translation_y;
		Vector2D_UpdateRot(translation);
		Vector2D_Rotate(translation, -heading);
		for (int i=POD_FR; i<(int)POD_NUM; i++) {
			rotation[i].r = rotation_global;
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
			float originalMaxPower = g_FullPower; // If there was a false positive, this ensures nothing changes.
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
		for (int i=POD_FR; i<(int)POD_NUM; i++) {
			g_MotorData[i].fineTuneFactor = fine_tune_factor;
		}
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
	float kP[POD_NUM] = {1.1,	1.1,	1.1,	1.1}; // MAGIC_NUM: TODO: PID tuning.
	float kI[POD_NUM] = {0.0,	0.0,	0.0,	0.0};
	float kD[POD_NUM] = {0.0,	0.0,	0.0,	0.0};
	float error_prev_pod[POD_NUM] = {0,0,0,0}; // Easier than using the `error_accumulated` array, and prevents the case where that array is size <=1.
	float error_rate_pod[POD_NUM] = {0,0,0,0};
	Aligned isAligned = ALIGNED_CLOSE; // If false, cut motor power so that wheel pod can get aligned.
	const int turnLimit = 3; // On each side. To prevent the wires from getting too twisted.
	int pod_pos_prev[POD_NUM] = {0,0,0,0};

	// Variables for lift PID calculations.
	float kP_lift_up	= 0.3; // TODO: PID tuning. MAGIC_NUM.
	float kP_lift_down	= 0.085;
	float kD_lift_up	= 0.0;
	float kD_lift_down	= 0.0;
	float error_lift = 0.0;
	float error_prev_lift = 0.0;
	float error_rate_lift = 0.0;
	float term_P_lift = 0.0;
	float term_D_lift = 0.0;

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
		nxtDisplayTextLine(7, "%d", isAligned);

		// "Damp" motors depending on how far the wheel pods are from their targets.
		for (int i=MOTOR_FR; i<(int)MOTOR_NUM; i++) {
			switch (isAligned) {
				case ALIGNED_FAR:
					g_MotorData[i].fineTuneFactor *= 0; // Zeroes motor power.
					break;
				case ALIGNED_MEDIUM:
					g_MotorData[i].fineTuneFactor *= 1;
					//g_MotorData[i].fineTuneFactor *= 1/abs(error_pod[i])*10; // Ranges from 28~83%
					break;
				case ALIGNED_CLOSE:
					g_MotorData[i].fineTuneFactor *= 1;
					break;
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
		//Motor_SetPower(power_lift, motor_lift);
	}
}



// Task for displaying data on the NXT's LCD screen.
// TODO: Put a lot of the display stuff into loops. Do we want to?
task Display()
{
	typedef enum DisplayMode {
		DISP_FCS,				// Default FCS screen.
		DISP_SWERVE_DEBUG,		// Encoders, target values, PID output, power levels.
		DISP_SWERVE_PID,		// Error, P-term, I-term, D-term.
		DISP_ENCODERS,			// Raw encoder values (7? 8?).
		DISP_JOYSTICKS,			// For convenience. TODO: Add buttons, D-pad, etc.?
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
				//nxtDisplayTextLine(4, " chg%+4d pow%+4d", correction_pod[POD_FR], g_MotorData[POD_FR].power);
				//nxtDisplayTextLine(5, " chg%+4d pow%+4d", correction_pod[POD_FL], g_MotorData[POD_FL].power);
				//nxtDisplayTextLine(6, " chg%+4d pow%+4d", correction_pod[POD_BL], g_MotorData[POD_BL].power);
				//nxtDisplayTextLine(7, " chg%+4d pow%+4d", correction_pod[POD_BR], g_MotorData[POD_BR].power);

				nxtDisplayTextLine(4, " chg%+4d pow%+4d", correction_pod[POD_FR], g_MotorData[POD_FR].fineTuneFactor);
				nxtDisplayTextLine(5, " chg%+4d pow%+4d", correction_pod[POD_FL], g_MotorData[POD_FL].fineTuneFactor);
				nxtDisplayTextLine(6, " chg%+4d pow%+4d", correction_pod[POD_BL], g_MotorData[POD_BL].fineTuneFactor);
				nxtDisplayTextLine(7, " chg%+4d pow%+4d", correction_pod[POD_BR], g_MotorData[POD_BR].fineTuneFactor);
				break;
			case DISP_SWERVE_PID :
				nxtDisplayTextLine(0, "FR err%d P:%d", error_pod[POD_FR], term_P_pod[POD_FR]);
				nxtDisplayTextLine(1, "FL err%d P:%d", error_pod[POD_FL], term_P_pod[POD_FL]);
				nxtDisplayTextLine(2, "BL err%d P:%d", error_pod[POD_BL], term_P_pod[POD_BL]);
				nxtDisplayTextLine(3, "BR err%d P:%d", error_pod[POD_BR], term_P_pod[POD_BR]);
				nxtDisplayTextLine(4, "FR I:%d D:%d", term_I_pod[POD_FR], term_D_pod[POD_FR]);
				nxtDisplayTextLine(5, "FL I:%d D:%d", term_I_pod[POD_FL], term_D_pod[POD_FL]);
				nxtDisplayTextLine(6, "BL I:%d D:%d", term_I_pod[POD_BL], term_D_pod[POD_BL]);
				nxtDisplayTextLine(7, "BR I:%d D:%d", term_I_pod[POD_BR], term_D_pod[POD_BR]);
				break;
			case DISP_ENCODERS :
				nxtDisplayTextLine(0, "FR:   %+6d", pod_raw[POD_FR]);
				nxtDisplayTextLine(1, "FL:   %+6d", pod_raw[POD_FL]);
				nxtDisplayTextLine(2, "BL:   %+6d", pod_raw[POD_BL]);
				nxtDisplayTextLine(3, "BR:   %+6d", pod_raw[POD_BR]);
				nxtDisplayTextLine(4, "Lift: %+6d", lift_pos);
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
