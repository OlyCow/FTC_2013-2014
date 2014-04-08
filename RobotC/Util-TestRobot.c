#pragma config(Hubs,  S1, HTServo,  HTServo,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTServo,  HTServo,  HTMotor,  HTMotor)
#pragma config(Sensor, S3,     sensor_IR,      sensorI2CCustomFastSkipStates9V)
#pragma config(Sensor, S4,     sensor_protoboard, sensorI2CCustom9V)
#pragma config(Motor,  motorA,          motor_assist_L,   tmotorNXT, PIDControl, reversed, encoder)
#pragma config(Motor,  motorB,          motor_assist_R,   tmotorNXT, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     motor_BR,         tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_2,     motor_lift_back,  tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     motor_lift_front, tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C4_2,     motor_FR,         tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C3_1,     motor_FL,         tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C3_2,     motor_sweeper,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C4_1,     motor_flag,       tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S2_C4_2,     motor_BL,         tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S1_C1_1,    servo_flip_R,         tServoStandard)
#pragma config(Servo,  srvo_S1_C1_2,    servo_auton,          tServoStandard)
#pragma config(Servo,  srvo_S1_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_6,    servo_omni_R,         tServoStandard)
#pragma config(Servo,  srvo_S1_C2_1,    servo7,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_2,    servo8,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_3,    servo_climb_R,        tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo_FR,             tServoStandard)
#pragma config(Servo,  srvo_S1_C2_6,    servo_BR,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_1,    servo_dump,           tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    servo14,              tServoNone)
#pragma config(Servo,  srvo_S2_C1_3,    servo15,              tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    servo16,              tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo17,              tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo_climb_L,        tServoStandard)
#pragma config(Servo,  srvo_S2_C2_1,    servo_omni_L,         tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    servo20,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_3,    servo21,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_4,    servo_flip_L,         tServoStandard)
#pragma config(Servo,  srvo_S2_C2_5,    servo_FL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C2_6,    servo_BL,             tServoStandard)

#include "includes.h"
#include "swerve-drive.h"

task PID(); // Sets winch servos' position, wheel pod motors' power, and lift motor's power. Others set in main.

// For PID:
float power_lift = 0.0;
int lift_target = 0;
float lift_pos = 0.0; // Really should be an int; using a float so I don't have to cast all the time.



task main()
{
	initializeGlobalVariables(); // Defined in "initialize.h", this intializes all struct members.
	initializeRobotVariables();
	Task_Kill(displayDiagnostics); // This is set separately in the "Display" task.
	Task_Spawn(PID);

	const int TEST_INTERVAL			= 200;
	const int LIFT_TEST_POS			= 1200;
	const int LIFT_TEST_INTERVAL	= 1300; // How much time to wait for lift to reach pos.
	const int PICKUP_LOWER_DELAY    = 1000;
	const int PICKUP_TEST_DELAY     = 1000;
	const int PICKUP_REVERSE_DELAY  = 1000;
	const int PICKUP_RAISE_DELAY    = 1000;
	const int FLAG_TEST_DELAY		= 1000;
	const int AUTON_TEST_DELAY		= 1000;

	// Move each drive motor back and forth.
	for (int i=POD_FR; i<(int)POD_NUM; i++) {
		Motor_SetPower(g_FullPower, Motor_Convert((WheelPod)i));
		Time_Wait(TEST_INTERVAL);
		Motor_SetPower(0, Motor_Convert((WheelPod)i));
		Time_Wait(TEST_INTERVAL);
		Motor_SetPower(-g_FullPower, Motor_Convert((WheelPod)i));
		Time_Wait(TEST_INTERVAL);
		Motor_SetPower(0, Motor_Convert((WheelPod)i));
	}

	// Move lift up a bit and dump; move lift back down.
	lift_target = LIFT_TEST_POS;
	Time_Wait(LIFT_TEST_INTERVAL);
	dumpCubes(1); // MAGIC_NUM: Just a short test.
	lift_target = lift_pos_pickup;
	Time_Wait(LIFT_TEST_INTERVAL);


	// Move vertical roller down.
	Servo_SetPosition(servo_flip_L, servo_flip_L_down);
	Servo_SetPosition(servo_flip_R, servo_flip_R_down);
	Time_Wait(PICKUP_LOWER_DELAY);

	// Turn pickup on and off.
	Motor_SetPower(g_FullPower, motor_sweeper);
	Motor_SetPower(g_FullPower, motor_assist_L);
	Motor_SetPower(g_FullPower, motor_assist_R);
	Time_Wait(PICKUP_TEST_DELAY);

	// Reverse vertical roller.
	Motor_SetPower(-g_FullPower, motor_sweeper);
	Motor_SetPower(-g_FullPower, motor_assist_L);
	Motor_SetPower(-g_FullPower, motor_assist_R);
	Time_Wait(PICKUP_REVERSE_DELAY);

	// Turn vertical roller off.
	Motor_SetPower(0, motor_sweeper);
	Motor_SetPower(0, motor_assist_L);
	Motor_SetPower(0, motor_assist_R);

	// Move vertical roller up.
	Servo_SetPosition(servo_flip_L, servo_flip_L_up);
	Servo_SetPosition(servo_flip_R, servo_flip_R_up);
	Time_Wait(PICKUP_RAISE_DELAY);

	// Spin flag.
	Motor_SetPower(g_FullPower, motor_flag);
	Time_Wait(FLAG_TEST_DELAY);
	Motor_SetPower(0, motor_flag);

	// Flip autonomous servo.
	Servo_SetPosition(servo_auton, 255);
	Time_Wait(AUTON_TEST_DELAY);
	Servo_SetPosition(servo_auton, 0);
	Time_Wait(AUTON_TEST_DELAY);
	Servo_SetPosition(servo_auton, 128);

	// Play loud warning noise.
	//TODO
	// Release climbing on non-NXt side.
	// Release climbing on other side.
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

		//Task_ReleaseCPU();
		//Task_EndTimeslice(); // TODO: Is this command superfluous? (This needs a check on the forums.)
	}
}
