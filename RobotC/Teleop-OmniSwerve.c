#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S2, HTServo,  HTServo,  none,     none)
#pragma config(Sensor, S3,     ,               sensorI2CCustom)
#pragma config(Sensor, S4,     ,               sensorI2CCustom9V)
#pragma config(Motor,  mtr_S1_C1_1,     motor_FL,      tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motor_FR,      tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motor_BL,      tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motor_BR,      tmotorTetrix, PIDControl, encoder)
#pragma config(Servo,  srvo_S2_C1_1,    servo_FR,             tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_2,    servo_FL,             tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_3,    servo_BL,             tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_4,    servo_BR,             tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_1,    servo7,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_2,    servo8,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_3,    servo9,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo12,              tServoNone)

#include "Headers\includes.h"
#include "Teleop-OmniSwerve.h"

#pragma autoStartTasks



task PID();

task main() {
	initializeGlobalVariables();
	g_task_main = Task_GetCurrentIndex();
	float gyro_angle = 0;
	float rotation_magnitude = 0;
	float rotation_angle[4] = {0,0,0,0}; //4=# of drive base motors
	float rotation_x[4] = {0,0,0,0}; //4=# of drive base motors
	float rotation_y[4] = {0,0,0,0}; //4=# of drive base motors
	float translation_magnitude = 0;
	float translation_angle = 0;
	float translation_x = 0;
	float translation_y = 0;
	float combined_input_magnitude = 0;
	float combined_angle[4] = {0,0,0,0}; //4=# of drive base motors
	float combined_x[4] = {0,0,0,0}; //4=# of drive base motors
	float combined_y[4] = {0,0,0,0}; //4=# of drive base motors
	float motor_power[4] = {0,0,0,0}; //4=# of drive base motors
	float servo_angle[4] = {0,0,0,0}; //4=# of drive base motors
	Joystick_WaitForStart();

	while (true) {
		Joystick_UpdateData();

		// This part is temporary. We are assigning a gyro angle from a joystick.
		// An operator moves the joystick to correspond with a flag mounted on the
		// frame of the drive base, to simulate the input from an actual joystick.
		// No word as to when we will be able to purchase a prototype board and
		// the MPU-6050 breakout board.
		gyro_angle = Math_RadToDeg(atan2(
							Joystick_Joystick(JOYSTICK_R, AXIS_Y, CONTROLLER_2),
							Joystick_Joystick(JOYSTICK_R, AXIS_X, CONTROLLER_2) ));

		// Actual code starts here. I would advise against touching it (or even
		// looking at it). It is also difficult to understand without looking at
		// documentation (which doesn't exist yet) or the engineering notebook
		// (I haven't finished writing that either). The following is an attempt
		// at describing the system without using any diagrams (O.o).
		// Basically, the orientation of the continuous rotation (CR) servos can
		// be divided into two cases: where they are all facing the same direction,
		// or if they are arranged in a "circle" (all rotated 45 deg). If there is
		// any rotation component to the input(s), then the CR servos are rotated
		// to the latter case (else, the former).
		// The position that the servo needs to move to is then sent to a different
		// task: a PID control loop that constantly monitors the input from the
		// respective encoder, and tries to move the servo to the position fed to
		// it by `task main()`.
		// When the input is completely linear, the motor is simply assigned a
		// (normalized) value of the joystick's magnitude. When there is a rotation
		// component involved, then the two are added together and normalized. It
		// is only normalized in this case if the sum of the two magnitudes exceeds
		// 100% motor power. (Of course, this is after the joystick values are
		// themselves normalized from 127 to 100.) The translation and rotation
		// vectors on each servo are then combined, and then the "cross product"
		// is taken in the direction the servo is pointing (finding the component
		// of the vector in the same direction as the servo is pointing). That
		// equals the power assigned to the motor of that wheel pod.
		rotation_magnitude = Joystick_GetRotationMagnitude();
		translation_x = Joystick_GetTranslationX();
		translation_y = Joystick_GetTranslationY();
		translation_magnitude = sqrt(pow(translation_x,2)+pow(translation_y,2)); //Pythagoras
		combined_input_magnitude = translation_magnitude+rotation_magnitude;
		if (combined_input_magnitude>g_FullPower) {
			Math_Normalize(rotation_magnitude, g_JoystickMax, combined_input_magnitude);
			Math_Normalize(translation_magnitude, g_JoystickMax, combined_input_magnitude);
		}
		for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
			rotation_angle[i] = g_MotorData[i].angleOffset+gyro_angle;
			rotation_x[i] = rotation_magnitude*sinDegrees(rotation_angle[i]);
			rotation_y[i] = rotation_magnitude*cosDegrees(rotation_angle[i]);
			combined_x[i] = translation_x+rotation_x;
			combined_y[i] = translation_y+rotation_y;
			combined_angle[i] = Math_RadToDeg(atan2(combined_y[i],combined_x[i]));
			motor_power[i] = sqrt(pow(combined_x[i],2)+pow(combined_y[i],2))*sinDegrees(combined_angle[i]-rotation_angle[i]);
			if (abs(rotation_magnitude)>0) {
				servo_angle[i] = g_MotorData[i].angleOffset;
			} else {
				translation_angle = Math_RadToDeg(atan2(translation_y,translation_x))-gyro_angle-90; //degrees
				for (int j=MOTOR_FR; j<=(int)MOTOR_BR; j++) {
					servo_angle[i] = translation_angle;
				}
			}
			g_MotorData[i].power = motor_power[i];
			g_ServoData[i].angle = servo_angle[i];
		}

		Task_EndTimeslice();
	}
}



task PID() {
	g_task_PID = Task_GetCurrentIndex();
	float kP = 25; //might need to make these terms arrays, if inverse-ness is an issue
	float kI = 0; //ditto^
	float kD = 0; //ditto^
	float error[4] = {0,0,0,0};
	float term_P[4] = {0,0,0,0};
	float term_I[4] = {0,0,0,0};
	float term_D[4] = {0,0,0,0};
	float total_correction[4] = {0,0,0,0};
	for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
		Motor_SetEncoder(Math_Normalize(90, 360, 1440), Motor_Convert((Motor)i)); // Might not be 90, could be some other number :P
	}
	Joystick_WaitForStart();

	while (true) {
		Task_HogCPU(); //This may be unnecessary/too greedy. Remember to remove `Task_ReleaseCPU()` if not needed.

		// Assigns power to servos in same loop as power is calculated (kinda is rolling assignment)
		for (int i=POD_FR; i<(int)POD_NUM; i++) {
			error[i] = g_ServoData[i].angle - Math_Normalize(Motor_GetEncoder(Motor_Convert((Motor)i)), 1440, 360);
			term_P[i] = kP*error[i]; //kP might become an array; see declaration
			term_I[i] = 0; //TODO! Has timers :P
			term_D[i] = 0; //TODO! Has timers :P
			total_correction[i] = term_P[i]+term_I[i]+term_D[i];
			Servo_SetPower(Servo_Convert((Servo)i), total_correction[i]);
		}

		// Parse the motor settings and assign them to the motors (in same loop).
		for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
			if (g_MotorData[i].isReversed==true) {
				g_MotorData[i].power *= -1;
			}
			Motor_SetPower(g_MotorData[i].power, Motor_Convert((Motor)i));
		}

		Task_ReleaseCPU();
		Task_EndTimeslice(); //Unless `Task_ReleaseCPU()` automatically switches tasks?
	}
}
