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



task main() {
	initializeGlobalVariables();
	float gyro_angle = 0;
	float rotation_magnitude = 0;
	float rotation_angle[4] = {0,0,0,0}; //4=# of drive base motors
	float rotation_x[4] = {0,0,0,0}; //4=# of drive base motors
	float rotation_y[4] = {0,0,0,0}; //4=# of drive base motors
	float translation_magnitude = 0;
	float translation_angle = 0;
	float translation_x = 0;
	float translation_y = 0;
	float combined_angle[4] = {0,0,0,0}; //4=# of drive base motors
	float combined_x[4] = {0,0,0,0}; //4=# of drive base motors
	float combined_y[4] = {0,0,0,0}; //4=# of drive base motors
	float motor_power[4] = {0,0,0,0}; //4=# of drive base motors
	float servo_angle[4] = {0,0,0,0}; //4=# of drive base motors

	while (true) {
		Joystick_UpdateData();

		// This part is temporary. We are assigning a gyro angle from a joystick.
		// An operator moves the joystick to correspond with a flag mounted on the
		// frame of the drive base, to simulate the input from an actual joystick.
		// No word as to when we will be able to purchase a prototype board and
		// the MPU-6050 breakout board.
		gyro_angle = radiansToDegrees(atan2(
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
		rotation_magnitude = Joystick_Joystick(JOYSTICK_L, AXIS_X);
		translation_x = Joystick_Joystick(JOYSTICK_R, AXIS_X);
		translation_y = Joystick_Joystick(JOYSTICK_R, AXIS_X);
		translation_magnitude = sqrt(pow(translation_x,2)+pow(translation_y,2)); //Pythagoras
		for (int i=(int)MOTOR_FR; i<=(int)MOTOR_BR; i++) {
			rotation_angle[i] = g_MotorData[i].angleOffset+gyro_angle;
			rotation_x[i] = rotation_magnitude*sinDegrees(rotation_angle[i]);
			rotation_y[i] = rotation_magnitude*cosDegrees(rotation_angle[i]);
			combined_x[i] = translation_x+rotation_x;
			combined_y[i] = translation_y+rotation_y;
			combined_angle[i] = radiansToDegrees(atan2(combined_y[i],combined_x[i]));
			motor_power[i] = sqrt(pow(combined_x[i],2)+pow(combined_y[i],2))*sinDegrees(combined_angle[i]-rotation_angle[i]);
			if (abs(rotation_magnitude)>0) {
				servo_angle[i] = g_MotorData[i].angleOffset;
			} else {
				translation_angle = radiansToDegrees(atan2(translation_y,translation_x))-gyro_angle-90; //degrees
				for (int j=(int)MOTOR_FR; j<=(int)MOTOR_BR; j++) {
					servo_angle[i] = translation_angle;
				}
			}
		}
	}
}
