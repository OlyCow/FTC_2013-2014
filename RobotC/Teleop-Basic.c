#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S2, HTServo,  none,     none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     motor_FR,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motor_FL,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motor_BL,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motor_BR,      tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S2_C1_1,    servo_FR,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    servo_FL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    servo_BL,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_4,    servo_BR,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo6,               tServoNone)

#include "Headers\includes.h"
#include "Teleop-Basic.h"

// Automatically starts all tasks defined in this file. (May not be necessary,
// now that we've merged `task PID()` into `task main()`. This change can be
// reversed if our code gets too complicated/causes too much lag.)
#pragma autoStartTasks

#warning "I can declare warnings at will!"
//#define WILL_EXPLODE // Un-comment this line (Ctrl-Q) to prevent this from compiling!
#ifdef WILL_EXPLODE
#error "This code will explode!"
#endif

//---------------- README!!! ------------------------------------------------>>
//
//     The version of the code in the commit "WORKING VERSION!!!" is "working".
// If you somehow mess up the code, you can always revert. So feel free to play
// around. (Commit hash: "bdf8abe6a7959a4b2f11036f3a48511afd36288d") To actually
// use that commit, don't forget to switch the encoder wire from FR to BL (the
// encoder wasn't working at the time).
//
//     Set the robot up as follows: with the front (where the NXT is mounted)
// facing towards you, the side of the wheel pods with ABS plastic should be on
// the left. As defined in "enums.h", the wheel pods are "numbered": `FR`, `FL`,
// `BL`, `BR`. As of this writing, `POD_BL` is missing a bevel gear and encoder,
// and thus has an omni-wheel attached to it. (This may or may not affect moving
// forward and rotating at the same time - I cannot test the code because the
// robot is out of batteries.)
//
//     The code is split into a few chunks: #1 Find the target angle, velocity,
// etc. #2 Adjust the current angle, velocity, etc. through a PID control. The
// PID loop doesn't have an "I" or "D" term yet, so it either overshoots (and
// then starts oscillating), or it never reaches the set-point ("steady-state
// error"). Go ahead and implement that if you wish! #3 Display data on the NXT
// screen for debugging purposes. Believe me, this is useful :)
//
// CONTROLS:	Joystick_R, Controller_1: Movement.
//				Button_LB/RB, Controller_1: Rotation.
//				Button_LT, Controller_1: Stop motors.
//				Button_RT, Controller_1: Fine-tune motors.
//
//--------------------------------------------------------------------------->>



task main() {
	initializeGlobalVariables(); // Defined in "global vars.h", this intializes all struct members.
	disableDiagnosticsDisplay(); // This disables the "samostat.rxe"-like diagnostics screen.

	// For finding target values:
	//g_task_main = Task_GetCurrentIndex(); // This was used when we had multiple tasks.
	bool  shouldNormalize = false; // This flag is set if motor values go over 100. All motor values will be scaled down.
	float gyro_angle = 0;
	float rotation_magnitude = 0; // Components of the vector of rotation.
	float rotation_angle[POD_NUM] = {0,0,0,0};
	float rotation_x[POD_NUM] = {0,0,0,0};
	float rotation_y[POD_NUM] = {0,0,0,0};
	float translation_magnitude = 0; // Components of the vector of translation.
	float translation_angle = 0;
	float translation_x = 0;
	float translation_y = 0;
	float combined_magnitude[POD_NUM] = {0,0,0,0}; // Components of the final (assigned) vector.
	float combined_angle[POD_NUM] = {0,0,0,0};
	float combined_x[POD_NUM] = {0,0,0,0};
	float combined_y[POD_NUM] = {0,0,0,0};

	// For PID control:
	float kP = 0.9; // Slightly low. When terms "I" and "D" are implemented, increase `kI` or `kD`.
	float kI = 0;
	float kD = 0;
	float current_encoder[POD_NUM] = {0,0,0,0};
	float error[POD_NUM] = {0,0,0,0}; // Difference between set-point and measured value.
	float term_P[POD_NUM] = {0,0,0,0};
	float term_I[POD_NUM] = {0,0,0,0};
	float term_D[POD_NUM] = {0,0,0,0};
	float total_correction[POD_NUM] = {0,0,0,0}; // Simply "term_P + term_I + term_D".

	Joystick_WaitForStart();

	while (true) {
		Joystick_UpdateData();

		// This part is temporary. We are assigning a gyro angle from a joystick.
		// An operator moves the joystick to correspond with the front of the drive
		// base, to simulate the input from an actual gyro. When we get a gyro, fix
		// this so it returns the actual value of the gyro.
		gyro_angle = 0; // No "relative" movement - unnecessary complexity for now.
		//gyro_angle = Math_RadToDeg(atan2(
		//					Joystick_Joystick(JOYSTICK_R, AXIS_Y, CONTROLLER_2),
		//					Joystick_Joystick(JOYSTICK_R, AXIS_X, CONTROLLER_2) ));
		//// ^Doesn't work(?) :(

		// Actual code starts here. It is ridiculously simple, but oddly counter-
		// intuitive. The rotation vector and the translation vector are combined,
		// then the final vector is normalized. That is it. This algorithm has the
		// property of seeming like the direction of the resultant vector is the
		// average of the input vectors' directions, which it is. It might seem like
		// there would be inevitable slipping involved in this scheme, but a simple
		// derivative analysis of a generalized cycloid shows the above algorithm
		// to describe the direction of the wheel pods perfectly.
		rotation_magnitude = Joystick_GetRotationMagnitude(); // Either LB/RB or X-axis of other joystick.
		translation_x = Joystick_GetTranslationX();
		translation_y = Joystick_GetTranslationY();
		translation_magnitude = sqrt(pow(translation_x,2)+pow(translation_y,2)); // Pythagorean Theorem.
		translation_angle = Math_RadToDeg(atan2(translation_y, translation_x)); // -180deg ~ 180deg
		for (int i=POD_FR; i<=(int)POD_BR; i++) {
			rotation_angle[i] = g_MotorData[i].angleOffset+gyro_angle+90; // Tangent to circle, so +90deg.
			rotation_x[i] = rotation_magnitude*cosDegrees(rotation_angle[i]); // Simple algebra.
			rotation_y[i] = rotation_magnitude*sinDegrees(rotation_angle[i]);
			combined_x[i] = translation_x+rotation_x[i]; // Combining the vectors' components.
			combined_y[i] = translation_y+rotation_y[i];
			combined_magnitude[i] = sqrt(pow(combined_x[i],2)+pow(combined_y[i],2));
			if (combined_magnitude[i]>g_FullPower) {
				shouldNormalize = true;
			}
			combined_angle[i] = (Math_RadToDeg(atan2(combined_y[i], combined_x[i]))+360)%360;
			g_ServoData[i].angle = combined_angle[i];
		}

		// Normalize our motors' power values if a motor's power went above g_FullPower.
		if (shouldNormalize==true) {
			float originalMaxPower = combined_magnitude[0];
			for (int i=POD_FR; i<=(int)POD_BR; i++) {
				if (combined_magnitude[i]>originalMaxPower) {
					originalMaxPower = combined_magnitude[i];
				}
			}
			for (int i=POD_FR; i<=(int)POD_BR; i++) {
				Math_Normalize(combined_magnitude[i], originalMaxPower, g_FullPower);
			}
		}
		for (int i=POD_FR; i<=(int)POD_BR; i++) {
			g_MotorData[i].power = combined_magnitude[i];
		}

		// Set our "fine-tune" factor (amount motor power is divided by).
		if (Joystick_Button(BUTTON_LT)==true) {
			for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
				g_MotorData[i].fineTuneFactor = 0; // Since power is fine-tuned w/ multiplication, this zeroes motor power.
			}
		} else if (Joystick_Button(BUTTON_RT)==true) {
			for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
				g_MotorData[i].fineTuneFactor = 4;
			}
		} else {
			for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
				g_MotorData[i].fineTuneFactor = 1;
			}
		}



		//PID control loop:
		for (int i=POD_FR; i<(int)POD_NUM; i++) {
			current_encoder[i] = ((Math_Normalize(Motor_GetEncoder(Motor_Convert((Motor)i)), (float)1440, 360)+360)%360)/2; // Encoders are geared up by 2.
			error[i] = g_ServoData[i].angle-current_encoder[i];
			term_P[i] = kP*error[i]; //kP might become an array
			term_I[i] = 0; //TODO! Has timers :P
			term_D[i] = 0; //TODO! Has timers :P
			total_correction[i] = Math_Limit((term_P[i]+term_I[i]+term_D[i]), 128);
			if (error[i]>180) {
				error[i] = error[i]-360;
			}
		}



		// Assign the power settings to the motors (already parsed).
		for (int i=MOTOR_FR; i<=(int)MOTOR_BR; i++) {
			if (g_MotorData[i].isReversed==true) {
				g_MotorData[i].power *= -1;
			}
			g_MotorData[i].power /= g_MotorData[i].fineTuneFactor;
			Motor_SetPower(g_MotorData[i].power, Motor_Convert((Motor)i));
		}

		// Assign the power settings to the servos. Can't make a loop yet, since one
		// of the assignments is different from the rest. >:(
		servo[servo_FR] = total_correction[POD_FR]+128; // +128, because that's how continuous rotation servos work.
		servo[servo_FL] = total_correction[POD_FL]+128;
		servo[servo_BL] = 128; // We don't have an encoder mounted for this servo... derp.
		//servo[servo_BL] = total_correction[POD_BL]+128; // Un-comment this when we do mount one.
		servo[servo_BR] = total_correction[POD_BR]+128;



		// Troubleshooting display:
		nxtDisplayTextLine(0, "FR set%d chg%d", g_ServoData[POD_FR].angle, total_correction[POD_FR]);
		nxtDisplayTextLine(1, "FL set%d chg%d", g_ServoData[POD_FL].angle, total_correction[POD_FL]);
		nxtDisplayTextLine(2, "BL set%d chg%d", g_ServoData[POD_BL].angle, total_correction[POD_BL]);
		nxtDisplayTextLine(3, "BR set%d chg%d", g_ServoData[POD_BR].angle, total_correction[POD_BR]);
		nxtDisplayTextLine(4, "FR encdr%d pow%d", current_encoder[POD_FR], g_MotorData[POD_FR].power);
		nxtDisplayTextLine(5, "FL encdr%d pow%d", current_encoder[POD_FL], g_MotorData[POD_FL].power);
		nxtDisplayTextLine(6, "BL encdr%d pow%d", current_encoder[POD_BL], g_MotorData[POD_BL].power);
		nxtDisplayTextLine(7, "BR encdr%d pow%d", current_encoder[POD_BR], g_MotorData[POD_BR].power);

		//Task_EndTimeslice(); // This was used when we had multiple tasks.
	}
}
