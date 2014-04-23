#ifndef AUTON_H
#define AUTON_H



// Motor Assignments!
tMotor omniL = motor_FL;
tMotor omniR = motor_FR;

// For debugging display.
float pos_L = 0.0;
float pos_R = 0.0;
float pos_avg = 0.0;
float error = 0.0;
float heading = 0.0; // Because f_angle_z is an int.
bool isStalled = false;



// Forward declarations of tasks.
task Gyro(); // Constantly updates the heading of the robot.
task MonitorStall(); // Makes sure the motors don't burn out.

// Forward declarations of functions.
void config_values(	bool &key,	string txt_disp,
					bool val_L,	string txt_L,
					bool val_R,	string txt_R);
void DumpAutonCube();
void LowerAutonArm();
void MoveForward(float inches, bool doBrake=true);
void MoveBackward(float inches, bool doBrake=true);
void ChargeForward(int milliseconds, int power, bool doBrake=true, bool doLowerOmni=false);
void ChargeBackward(int milliseconds, int power, bool doBrake=true, bool doLowerOmni=false);
void TurnLeft(int degrees);
void TurnRight(int degrees);
void Brake(bool doSettle=true);
void Settle();
void DefendRamp();

// Definitions of tasks.
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
task MonitorStall()
{
	bool isCheckingStall = false;
	const int stall_threshold = 10; // encoder counts
	const int stall_delay = 500; // milliseconds
	int omni_L_pos		= 0;
	int omni_R_pos		= 0;
	int omni_L_pos_prev	= 0;
	int omni_R_pos_prev	= 0;
	int change_L		= 0;
	int change_R		= 0;
	int timer_stall		= 0;
	Time_ClearTimer(timer_stall);
	
	while (true) {
		omni_L_pos_prev = omni_L_pos;
		omni_R_pos_prev = omni_R_pos;
		omni_L_pos = Motor_GetEncoder(omniL);
		omni_R_pos = Motor_GetEncoder(omniR);
		change_L = omni_L_pos - omni_L_pos_prev;
		change_R = omni_R_pos - omni_R_pos_prev;
		change_L = abs(change_L);
		change_R = abs(change_R);
		if ((change_L<stall_threshold) || (change_R<stall_threshold)) {
			if (!isCheckingStall) {
				Time_ClearTimer(timer_stall);
				isCheckingStall = true;
			} else {
				if (Time_ClearTimer>stall_delay) {
					isStalled = true;
				}
			}
		}
		Time_Wait(10);
	}
}

// Definitions of functions.
void config_values(	bool &key,	string txt_disp,
					bool val_L,	string txt_L,
					bool val_R,	string txt_R)
{
	Task_Kill(Display);
	Task_Kill(displayDiagnostics);
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
		nxtDisplayString(4, disp_R);

		Time_Wait(10);
	}
	if (isL==true) {
		key = val_L;
	} else if (isR==true) {
		key = val_R;
	}

	Task_Spawn(displayDiagnostics);
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
	float kP = 0.03;
	bool isMoving = true;
	int timer_timeout = 0;
	int timeout_threshold = 5000; // MAGIC_NUM: 5 seconds. Plenty, no?
	Time_ClearTimer(timer_timeout);

	Motor_ResetEncoder(omniL);
	Motor_ResetEncoder(omniR);
	Task_Spawn(MonitorStall);

	while (isMoving && (Time_GetTime(timer_timeout)<timeout_threshold)){
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
			// TODO: Add this back in if distances aren't accurate enough.
			//if (abs(vel)<20) {
				isMoving = false;
			//}
		}
		if (isStalled) {
			break;
		}
	}
	
	Task_Kill(MonitorStall);
	isStalled = false;
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
void ChargeForward(int milliseconds, int power, bool doBrake, bool doLowerOmni)
{
	Servo_SetPosition(servo_omni_L, servo_omni_L_up);
	Servo_SetPosition(servo_omni_R, servo_omni_R_up);
	for (int i=0; i<milliseconds; ++i) {
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
void ChargeBackward(int milliseconds, int power, bool doBrake, bool doLowerOmni)
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
	float kP = 5.7;
	bool isFineTune = false;
	int finish_timer = 0;
	int timer_timeout = 0;
	int timeout_threshold = 5000; // MAGIC_NUM: 5 seconds. Plenty, no?
	
	Task_Spawn(MonitorStall);
	Time_ClearTimer(timer_timeout);

	while (isTurning && (Time_GetTime(timer_timeout)<timeout_threshold)) {
		current_pos = heading;
		error = target-current_pos;
		if (error>60) {
			power = g_FullPower;
		} else if (error<-60) {
			power = -g_FullPower;
		} else {
			power = kP*error;
		}
		if (abs(power)<20) {
			if (power>0) {
				power = 20;
			} else if (power<0) {
				power = -20;
			}
		}
		power_neg = -power;
		Motor_SetPower(power, motor_FL);
		Motor_SetPower(power, motor_BL);
		Motor_SetPower(power_neg, motor_FR);
		Motor_SetPower(power_neg, motor_BR);
		if (abs(error)<2) {
			if (isFineTune==false) {
				Time_ClearTimer(finish_timer);
				isFineTune = true;
			} else if (Time_GetTime(finish_timer)>500) {
				isTurning = false;
			}
		}
		if (isStalled) {
			break;
		}
	}
	
	Task_Kill(MonitorStall);
	isStalled = false;
	Motor_SetPower(0, motor_FL);
	Motor_SetPower(0, motor_BL);
	Motor_SetPower(0, motor_FR);
	Motor_SetPower(0, motor_BR);
}
void TurnRight(int degrees)
{
	TurnLeft(-degrees);
}
void Brake(bool doSettle)
{
	Motor_SetPower(0, motor_FL);
	Motor_SetPower(0, motor_BL);
	Motor_SetPower(0, motor_FR);
	Motor_SetPower(0, motor_BR);
	if (doSettle) {
		Settle();
	}
}
void Settle()
{
	Time_Wait(500);
}
void DefendRamp()
{
	Servo_SetPosition(servo_omni_L, servo_omni_L_down);
	Servo_SetPosition(servo_omni_R, servo_omni_R_down);
	Motor_ResetEncoder(omniL);
	Motor_ResetEncoder(omniR);
	float kP = -0.02;
	float power_L = 0.0;
	float power_R = 0.0;

	while (true) {
		pos_L = Motor_GetEncoder(omniL);
		pos_R = -Motor_GetEncoder(omniR);

		if (abs(pos_L)>120) {
			power_L = kP*pos_L;
		} else {
			power_L = 0.0;
			if (abs(power_L)<20) {
				if (power_L>0) {
					power_L = 20.0;
				} else {
					power_L = -20.0;
				}
			}
		}
		if (abs(pos_R)>120) {
			power_R = kP*pos_R;
			if (abs(power_R)<20) {
				if (power_R>0) {
					power_R = 20.0;
				} else {
					power_R = -20.0;
				}
			}
		} else {
			power_R = 0.0;
		}

		Motor_SetPower(power_L, motor_FL);
		Motor_SetPower(power_L, motor_BL);
		Motor_SetPower(power_R, motor_FR);
		Motor_SetPower(power_R, motor_BR);
		Time_Wait(8);	// MAGIC_NUM: Arbitrary delay.
	}
}



#endif // AUTON_H
