#ifndef SUBROUTINES_H
#define SUBROUTINES_H



void sub_LiftToHeight(int height)
{
	// Version 1 ("Theoretical Form")
	//int Kp = 1;
	//int Ki = 1;
	//int pastError = 0;
	//int presentError = 0;
	//int previousTime = 0;
	//int dTime = 0;
	//dTime = Time[1]-previousTime;
	//previousTime += dTime;
	//presentError = height-Motor_GetEncoder(motor_lift);
	//pastError += dTime*presentError;
	//power_lift = Kp*presentError;

	//// Version 2 ("Standard Form")
	//int Kp = 1;
	//int Ti = 1;
	//int pastError = 0;
	//int presentError = 0;
	//int previousTime = 0;
	//int dTime = 0;
	//dTime = Time[1]-previousTime;
	//previousTime += dTime;
	//presentValue = Motor_GetEncoder(motor_lift);
	//pastError += dTime*presentError;
	//power_lift = Kp*(presentValue+pastError/Ti);
}


//// We aren't using this function at all--therefore, it is commented out.
//task sub_MOO()
//{
//	PlaySoundFile("moo.rso");
//
//	// Uncomment this next section if "moo.rso" won't play.
//	while (bSoundActive)
//	{
//		Time_Wait(10);
//		Joystick_UpdateData();
//		EndTimeSlice();
//	}
//
//	StopTask(sub_MOO);
//}



#endif
