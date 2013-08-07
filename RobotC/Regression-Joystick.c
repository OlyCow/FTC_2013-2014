// To use this regression file: un-comment one "chunk", compile+download,
// and run the program. See if it behaves as expected. Then, comment out
// that "chunk", un-comment the next "chunk", and repeat until all "chunks"
// have been tested. Then move on to the next regression file.

#include "Headers\includes.h"

task main() {

	// This function is only included for the sake of "good practice".
	// Possibly superfluous (and bad?) since this is a regression file,
	// and problems need to be isolated as much as possible :P
	initializeGlobalVariables();

	// Prevents "JoystickDriver.c" from taking over the screen (like samostat).
	disableDiagnosticsDisplay();

	// Fields for 8 lines of the display, each split into columns for convenience.
	float number[8][4];
	string character[8][4];
	for (int i=0; i<8; i++) {
		for (int j=0; j<4; j++) {
			number[i][j] = 0;
			character[i][j] = "";
		}
	}

	while (true) {
		Joystick_UpdateData();



		//// BEGIN CHUNK I: `Joystick_Joystick(...)` ============================>>>>
		//// Reads all the possible inputs for `Joystick_Joystick(...)`.
		//number[0][0] = Joystick_Joystick(JOYSTICK_L, AXIS_X);
		//number[1][0] = Joystick_Joystick(JOYSTICK_L, AXIS_X, CONTROLLER_1);
		//number[1][1] = Joystick_Joystick(JOYSTICK_L, AXIS_X, CONTROLLER_2);
		//number[2][0] = Joystick_Joystick(JOYSTICK_L, AXIS_Y);
		//number[3][0] = Joystick_Joystick(JOYSTICK_L, AXIS_Y, CONTROLLER_1);
		//number[3][1] = Joystick_Joystick(JOYSTICK_L, AXIS_Y, CONTROLLER_2);
		//number[4][0] = Joystick_Joystick(JOYSTICK_R, AXIS_X);
		//number[5][0] = Joystick_Joystick(JOYSTICK_R, AXIS_X, CONTROLLER_1);
		//number[5][1] = Joystick_Joystick(JOYSTICK_R, AXIS_X, CONTROLLER_2);
		//number[6][0] = Joystick_Joystick(JOYSTICK_R, AXIS_Y);
		//number[7][0] = Joystick_Joystick(JOYSTICK_R, AXIS_Y, CONTROLLER_1);
		//number[7][1] = Joystick_Joystick(JOYSTICK_R, AXIS_Y, CONTROLLER_2);

		//// Display what was read. "LXD"="Left,X-axis,default controller", etc.
		//nxtDisplayTextLine(0, "LXD %d", number[0][0]);
		//nxtDisplayTextLine(1, "LX1 %d | LX2 %d", number[1][0], number[1][1]);
		//nxtDisplayTextLine(2, "LYD %d", number[2][0]);
		//nxtDisplayTextLine(3, "LY1 %d | LY2 %d", number[3][0], number[3][1]);
		//nxtDisplayTextLine(4, "RXD %d", number[4][0]);
		//nxtDisplayTextLine(5, "RX1 %d | RX2 %d", number[5][0], number[5][1]);
		//nxtDisplayTextLine(6, "RYD %d", number[6][0]);
		//nxtDisplayTextLine(7, "RY1 %d | RY2 %d", number[7][0], number[7][1]);
		//// END CHUNK I: `Joystick_Joystick(...)` ==============================>>>>



		//// BEGIN CHUNK II: `Joystick_Direction(Controller)` ===================>>>>
		//// Reads the direction of each D-pad; temporarily storing it in a buffer.
		//number[2][0] = Joystick_Direction();
		//number[4][0] = Joystick_Direction(CONTROLLER_1);
		//number[6][0] = Joystick_Direction(CONTROLLER_2);

		//// Converts the values stored in the buffer to a (more readable) string.
		//// The index starts at 2 and takes on the values 2,4,6 to slightly improve
		//// performance. (Instead of going through 0,1,2.) This is because the
		//// calculation for which entry to use only needs to be done once each loop.
		//for (int i=2; i<=6; i+=2) {
		//	string temp = "";
		//	switch (number[i][0]) {
		//		case DIRECTION_NONE:
		//			temp = "none";
		//			break;
		//		case DIRECTION_F:
		//			temp = "front";
		//			break;
		//		case DIRECTION_FR:
		//			temp = "front-right";
		//			break;
		//		case DIRECTION_R:
		//			temp = "right";
		//			break;
		//		case DIRECTION_BR:
		//			temp = "back-right";
		//			break;
		//		case DIRECTION_B:
		//			temp = "back";
		//			break;
		//		case DIRECTION_BL:
		//			temp = "back-left";
		//			break;
		//		case DIRECTION_L:
		//			temp = "left";
		//			break;
		//		case DIRECTION_FL:
		//			temp = "front-left";
		//			break;
		//	}
		//	character[i][0] = temp;
		//}

		//// Displays the previously-converted strings.
		//nxtDisplayTextLine(1, "Default:");
		//nxtDisplayCenteredTextLine(2, "%s", character[2][0]);
		//nxtDisplayTextLine(3, "Controller 1:");
		//nxtDisplayCenteredTextLine(4, "%s", character[4][0]);
		//nxtDisplayTextLine(5, "Controller 2:");
		//nxtDisplayCenteredTextLine(6, "%s", character[6][0]);
		//// END CHUNK II: `Joystick_Direction(Controller)` =====================>>>>



		//// BEGIN CHUNK III: `Joystick_Direction(Direction, ...)` ==============>>>>
		//// Check whether the specified direction is pressed, and also convert chars
		//// in our array depending on the result ("#" pressed; "-" not pressed).
		//for (Direction i=DIRECTION_F; i<=DIRECTION_FL; i++) {
		//	if (Joystick_Direction(i)==true) {
		//		character[i][0] = "#";
		//	} else {
		//		character[i][0] = "-";
		//	}
		//	if (Joystick_Direction(i, CONTROLLER_1)==true) {
		//		character[i][1] = "#";
		//	} else {
		//		character[i][1] = "-";
		//	}
		//	if (Joystick_Direction(i, CONTROLLER_2)==true) {
		//		character[i][2] = "#";
		//	} else {
		//		character[i][2] = "-";
		//	}
		//}

		//// These are for formatting. They stay static, but due to the "chunk"
		//// structure of this file, these are declared over again every loop.
		//// The impact on performance is negligible though, so we don't care.
		//character[0][3] = "F >";
		//character[1][3] = "FR>";
		//character[2][3] = "R >";
		//character[3][3] = "BR>";
		//character[4][3] = "B >";
		//character[5][3] = "BL>";
		//character[6][3] = "L >";
		//character[7][3] = "FL>";

		//// Displays our huge array, all nicely formatted.
		//for (int i=0; i<8; i++) {
		//	nxtDisplayCenteredTextLine( i, "%s D:%s 1:%s 2:%s",
		//								character[i][3],
		//								character[i][0],
		//								character[i][1],
		//								character[i][2] );
		//}
		//// END CHUNK III: `Joystick_Direction(Direction, ...)` ================>>>>



		//// BEGIN CHUNK IV: `Joystick_ButtonPressed(...)` ======================>>>>

		//// END CHUNK IV: `Joystick_ButtonPressed(...)` ========================>>>>



		// BEGIN CHUNK V: `Joystick_ButtonReleased(...)` ======================>>>>

		// END CHUNK V: `Joystick_ButtonReleased(...)` ========================>>>>



		// BEGIN CHUNK VI: `Joystick_Button(...)` =============================>>>>

		// END CHUNK VI: `Joystick_Button(...)` ===============================>>>>



		// BEGIN CHUNK VII: `Joystick_DirectionPressed(...)` ==================>>>>

		// END CHUNK VII: `Joystick_DirectionPressed(...)` ====================>>>>



		// BEGIN CHUNK VIII: `Joystick_DirectionReleased(...)` ================>>>>

		// END CHUNK VIII: `Joystick_DirectionReleased(...)` ==================>>>>



		 Reset all display fields to "0".
		for (int i=0; i<8; i++) {
			for (int j=0; j<4; j++) {
				number[i][j] = 0;
				character[i][j] = "";
			}
		}

		// Give the LCD display some breathing room.
		wait1Msec(100);
	}
}
