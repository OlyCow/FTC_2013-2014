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

	// Fields for 8 lines of the display, each split into 2 columns for convenience.
	int line[8][2] = {{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};

	while (true) {
		Joystick_UpdateData();

		//// BEGIN CHUNK I: `Joystick_Joystick(...)` ============================>>>>
		//// Reads all the possible inputs for `Joystick_Joystick(...)`.
		//line[0][0] = Joystick_Joystick(JOYSTICK_L, AXIS_X);
		//line[1][0] = Joystick_Joystick(JOYSTICK_L, AXIS_X, CONTROLLER_1);
		//line[1][1] = Joystick_Joystick(JOYSTICK_L, AXIS_X, CONTROLLER_2);
		//line[2][0] = Joystick_Joystick(JOYSTICK_L, AXIS_Y);
		//line[3][0] = Joystick_Joystick(JOYSTICK_L, AXIS_Y, CONTROLLER_1);
		//line[3][1] = Joystick_Joystick(JOYSTICK_L, AXIS_Y, CONTROLLER_2);
		//line[4][0] = Joystick_Joystick(JOYSTICK_R, AXIS_X);
		//line[5][0] = Joystick_Joystick(JOYSTICK_R, AXIS_X, CONTROLLER_1);
		//line[5][1] = Joystick_Joystick(JOYSTICK_R, AXIS_X, CONTROLLER_2);
		//line[6][0] = Joystick_Joystick(JOYSTICK_R, AXIS_Y);
		//line[7][0] = Joystick_Joystick(JOYSTICK_R, AXIS_Y, CONTROLLER_1);
		//line[7][1] = Joystick_Joystick(JOYSTICK_R, AXIS_Y, CONTROLLER_2);

		//// Display what was read. "LXD"="Left,X-axis,default controller", etc.
		//nxtDisplayTextLine(0, "LXD %d", line[0][0]);
		//nxtDisplayTextLine(1, "LX1 %d | LX2 %d", line[1][0], line[1][1]);
		//nxtDisplayTextLine(2, "LYD %d", line[2][0]);
		//nxtDisplayTextLine(3, "LY1 %d | LY2 %d", line[3][0], line[3][1]);
		//nxtDisplayTextLine(4, "RXD %d", line[4][0]);
		//nxtDisplayTextLine(5, "RX1 %d | RX2 %d", line[5][0], line[5][1]);
		//nxtDisplayTextLine(6, "RYD %d", line[6][0]);
		//nxtDisplayTextLine(7, "RY1 %d | RY2 %d", line[7][0], line[7][1]);
		//// END CHUNK I: `Joystick_Joystick(...)` ==============================>>>>

		// BEGIN CHUNK II: `Joystick_Direction(Controller)` =====================>>>>
		line[0][0] = Joystick_Direction();
		// END CHUNK II: `Joystick_Direction(Controller)` =======================>>>>

		// BEGIN CHUNK III: `Joystick_Direction(Direction, ...)` ================>>>>

		// END CHUNK III: `Joystick_Direction(Direction, ...)` ==================>>>>

		// Reset all display fields to "0".
		for (int i=0; i<8; i++) {
			for (int j=0; j<2; j++) {
				line[i][j] = 0;
			}
		}
	}
}
