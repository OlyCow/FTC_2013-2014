#ifndef DISPLAY_C
#define DISPLAY_C
#pragma systemFile
#include "..\Headers\Display.h"
// For default values, see above header file.



void Display_Clear(bool isNegative) {
	switch (isNegative) {
		case false:
			eraseDisplay();
			break;
		case true:
			nxtFillRect(0, 63, 99, 0); // edges of the screen
			break;
	}
}
void Display_DrawPixel(int x, int y, bool isNegative) {
	switch (isNegative) {
		case false:
			nxtSetPixel(x, y);
			break;
		case true:
			nxtClearPixel(x, y);
			break;
	}
}
void Display_DrawLine(int x1, int y1, int x2, int y2, bool isNegative) {
	switch (isNegative) {
		case false:
			nxtDrawLine(x1, y1, x2, y2);
			break;
		case true:
			nxtEraseLine(x1, y1, x2, y2);
			break;
	}
}
void Display_DrawLineXOR(int x1, int y1, int x2, int y2, bool isNegative) {
	for (int i=false; i<=isNegative; i++) { // if isNegative==true, iterates twice; otherwise once
		nxtInvertLine(x1, y1, x2, y2);
	}
}
void Display_DrawRect(int L, int T, int R, int B, bool isNegative, bool isFilled) {
	switch (isNegative) {
		case false:
			switch (isFilled) {
				case false: // not negative, not filled
					nxtDrawRect(L, T, R, B);
					break;
				case true: // not negative, is filled
					nxtFillRect(L, T, R, B);
					break;
			}
			break;
		case true:
			switch (isFilled) {
				case false: // is negative, not filled
					drawInvertRect(L, T, R, B);
					break;
				case true: // is negative, is filled
					nxtEraseRect(L, T, R, B);
					break;
			}
			break;
	}
}
void Display_DrawCircle(int h, int k, int diameter, bool isNegative, bool isFilled) {
	int r = diameter/2;
	int L = h-r;
	int T = k+r;
	int R = L+diameter-1; //not sure if this -1 is necessary, but it's in "RobotCIntrinsics.c", so...
	int B = T-diameter+1; //^ditto
	switch (isNegative) {
		case false:
			switch (isFilled) {
				case false: // not negative, not filled
					nxtDrawCircle(L, T, diameter); //nxtDrawEllipse calls this, so might as well do this directly
					break;
				case true: // not negative, is filled
					nxtFillEllipse(L, T, R, B);
					break;
			}
			break;
		case true:
			switch (isFilled) {
				case false: // is negative, not filled
					drawInvertEllipse(L, T, R, B);
					break;
				case true: // is negative, is filled
					nxtEraseEllipse(L, T, R, B);
					break;
			}
			break;
	}
}
void Display_DrawEllipse(int L, int T, int R, int B, bool isNegative, bool isFilled) {
	switch (isNegative) {
		case false:
			switch (isFilled) {
				case false: // not negative, not filled
					nxtDrawEllipse(L, T, R, B);
					break;
				case true: // not negative, is filled
					nxtFillEllipse(L, T, R, B);
					break;
			}
			break;
		case true:
			switch (isFilled) {
				case false: // is negative, not filled
					drawInvertEllipse(L, T, R, B);
					break;
				case true: // is negative, is filled
					nxtEraseEllipse(L, T, R, B);
					break;
			}
			break;
	}
}
void Display_DrawRIC(int left, int bottom, string fileName) {
	nxtDisplayRICFile(left, bottom, fileName);
}
void Display_SetContrast(int contrast) {
	nLCDContrast = contrast;
}
int  Display_GetContrast() {
	return nLCDContrast;
}
// DO NOT! use the following function. It is written to flash, which has a
// limited (10k~100k) R/W lifespan. So, DO NOT USE THE FOLLOWING FUNCTION!
void Display_SetDefaultContrast(int contrast) {
	nLCDContrastDefault = contrast;
}
int  Display_GetDefaultContrast() {
	return nLCDContrastDefault;
}
void Display_SetStatusBarVisibility(bool isVisible) {
	bNxtLCDStatusDisplay = isVisible;
}
bool Display_GetStatusBarVisibility() {
	return bNxtLCDStatusDisplay;
}



#endif // DISPLAY_C
