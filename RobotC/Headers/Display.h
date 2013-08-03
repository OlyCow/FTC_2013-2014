#ifndef DISPLAY_H
#define DISPLAY_H
#pragma systemFile



void Display_Clear(bool isNegative=false);
void Display_DrawPixel(int x, int y, bool isNegative=false);
void Display_DrawLine(int x1, int y1, int x2, int y2, bool isNegative=false);
void Display_DrawLineXOR(int x1, int y1, int x2, int y2, bool isNegative=false);
void Display_DrawRect(int L, int T, int R, int B, bool isNegative=false, bool isFilled=false);
void Display_DrawCircle(int h, int k, int diameter, bool isNegative=false, bool isFilled=false);
void Display_DrawEllipse(int L, int T, int R, int B, bool isNegative=false, bool isFilled=false);
void Display_DrawRIC(int left, int bottom, string fileName);
void Display_SetContrast(int contrast=90); //default value (in NXT-G)
int  Display_GetContrast();
// DO NOT! use the following function very many times. It is written to flash memory,
// which has a limited (10k~100k) R/W limit. So, DO NOT USE THE FOLLOWING FUNCTION! (:P)
// (see what I did with that smiley up there? :)
void Display_SetDefaultContrast(int contrast=90); //default value (in NXT-G)
int  Display_GetDefaultContrast();
void Display_SetStatusBarVisibility(bool isVisible);
bool Display_GetStatusBarVisibility();



//---eraseDisplay()
//---nxtSetPixel(xPos, yPos)
//---nxtClearPixel(xPos, yPos)
//nxtDisplayBigStringAt(xPos, yPos, pChar, ...);
//nxtDisplayBigStringAt(xPos, yPos, sFormatString, ...);
//nxtDisplayBigTextLine(nLineNumber, pChar, ...);
//nxtDisplayBigTextLine(nLineNumber, sFormatString, ...);
//nxtDisplayCenteredBigTextLine(nLineNumber, pChar, ...);
//nxtDisplayCenteredBigTextLine(nLineNumber, sFormatString, ...);
//nxtDisplayCenteredTextLine(nLineNumber, pChar, ...);
//nxtDisplayCenteredTextLine(nLineNumber, sFormatString, ...);
//nxtDisplayRICFile(nleft, nBottom, sFileName);
//nxtDisplayString(nLineNumber, pChar, ...);
//nxtDisplayString(nLineNumber, sFormatString, ...);
//nxtDisplayStringAt(nLineNumber, pChar, ...);
//nxtDisplayStringAt(nLineNumber, sFormatString, ...);
//nxtDisplayTextLine(nLineNumber, pChar, ...);
//nxtDisplayTextLine(nLineNumber, sFormatString, ...);
//---nxtDrawEllipse(Left, Top, Right, Bottom);
//---nxtDrawLine(xPos, yPos, xPosTo, yPosTo);
//---nxtDrawRect(Left, Top, Right, Bottom);
//---nxtEraseEllipse(Left, Top, Right, Bottom);
//---nxtEraseLine(xPos, yPos, xPosTo, yPosTo);
//---nxtEraseRect(Left, Top, Right, Bottom);
//---nxtFillEllipse(Left, Top, Right, Bottom);
//---nxtFillRect(Left, Top, Right, Bottom);
//---nxtInvertLine(xPos, yPos, xPosTo, yPosTo);
//nxtScrollText(pChar, ...);
//nxtScrollText(sFormatString);
//---bNxtLCDStatusDisplay
//---nLCDContrast
//---nLCDContrastDefault
#include "..\Libraries\Display.c"
#endif // DISPLAY_H
