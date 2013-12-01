#ifndef VECTORS_2D_C
#define VECTORS_2D_C
#pragma systemFile
#include "..\Headers\Vectors-2D.h"
// For default values, see above header file.



void Vector2D_UpdatePos(vector2D &inputVector) {
	inputVector.x = inputVector.r*cosDegrees(inputVector.theta);
	inputVector.y = inputVector.r*sinDegrees(inputVector.theta);
}
void Vector2D_UpdateRot(vector2D &inputVector) {
	inputVector.theta = radiansToDegrees(atan2(inputVector.y, inputVector.x));
	inputVector.r = sqrt((inputVector.y)*(inputVector.y)+(inputVector.x)*(inputVector.x));
}
void Vector2D_Add(vector2D vectorA, vector2D vectorB, vector2D &result) {
	result.x = vectorA.x+vectorB.x;
	result.y = vectorA.y+vectorB.y;
	Vector2D_UpdateRot(result);
}
void Vector2D_Subtract(vector2D vectorA, vector2D vectorB, vector2D &result) {
	result.x = vectorA.x-vectorB.x;
	result.y = vectorA.y-vectorB.y;
	Vector2D_UpdateRot(result);
}
void Vector2D_Scale(float scalar, vector2D inputVector, vector2D &result) {
	result.x = scalar*inputVector.x;
	result.y = scalar*inputVector.y;
	Vector2D_UpdateRot(result);
}
void Vector2D_Translate(vector2D &inputVector, float x, float y) {
	inputVector.x += x;
	inputVector.y += y;
	Vector2D_UpdateRot(inputVector);
}
void Vector2D_Rotate(vector2D &inputVector, float theta, AngleUnit units) {
	inputVector.x = inputVector.x*cosDegrees(theta)-inputVector.y*sinDegrees(theta);
	inputVector.y = inputVector.y*sinDegrees(theta)+inputVector.y*cosDegrees(theta);
	Vector2D_UpdateRot(inputVector);
}



#endif // VECTORS_2D_C
