#ifndef VECTORS_2D_H
#define VECTORS_2D_H
#pragma systemFile
#include "Math.h"



typedef struct {
	float x;
	float y;
	float theta;
	float r;
} vector2D;



void Vector2D_UpdatePos(vector2D &inputVector);
void Vector2D_UpdateRot(vector2D &inputVector);
void Vector2D_Add(vector2D vectorA, vector2D vectorB, vector2D &result);
void Vector2D_Subtract(vector2D vectorA, vector2D vectorB, vector2D &result);
void Vector2D_Scale(float scalar, vector2D inputVector, vector2D &result);
void Vector2D_Translate(vector2D &inputVector, float x, float y);
void Vector2D_Rotate(vector2D &inputVector, float theta, AngleUnit units=UNIT_DEG);
// TODO: Add access functions?



#include "..\Libraries\Vectors-2D.c"
#endif // VECTORS_2D_H
