#include "main.h"

float* Color::GetFloatColor()
{
	float* aFloatColor = (float*)malloc(sizeof(float) * 4);
	aFloatColor[0] = (float)r / 255.f;
	aFloatColor[1] = (float)g / 255.f;
	aFloatColor[2] = (float)b / 255.f;
	aFloatColor[3] = (float)a / 255.f;
	return aFloatColor;
}
