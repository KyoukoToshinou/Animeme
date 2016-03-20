#pragma once

class Color
{
public:
	unsigned char r, g, b, a;

	Color()
	{
		r = 255;
		g = 255;
		b = 255;
		a = 255;
	}

	Color(int red, int green, int blue, int alpha = 255)
	{
		r = red;
		g = green;
		b = blue;
		a = alpha;
	}

	float* GetFloatColor();
};