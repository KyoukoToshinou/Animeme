#pragma once

enum
{
	TEXT_ALIGN_LEFT,
	TEXT_ALIGN_CENTER,
	TEXT_ALIGN_RIGHT
};

enum
{
	TEXT_ALIGN_TOP,
	TEXT_ALIGN_DOWN = 2
};

namespace draw
{
	int GetTextWidth(float, const char*, ...);
	int GetTextHeight(float, const char*, ...);
	int String(int, int, int, int, Color, float, const char*, ...);
	void FilledRectangle(int, int, int, int, Color);
	void Rectangle(int, int, int, int, Color);
	void RoundedFilledRectangle(int, int, int, int, Color);
	void Line(int, int, int, int, Color);
}