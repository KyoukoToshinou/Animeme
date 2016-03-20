#include "main.h"

Vector2D okaeri::Mouse::vecMouse = Vector2D();

inline void okaeri::Mouse::Refresh()
{
	POINT pointMouse;
	GetCursorPos(&pointMouse); ScreenToClient(GetActiveWindow(), &pointMouse);
	vecMouse = Vector2D(pointMouse.x, pointMouse.y);
}

inline bool okaeri::Mouse::IsOver(float x, float y, float w, float h)
{
	return (vecMouse.x >= x && vecMouse.x <= x + w && vecMouse.y >= y && vecMouse.y <= y + h);
}

inline bool okaeri::Mouse::IsPressing(float x, float y, float w, float h)
{
	return (IsOver(x, y, w, h) && GetAsyncKeyState(VK_LBUTTON));
}

inline bool okaeri::Mouse::Down()
{
	return GetAsyncKeyState(VK_LBUTTON);
}
