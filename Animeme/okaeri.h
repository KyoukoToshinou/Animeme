#pragma once

namespace okaeri
{
	namespace Mouse
	{
		extern Vector2D vecMouse;
		void Refresh();
		bool IsOver(float x, float y, float w, float h);
		bool IsPressing(float x, float y, float w, float h);
		bool Down();
	}
}