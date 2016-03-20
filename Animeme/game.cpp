#include "main.h"

bool game::WorldToScreen(Vector vecPosition, float& flX, float& flY)
{
	if (gamemanager::gameType == gamemanager::game_bf3) {
		Vector vecScreen;
		bool bResult = fb::worldToScreen(vecPosition, vecScreen);
		flX = vecScreen.x; flY = vecScreen.y;
		return bResult;
	}
	else if (gamemanager::gameType == gamemanager::game_cod4)
		return cod4::sdk::WorldToScreen(vecPosition, flX, flY);
	else if (gamemanager::gameType == gamemanager::game_cod8) {
		cod8::sdk::ScreenMatrix* pScreenMatrix = cod8::sdk::GetScreenMatrix();
		if (pScreenMatrix) {
			float* flScreenCoords = (float*)malloc(sizeof(float) * 2);
			bool bResult = cod8::sdk::WorldToScreen(0, pScreenMatrix, (float*)&vecPosition, flScreenCoords);
			flX = flScreenCoords[0];
			flY = flScreenCoords[1];
			return bResult;
		}
	}

	return false;
}
