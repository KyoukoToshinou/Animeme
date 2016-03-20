#pragma once

namespace gamemanager
{
	enum
	{
		game_cod4,
		game_cod8,
		game_bf3,
	};

	void Init();
	void Input(), Visuals();
	extern int gameType, steamAppID;
}