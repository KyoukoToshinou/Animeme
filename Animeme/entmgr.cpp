#include "main.h"

int entmgr::GetMaxPlayers()
{
	if (gamemanager::gameType == gamemanager::game_bf3)
		return bf3::pointers::pClientPlayerManager->m_pPlayers.Size();
	else if (gamemanager::gameType == gamemanager::game_cod8)
		return 18; // wtf?

	return 0;
}

int entmgr::GetMaxEntities()
{
	if (gamemanager::gameType == gamemanager::game_bf3)
		return GetMaxPlayers();
	else if (gamemanager::gameType == gamemanager::game_cod8)
		return 2048;

	return 0;
}

player* entmgr::GetPlayer(int i)
{
	if (i <= GetMaxPlayers()) {
		if (gamemanager::gameType == gamemanager::game_bf3)
			return (player*)bf3::pointers::pClientPlayerManager->m_pPlayers[i];
		else if (gamemanager::gameType == gamemanager::game_cod8)
			return (player*)(cod8::pointers::ulEntities + ((int)cod8::pointers::ulEntitiesSize * i));
	}

	return nullptr;
}

entity* entmgr::GetEntity(int i)
{
	if (i <= GetMaxEntities()) {
		if (gamemanager::gameType == gamemanager::game_bf3)
			return GetPlayer(i);
		else if (gamemanager::gameType == gamemanager::game_cod8)
			return (entity*)(cod8::pointers::ulEntities + ((int)cod8::pointers::ulEntitiesSize * i));
	}

	return nullptr;
}