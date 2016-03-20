#pragma once

namespace entmgr
{
	int GetMaxPlayers();
	int GetMaxEntities();
	entity* GetEntity(int);
	player* GetPlayer(int);
}