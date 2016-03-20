#include "main.h"

#define soldier (fb::ClientSoldierEntity*)(((fb::ClientPlayer2*)this)->m_controlledControllable)

bool entity::IsValid()
{
	if (gamemanager::gameType == gamemanager::game_bf3) {
		fb::ClientSoldierEntity* pClientSoldierEntity = soldier;
		if (pClientSoldierEntity == nullptr)
			return false;
	}

	return true;
}

char* entity::GetName(int iIndex)
{
	if (gamemanager::gameType == gamemanager::game_bf3) {
		fb::ClientPlayer2* pClientPlayer = (fb::ClientPlayer2*)this;
		if (pClientPlayer != nullptr)
			return pClientPlayer->m_name;
	}
	else if (gamemanager::gameType == gamemanager::game_cod8) {
		if (this->GetType() == ent_player) {
			cod8::sdk::client_t* pClient = (cod8::sdk::client_t*)(cod8::pointers::ulClientInfo + ((int)cod8::pointers::ulClientInfoSize * iIndex));
			return pClient->Name;
		}
	}

	return "";
}

Vector entity::GetPosition()
{
	if (gamemanager::gameType == gamemanager::game_bf3) {
		fb::ClientSoldierEntity* pClientSoldierEntity = soldier;
		if (pClientSoldierEntity != nullptr) {
			fb::ClientSoldierReplication* pReplicatedController = pClientSoldierEntity->m_replicatedController;
			if (pReplicatedController != nullptr)
				return Vec3toVec(pReplicatedController->m_state.position);
		}
	}

	return Vector();
}

Vector entity::GetAngles()
{
	return Vector();
}

int entity::GetType()
{
	if (gamemanager::gameType == gamemanager::game_cod8) {
		int iType = ent_none;
		switch (((cod8::sdk::entity_t*)this)->Type) {
		case ET_PLAYER:
			iType = ent_player;
			break;

		case ET_EXPLOSIVE:
			iType = ent_special;
			break;
		}
	}

	return ent_player;
}

weapon* npc::GetWeapon()
{
	if (gamemanager::gameType == gamemanager::game_bf3) {
		fb::ClientSoldierEntity* pClientSoldierEntity = soldier;
		if (pClientSoldierEntity != nullptr)
			return (weapon*)pClientSoldierEntity->m_soldierWeaponsComponent->m_currentAnimatedWeaponHandler->m_currentAnimatedWeapon;
	}

	return nullptr;
}

int npc::GetHealth()
{
	if (gamemanager::gameType == gamemanager::game_bf3) {
		fb::ClientSoldierEntity* pClientSoldierEntity = soldier;
		if (pClientSoldierEntity != nullptr)
			return pClientSoldierEntity->m_health;
	}

	return 0;
}

int npc::GetArmor()
{
	return 0;
}

bool npc::IsAlive()
{
	if (gamemanager::gameType == gamemanager::game_bf3) {
		fb::ClientSoldierEntity* pClientSoldierEntity = soldier;
		if (pClientSoldierEntity != nullptr)
			return pClientSoldierEntity->isAlive();
	}

	return false;
}

bool player::IsTeammate()
{
	return false;
}

bool player::IsFriend()
{
	return false;
}
