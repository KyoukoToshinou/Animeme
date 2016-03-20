#pragma once

enum
{
	ent_none,
	ent_player,
	ent_npc,
	ent_item,
	ent_special
};

class weapon
{

};

class entity
{
public:
	bool IsValid();
	char* GetName(int);
	Vector GetPosition();
	Vector GetAngles();
	int GetType();
};

class npc : public entity
{
public:
	weapon* GetWeapon();
	int GetHealth();
	int GetArmor();
	bool IsAlive();
};

class player : public npc
{
public:
	bool IsTeammate();
	bool IsFriend();
};