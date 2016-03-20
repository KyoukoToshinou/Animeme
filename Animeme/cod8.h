#pragma once

namespace cod8
{
	namespace sdk
	{
#define FL_NONE 0
#define FL_CROUCH 0x4
#define FL_PRONE 0x8
#define FL_DEAD 0x40000
#define FL_ZOOM 0x80000
#define FL_FIRE 0x800000

#define ET_GENERAL 0
#define ET_PLAYER 1
#define ET_PLAYER_CORPSE 2
#define ET_ITEM 3
#define ET_EXPLOSIVE 4
#define ET_INVISIBLE 5
#define ET_SCRIPTMOVER 6
#define ET_SOUND_BLEND 7
#define ET_FX 8
#define ET_LOOP_FX 9
#define ET_PRIMARY_LIGHT 10
#define ET_TURRET 11
#define ET_HELICOPTER 12
#define ET_PLANE 13
#define ET_VEHICLE 14
#define ET_VEHICLE_COLLMAP 15
#define ET_VEHICLE_CORPSE 16
#define ET_VEHICLE_SPAWNER 17

#define PLAYERSMAX 18
#define ENTITIESMAX 2048

#define BONESMAX 20

		// typedef float Color[4];
		typedef struct {
			char _0x0000[64];
		} ScreenMatrix;

		typedef struct {
			float fraction;
			char _0x004[44];
		} Trace_T; //Size=0x002C

		typedef struct {
			char _0x0000[2];
			SHORT Valid; //0x0002
			char _0x0004[16];
			Vector Origin; //0x0014
			Vector Angles; //0x0020
			char _0x002C[60];
			int Flags; //0x0068
			char _0x006C[12];
			Vector OldOrigin; //0x0078
			char _0x0084[76];
			int ClientNum; //0x00D0
			SHORT Type; //0x00D4
			char _0x00D6[18];
			Vector NewOrigin; //0x00E8
			char _0x00F4[64];
			BYTE OldWeaponID; //0x0134
			char _0x0135[99];
			BYTE WeaponID; //0x0198
			char _0x0199[1];
			SHORT Alive; //0x019A
			char _0x019C[8];
			float Flags2; //0x01A4
			char _0x01A8[40];
			int IsAlive; //0x01D0
			char _0x01D4[36];
		} entity_t; //Size=0x01F8

		typedef struct {
			char _0x0000[0x8];
			int Width;
			int Height;
			int FovX;
			int FovY;
			Vector ViewOrg;
			Vector ViewAxis[3];
			char _0x0048[0x24];
			float ZoomProgress;
			char _0x0070[0x4ABC];
			float ViewAngles[3];
			char _0x4B34[0x30];
			float WeaponViewAngleX;
			float WeaponViewAngleY;

		} refdef_t; //Size=0x0070

		typedef struct {
			int servertime; //0x0000
			int playerstate; //0x0004
			int staminatimer; //0x0008
			unsigned short playerstance; //0x000C
			char _pad[10];
			int velocity; //0x0018
			float Origin[3]; //0x001C
			float Velocity[3]; //0x0028
			char _pad1[44];
			float refdefViewAngleY; //0x0060
			float refdefViewAngleX; //0x0064
			char _pad2[232];
			int ClientNum; //0x0150
			char _pad3[4];
			float viewAngleY; //0x0158
			float viewAngleX; //0x015C
			char _pad4[4];
			int   playerStanceInt; //0x0164
			float playerStanceFlt; //0x0168
			char _pad5[80];
			int MaxEntities; //0x01BC
			char _0x01C0[68];
			int AdvancedUAV;                                 //0x204 (0x9D64A4)
			char _0x0208[0x38];                              //0x208
			int NextAttack;                                  //0x240 (0x9D64E0)
			char _0x0244[0x12C];                             //0x244
			int WeaponID;
		} cg_t; //Size=0x0438

		typedef struct {
			char _0x0000[8];
			int Width; //0x0008
			int Height; //0x000C
			char _0x0010[20];
			char GameType[4]; //0x0024
			char _0x0028[28];
			char HostName[64]; //0x0044
			char _0x0084[196];
			int MaxClients; //0x0148
			char _0x014C[4];
			char MapName[64]; //0x0150

		} cgs_t; //Size=0x0190

		typedef struct {
			int Valid;                                       //0x0 (0xAD28F8)
			char _0x0004[0x8];                               //0x4
			char Name[16];                                   //0xC (0xAD2904)
			int Team;                                        //0x1C (0xAD2914)
			char _0x0020[0x4];                               //0x20
			int Rank;                                        //0x24 (0xAD291C)
			char _0x0028[0x10];                              //0x28
			int Perk;                                        //0x38 (0xAD2930)
			char _0x003C[0x8];                               //0x3C
			int Score;                                       //0x44 (0xAD293C)
			char _0x0048[0x458];                             //0x48
			int Attacking;                                   //0x4A0 (0xAD2D98)
			char _0x04A4[0x4];                               //0x4A4
			int Zooming;                                     //0x4A8 (0xAD2DA0)
			char _0x04AC[0xB8];                              //0x4AC
		} client_t; //[Addr: 0xAD28F8] [Size: 0x564]

		typedef struct {
			Vector Recoil; //0x0044
			Vector vOrigin; //0x0050
			float DeltaOrigin[3]; //0x005C
			float ReadViewAngleY; //0x0068
			float ReadViewAngleX; //0x006C
			char _0x0070[108];
			float SetViewAngleY; //0x00DC
			float SetViewAngleX; //0x00E0

		} viewMatrix_t; //Size=0x00A4

		typedef struct {
			char *szName; //0x0000 
			char _0x0004[4];
			char *szLocalizedName; //0x0008 
			char _0x000C[60];
			float flZoomedFOV; //0x0048 
			char _0x004C[8];
			__int32 iMaxClipRounds; //0x0054 
			__int32 iBulletImpactEffectType; //0x0058 
			__int32 iWeaponFireDelay; //0x005C 
			__int32 iWeaponBulletType; //0x0060 
			char _0x0064[4];
			Vector vWeaponWeight; //0x0068 
			char _0x0074[16];
			__int32 Icon; //0x0084 
			char _0x0088[920];
			float Spread0; //0x0420
			float Spread1; //0x0424
			float Spread2; //0x0428
			float Spread3; //0x042C
			float Spread4; //0x0430
			float Spread5; //0x0434
			float Spread6; //0x0438
			float Spread7; //0x043C
			float Spread8; //0x0440
			float Spread9; //0x0444
			char _0x0448[356];
			float Recoil1; //0x05AC
			float Recoil2; //0x05B0
			float Recoil3; //0x05B4
			float Recoil4; //0x05B8
			char _0x05BC[52];
			float Recoil5; //0x05F0
			float Recoil6; //0x05F4
			float Recoil7; //0x05F8
			float Recoil8; //0x05FC

		} weapon_t; //Size=0x0600

		typedef struct {
			int MaxEntNum;
			int EntitieNum;
			float visible;
			int z_crap0;
			char lol[4];
			float viewOrigin[3];
			float start[3];
			float end[3];
			float viewAngle[3];
		} BulletTrace_T;

		typedef struct {
			char Name[20];
			int x;
			int y;
			int width;
			int height;
		}shader_t;

		//yay My first reversing...
		typedef struct {
			BYTE Active;
			BYTE Pressed;
			char padding[18];
		}Key;

		typedef struct {
			char unk1[216];
			Key Jump;
			char unk2[60];
			Key Weapon;
			Key HoldBreath;
			char unk3[20];
			Key Lethal;
			Key Tactical;
			Key Knife;
			Key Use;
			Key Reload;
		}Input_t;

		typedef int(*DrawRotatedPic_t)(ScreenMatrix*, float, float, float, float, float, float*, int*);
		extern DrawRotatedPic_t DrawRotatedPic;

		typedef int*(*RegisterShader_t)(char* szShadername);
		extern RegisterShader_t RegisterShader;

		typedef void*(*RegisterFont_t)(char* szFontname);
		extern RegisterFont_t RegisterFont;

		typedef void(*UIShowList_t)(int, int, int, int);
		extern UIShowList_t UIShowList;

		typedef bool(*WorldToScreen_t)(int, ScreenMatrix*, float*, float*);
		extern WorldToScreen_t WorldToScreen;

		typedef int(*DrawEngineText_t)(char*, int, void*, float, float, float, float, float, float*, int);
		extern DrawEngineText_t DrawEngineText;

		typedef ScreenMatrix*(*GetScreenMatrix_t)();
		extern GetScreenMatrix_t GetScreenMatrix;
	}

	namespace pointers
	{
		extern unsigned long ulEntities;
		extern unsigned long ulEntitiesSize;
		extern unsigned long ulClientInfo;
		extern unsigned long ulClientInfoSize;

		extern int* pWhiteShader;
		extern void* pFont;
	}

	void newUIShowList(int, int, int, int);
	bool Init();
}
