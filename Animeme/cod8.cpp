#include "main.h"
#include <Psapi.h>


namespace shitsig {
	MODULEINFO GetModuleInfo(LPCSTR szModule) {
		MODULEINFO modinfo = { 0 };
		HMODULE hModule = GetModuleHandleA(szModule);
		if (hModule == 0) return modinfo;
		GetModuleInformation(GetCurrentProcess(), hModule, &modinfo, sizeof(MODULEINFO));
		return modinfo;
	}

	BOOL Match(const BYTE* pData, const BYTE* bMask, const char* szMask) {
		for (; *szMask; ++szMask, ++pData, ++bMask)
			if (*szMask == 'x' && *pData != *bMask)
				return false;
		return (*szMask) == NULL;
	}

	DWORD FindPattern(char* szSig, char* szMask) {
		DWORD dwAddress = (DWORD)GetModuleHandleA(NULL);
		DWORD dwLen = GetModuleInfo(NULL).SizeOfImage;

		for (DWORD i = 0; i < dwLen; i++)
			if (Match((BYTE*)(dwAddress + i), (BYTE*)szSig, szMask))
				return (DWORD)(dwAddress + i);

		return 0;
	}

	DWORD FindPattern(DWORD dwAddress, DWORD dwLen, char* bMask, char* szMask) {
		for (DWORD i = 0; i < dwLen; i++)
			if (Match((BYTE*)(dwAddress + i), (BYTE*)bMask, szMask))
				return (DWORD)(dwAddress + i);

		return 0;
	}
}

int* cod8::pointers::pWhiteShader = nullptr;
cod8::sdk::RegisterShader_t cod8::sdk::RegisterShader = nullptr;
cod8::sdk::RegisterFont_t cod8::sdk::RegisterFont = nullptr;
cod8::sdk::UIShowList_t cod8::sdk::UIShowList = nullptr;
void* cod8::pointers::pFont = nullptr;
cod8::sdk::WorldToScreen_t cod8::sdk::WorldToScreen = nullptr;
cod8::sdk::DrawEngineText_t cod8::sdk::DrawEngineText = nullptr;
cod8::sdk::DrawRotatedPic_t cod8::sdk::DrawRotatedPic = nullptr;
cod8::sdk::GetScreenMatrix_t cod8::sdk::GetScreenMatrix = nullptr;
unsigned long cod8::pointers::ulEntities = 0;
unsigned long cod8::pointers::ulEntitiesSize = 0;
unsigned long cod8::pointers::ulClientInfo = 0;
unsigned long cod8::pointers::ulClientInfoSize = 0;

void cod8::newUIShowList(int a1, int a2, int a3, int a4)
{
	_asm pushad;
	static bool bInitialized = false;
	if (!bInitialized || GetAsyncKeyState(VK_F1))
	{
		pointers::pFont = sdk::RegisterFont("fonts/normalFont");
		pointers::pWhiteShader = sdk::RegisterShader("white");
		bInitialized = true;
	}

	gamemanager::Visuals();
	sdk::UIShowList(a1, a2, a3, a4);
	__asm popad;
}

bool cod8::Init()
{
	sdk::DrawEngineText = (sdk::DrawEngineText_t)shitsig::FindPattern("\x8b\x44\x24\x04\x80\x38\x00\x0f\x84\x00\x00\x00\x00", "xxxxxxxxx????");
	sdk::RegisterShader = (sdk::RegisterShader_t)shitsig::FindPattern("\x8b\x44\x24\x04\x80\x38\x00\x75\x00", "xxxxxxxx?");
	sdk::RegisterFont = (sdk::RegisterFont_t)utils::FindPattern("\x8b\x44\x24\x04\x6a\x01\x50\x6a\x18");
	sdk::UIShowList = (sdk::UIShowList_t)DetourCreate((void*)shitsig::FindPattern("\xA1\x00\x00\x00\x00\x81\xEC\x00\x00\x00\x00\x80\x78\x0C\x00\x74\x29", "x????xx????xxxxxx"), (void*)newUIShowList, DETOUR_TYPE_NOP_NOP_JMP);
	sdk::WorldToScreen = (sdk::WorldToScreen_t)shitsig::FindPattern("\x83\xEC\x0C\x8B\x44\x24\x18\xD9\x00", "xxxxxxxxx");
	sdk::GetScreenMatrix = (sdk::GetScreenMatrix_t)shitsig::FindPattern("\xA1\x00\x00\x00\x00\x83\xF8\x03\x77\x0D", "x????xxxxx");
	sdk::DrawRotatedPic = (sdk::DrawRotatedPic_t)shitsig::FindPattern("\x83\xEC\x50\xD9\x44\x24\x68", "xxxxxxx");
	unsigned long ulEntityMemory = shitsig::FindPattern("\x69\xc0\x00\x00\x00\x00\x05\x00\x00\x00\x00\x83\xb8\xd4\x00\x00\x00\x01", "xx????x????xxxxxxx");
	pointers::ulEntities = *(unsigned long*)(ulEntityMemory + 0x7);
	pointers::ulEntitiesSize = *(unsigned long*)(ulEntityMemory + 0x2);
	unsigned long ulClientInfoMemory = shitsig::FindPattern("\x69\xc9\x00\x00\x00\x00\x6a\x00\x81\xc1\x00\x00\x00\x00", "xx????xxxx????");
	pointers::ulClientInfo = *(unsigned long*)(ulEntityMemory + 0xA);
	pointers::ulClientInfoSize = *(unsigned long*)(ulEntityMemory + 0x2);
	return true;
}
