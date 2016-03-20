#include "main.h"

int gamemanager::gameType = 0, gamemanager::steamAppID = 0;

void gamemanager::Init()
{
	char* szExecutableName = (char*)malloc(MAX_PATH);
	GetModuleFileNameA(nullptr, szExecutableName, MAX_PATH);
	const char* szFilename = file::RemovePath(szExecutableName);
	console::Print(szFilename);

	if (strstr(szFilename, "vu.exe") || strstr(szFilename, "bf3.exe")) {
		gameType = game_bf3;
		bf3::Init();
	}
	else if (strstr(szFilename, "iw3mp.exe")) {
		gameType = game_cod4;
		cod4::Init();
	}
	else if (strstr(szFilename, "iw5mp.exe")) {
		gameType = game_cod8;
		cod8::Init();
	}
}

void gamemanager::Input()
{
}

void gamemanager::Visuals()
{
	if (GetAsyncKeyState(VK_F1))
		language::ChangeLanguage("fr");
	else if (GetAsyncKeyState(VK_F2))
		language::ChangeLanguage("en");
	else if (GetAsyncKeyState(VK_F3))
		language::ChangeLanguage("jp");

	/*if (GetKeyState(VK_F7)) {
		int iMaxEntities = entmgr::GetMaxPlayers();
		for (int i = 0; i <= iMaxEntities; i++) {
			entity* pEntity = entmgr::GetEntity(i);
			if (pEntity == nullptr)
				continue;
			if (!pEntity->IsValid())
				continue;
			if (pEntity->GetType() != ent_player)
				continue;

			printf(":: %s ::\n", pEntity->GetName(i));
		}
	}*/
	if (GetKeyState(VK_F7))
	for (int i = 0; i < 18; i++) {
		cod8::sdk::client_t* pShit = (cod8::sdk::client_t*)(cod8::pointers::ulClientInfo + ((int)cod8::pointers::ulClientInfoSize * i));
		if (!pShit)
			continue;

		printf("wow!\n");
		printf("%s\n", pShit->Name);
	}

	draw::FilledRectangle(50, 50, 100, 100, Color(255, 0, 0));
	draw::Rectangle(75, 75, 50, 50, Color(0, 0, 255));
	draw::Line(25, 25, 175, 175, Color(0, 255, 0));
	draw::String(0, 0, 0, 0, Color(rand() % 255, rand() % 255, rand() % 255), 1, "#EntryPoint_Welcome");

	if (GetKeyState(VK_F5)) {
		float x, y;
		game::WorldToScreen(Vector(0, 0, 0), x, y);
		draw::String(0, 0, x, y, Color(50, 150, 255), 1, "Vector(0, 0, 0)");
	}

	/*float ScrW = 1920, ScrH = 1080;
	Color colHeader = Color(255, 100, 0, 200);
	draw::FilledRectangle(ScrW / 2 - 15, ScrH / 2 - 2, 30, 3, Color(0, 0, 0, 200));
	draw::FilledRectangle(ScrW / 2 - 2, ScrH / 2 - 15, 3, 30, Color(0, 0, 0, 200));

	draw::Line(ScrW / 2 - 25, ScrH / 2, ScrW / 2 + 25, ScrH / 2, Color(190, 190, 190, 200));
	draw::Line(ScrW / 2, ScrH / 2 - 25, ScrW / 2, ScrH / 2 + 25, Color(190, 190, 190, 200));

	draw::Line(ScrW / 2 - 15, ScrH / 2, ScrW / 2 + 15, ScrH / 2, colHeader);
	draw::Line(ScrW / 2, ScrH / 2 - 15, ScrW / 2, ScrH / 2 + 15, colHeader);*/
}
