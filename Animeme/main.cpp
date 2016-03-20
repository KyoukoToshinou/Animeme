#include "main.h"


int __stdcall DllMain(void* pBaseExecutable, int iAttachReason, void*)
{
	if (iAttachReason == 1) {
		if (!console::Init())
			goto dll_main_end;

		language::ChangeLanguage("fr");
		console::Print(console::con_info, "#EntryPoint_Welcome");
		language::ChangeLanguage("en");
		console::Print(console::con_info, "#EntryPoint_Welcome");
		language::ChangeLanguage("jp");
		console::Print(console::con_info, "#EntryPoint_Welcome");

		gamemanager::Init();

		

		//if (szExecutableName)
		//console::Print("%i", std::stoi(file::Read(".//steam_appid.txt")));
		/*if ((unsigned long)base_executable == 0x6DF30000) {
			console::print("game is call of doteh foure");
		}*/

	}

dll_main_end:
	return 1;
}