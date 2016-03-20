#include "main.h"

void* console::pHandle = nullptr;

bool console::Init()
{
	AllocConsole();
	freopen("CONIN$", "r", stdin);
	freopen("CONOUT$", "w", stdout);
	freopen("CONOUT$", "w", stderr);
	pHandle = GetStdHandle(STD_OUTPUT_HANDLE);
	if (pHandle == nullptr)
		printf("fatal error > couldn't get handle to console\n");

	return pHandle;
}

int console::Color(int iColor)
{
	if (pHandle == nullptr)
		return con_gray;

	_CONSOLE_SCREEN_BUFFER_INFO coninfo;
	if (iColor >= 0)
		SetConsoleTextAttribute(pHandle, iColor);

	return (int)(GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &coninfo) ? coninfo.wAttributes : con_gray);
}

void console::Print(const char* szString, ...)
{
	if (pHandle == nullptr)
		return;

	char* szConsoleString = (char*)malloc(2048);

	va_list vaList;
	va_start(vaList, szString);
	_vsnprintf(szConsoleString, 2048, szString, vaList);
	va_end(vaList);

	printf(szConsoleString);
	printf("\n");
}

void console::Print(int iLevel, const char* szString, ...)
{
	if (pHandle == nullptr)
		return;

	char* szConsoleString = (char*)malloc(2048);
	char con_output[2048];

	if (language::SentenceExists(szString))
		strcpy(szConsoleString, language::GetSentence(szString));
	else {
		va_list vaList;
		va_start(vaList, szString);
		_vsnprintf(szConsoleString, 2048, szString, vaList);
		va_end(vaList);
	}

	int con_color = Color();

	switch (iLevel)
	{
	case con_none:
		Color(con_gray);
		sprintf(con_output, "%s", szConsoleString);
		break;
	case con_info:
		Color(con_blue);
		sprintf(con_output, "%s > %s", language::GetSentence("#Console_Info"), szConsoleString);
		break;
	case con_warning:
		Color(con_yellow);
		sprintf(con_output, "%s > %s", language::GetSentence("#Console_Warning"), szConsoleString);
		break;
	case con_error:
		Color(con_red);
		sprintf(con_output, "%s > %s", language::GetSentence("#Console_Error"), szConsoleString);
		break;
	}

	printf(con_output);
	printf("\n");

	Color(con_color);
}