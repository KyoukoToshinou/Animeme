#include "main.h"

#define MODULE_NAME "animeme"
std::map<const char*, const char*> language::aSentences;

const char* language::GetSentence(const char* szKey)
{
	if (aSentences.find(szKey) == aSentences.end())
		return "*** GetSentence error ***";

	char* oshit = (char*)malloc(strlen(szKey));
	strcpy(oshit, aSentences[szKey]);
	aSentences[szKey] = oshit;
	return oshit;
}

const char* language::SetSentence(const char* szKey, const char* szSentence)
{
	return aSentences[szKey] = szSentence;
}

const char* language::SetFormatSentence(const char* szKey, const char* szSentence, ...)
{
	char szNewSentence[2048];
	va_list vaList;
	va_start(vaList, szSentence);
	_vsnprintf(szNewSentence, sizeof(szNewSentence), szSentence, vaList);
	va_end(vaList);

	return aSentences[szKey] = szNewSentence;
}

bool language::SentenceExists(const char* szKey)
{
	return (aSentences.find(szKey) != aSentences.end());
}

void language::ChangeLanguage(const char* szLanguage)
{
	if (strstr(szLanguage, "fr")) {
		SetSentence("#Console_Info", "info");
		SetSentence("#Console_Error", "erreur");
		SetSentence("#Console_Warning", "avertissement");
		SetFormatSentence("#EntryPoint_Welcome", "bienvenue dans %s!", MODULE_NAME);
	}
	else if (strstr(szLanguage, "jp")) {
		SetSentence("#Console_Info", "info");
		SetSentence("#Console_Error", "era");
		SetSentence("#Console_Warning", "keikoku");
		SetFormatSentence("#EntryPoint_Welcome", "%s e youkoso!", MODULE_NAME);
	}
	else if (strstr(szLanguage, "en")) {
		SetSentence("#Console_Info", "info");
		SetSentence("#Console_Error", "error");
		SetSentence("#Console_Warning", "warning");
		SetFormatSentence("#EntryPoint_Welcome", "welcome to %s!", MODULE_NAME);
	}
}
