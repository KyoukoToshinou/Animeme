#pragma once

namespace language
{
	extern std::map<const char*, const char*> aSentences;
	const char* GetSentence(const char*);
	const char* SetSentence(const char*, const char*);
	const char* SetFormatSentence(const char*, const char*, ...);
	bool SentenceExists(const char*);
	void ChangeLanguage(const char*);
}