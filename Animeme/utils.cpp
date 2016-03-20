#include "main.h"

unsigned long utils::FindPattern(const char* pattern)
{
	return FindPattern((unsigned long)GetModuleHandle(0), (unsigned long)-1, pattern);
}

unsigned long utils::FindPattern(const char* dll, const char* pattern)
{
	return FindPattern((unsigned long)GetModuleHandle(dll), (unsigned long)-1, pattern);
}

unsigned long utils::FindPattern(unsigned long address, unsigned long size, const char* pattern)
{
	for (unsigned long i = 0; i < size; ++i, ++address)
		if (DataCompare((const char*)address, pattern))
			return address;

	return NULL;
}

bool utils::DataCompare(const char* base, const char* pattern)
{
	for (; *pattern; ++base, ++pattern)
		if (*pattern != '?' && *base != *pattern)
			return 0;

	return *pattern == 0;
}
