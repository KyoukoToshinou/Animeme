#pragma once

namespace utils
{
	unsigned long FindPattern(const char*);
	unsigned long FindPattern(const char*, const char*);
	unsigned long FindPattern(unsigned long, unsigned long, const char*);
	bool DataCompare(const char*, const char*);
}