#pragma once

namespace file
{
	const char* Read(const char*);
	void Write(const char*, const char*, bool = false);
	bool Exists(const char*);
	bool IsFolder(const char*);
	const char* RemovePath(const char*);
}