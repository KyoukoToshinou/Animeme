#include "main.h"

_iobuf* iopen(const char* fn, const char* m)
{
	if (access(fn, 0) == -1)
	{
		char p[MAX_PATH]; strcpy(p, fn);

		for (unsigned i = 0; i < strlen(p); i++)
			if (p[i] == '/') {
				char oldch = p[i]; p[i] = 0;
				CreateDirectory(p, 0);
				p[i] = oldch;
			}
	}

	return fopen(fn, m);
}

const char* file::Read(const char* szFilename)
{
	long lFilesize;
	char* szResult = "";
	_iobuf* pFile = iopen(szFilename, "r"); {
		if (pFile != nullptr) {
			fseek(pFile, 0, SEEK_END); {
				lFilesize = ftell(pFile);
				szResult = (char*)malloc(lFilesize + 1);
			} rewind(pFile);

			fread(szResult, sizeof(char), lFilesize, pFile);
			szResult[lFilesize] = '\0';
		}
	} fclose(pFile);

	return szResult;
}

void file::Write(const char* szFilename, const char* szContent, bool bAppend)
{
	_iobuf* pFile = iopen(szFilename, bAppend ? "a" : "w"); {
		if (pFile != nullptr)
			fwrite(szContent, sizeof(char), strlen(szContent), pFile);
	} fclose(pFile);
}

bool file::Exists(const char* szFilename)
{
	struct stat oStat;
	return (stat(szFilename, &oStat) == 0);
}

bool file::IsFolder(const char* szFilename)
{
	struct stat oStat;
	if (stat(szFilename, &oStat) == 0)
		if (oStat.st_mode & S_IFDIR)
			return true;

	return false;
}

const char* file::RemovePath(const char* szFilepath)
{
	int j = 0, k = 0;
	do {
		if (*szFilepath == '\\')
			k = j + 1;
		j++;
	} while (*szFilepath++ != '\0');
	szFilepath -= j;

	int iFilenameSize = j - k;
	char* szFilename = (char*)malloc(iFilenameSize);
	memcpy(szFilename, szFilepath + k, iFilenameSize);
	return szFilename;
}
