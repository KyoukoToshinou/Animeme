#pragma once

class VTableTools
{
public:
	int iTableSize;
	unsigned long *originalTable, *newTable; // unsigned long* originalTable, newTable; <--- doesn't make newTable a pointer

											 // ** = pointer to array?
	template<typename fTypeDef>
	VTableTools(fTypeDef pPointer)
	{
		unsigned long** pToClassArray = (unsigned long**)pPointer; // We could save this to unhook later
		if (!pToClassArray) {
			//Console::Print("pToClassArray is invalid!", Console::Level::Error, "VTableTools"); // con::Print(con::console::RED, "VTableTools: pToClassArray is invalid!");
			return;
		}

		originalTable = *pToClassArray;

		for (iTableSize = 0; originalTable[iTableSize]; iTableSize++)
			if (IsBadCodePtr((FARPROC)originalTable[iTableSize]))
				break;

		newTable = new unsigned long[iTableSize]; // We create a new ulong array and we assign it to newTable pointer
												  // con::Print(con::console::RED, "%i, %i", sizeof(newTable), (sizeof(DWORD)* iTableSize));
		memcpy(newTable, originalTable, sizeof(unsigned long)* iTableSize); // We copy the content of the original table into the new one
		*pToClassArray = newTable; // We override the original table with the new one
	}

	unsigned long Hook(unsigned long hookedFunction, int iIndex)
	{
		// This works because when the game queries a vfunc, it'll come grab it on newTable since we've set the class array to ours
		newTable[iIndex] = hookedFunction;

		return originalTable[iIndex];
	}
};