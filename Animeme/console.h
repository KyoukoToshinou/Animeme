#pragma once

namespace console
{
	extern void* pHandle;
	enum
	{
		con_none,		// white
		con_info,		// blue
		con_warning,	// yellow
		con_error,		// red
	};

	enum
	{
		con_dark,
		con_darkblue,
		con_darkgreen,
		con_darkteal,
		con_darkred,
		con_darkpink,
		con_darkyellow,
		con_gray,
		con_darkgray,
		con_blue,
		con_green,
		con_teal,
		con_red,
		con_pink,
		con_yellow,
		con_white
	};

	bool Init();
	int Color(int color = -1);
	void Print(const char*, ...);
	void Print(int, const char*, ...);
}
