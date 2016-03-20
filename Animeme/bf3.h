#pragma once
#include "bf3_sdk.h"

#define CONCAT_IMPL(x, y) x##y
#define MACRO_CONCAT(x, y) CONCAT_IMPL(x, y)
#define PAD(SIZE) BYTE MACRO_CONCAT(_pad, __COUNTER__)[SIZE];

namespace bf3
{
	namespace sdk
	{
		class RenderScreenInfo
		{
		public:
			UINT m_nWidth;					// this+0x0
			UINT m_nHeight;					// this+0x4
			UINT m_nWindowWidth;			// this+0x8
			UINT m_nWindowHeight;			// this+0xC
			FLOAT fRefreshRate;				// this+0x10
		};

		class DxRenderer
		{
		public:
			BYTE Pad_000[0x8];				// 0x00
			UINT m_nFrameCounter;			// 0x08
			BOOL m_bFrameInProgress;		// 0x0C
			HWND m_hWnd;					// 0x10
			BYTE Pad_014[0x4];				// 0x14
			BYTE m_bFullscreenWanted;		// 0x18
			BYTE m_bFullscreenActive;		// 0x19
			BYTE m_bMinimized;				// 0x1A
			BYTE m_bMinimizing;				// 0x1B
			BYTE m_bResizing;				// 0x1C
			BYTE m_bOccluded;				// 0x1D
			BYTE m_bVSync;					// 0x1E
			PAD(0x1);						// 0x1F
			RenderScreenInfo m_screenInfo;	// 0x20
			PAD(0xA4);						// 0x34
			void* pDevice;					// 0xD8
			void* pContext;					// 0xDC
			PAD(0x14);						// 0xE0
			void* pSwapChain;				// 0xF4
		};

		typedef void*(__stdcall* GetDebugRenderer2_t)();
		extern GetDebugRenderer2_t GetDebugRenderer2;

		typedef int(__stdcall* SwapChain_t)(int, int, int);
		extern SwapChain_t SwapChain;

		typedef void(__thiscall* DrawText_t)(void*, int, int, char*, Color, float);
		extern DrawText_t String;

		typedef void(__thiscall* Rectangle_t)(void*, float*, float*, Color);
		extern Rectangle_t Rectangle;

		typedef void(__thiscall* FilledRectangle_t)(void*, float*, float*, Color);
		extern FilledRectangle_t FilledRectangle;

		typedef void(__thiscall* Line_t)(void*, float*, float*, Color);
		extern Line_t Line;
	}

	namespace pointers
	{
		extern fb::ClientGameContext* pClientGameContext;
		extern fb::ClientPlayerManager* pClientPlayerManager;
		extern sdk::DxRenderer* pDxRenderer;
		extern void* pDebugRenderer2;
	}

	int __stdcall newSwapChain(int, int, int);
	bool Init();
}