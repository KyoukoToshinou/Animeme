#include "main.h"

bool fb::AutoFire = false;

#define OFF_DEBUGRENDERER2_DRAWTEXT		0x004B7610
#define OFF_DEBUGRENDERER2_DRAWLINE		0x004B9980
#define OFF_DEBUGRENDERER2_DRAWRECT		0x4BA4F0
#define OFF_DEBUGRENDERER2_FILLRECT		0x4BA4F0
#define OFF_DEBUGRENDERER2_SINGLETON	0x4B2EA0
#define OFF_DEBUGRENDERER2_SPHERE		0x0043AF20
#define OFFSET_DXRENDERER				0x023577D4

fb::ClientGameContext* bf3::pointers::pClientGameContext = nullptr;
fb::ClientPlayerManager* bf3::pointers::pClientPlayerManager = nullptr;
bf3::sdk::DxRenderer* bf3::pointers::pDxRenderer = nullptr;
void* bf3::pointers::pDebugRenderer2 = nullptr;
bf3::sdk::GetDebugRenderer2_t bf3::sdk::GetDebugRenderer2 = nullptr;
bf3::sdk::SwapChain_t bf3::sdk::SwapChain = nullptr;
bf3::sdk::DrawText_t bf3::sdk::String = nullptr;
bf3::sdk::Rectangle_t bf3::sdk::Rectangle = nullptr;
bf3::sdk::FilledRectangle_t bf3::sdk::FilledRectangle = nullptr;
bf3::sdk::Line_t bf3::sdk::Line = nullptr;

int __stdcall bf3::newSwapChain(int a1, int a2, int a3)
{
	int iResult = sdk::SwapChain(a1, a2, a3);

	pointers::pDebugRenderer2 = sdk::GetDebugRenderer2();
	gamemanager::Visuals();

	return iResult;
}

bool bf3::Init()
{
	sdk::GetDebugRenderer2 = (sdk::GetDebugRenderer2_t)OFF_DEBUGRENDERER2_SINGLETON;
	sdk::String = (sdk::DrawText_t)OFF_DEBUGRENDERER2_DRAWTEXT;
	sdk::Rectangle = (sdk::Rectangle_t)OFF_DEBUGRENDERER2_DRAWRECT;
	sdk::FilledRectangle = (sdk::FilledRectangle_t)OFF_DEBUGRENDERER2_FILLRECT;
	sdk::Line = (sdk::Line_t)OFF_DEBUGRENDERER2_DRAWLINE;

	pointers::pClientGameContext = *(fb::ClientGameContext**)OFFSET_CLIENTGAMECONTEXT;
	pointers::pClientPlayerManager = pointers::pClientGameContext->m_clientPlayerManager;
	pointers::pDxRenderer = *(sdk::DxRenderer**)OFFSET_DXRENDERER;

	VTableTools* SwapChainVTable = new VTableTools((DWORD**)pointers::pDxRenderer->pSwapChain);
	sdk::SwapChain = (sdk::SwapChain_t)SwapChainVTable->Hook((DWORD)newSwapChain, 8);

	return true;
}
