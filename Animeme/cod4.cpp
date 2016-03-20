#include "main.h"

int cod4::sdk::iWhiteShader = 0;
cod4::sdk::RegisterShader_t cod4::sdk::RegisterShader = nullptr;
cod4::sdk::RegisterFont_t cod4::sdk::RegisterFont = nullptr;
cod4::sdk::EndFrame_t cod4::sdk::EndFrame = nullptr;
void* cod4::pointers::pFont = nullptr;
cod4::sdk::refdef_t* cod4::pointers::pRefdef = nullptr;
cod4::sdk::entity_t* cod4::pointers::pEntities = nullptr;
cod4::sdk::clientInfo_t* cod4::pointers::pClientInfo = nullptr;

void cod4::sdk::RotatedPic(float x, float y, float w, float h, float flAngle, float* pColor, int iShader)
{
	__asm
	{
		mov edx, 0x00E343D8
		mov ebx, 0x431490
		push iShader
		push pColor
		push flAngle
		push h
		push w
		push y
		push x
		call ebx
		add esp, 0x1C
	}
}

void cod4::sdk::StretchPic(float x, float y, float w, float h, float uk1, float uk2, float uk3, float uk4, float* pColor, int iMaterial)
{
	DWORD dwCall = 0x5F65F0;

	__asm
	{
		push	pColor
		push	uk4
		push	uk3
		push	uk2
		push	uk1
		push	h
		push	w
		push	y
		push	x
		mov		eax, iMaterial
		call[dwCall]
		add		esp, 0x24
	}
}

void cod4::sdk::Shader(float x, float y, float w, float h, float* colColor, int iShader)
{
	StretchPic(x, y, w, h, 0.0f, 0.0f, 1.0f, 1.0f, colColor, iShader);
}

void cod4::sdk::String(float x, float y, void* pFont, char* pText, float* pColor, float sX, float sY, float f1, float f2)
{
	DWORD dwCall = 0x5F6B00;

	sX *= 1.153846153846154f;
	sY *= 1.153846153846154f;

	__asm
	{
		push	f2
		push	f1
		push	sY
		push	sX
		push	y
		push	x
		push	pFont
		push	0x7FFFFFFF
		push	pText
		mov		ecx, pColor
		call[dwCall]
		add		esp, 0x28
	}
}

bool cod4::sdk::WorldToScreen(Vector vecWorldLocation, float& flScreenX, float& flScreenY)
{
	int iCenterX = pointers::pRefdef->Width / 2;
	int iCenterY = pointers::pRefdef->Height / 2;

	Vector vLocal, vTransForm;
	Vector vright = pointers::pRefdef->viewaxis[1],
		vup = pointers::pRefdef->viewaxis[2],
		vfwd = pointers::pRefdef->viewaxis[0];

	vLocal = vecWorldLocation - pointers::pRefdef->vieworg;

	vTransForm[0] = DotProduct(vLocal, vright);
	vTransForm[1] = DotProduct(vLocal, vup);
	vTransForm[2] = DotProduct(vLocal, vfwd);

	if (vTransForm.z < 0.01)
		return 0;

	flScreenX = iCenterX * (1 - (vTransForm.x / pointers::pRefdef->Fovx / vTransForm.z));
	flScreenY = iCenterY * (1 - (vTransForm.y / pointers::pRefdef->Fovy / vTransForm.z));
	return vTransForm.z > 0;
}

signed int cod4::newEndFrame()
{
	static bool bInitialized = false;
	if (!bInitialized || GetAsyncKeyState(VK_F1))
	{
		pointers::pFont = sdk::RegisterFont("fonts/normalfont", 1);
		sdk::iWhiteShader = sdk::RegisterShader("white", 7);
		bInitialized = true;
	}

	gamemanager::Visuals();
	return sdk::EndFrame();
}

bool cod4::Init()
{
	pointers::pEntities = (cod4::sdk::entity_t*)0x84F2D8;
	pointers::pClientInfo = (cod4::sdk::clientInfo_t*)0x839270;
	pointers::pRefdef = (cod4::sdk::refdef_t*)0x797600;
	sdk::RegisterShader = (sdk::RegisterShader_t)0x5F2A80;
	sdk::RegisterFont = (sdk::RegisterFont_t)0x5F1EC0;
	sdk::EndFrame = (sdk::EndFrame_t)DetourCreate((void*)0x45CEF0, (void*)newEndFrame, DETOUR_TYPE_NOP_NOP_JMP);
	return true;
}