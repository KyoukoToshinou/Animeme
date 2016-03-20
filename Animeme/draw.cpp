#include "main.h"

int draw::GetTextWidth(float flSize, const char* szString, ...)
{
	int iResult = 0;

	if (gamemanager::gameType == gamemanager::game_bf3) {
		do {
			switch (*szString) {
			case ' ':
			case '_':
				iResult += 10;
				break;

			default:
				iResult += 8;
				break;
			}
		} while (*szString++ != '\0');
		iResult *= flSize;
	}
	else if (gamemanager::gameType == gamemanager::game_cod4 || gamemanager::gameType == gamemanager::game_cod8) {
		do {
			switch (*szString) {
			case 32:
			case 114:
				iResult += 4;
				break;
			case 82:
			case 101:
			case 107:
				iResult += 6;
				break;
			case 73:
			case 105:
			case 108:
				iResult += 2;
				break;
			case 106:
				iResult += 3;
				break;
			case 109:
				iResult += 10;
				break;
			case 65:
			case 67:
			case 72:
			case 75:
			case 79:
			case 80:
			case 81:
			case 86:
			case 89:
			case 118:
			case 121:
				iResult += 7;
				break;
			case 77:
			case 119:
				iResult += 9;
				break;
			case 87:
				iResult += 11;
				break;
			case 88:
				iResult += 8;
				break;
			default:
				iResult += 5;
				break;
			}
		} while (*szString++ != '\0');
	}

	return iResult;
}

int draw::GetTextHeight(float flSize, const char* szString, ...)
{
	int iResult = 0;
	if (gamemanager::gameType == gamemanager::game_bf3)
		iResult = (20 / 1.5f) * flSize;
	else if (gamemanager::gameType == gamemanager::game_cod4 || gamemanager::gameType == gamemanager::game_cod8)
		iResult = 13 * flSize;

	return iResult;
}

int draw::String(int iAlignXAxis, int iAlignYAxis, int x, int y, Color colColor, float flSize, const char* szString, ...)
{
	int iResult = 0;
	char* szFormattedString = (char*)malloc(2048);

	if (language::SentenceExists(szString))
		strcpy(szFormattedString, language::GetSentence(szString));//strcpy(szFormattedString, language::GetSentence(szString));
	else {
		va_list vaList;
		va_start(vaList, szString);
		_vsnprintf(szFormattedString, 2048, szString, vaList);
		va_end(vaList);
	}

	int iTextWidth = GetTextWidth(flSize, szFormattedString), iTextHeight = GetTextHeight(flSize, szFormattedString);
	x += iTextWidth * (iAlignXAxis == 2 ? 1 : iAlignXAxis == 1 ? 0.5f : 0);
	y += iTextHeight * (iAlignYAxis == 2 ? 1 : iAlignYAxis == 1 ? 0.5f : 0);
	iResult = iTextHeight;

	if (gamemanager::gameType == gamemanager::game_bf3)
		bf3::sdk::String(bf3::pointers::pDebugRenderer2, x, y, szFormattedString, colColor, flSize);
	else if (gamemanager::gameType == gamemanager::game_cod4)
		cod4::sdk::String(x, y + iTextHeight, cod4::pointers::pFont, szFormattedString, colColor.GetFloatColor(), flSize, flSize, 0.0f, 0.0f);
	else if (gamemanager::gameType == gamemanager::game_cod8)
		cod8::sdk::DrawEngineText(szFormattedString, strlen(szFormattedString), cod8::pointers::pFont, x, y + iTextHeight, 0.5f * flSize, 0.5f * flSize, 0, colColor.GetFloatColor(), 1);

	return iResult;
}

void draw::FilledRectangle(int x, int y, int w, int h, Color colColor)
{
	if (gamemanager::gameType == gamemanager::game_bf3) {
		float* flMins = (float*)malloc(sizeof(float) * 2), *flMaxs = (float*)malloc(sizeof(float) * 2);
		flMins[0] = x; flMins[1] = y;
		flMaxs[0] = flMins[0] + w; flMaxs[1] = flMins[1] + h;
		bf3::sdk::FilledRectangle(bf3::pointers::pDebugRenderer2, flMins, flMaxs, colColor);
	}
	else if (gamemanager::gameType == gamemanager::game_cod4)
		cod4::sdk::Shader(x, y, w, h, colColor.GetFloatColor(), cod4::sdk::iWhiteShader);
	else if (gamemanager::gameType == gamemanager::game_cod8) {
		cod8::sdk::ScreenMatrix* pScreenMatrix = cod8::sdk::GetScreenMatrix();
		if (pScreenMatrix)
			cod8::sdk::DrawRotatedPic(pScreenMatrix, x, y, w, h, 0, colColor.GetFloatColor(), cod8::pointers::pWhiteShader);
	}
}

void draw::Rectangle(int x, int y, int w, int h, Color colColor)
{
	FilledRectangle(x, y, w, 1, colColor);
	FilledRectangle(x, y + h - 1, w, 1, colColor);
	FilledRectangle(x, y, 1, h, colColor);
	FilledRectangle(x + w - 1, y, 1, h, colColor);
}

void draw::RoundedFilledRectangle(int x, int y, int w, int h, Color colColor)
{
	FilledRectangle(x + 1, y + 1, w - 2, h - 2, colColor);
	FilledRectangle(x + 1, y, w - 2, 1, colColor);
	FilledRectangle(x + 1, y + h - 1, w - 2, 1, colColor);
	FilledRectangle(x, y + 1, 1, h - 2, colColor);
	FilledRectangle(x + w - 1, y + 1, 1, h - 2, colColor);
}

void draw::Line(int x1, int y1, int x2, int y2, Color colColor)
{
	if (gamemanager::gameType == gamemanager::game_bf3) {
		float* flStart = (float*)malloc(sizeof(float) * 2), *flEnd = (float*)malloc(sizeof(float) * 2);
		flStart[0] = x1; flStart[1] = y1;
		flEnd[0] = x2; flEnd[1] = y2;
		bf3::sdk::Line(bf3::pointers::pDebugRenderer2, flStart, flEnd, colColor);
	}
	else if (gamemanager::gameType == gamemanager::game_cod4) {
		float x, y, angle, l1, l2, h1;
		h1 = y2 - y1;
		l1 = x2 - x1;
		l2 = sqrt(l1 * l1 + h1 * h1);
		x = x1 + ((l1 - l2) / 2);
		y = y1 + (h1 / 2);
		angle = (float)atan(h1 / l1) * (180 / pi);
		cod4::sdk::RotatedPic(x, y, l2, 1, angle, colColor.GetFloatColor(), cod4::sdk::iWhiteShader);
	}
	else if (gamemanager::gameType == gamemanager::game_cod8) {
		cod8::sdk::ScreenMatrix* pScreenMatrix = cod8::sdk::GetScreenMatrix();
		if (pScreenMatrix) {
			float x, y, angle, l1, l2, h1;
			h1 = y2 - y1;
			l1 = x2 - x1;
			l2 = sqrt(l1 * l1 + h1 * h1);
			x = x1 + ((l1 - l2) / 2);
			y = y1 + (h1 / 2);
			angle = (float)atan(h1 / l1) * (180 / pi);
			cod8::sdk::DrawRotatedPic(pScreenMatrix, x, y, l2, 1, angle, colColor.GetFloatColor(), cod8::pointers::pWhiteShader);
		}
	}
}
