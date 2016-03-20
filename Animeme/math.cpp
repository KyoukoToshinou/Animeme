#include "main.h"

float math::Lerp(float flFraction, float flStart, float flEnd)
{
	return (1 - flFraction) * flStart + flFraction * flEnd;
}
