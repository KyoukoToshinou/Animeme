#pragma once

namespace math
{
	template<typename type_t>
	inline type_t Clamp(type_t a1, type_t min, type_t max)
	{
		return a1 > max ? max : a1 < min ? min : a1;
	}

	float Lerp(float, float, float);
}