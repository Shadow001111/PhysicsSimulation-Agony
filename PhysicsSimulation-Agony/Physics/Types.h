#pragma once
#include <cstdint>

namespace PS_AGONY
{
	using Real = double;
	using BodyIndex = uint32_t;
	using MaterialIndex = uint32_t;
	using BodyTextureId = uint32_t;

	struct BodyPair
	{
		BodyIndex a, b;
	};

	enum class BodyType : uint8_t
	{
		Circle,
		Box,
		Polygon,
		COUNT
	};
}