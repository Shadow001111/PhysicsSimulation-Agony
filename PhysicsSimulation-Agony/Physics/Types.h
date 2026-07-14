#pragma once
#include <cstdint>

namespace PS_AGONY
{
	// Can be changed.

	using Real = float;

	// Can't be changed.

	using BodyIndex = uint32_t;
	using MaterialIndex = uint32_t;
	using BodyTextureId = uint32_t;

	struct BodyPair
	{
		BodyIndex a, b;
	};

	struct BodyDeletion
	{
		BodyIndex deletedIndex;
		BodyIndex swappedFromIndex;
	};

	enum class BodyType : uint8_t
	{
		Circle,
		Box,
		Polygon,
		COUNT
	};

	struct AABB
	{
		Real minX, minY;
		Real maxX, maxY;
	};

	enum class ConstraintType : uint32_t
	{
		Spring,
		COUNT
	};

	struct BodyAttachment
	{
		ConstraintType type;
		uint32_t objectIndex;
	};
}