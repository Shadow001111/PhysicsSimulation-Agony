#pragma once
#include <cstdint>

namespace PS_AGONY
{
	// Can be changed.

	using Real = float;

	// Can't be changed.

	using ObjectIndex = uint32_t;
	using ColliderIndex = ObjectIndex;
	using MaterialIndex = uint32_t;
	using BodyTextureId = uint32_t;

	struct ObjectPair
	{
		ObjectIndex a, b;
	};

	struct ObjectDeletion
	{
		ObjectIndex deletedIndex;
		ObjectIndex swappedFromIndex;
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
		Joint,
		COUNT
	};

	struct BodyAttachment
	{
		ConstraintType type;
		ObjectIndex objectIndex;
	};
}