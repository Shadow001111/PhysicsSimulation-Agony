#pragma once
#include <cstdint>
#include <glm/glm.hpp>

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
		Capsule,
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
		Rod,
		Prismatic,
		Slider,
		Motor,
		COUNT
	};

	struct BodyAttachment
	{
		ConstraintType type;
		ObjectIndex objectIndex;
	};

	// Glm.
	using Vec2 = glm::vec<2, Real>;
	using Vec3 = glm::vec<3, Real>;
	using Vec4 = glm::vec<4, Real>;

	using Mat3 = glm::mat<3, 3, Real>;
	using Mat4 = glm::mat<4, 4, Real>;
}