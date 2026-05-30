#pragma once
#include "Types.h"

#include <vector>

namespace PS_AGONY
{
	enum class BodyType : uint8_t
	{
		Circle,
		Box,
		Polygon
	};

	struct AABBSoA
	{
		std::vector<Real> minX;
		std::vector<Real> minY;
		std::vector<Real> maxX;
		std::vector<Real> maxY;
	};

	struct BodiesSoA
	{
		std::vector<Real> positionX;
		std::vector<Real> positionY;

		std::vector<Real> velocityX;
		std::vector<Real> velocityY;

		std::vector<Real> rotation;

		std::vector<Real> angularVelocity;

		std::vector<Real> mass;
		std::vector<Real> invMass;

		std::vector<Real> inertia;
		std::vector<Real> invInertia;

		// std::vector<Real> localCenterOfMassX;
		// std::vector<Real> localCenterOfMassY;

		std::vector<MaterialIndex> materialIndex;

		AABBSoA aabb;

		std::vector<BodyType> bodyType;
		std::vector<BodyIndex> shapeIndex;

		mutable std::vector<uint8_t> collisionDebug;

		size_t getCount() const noexcept { return positionX.size(); }
	};

	struct CirclesSoA
	{
		std::vector<Real> radius;

		std::vector<BodyIndex> bodyIndices;

		size_t getCount() const noexcept { return radius.size(); }
	};
}
