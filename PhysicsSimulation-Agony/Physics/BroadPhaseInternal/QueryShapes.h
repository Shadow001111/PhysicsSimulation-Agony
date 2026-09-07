#pragma once
#include "BvhNode.h"

namespace PS_AGONY::BroadPhaseInternal
{
	using RealSimd = Ecstasy::Core::Simd<Real>;

	struct AABBQuery
	{
		AABB aabb;
		RealSimd aabbMinXV;
		RealSimd aabbMaxXV;
		RealSimd aabbMinYV;
		RealSimd aabbMaxYV;

		explicit AABBQuery(const AABB& inAABB) noexcept :
			aabb(inAABB),
			aabbMinXV(inAABB.minX),
			aabbMaxXV(inAABB.maxX),
			aabbMinYV(inAABB.minY),
			aabbMaxYV(inAABB.maxY)
		{
		}

		[[nodiscard]] bool overlaps(const BvhNode& node) const noexcept
		{
			return node.minX < aabb.maxX && node.maxX > aabb.minX &&
				node.minY < aabb.maxY && node.maxY > aabb.minY;
		}

		[[nodiscard]] bool contains(const BvhNode& node) const noexcept
		{
			return node.minX >= aabb.minX && node.maxX <= aabb.maxX &&
				node.minY >= aabb.minY && node.maxY <= aabb.maxY;
		}

		[[nodiscard]] auto simdOverlap(RealSimd minXV, RealSimd maxXV, RealSimd minYV, RealSimd maxYV) const noexcept
		{
			return (minXV < aabbMaxXV) & (maxXV > aabbMinXV) &
				(minYV < aabbMaxYV) & (maxYV > aabbMinYV);
		}
	};

	struct CircleQuery
	{
		Vec2 pos;
		Real radiusSq;
		RealSimd posXV;
		RealSimd posYV;
		RealSimd zeroV;
		RealSimd radiusSqV;

		CircleQuery(Vec2 position, Real radius) noexcept :
			pos(position),
			radiusSq(std::fmax(Real(0), radius)* std::fmax(Real(0), radius)),
			posXV(position.x),
			posYV(position.y),
			zeroV(Real(0)),
			radiusSqV(radiusSq)
		{
		}

		[[nodiscard]] bool overlaps(const BvhNode& node) const noexcept
		{
			const Real dx = std::fmax(node.minX - pos.x, std::fmax(Real(0), pos.x - node.maxX));
			const Real dy = std::fmax(node.minY - pos.y, std::fmax(Real(0), pos.y - node.maxY));
			return (dx * dx + dy * dy) <= radiusSq;
		}

		[[nodiscard]] bool contains(const BvhNode& node) const noexcept
		{
			const Real maxDx = std::fmax(std::fabs(node.minX - pos.x), std::fabs(node.maxX - pos.x));
			const Real maxDy = std::fmax(std::fabs(node.minY - pos.y), std::fabs(node.maxY - pos.y));
			return (maxDx * maxDx + maxDy * maxDy) <= radiusSq;
		}

		[[nodiscard]] auto simdOverlap(RealSimd minXV, RealSimd maxXV, RealSimd minYV, RealSimd maxYV) const noexcept
		{
			const RealSimd dx = RealSimd::max(minXV - posXV, RealSimd::max(zeroV, posXV - maxXV));
			const RealSimd dy = RealSimd::max(minYV - posYV, RealSimd::max(zeroV, posYV - maxYV));
			const RealSimd distSq = RealSimd::mulAdd(dx, dx, dy * dy);
			return distSq <= radiusSqV;
		}
	};
}