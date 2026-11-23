#pragma once

// Private helpers shared by the SimulationRenderer translation units.

#include "PS_AGONY2D/ObjectSoA.h"
#include "PS_AGONY2D/Constants.h"
#include "PS_AGONY2D/FastCosSin.h"
#include "Ecstasy/Core/Portablity.h"

#include <cstdint>

namespace PS_AGONY_RENDER
{
	using namespace PS_AGONY2D;

	[[nodiscard]] inline uint32_t idToHexColor(uint32_t x) noexcept
	{
		x ^= x >> 16;
		x *= 0x7feb352dU;
		x ^= x >> 15;
		x *= 0x846ca68bU;
		x ^= x >> 16;
		return x & 0xFFFFFF;
	}

	struct CullRect
	{
		float minX, minY, maxX, maxY;

		explicit CullRect(const AABB& aabb) noexcept
			: minX(static_cast<float>(aabb.minX)), minY(static_cast<float>(aabb.minY))
			, maxX(static_cast<float>(aabb.maxX)), maxY(static_cast<float>(aabb.maxY))
		{
		}

		[[nodiscard]] bool contains(float x, float y) const noexcept
		{
			return x >= minX && x <= maxX && y >= minY && y <= maxY;
		}

		[[nodiscard]] bool overlaps(float otherMinX, float otherMinY, float otherMaxX, float otherMaxY) const noexcept
		{
			return otherMaxX >= minX && otherMinX <= maxX && otherMaxY >= minY && otherMinY <= maxY;
		}
	};

	// Result is wrapped into [0, 2*pi) so it can be fed into FastCosSin.
	[[nodiscard]] __forceinline Real interpolateAndWrapRotation(Real rot, Real oldRot, Real wrapCount, Real alpha) noexcept
	{
		const Real fullRotation = rot + wrapCount * Constants::TWO_PI;

		Real interpolatedRotation = oldRot + (fullRotation - oldRot) * alpha;

		const Real wrapMultiple = std::trunc(interpolatedRotation * Constants::INV_TWO_PI);
		interpolatedRotation -= wrapMultiple * Constants::TWO_PI;

		const Real isRotNegativeMask = interpolatedRotation < 0;
		interpolatedRotation += isRotNegativeMask * Constants::TWO_PI;

		return interpolatedRotation;
	}

	struct BodyTransformSample
	{
		Real x, y; // World center of mass.
		Real cosR, sinR;
		Real rotation;
	};

	// Resolves a body's render transform. Built once per frame.
	// With interpolate == false the lerp and rotation wrap are skipped.
	struct BodyTransformContext
	{
		const Real* ECSTASY_RESTRICT oldX;
		const Real* ECSTASY_RESTRICT oldY;
		const Real* ECSTASY_RESTRICT oldRot;
		const Real* ECSTASY_RESTRICT wrapCount;

		const Real* ECSTASY_RESTRICT x;
		const Real* ECSTASY_RESTRICT y;
		const Real* ECSTASY_RESTRICT rot;

		Real alpha;
		bool interpolate;

		[[nodiscard]] static BodyTransformContext make(const BodySoAViewer& bodies, Real alpha, bool interpolate) noexcept
		{
			return BodyTransformContext{
				bodies.renderOldComX, bodies.renderOldComY, bodies.renderOldRotation, bodies.renderRotationWrapCount,
				bodies.comX, bodies.comY, bodies.rotation,
				alpha, interpolate
			};
		}

		[[nodiscard]] __forceinline BodyTransformSample sample(ObjectDenseIndex bodyIndex) const noexcept
		{
			if (interpolate)
			{
				const Real posX = x[bodyIndex];
				const Real posY = y[bodyIndex];
				const Real oldPosX = oldX[bodyIndex];
				const Real oldPosY = oldY[bodyIndex];

				const Real interpolatedRotation = interpolateAndWrapRotation(rot[bodyIndex], oldRot[bodyIndex], wrapCount[bodyIndex], alpha);
				const auto [c, s] = FastCosSin::order5Scalar(interpolatedRotation);

				return {
					oldPosX + (posX - oldPosX) * alpha,
					oldPosY + (posY - oldPosY) * alpha,
					c, s, interpolatedRotation
				};
			}
			else
			{
				const Real rotation = rot[bodyIndex];
				const auto [c, s] = FastCosSin::order5Scalar(rotation);

				return { x[bodyIndex], y[bodyIndex], c, s, rotation };
			}
		}
	};

	struct ColliderPlacement
	{
		float x, y;
		float comX, comY;
		float rotation;
		ObjectDenseIndex shapeIndex;
	};

	// Resolves a collider's world transform from its body's render transform.
	struct ColliderPlacer
	{
		const BodyTransformContext& transform;

		const ObjectDenseIndex* ECSTASY_RESTRICT bodyIndex;
		const ObjectDenseIndex* ECSTASY_RESTRICT shapeIndex;
		const Real* ECSTASY_RESTRICT comOffsetX;
		const Real* ECSTASY_RESTRICT comOffsetY;
		const Real* ECSTASY_RESTRICT rotation;

		[[nodiscard]] static ColliderPlacer make(const BodySoAViewer&, const ColliderSoAViewer& colliders, const BodyTransformContext& transform) noexcept
		{
			return ColliderPlacer{
				transform,
				colliders.bodyIndex, colliders.shapeIndex,
				colliders.comOffsetX, colliders.comOffsetY, colliders.localRotation
			};
		}

		[[nodiscard]] __forceinline ColliderPlacement place(ObjectDenseIndex colliderIndex) const noexcept
		{
			const ObjectDenseIndex body = bodyIndex[colliderIndex];
			const BodyTransformSample sample = transform.sample(body);

			const Real offX = comOffsetX[colliderIndex];
			const Real offY = comOffsetY[colliderIndex];

			return {
				static_cast<float>(sample.x + (offX * sample.cosR - offY * sample.sinR)),
				static_cast<float>(sample.y + (offX * sample.sinR + offY * sample.cosR)),
				0.0f, // Shaders rotate about the collider origin, which is already placed in world space.
				0.0f,
				static_cast<float>(sample.rotation + rotation[colliderIndex]),
				shapeIndex[colliderIndex]
			};
		}
	};
}