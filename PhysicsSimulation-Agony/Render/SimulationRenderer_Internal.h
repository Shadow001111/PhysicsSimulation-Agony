#pragma once

// Private implementation details shared by SimulationRenderer.cpp,
// SimulationRenderer_Colliders.cpp and SimulationRenderer_Constraints.cpp.
// Not included by the public SimulationRenderer.h - keeps the class's
// public surface unchanged while letting the three translation units
// share code.

#include "Physics/ObjectSoA.h"
#include "Physics/Constants.h"
#include "Physics/FastCosSin.h"

#include <cstdint>

namespace Render
{
	using namespace PS_AGONY;

	[[nodiscard]] static inline uint32_t idToHexColor(uint32_t x) noexcept
	{
		x ^= x >> 16;
		x *= 0x7feb352dU;
		x ^= x >> 15;
		x *= 0x846ca68bU;
		x ^= x >> 16;
		return x & 0xFFFFFF;
	}

	// Interpolates between the old and new rotation of a body and wraps the
	// result into [0, 2*pi) so it can be safely fed into FastCosSin.
	[[nodiscard]] __forceinline static Real interpolateAndWrapRotation(
		const Real& rot,
		const Real& oldRot,
		const Real& wrapCount,
		const Real& alpha)
	{
		const Real fullRotation = rot + wrapCount * PS_AGONY::Constants::TWO_PI;

		Real interpolatedRotation = oldRot + (fullRotation - oldRot) * alpha;

		const Real wrapMultiple = std::trunc(interpolatedRotation * Constants::INV_TWO_PI);
		interpolatedRotation -= wrapMultiple * Constants::TWO_PI;

		const Real isRotNegativeMask = interpolatedRotation < 0;
		interpolatedRotation += isRotNegativeMask * Constants::TWO_PI;

		return interpolatedRotation;
	}

	// Resolved world-space transform of a single body for this frame.
	struct BodyTransformSample
	{
		Real x, y;
		Real cosR, sinR;
		Real rotation;
	};

	// Bundles the body SoA pointers needed to resolve a body's render
	// transform, plus the render alpha and whether interpolation is active.
	// Built once per renderSimulation() call and passed by const-ref to
	// every sub-function, instead of each one re-fetching the same 7
	// pointers and re-deriving the same math.
	//
	// interpolate == false skips the old-position lerp and the rotation
	// wrap/trunc math entirely (RenderOptions::interpolation).
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
				bodies.renderOldOffsetX, bodies.renderOldOffsetY, bodies.renderOldRotation, bodies.renderRotationWrapCount,
				bodies.offsetX, bodies.offsetY, bodies.rotation,
				alpha, interpolate
			};
		}

		[[nodiscard]] __forceinline BodyTransformSample sample(ObjectIndex bodyIndex) const noexcept
		{
			if (interpolate)
			{
				const Real posX = x[bodyIndex];
				const Real posY = y[bodyIndex];
				const Real oldPosX = oldX[bodyIndex];
				const Real oldPosY = oldY[bodyIndex];

				const Real interpolatedX = oldPosX + (posX - oldPosX) * alpha;
				const Real interpolatedY = oldPosY + (posY - oldPosY) * alpha;

				const Real interpolatedRotation = interpolateAndWrapRotation(
					rot[bodyIndex],
					oldRot[bodyIndex],
					wrapCount[bodyIndex],
					alpha
				);

				const auto [c, s] = FastCosSin::order4Scalar(interpolatedRotation);

				return { interpolatedX, interpolatedY, c, s, interpolatedRotation };
			}
			else
			{
				const Real rotation = rot[bodyIndex];
				const auto [c, s] = FastCosSin::order4Scalar(rotation);

				return { x[bodyIndex], y[bodyIndex], c, s, rotation };
			}
		}
	};
}