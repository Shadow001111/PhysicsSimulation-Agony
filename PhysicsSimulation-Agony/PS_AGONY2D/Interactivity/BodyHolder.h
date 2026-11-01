#pragma once
#include "PS_AGONY2D/Types/Types.h"
#include "PS_AGONY2D/HandleTable.h"

#include <vector>

namespace PS_AGONY2D::Interactivity
{
	// A moving target that pulls any number of bodies. The holder has one position and one velocity,
	// each held body has its own anchor (localOffset) and its own offset from the holder (targetOffset).
	class BodyHolder
	{
		Vec2 holderPosition{ 0, 0 };
		Vec2 holderVelocity{ 0, 0 };
	public:
		struct HeldBody
		{
			Handle body;
			Vec2 localOffset;  // Anchor in body space, relative to the body world center.
			Vec2 targetOffset; // Anchor target = holder position + targetOffset.
		};

		std::vector<HeldBody> heldBodies;

		bool isHolding() const noexcept { return !heldBodies.empty(); }

		// Moves the holder and derives its velocity. deltaTime <= 0 gives zero velocity.
		void setPosition(const Vec2& newPosition, Real deltaTime)
		{
			if (deltaTime <= 0) [[unlikely]]
			{
				holderVelocity = Vec2();
			}
			else
			{
				holderVelocity = (newPosition - holderPosition) / deltaTime;
			}
			holderPosition = newPosition;
		}

		// Moves the holder with zero velocity. Call before the first grab, otherwise the stale position
		// produces a huge velocity on the first step.
		void teleport(const Vec2& newPosition) noexcept
		{
			holderPosition = newPosition;
			holderVelocity = Vec2();
		}

		const Vec2& getPosition() const noexcept { return holderPosition; }
		const Vec2& getVelocity() const noexcept { return holderVelocity; }
	};
}