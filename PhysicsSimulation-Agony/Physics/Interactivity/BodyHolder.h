#pragma once
#include "Physics/GlmTypes.h"

#include <optional>

namespace PS_AGONY::Interactivity
{
	class BodyHolder
	{
		Vec2 holderPosition;
		Vec2 holderVelocity;
	public:
		std::optional<ObjectIndex> heldBody;
		Vec2 bodyOffset;

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

		const Vec2& getPosition() const noexcept { return holderPosition; }
		const Vec2& getVelocity() const noexcept { return holderVelocity; }
	};
}
