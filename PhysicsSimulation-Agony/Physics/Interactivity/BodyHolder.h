#pragma once
#include "Physics/GlmTypes.h"

#include <optional>

namespace PS_AGONY::Interactivity
{
	struct BodyHolder
	{
		std::optional<BodyIndex> heldBody;
		Vec2 holderPosition;
		Vec2 holderVelocity;
		Vec2 bodyOffset;
	};
}
