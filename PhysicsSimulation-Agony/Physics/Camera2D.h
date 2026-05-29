#pragma once
#include "GlmTypes.h"

namespace PS_AGONY
{
	class Camera2D
	{
	public:
		Vec2 position;
		Vec2 viewRange;

		// Setters
		void setViewRangeW(Real w, Real ratio);
		void setViewRangeH(Real h, Real ratio);

		// Getters
		[[nodiscard]]
		Mat4 getViewMatrix() const noexcept;

		[[nodiscard]]
		Mat4 getProjectionMatrix() const noexcept;
	};
}

