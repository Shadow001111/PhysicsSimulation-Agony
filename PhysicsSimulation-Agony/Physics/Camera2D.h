#pragma once
#include "GlmTypes.h"

namespace PS_AGONY
{
	class Camera2D
	{
	public:
		Vec2 position{ 0.0, 0.0 };
		Real aspectRatio{ 1.0} ;
		Real viewRange{ 1.0 }; // Inverse zoom.

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

