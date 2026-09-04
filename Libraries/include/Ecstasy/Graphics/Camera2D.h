#pragma once
#include "Ecstasy/Core/FloatingPoint.h"

#include <glm/gtc/matrix_transform.hpp>

namespace Ecstasy::Graphics
{
	template<Core::IsFloatingPointSystem FPSystem = Core::FloatingPointSystem<float>>
	class Camera2D
	{
		using Real = typename FPSystem::Real;
		using Vec2 = typename FPSystem::Vec2;
		using Vec3 = typename FPSystem::Vec3;
		using Mat4 = typename FPSystem::Mat4;
	public:
		Vec2 position{ 0.0, 0.0 };
		Real aspectRatio{ 1.0} ;
		Real viewRange{ 1.0 }; // Inverse zoom.

		// Transformation.

		// NDC space.
		Vec2 screenToWorldSpace(const Vec2& screenPoint) const noexcept
		{
			const Real halfRange = viewRange * Real(0.5);
			const Real halfW = halfRange * aspectRatio;
			const Real halfH = halfRange;

			return Vec2(
				screenPoint.x * halfW + position.x,
				screenPoint.y * halfH + position.y
			);
		}

		// Setters
		void setViewRangeW(Real w, Real ratio)
		{
			viewRange = w / ratio;
			aspectRatio = ratio;
		}

		void setViewRangeH(Real h, Real ratio)
		{
			viewRange = h;
			aspectRatio = ratio;
		}

		// Getters
		[[nodiscard]]
		Mat4 getViewMatrix() const noexcept
		{
			return glm::translate(
				Mat4(Real(1.0)),
				Vec3(-position, Real(0.0))
			);
		}

		[[nodiscard]]
		Mat4 getProjectionMatrix() const noexcept
		{
			const Real halfRange = viewRange * Real(0.5);
			const Real halfW = halfRange * aspectRatio;
			const Real halfH = halfRange;

			return glm::ortho(
				-halfW,
				halfW,
				-halfH,
				halfH
			);
		}
	};
}

