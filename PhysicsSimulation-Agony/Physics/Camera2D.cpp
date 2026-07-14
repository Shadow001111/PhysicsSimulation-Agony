#include "Camera2D.h"

#include <glm/gtc/matrix_transform.hpp>

namespace PS_AGONY
{
    Vec2 Camera2D::screenToWorldSpace(const Vec2& screenPoint) const noexcept
    {
        const Real halfRange = viewRange * Real(0.5);
        const Real halfW = halfRange * aspectRatio;
        const Real halfH = halfRange;

        return Vec2(
            screenPoint.x * halfW + position.x,
            screenPoint.y * halfH + position.y
        );
    }

    void Camera2D::setViewRangeW(Real w, Real ratio)
    {
		viewRange = w / ratio;
		aspectRatio = ratio;
    }

    void Camera2D::setViewRangeH(Real h, Real ratio)
    {
		viewRange = h;
		aspectRatio = ratio;
    }

    Mat4 Camera2D::getViewMatrix() const noexcept
	{
		return glm::translate(
			Mat4(1.0),
			Vec3(-position, 0.0)
		);
	}

	Mat4 Camera2D::getProjectionMatrix() const noexcept
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
}