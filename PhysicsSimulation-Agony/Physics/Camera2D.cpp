#include "Camera2D.h"

#include <glm/gtc/matrix_transform.hpp>

namespace PS_AGONY
{
    void Camera2D::setViewRangeW(Real w, Real ratio)
    {
        viewRange.x = w;
        viewRange.y = w / ratio;
    }

    void Camera2D::setViewRangeH(Real h, Real ratio)
    {
        viewRange.x = h * ratio;
        viewRange.y = h;
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
        const Real halfW = viewRange.x * 0.5;
        const Real halfH = viewRange.y * 0.5;

        return glm::ortho(
            -halfW,
            halfW,
            -halfH,
            halfH
        );
    }
}