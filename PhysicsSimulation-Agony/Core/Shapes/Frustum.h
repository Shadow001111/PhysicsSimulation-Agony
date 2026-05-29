#pragma once
#include "Shapes.h"

#include "Core/Simd.h"

template<typename T>
struct Frustum
{
	using value_type = T;
	using Vec3Type = glm::vec<3, value_type>;
	using PlaneType = Plane<value_type>;

	PlaneType nearPlane, farPlane, rightPlane, leftPlane, topPlane, bottomPlane; // Order for better cache locality in check methods, hopefully

	bool checkBox(const Box<value_type>& box) const noexcept
	{
		return
			isBoxOnOrForwardPlane(box, nearPlane) &&
			isBoxOnOrForwardPlane(box, farPlane) &&
			isBoxOnOrForwardPlane(box, rightPlane) &&
			isBoxOnOrForwardPlane(box, leftPlane) &&
			isBoxOnOrForwardPlane(box, topPlane) &&
			isBoxOnOrForwardPlane(box, bottomPlane);
	}

	void update(
		const Vec3Type& position,
		const Vec3Type& forwardDirection, const Vec3Type& rightDirection, const Vec3Type& upDirection,
		value_type fov,
		value_type aspectRatio,
		value_type nearPlaneDistance, value_type farPlaneDistance
	) noexcept
	{
		value_type halfVSide = farPlaneDistance * glm::tan(fov * value_type(0.5));
		value_type halfHSide = halfVSide * aspectRatio;

		Vec3Type nearMultFwd = nearPlaneDistance * forwardDirection;
		Vec3Type farMultFwd = farPlaneDistance * forwardDirection;

		nearPlane = PlaneType(position + nearMultFwd, forwardDirection);
		farPlane = PlaneType(position + farMultFwd, -forwardDirection);

		rightPlane = PlaneType(position, glm::cross(farMultFwd - rightDirection * halfHSide, upDirection));
		leftPlane = PlaneType(position, glm::cross(upDirection, farMultFwd + rightDirection * halfHSide));

		topPlane = PlaneType(position, glm::cross(rightDirection, farMultFwd - upDirection * halfVSide));
		bottomPlane = PlaneType(position, glm::cross(farMultFwd + upDirection * halfVSide, rightDirection));
	}
};

template<typename T>
struct FrustumSimd
{
	using value_type = T;
	using Vec3Type = glm::vec<3, value_type>;
	using PlaneType = Plane<value_type>;

	using SimdFloat = Simd<value_type>;

	PlaneSoA<value_type, 6> planes; // near, far, right, left, top, bottom

	bool checkBox(const Box<value_type>& box) const noexcept
	{
		constexpr size_t L = SimdFloat::lanes;

		// Broadcast box data into full SIMD registers
		SimdFloat cx(box.center.x);
		SimdFloat cy(box.center.y);
		SimdFloat cz(box.center.z);
		SimdFloat hx(box.halfExtents.x);
		SimdFloat hy(box.halfExtents.y);
		SimdFloat hz(box.halfExtents.z);

		// Mask for absolute value
		const SimdFloat absMask = SimdFloat::get_abs_mask();

		// Helper: check a group of `count` planes starting at array index `start`
		auto checkGroup = [&](size_t start, size_t count) -> bool
			{
				// Load plane data (contiguous loads – very fast)
				SimdFloat nx = SimdFloat::loadu(&planes.normalX[start]);
				SimdFloat ny = SimdFloat::loadu(&planes.normalY[start]);
				SimdFloat nz = SimdFloat::loadu(&planes.normalZ[start]);
				SimdFloat nd = SimdFloat::loadu(&planes.d[start]);

				// Distance = dot(center, normal) + d
				SimdFloat dist = SimdFloat::mul_add(cz, nz, nd); // dist = cz * nz + nd
				dist = SimdFloat::mul_add(cy, ny, dist); // dist = cy * ny + dist
				dist = SimdFloat::mul_add(cx, nx, dist); // dist = cx * nx + dist

				// Effective radius = dot(halfExtents, abs(normal))
				SimdFloat absNx = SimdFloat::bitwise_and(nx, absMask);
				SimdFloat absNy = SimdFloat::bitwise_and(ny, absMask);
				SimdFloat absNz = SimdFloat::bitwise_and(nz, absMask);

				SimdFloat radius = hx * absNx;
				radius = SimdFloat::mul_add(hy, absNy, radius);
				radius = SimdFloat::mul_add(hz, absNz, radius);

				// dist >= -radius  <=>  dist + radius >= 0
				SimdFloat sum = dist + radius;
				SimdFloat mask = sum >= SimdFloat::fill_lanes_with_zero();

				int bits = mask.movemask();
				int validMask = (1 << count) - 1;
				return (bits & validMask) == validMask;
			};

		if constexpr (L == 8)
		{
			// All 6 planes + 2 dummy safe planes
			return checkGroup(0, 6);
		}
		else if (L == 4)
		{
			// Two groups of 4 (with dummy planes padding the second group)
			bool ok = checkGroup(0, 4); // near, far, right, left
			if (!ok) return false;

			ok = checkGroup(4, 2); // top, bottom, dummy, dummy
			return ok;
		}
		else if (L == 2)
		{
			// Three groups of 2 (with dummy planes padding the last group)
			bool ok = checkGroup(0, 2); // near, far
			if (!ok) return false;

			ok = checkGroup(2, 2); // right, left
			if (!ok) return false;

			ok = checkGroup(4, 2); // top, bottom, dummy, dummy
			return ok;
		}
		else
		{
			static_assert(L == 8 || L == 4 || L == 2, "Unsupported SIMD width");
			return false; // Should never reach here
		}
	}

	void update(
		const Vec3Type& position,
		const Vec3Type& forwardDirection, const Vec3Type& rightDirection, const Vec3Type& upDirection,
		value_type fov,
		value_type aspectRatio,
		value_type nearPlaneDistance, value_type farPlaneDistance
	) noexcept
	{
		value_type halfVSide = farPlaneDistance * glm::tan(fov * value_type(0.5));
		value_type halfHSide = halfVSide * aspectRatio;

		Vec3Type nearMultFwd = nearPlaneDistance * forwardDirection;
		Vec3Type farMultFwd = farPlaneDistance * forwardDirection;

		PlaneType nearPlane(position + nearMultFwd, forwardDirection);
		PlaneType farPlane(position + farMultFwd, -forwardDirection);

		PlaneType rightPlane(position, glm::cross(farMultFwd - rightDirection * halfHSide, upDirection));
		PlaneType leftPlane(position, glm::cross(upDirection, farMultFwd + rightDirection * halfHSide));

		PlaneType topPlane(position, glm::cross(rightDirection, farMultFwd - upDirection * halfVSide));
		PlaneType bottomPlane(position, glm::cross(farMultFwd + upDirection * halfVSide, rightDirection));
		
		planes.set(0, nearPlane);
		planes.set(1, farPlane);
		planes.set(2, rightPlane);
		planes.set(3, leftPlane);
		planes.set(4, topPlane);
		planes.set(5, bottomPlane);
	}
};