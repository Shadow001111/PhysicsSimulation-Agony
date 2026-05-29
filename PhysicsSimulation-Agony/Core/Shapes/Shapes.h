#pragma once
#include "Box.h"
#include "Plane.h"

//template<typename T>
//bool isBoxOnOrForwardPlane(const Box<T>& box, const Plane<T>& plane)
//{
//	T distance = plane.distanceToPoint(box.center);
//	T effectiveRadius = glm::dot(box.halfExtents, glm::abs(plane.normal));
//	return distance >= -effectiveRadius;
//}

template<typename T>
bool isBoxOnOrForwardPlane(const Box<T>& box, const Plane<T>& plane) noexcept
{
	T distance = plane.distanceToPoint(box.center);
	T effectiveRadius = glm::dot(box.halfExtents, glm::abs(plane.normal));
	return distance >= -effectiveRadius;
}