#pragma once
#include <glm/vec3.hpp>

template<typename T>
struct Box
{
	using value_type = T;

	glm::vec<3, T> center, halfExtents;

	Box() : center(T(0)), halfExtents(T(0)) {}

	Box(const glm::vec<3, T>& center, const glm::vec<3, T>& halfExtents)
		: center(center), halfExtents(halfExtents) {
	}
};
