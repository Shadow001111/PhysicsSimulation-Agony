#pragma once
#include <glm/glm.hpp>
#include <array>

template<typename T>
struct Plane
{
	using value_type = T;

	glm::vec<3, T> normal;
	T d;

	Plane() : normal(T(0)), d(T(0)) {}

	Plane(const glm::vec<3, T>& center, const glm::vec<3, T>& normal)
		: normal(normal), d(-glm::dot(normal, center)) {
	}

	Plane(const glm::vec<3, T>& normal, T d)
		: normal(normal), d(d) {
	}

	T distanceToPoint(const glm::vec<3, T>& point) const noexcept
	{
		return glm::dot(normal, point) + d;
	}
};

template<typename T, size_t N>
struct PlaneSoA
{
	using value_type = T;
	constexpr static size_t count = N;

	std::array<T, N> normalX{ 0.0 };
	std::array<T, N> normalY{ 0.0 };
	std::array<T, N> normalZ{ 0.0 };
	std::array<T, N> d{ 0.0 };

	PlaneSoA() = default;

	void set(size_t i, const Plane<T>& plane)
	{
		normalX[i] = plane.normal.x;
		normalY[i] = plane.normal.y;
		normalZ[i] = plane.normal.z;
		d[i] = plane.d;
	}
};