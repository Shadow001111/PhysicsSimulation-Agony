#pragma once
#include <type_traits>
#include <concepts>

#include <glm/glm.hpp>

namespace Ecstasy::Graphics
{
	template<std::floating_point TReal>
	struct FloatingPointSystem
	{
		using Real = TReal;

		using Vec2 = glm::vec<2, Real>;
		using Vec3 = glm::vec<3, Real>;
		using Vec4 = glm::vec<4, Real>;

		using Mat3 = glm::mat<3, 3, Real>;
		using Mat4 = glm::mat<4, 4, Real>;
	};

	template<typename T>
	concept IsFloatingPointSystem = requires {
		typename T::Real;
		typename T::Vec2;
		typename T::Vec3;
		typename T::Vec4;
		typename T::Mat3;
		typename T::Mat4;
		requires std::floating_point<typename T::Real>;
	};
}