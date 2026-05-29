#pragma once
#include <glm/vec3.hpp>

struct ivec3Hasher
{
public:
	size_t operator()(const glm::ivec3& other) const noexcept;
};

