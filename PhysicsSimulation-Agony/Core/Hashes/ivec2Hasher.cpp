#include "ivec2Hasher.h"

size_t ivec2Hasher::operator()(const glm::ivec2& other) const noexcept
{
	constexpr size_t addConst = 0x9e3779b97f4a7c15;
	size_t h = (size_t)other.x + addConst;
	h ^= (size_t)other.y + addConst + (h << 6) + (h >> 2);
	return h;
}