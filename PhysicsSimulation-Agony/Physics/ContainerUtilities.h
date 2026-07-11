#pragma once

namespace PS_AGONY
{
	template<typename Container>
	inline size_t getVectorMemoryUsage(const Container& container)
	{
		return container.capacity() * sizeof(container[0]);
	}
}