#pragma once
#include "Ecstasy/Core/MemoryAllocation/AlignedAllocator.h"
#include "Ecstasy/Core/Simd.h"

#include "Types.h"

#include <vector>

namespace PS_AGONY
{
	using namespace Ecstasy::Core::MemoryAllocation;

	template<typename T, size_t aligment>
	using AlignedVector = std::vector<T, AlignedAllocator<T, aligment>>;

	template<typename T>
	using AlignedVector64 = std::vector<T, AlignedAllocator<T, 64>>;

	template<typename T>
	using SimdAlignedVector = AlignedVector<T, Ecstasy::Core::Simd<T>::bytes>;

	template<typename T>
	using RealSimdAlignedVector = AlignedVector<T, Ecstasy::Core::Simd<Real>::bytes>;

	template<typename T>
	using CacheLineAlignedVector = AlignedVector<T, std::hardware_destructive_interference_size>;


	template<typename Container>
	inline size_t getVectorMemoryUsage(const Container& container)
	{
		return container.capacity() * sizeof(container[0]);
	}
}