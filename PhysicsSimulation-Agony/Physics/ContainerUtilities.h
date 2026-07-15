#pragma once
#include "Core/MemoryAllocation/AlignedAllocator.h"

#include "EcstasyCore/Simd.h"

#include "Types.h"

#include <vector>

namespace PS_AGONY
{
	template<typename T, size_t aligment>
	using AlignedVector = std::vector<T, AlignedAllocator<T, aligment>>;

	template<typename T>
	using AlignedVector64 = std::vector<T, AlignedAllocator<T, 64>>;

	template<typename T>
	using SimdAlignedVector = AlignedVector<T, Ecstasy::Simd<T>::bytes>;

	template<typename T>
	using RealSimdAlignedVector = AlignedVector<T, Ecstasy::Simd<Real>::bytes>;

	template<typename T>
	using CacheLineAlignedVector = AlignedVector<T, std::hardware_destructive_interference_size>;


	template<typename Container>
	inline size_t getVectorMemoryUsage(const Container& container)
	{
		return container.capacity() * sizeof(container[0]);
	}
}