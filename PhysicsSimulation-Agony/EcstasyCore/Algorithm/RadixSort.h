#pragma once
#include "../Portablity.h"

#include <array>
#include <cstring>
#include <type_traits>

// TODO: It works, but namings of arrays are swapped!
namespace Ecstasy::Algorithm
{
    template<
        uint32_t RADIX_BITS,
        std::integral T,
        std::unsigned_integral SizeType = size_t
    >
    void radixSort(
        T* ECSTASY_RESTRICT arrayToBeSorted,
        T* ECSTASY_RESTRICT tempArray,
        SizeType count)
    {
        static_assert(RADIX_BITS == 8 || RADIX_BITS == 16 || RADIX_BITS == 32 || RADIX_BITS == 64,
            "RADIX_BITS must be 8, 16, 32, or 64");

        constexpr uint32_t BITS_OF_T = sizeof(T) * 8;
        static_assert(BITS_OF_T > RADIX_BITS,
            "Number of bits of T must be greater than RADIX_BITS");

        constexpr uint32_t RADIX_SIZE = 1u << RADIX_BITS;
        constexpr uint32_t RADIX_MASK = RADIX_SIZE - 1u;

        // Count array for digit frequencies (zero-initialized).
        std::array<SizeType, RADIX_SIZE> counts;

        // Use unsigned type for shift/key extraction (avoids sign extension).
        using U = std::make_unsigned_t<T>;

        // Radix pass: stable sort by the current digit.
        auto radixPass = [&](uint32_t shift, const T* src, T* dst)
            {
                // Reset counts.
                counts.fill(0);

                // Count occurrences of each digit.
                for (SizeType i = 0; i < count; i++)
                {
                    const U val = static_cast<U>(src[i]);
                    const uint32_t key = (val >> shift) & RADIX_MASK;
                    ++counts[key];
                }

                // Exclusive prefix sum -> starting indices.
                SizeType sum = 0;
                for (uint32_t i = 0; i < RADIX_SIZE; i++)
                {
                    const SizeType c = counts[i];
                    counts[i] = sum;
                    sum += c;
                }

                // Scatter elements into destination (stable).
                for (SizeType i = 0; i < count; i++)
                {
                    const U val = static_cast<U>(src[i]);
                    const uint32_t key = (val >> shift) & RADIX_MASK;
                    dst[counts[key]++] = src[i];
                }
            };

        // Perform passes: shift by RADIX_BITS each time until all bits processed.
        T* src = arrayToBeSorted;
        T* dst = tempArray;
        for (uint32_t shift = 0; shift < BITS_OF_T; shift += RADIX_BITS)
        {
            radixPass(shift, src, dst);
            std::swap(src, dst);
        }

        // If final sorted array resides in temp, copy it back.
        if (src != arrayToBeSorted)
        {
            std::memcpy(arrayToBeSorted, src, count * sizeof(T));
        }
    }

    template<
        uint32_t RADIX_BITS,
        std::integral KeyType,
        std::unsigned_integral IndexType,
        std::unsigned_integral SizeType = size_t
    >
    void radixSortIndices(
        IndexType* ECSTASY_RESTRICT indexArrayToBeSorted,
        IndexType* ECSTASY_RESTRICT tempIndexArray,
        const KeyType* ECSTASY_RESTRICT keyArray,
        SizeType count)
    {
        static_assert(RADIX_BITS == 8 || RADIX_BITS == 16 || RADIX_BITS == 32 || RADIX_BITS == 64,
            "RADIX_BITS must be 8, 16, 32, or 64");

        constexpr uint32_t BITS_OF_KEY = sizeof(KeyType) * 8;
        static_assert(BITS_OF_KEY > RADIX_BITS,
            "Number of bits of KeyType must be greater than RADIX_BITS");

        constexpr uint32_t RADIX_SIZE = 1u << RADIX_BITS;
        constexpr uint32_t RADIX_MASK = RADIX_SIZE - 1u;

        std::array<SizeType, RADIX_SIZE> counts;

        using UKey = std::make_unsigned_t<KeyType>;

        auto radixPass = [&](uint32_t shift, const IndexType* srcIdx, IndexType* dstIdx)
            {
                counts.fill(0);

                // Count occurrences of each digit using the key values.
                for (SizeType i = 0; i < count; i++)
                {
                    const IndexType idx = srcIdx[i];
                    const UKey val = static_cast<UKey>(keyArray[idx]);   // get key for this index
                    const uint32_t key = (val >> shift) & RADIX_MASK;
                    ++counts[key];
                }

                // Exclusive prefix sum.
                SizeType sum = 0;
                for (uint32_t i = 0; i < RADIX_SIZE; i++)
                {
                    const SizeType c = counts[i];
                    counts[i] = sum;
                    sum += c;
                }

                // Scatter indices into destination (stable).
                for (SizeType i = 0; i < count; i++)
                {
                    const IndexType idx = srcIdx[i];
                    const UKey val = static_cast<UKey>(keyArray[idx]);
                    const uint32_t key = (val >> shift) & RADIX_MASK;
                    dstIdx[counts[key]++] = idx;
                }
            };

        IndexType* src = indexArrayToBeSorted;
        IndexType* dst = tempIndexArray;
        for (uint32_t shift = 0; shift < BITS_OF_KEY; shift += RADIX_BITS)
        {
            radixPass(shift, src, dst);
            std::swap(src, dst);
        }

        // If final sorted indices reside in the temporary buffer, copy them back.
        if (src != indexArrayToBeSorted)
        {
            std::memcpy(indexArrayToBeSorted, src, count * sizeof(IndexType));
        }
    }
}
