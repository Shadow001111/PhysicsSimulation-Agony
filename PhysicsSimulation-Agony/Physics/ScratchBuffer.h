#pragma once
#include <vector>
#include <cstddef>
#include <type_traits>
#include <utility>

#include "Core/MemoryAllocation/AlignedAllocator.h"

namespace PS_AGONY
{
    template <size_t MAX_ALIGNMENT>
    class ScratchBuffer
    {
        static_assert(MAX_ALIGNMENT > 0, "MAX_ALIGNMENT must be greater than 0");
        static_assert((MAX_ALIGNMENT& (MAX_ALIGNMENT - 1)) == 0, "MAX_ALIGNMENT must be a power of two");
    private:
        std::vector<std::byte, AlignedAllocator<std::byte, MAX_ALIGNMENT>> buffer;

    public:
        ScratchBuffer() noexcept
        {}

        ScratchBuffer(const ScratchBuffer& other) :
            buffer(other.buffer)
        {}

        ~ScratchBuffer() = default;

        ScratchBuffer& operator=(const ScratchBuffer& other)
        {
            if (this != &other)
            {
                buffer = other.buffer;
            }
            return *this;
        }

        ScratchBuffer(ScratchBuffer&& other) noexcept :
            buffer(std::move(other.buffer))
        {}

        ScratchBuffer& operator=(ScratchBuffer&& other) noexcept
        {
            if (this != &other)
            {
                buffer = std::move(other.buffer);
            }
            return *this;
        }

        // Allocates total bytes required to fit 'count' elements of type T.
        template <typename T>
        void allocate(size_t count)
        {
            static_assert(alignof(T) <= MAX_ALIGNMENT, "T's alignment exceeds MAX_ALIGNMENT");
            static_assert(std::is_trivial_v<T>, "T must be a trivial (POD) type");

            buffer.resize(count * sizeof(T));
        }

        // Returns a pointer cast to type T.
        template <typename T>
        [[nodiscard]] T* getPtr() noexcept
        {
            static_assert(alignof(T) <= MAX_ALIGNMENT, "T's alignment exceeds MAX_ALIGNMENT");
            static_assert(std::is_trivial_v<T>, "T must be a trivial (POD) type");

            return reinterpret_cast<T*>(buffer.data());
        }

        // Returns a const pointer cast to type T.
        template <typename T>
        [[nodiscard]] const T* getPtr() const noexcept
        {
            static_assert(alignof(T) <= MAX_ALIGNMENT, "T's alignment exceeds MAX_ALIGNMENT");
            static_assert(std::is_trivial_v<T>, "T must be a trivial (POD) type");

            return reinterpret_cast<const T*>(buffer.data());
        }

        // Returns the number of elements of type T currently allocated.
        template <typename T>
        [[nodiscard]] size_t getSize() const noexcept
        {
            static_assert(alignof(T) <= MAX_ALIGNMENT, "T's alignment exceeds MAX_ALIGNMENT");
            static_assert(std::is_trivial_v<T>, "T must be a trivial (POD) type");

            return buffer.size() / sizeof(T);
        }
    };
}