#pragma once
#include <vector>
#include <cstddef>
#include <type_traits>
#include <utility>
#include <stdexcept>
#include <span>

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
        void resize(size_t count)
        {
            static_assert(alignof(T) <= MAX_ALIGNMENT, "T's alignment exceeds MAX_ALIGNMENT");
            static_assert(std::is_trivial_v<T>, "T must be a trivial (POD) type");

            buffer.resize(count * sizeof(T));
        }

        // Resets size to zero.
        void clear()
        {
            buffer.resize(0);
        }

        // Appends an element by forwarding arguments to its constructor in-place.
        template <typename T, typename... Args>
        void emplaceBack(Args&&... args)
        {
            static_assert(alignof(T) <= MAX_ALIGNMENT, "T's alignment exceeds MAX_ALIGNMENT");
            static_assert(std::is_trivial_v<T>, "T must be a trivial (POD) type");

            const size_t currentBytes = buffer.size();
            buffer.resize(currentBytes + sizeof(T));

            // Construct the object directly in the newly allocated byte space
            ::new (static_cast<void*>(buffer.data() + currentBytes)) T(std::forward<Args>(args)...);
        }

        // Appends an element via copy.
        template <typename T>
        void pushBack(const T& value)
        {
            emplaceBack<T>(value);
        }

        // Appends an element via move.
        template <typename T>
        void pushBack(T&& value)
        {
            emplaceBack<T>(std::move(value));
        }

        // Removes the last element of type T from the buffer
        template <typename T>
        void popBack()
        {
            static_assert(alignof(T) <= MAX_ALIGNMENT, "T's alignment exceeds MAX_ALIGNMENT");
            static_assert(std::is_trivial_v<T>, "T must be a trivial (POD) type");

            if (buffer.size() < sizeof(T))
            {
                throw std::out_of_range("ScratchBuffer::popBack - buffer underflow");
            }

            buffer.resize(buffer.size() - sizeof(T));
        }

        // Returns a reference to the element at the specified index.
        template <typename T>
        [[nodiscard]] T& at(size_t index)
        {
            static_assert(alignof(T) <= MAX_ALIGNMENT, "T's alignment exceeds MAX_ALIGNMENT");
            static_assert(std::is_trivial_v<T>, "T must be a trivial (POD) type");

            const size_t byteOffset = index * sizeof(T);
            if (byteOffset + sizeof(T) > buffer.size())
            {
                throw std::out_of_range("ScratchBuffer::at - index out of range");
            }

            return *reinterpret_cast<T*>(buffer.data() + byteOffset);
        }

        // Returns a const reference to the element at the specified index.
        template <typename T>
        [[nodiscard]] const T& at(size_t index) const
        {
            static_assert(alignof(T) <= MAX_ALIGNMENT, "T's alignment exceeds MAX_ALIGNMENT");
            static_assert(std::is_trivial_v<T>, "T must be a trivial (POD) type");

            const size_t byteOffset = index * sizeof(T);
            if (byteOffset + sizeof(T) > buffer.size())
            {
                throw std::out_of_range("ScratchBuffer::at - index out of range");
            }

            return *reinterpret_cast<const T*>(buffer.data() + byteOffset);
        }

        // Returns a reference to the very last element of type T in the buffer.
        template <typename T>
        [[nodiscard]] T& atBack()
        {
            static_assert(alignof(T) <= MAX_ALIGNMENT, "T's alignment exceeds MAX_ALIGNMENT");
            static_assert(std::is_trivial_v<T>, "T must be a trivial (POD) type");

            if (buffer.size() < sizeof(T))
            {
                throw std::out_of_range("ScratchBuffer::atBack - buffer is empty or too small");
            }

            const size_t byteOffset = buffer.size() - sizeof(T);
            return *reinterpret_cast<T*>(buffer.data() + byteOffset);
        }

        // Returns a const reference to the very last element of type T in the buffer.
        template <typename T>
        [[nodiscard]] const T& atBack() const
        {
            static_assert(alignof(T) <= MAX_ALIGNMENT, "T's alignment exceeds MAX_ALIGNMENT");
            static_assert(std::is_trivial_v<T>, "T must be a trivial (POD) type");

            if (buffer.size() < sizeof(T))
            {
                throw std::out_of_range("ScratchBuffer::atBack - buffer is empty or too small");
            }

            const size_t byteOffset = buffer.size() - sizeof(T);
            return *reinterpret_cast<const T*>(buffer.data() + byteOffset);
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

        // Returns capacity in bytes.
        [[nodiscard]] size_t getCapacity() const noexcept
        {
            return buffer.capacity();
        }

        // Returns a std::span of type T over the buffer contents.
        template <typename T>
        [[nodiscard]] std::span<T> getSpan() noexcept
        {
            static_assert(alignof(T) <= MAX_ALIGNMENT, "T's alignment exceeds MAX_ALIGNMENT");
            static_assert(std::is_trivial_v<T>, "T must be a trivial (POD) type");

            return std::span<T>(getPtr<T>(), getSize<T>());
        }

        // Returns a std::span of type const T over the buffer contents.
        template <typename T>
        [[nodiscard]] std::span<const T> getSpan() const noexcept
        {
            static_assert(alignof(T) <= MAX_ALIGNMENT, "T's alignment exceeds MAX_ALIGNMENT");
            static_assert(std::is_trivial_v<T>, "T must be a trivial (POD) type");

            return std::span<const T>(getPtr<T>(), getSize<T>());
        }
    };
}