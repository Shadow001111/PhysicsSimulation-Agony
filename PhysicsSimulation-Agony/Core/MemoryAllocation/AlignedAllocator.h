#pragma once
#include <cstddef>
#include <new>
#include <limits>
#include <type_traits>
#include <utility>

template <class T, std::size_t Alignment = alignof(T)>
class AlignedAllocator
{
    static_assert(std::is_object_v<T>, "T must be an object type");
    static_assert(Alignment >= alignof(T), "Alignment must be at least alignof(T)");
    static_assert((Alignment& (Alignment - 1)) == 0, "Alignment must be a power of two");
public:
    using value_type = T;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;

    using propagate_on_container_move_assignment = std::true_type;
    using is_always_equal = std::true_type;

    template <class U>
    struct rebind
    {
        using other = AlignedAllocator<U, Alignment>;
    };

    AlignedAllocator() noexcept = default;

    template <class U>
    AlignedAllocator(const AlignedAllocator<U, Alignment>&) noexcept {}

    [[nodiscard]] T* allocate(std::size_t n) {
        if (n > max_size()) {
            throw std::bad_array_new_length{};
        }

        const std::size_t bytes = n * sizeof(T);

        #if defined(__cpp_aligned_new)
            return static_cast<T*>(::operator new(bytes, std::align_val_t{ Alignment }));
        #else
            // Fallback for older compilers/platforms:
            void* p = nullptr;

            #if defined(_MSC_VER)
                p = _aligned_malloc(bytes, Alignment);
                if (!p) throw std::bad_alloc{};
            #elif defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 200112L
                if (posix_memalign(&p, Alignment, bytes) != 0) {
                    throw std::bad_alloc{};
                }
            #else
                // Last-resort fallback: not guaranteed to provide requested alignment.
                p = ::operator new(bytes);
            #endif

            return static_cast<T*>(p);
        #endif
    }

    void deallocate(T* p, std::size_t) noexcept {
        #if defined(__cpp_aligned_new)
            ::operator delete(p, std::align_val_t{ Alignment });
        #else
            #if defined(_MSC_VER)
                _aligned_free(p);
            #else
                ::operator delete(p);
            #endif
        #endif
    }

    [[nodiscard]] constexpr std::size_t max_size() const noexcept{
        return (std::numeric_limits<std::size_t>::max)() / sizeof(T);
    }

    template <class U, std::size_t A>
    friend bool operator==(const AlignedAllocator&, const AlignedAllocator<U, A>&) noexcept {
        return A == Alignment;
    }

    template <class U, std::size_t A>
    friend bool operator!=(const AlignedAllocator& a, const AlignedAllocator<U, A>& b) noexcept {
        return !(a == b);
    }
};