#include "TracyProfiler.h"

#if TRACY_ENABLE

void* operator new(size_t size)
{
    void* ptr = malloc(size);
    if (!ptr) [[unlikely]] throw std::bad_alloc{};
    TracyAlloc(ptr, size);
    return ptr;
}

void operator delete(void* ptr) noexcept
{
    if (!ptr) [[unlikely]] return;
    TracyFree(ptr);
    free(ptr);
}

void* operator new[](size_t size)
{
    void* ptr = malloc(size);
    if (!ptr) [[unlikely]] throw std::bad_alloc{};
    TracyAlloc(ptr, size);
    return ptr;
}

void operator delete[](void* ptr) noexcept
{
    if (!ptr) [[unlikely]] return;
    TracyFree(ptr);
    free(ptr);
}

#endif