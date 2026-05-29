#pragma once
#include <atomic>
#include <type_traits>
#include <cstdint>

template <typename T>
class ResourcePtr;

template <typename Derived>
class ResourceReferenceCounter
{
    friend class ResourcePtr<Derived>;

    mutable std::atomic<uint32_t> refCount{ 0 };

    void addRef() const
    {
        refCount.fetch_add(1, std::memory_order_relaxed);
    }

    bool subRef() const
    {
        auto old = refCount.fetch_sub(1, std::memory_order_acq_rel);
        return old == 1; // count went from 1 to 0
    }
protected:
    ResourceReferenceCounter() = default;
    ~ResourceReferenceCounter() = default;

    // No copying/moving of the counter itself
    ResourceReferenceCounter(const ResourceReferenceCounter&) = delete;
    ResourceReferenceCounter& operator=(const ResourceReferenceCounter&) = delete;
    ResourceReferenceCounter(ResourceReferenceCounter&&) = delete;
    ResourceReferenceCounter& operator=(ResourceReferenceCounter&&) = delete;
};

template <typename T>
class ResourcePtr
{
    static_assert(std::is_base_of_v<ResourceReferenceCounter<T>, T>,
        "T must derive from ResourceReferenceCounter<T>");

    T* ptr = nullptr;

    void acquire(T* p)
    {
        if (p)
        {
            auto* base = static_cast<ResourceReferenceCounter<T>*>(p);
            base->addRef();
        }
    }

    void release()
    {
        if (ptr)
        {
            auto* base = static_cast<ResourceReferenceCounter<T>*>(ptr);
            bool shouldDelete = base->subRef();
            if (shouldDelete)
            {
                delete ptr;
            }
        }
        ptr = nullptr;
    }
public:
    ResourcePtr() noexcept : ptr(nullptr) {}
    ResourcePtr(T& obj) noexcept : ptr(&obj) { acquire(ptr); }
    ResourcePtr(T* obj) noexcept : ptr(obj) { acquire(ptr); }
    ~ResourcePtr() { release(); }

    ResourcePtr(const ResourcePtr& other) noexcept : ptr(other.ptr)
    {
        acquire(ptr);
    }

    ResourcePtr& operator=(const ResourcePtr& other) noexcept
    {
        if (this != &other)
        {
            release();
            ptr = other.ptr;
            acquire(ptr);
        }
        return *this;
    }

    ResourcePtr(ResourcePtr&& other) noexcept : ptr(other.ptr)
    {
        other.ptr = nullptr;
    }

    ResourcePtr& operator=(ResourcePtr&& other) noexcept
    {
        if (this != &other)
        {
            release();
            ptr = other.ptr;
            other.ptr = nullptr;
        }
        return *this;
    }

    explicit operator bool() const noexcept { return ptr != nullptr; }
    T* get() const noexcept { return ptr; }
    T* operator->() const noexcept { return ptr; }
    T& operator*() const noexcept { return *ptr; }
};