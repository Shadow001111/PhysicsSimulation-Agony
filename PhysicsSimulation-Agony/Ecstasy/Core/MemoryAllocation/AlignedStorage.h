#pragma once
#include <new>
#include <utility>

template <class T, size_t Alignment>
class AlignedStorage
{
    static_assert(Alignment != 0, "Alignment must be non-zero.");
    static_assert(((Alignment & (Alignment - 1))) == 0, "Alignment must be a power of two.");

    T* mData = nullptr;
    size_t mSize = 0;
public:
    AlignedStorage() noexcept = default;

    explicit AlignedStorage(size_t count)
    {
        allocate(count);
    }

    ~AlignedStorage()
    {
        release();
    }

    AlignedStorage(const AlignedStorage&) = delete;
    AlignedStorage& operator=(const AlignedStorage&) = delete;

    AlignedStorage(AlignedStorage&& other) noexcept :
        mData(std::exchange(other.mData, nullptr)),
        mSize(std::exchange(other.mSize, 0)) 
    {
    }

    AlignedStorage& operator=(AlignedStorage&& other) noexcept
    {
        if (this != &other)
        {
            release();
            mData = std::exchange(other.mData, nullptr);
            mSize = std::exchange(other.mSize, 0);
        }
        return *this;
    }

    void allocate(size_t count)
    {
        release();
        if (count == 0)
        {
            return;
        }

        mSize = count;
        mData = static_cast<T*>(
            ::operator new(sizeof(T) * mSize, static_cast<std::align_val_t>(Alignment))
            );
    }

    void release() noexcept
    {
        if (mData)
        {
            ::operator delete(mData, static_cast<std::align_val_t>(Alignment));
            mData = nullptr;
            mSize = 0;
        }
    }

    [[nodiscard]] T* data() noexcept { return mData; }
    [[nodiscard]] const T* data() const noexcept { return mData; }

    [[nodiscard]] size_t size() const noexcept { return mSize; }
    [[nodiscard]] bool empty() const noexcept { return mData == nullptr; }

    T& operator[](size_t i) noexcept { return mData[i]; }
    const T& operator[](size_t i) const noexcept { return mData[i]; }
};