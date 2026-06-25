#pragma once
#include <array>
#include <cstddef>
#include <utility>

template <class T, size_t N>
class SymmetricMatrix
{
    static constexpr size_t STORED_COUNT = N * (N + 1) / 2;
    std::array<T, STORED_COUNT> mData{};

    static constexpr size_t index(size_t r, size_t c) noexcept
    {
        if (r < c) std::swap(r, c);
        return ((r * (r + 1)) >> 1) + c;
    }
public:
    static constexpr size_t size = N;

    SymmetricMatrix() = default;
    ~SymmetricMatrix() = default;
    SymmetricMatrix(const SymmetricMatrix&) = default;
    SymmetricMatrix& operator=(const SymmetricMatrix&) = default;

    SymmetricMatrix(SymmetricMatrix&& other) noexcept
    {
        for (size_t i = 0; i < STORED_COUNT; i++)
        {
            mData[i] = std::move(other.mData[i]);
        }
    }

    SymmetricMatrix& operator=(SymmetricMatrix&& other) noexcept
    {
        if (this != &other)
        {
            for (size_t i = 0; i < STORED_COUNT; i++)
            {
                mData[i] = std::move(other.mData[i]);
            }
        }
        return *this;
    }

    constexpr T& operator()(size_t r, size_t c) noexcept
    {
        return mData[index(r, c)];
    }

    constexpr const T& operator()(size_t r, size_t c) const noexcept
    {
        return mData[index(r, c)];
    }

    auto& getDirectAccess() noexcept { return mData; }
    const auto& getDirectAccess() const noexcept { return mData; }
};