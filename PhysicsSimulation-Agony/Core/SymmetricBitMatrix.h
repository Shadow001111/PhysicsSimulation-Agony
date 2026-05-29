#pragma once
#include "Bitset.h"

template <size_t N>
class SymmetricBitMatrix
{
    static constexpr size_t SIZE = (N * (N + 1)) >> 1;
    Bitset<SIZE> data;

    constexpr size_t index(size_t i, size_t j) const
    {
        if (i > j) std::swap(i, j);
        return i * N - (i * (i - 1)) / 2 + (j - i);
    }

public:
    SymmetricBitMatrix() noexcept
    {
        reset();
    }

    SymmetricBitMatrix(const SymmetricBitMatrix& other)
    {
        data = other.data;
    }

    SymmetricBitMatrix& operator=(const SymmetricBitMatrix& other)
    {
        if (this != &other)
        {
            data = other.data;
        }
        return *this;
    }

    void reset() noexcept
    {
        data.reset();
    }

    bool read(size_t i, size_t j) const noexcept { return data.read(index(i, j)); }
    void set(size_t i, size_t j, bool val) noexcept { data.set(index(i, j), val); }
    bool all() const noexcept { return data.all(); }
};