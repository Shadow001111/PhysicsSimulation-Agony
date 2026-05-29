#pragma once
#include <type_traits>

template <typename T>
    requires (std::is_unsigned_v<T>)
struct Flags
{
private:
    T bits{ 0 };
public:
    void reset() noexcept
    {
        bits = 0;
    }

    void set(unsigned index, bool value) noexcept
    {
        T mask = static_cast<T>(T(1) << index);
        if (value)
        {
            bits |= mask;
        }
        else
        {
            bits &= ~mask;
        }
    }

    [[nodiscard]] bool read(unsigned index) const noexcept
    {
        T mask = static_cast<T>(T(1) << index);
        return (bits & mask) != 0;
    }

    [[nodiscard]] bool readAndSet(unsigned index, bool value) noexcept
    {
        T mask = static_cast<T>(T(1) << index);
        bool cond = (bits & mask) != 0;
        if (value)
        {
            bits |= mask;
        }
        else
        {
            bits &= ~mask;
        }
        return cond;
    }
};
