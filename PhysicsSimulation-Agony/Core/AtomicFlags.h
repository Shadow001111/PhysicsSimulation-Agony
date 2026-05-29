#pragma once
#include <atomic>
#include <type_traits>

template <typename T>
    requires (std::is_unsigned_v<T>)
class AtomicFlags
{
    std::atomic<T> bits{ 0 };
public:
    void reset() noexcept
    {
        bits.store(0, std::memory_order_release);
    }

    void set(unsigned index, bool value) noexcept 
    {
        T mask = static_cast<T>(T(1) << index);
        if (value)
        {
            bits.fetch_or(mask, std::memory_order_acq_rel);
        }
        else
        {
            bits.fetch_and(~mask, std::memory_order_acq_rel);
        }
    }

    [[nodiscard]] bool read(unsigned index) const noexcept
    {
        T mask = static_cast<T>(T(1) << index);
        return (bits.load(std::memory_order_acquire) & mask) != 0;
    }

    [[nodiscard]] bool readAndSet(unsigned index, bool value) noexcept
    {
        T mask = static_cast<T>(T(1) << index);
        if (value)
        {
            return (bits.fetch_or(mask, std::memory_order_acq_rel) & mask) != 0;
        }
        else
        {
            return (bits.fetch_and(~mask, std::memory_order_acq_rel) & mask) != 0;
        }
	}
};
