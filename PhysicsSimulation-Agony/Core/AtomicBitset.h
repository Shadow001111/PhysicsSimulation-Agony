#pragma once
#include <atomic>
#include <array>
#include <cstddef>
#include <type_traits>

template <size_t Bits, typename Word = unsigned int>
    requires (std::is_unsigned_v<Word>)
class AtomicBitset
{
    static constexpr size_t BitsPerWord = sizeof(Word) * 8;
    static_assert((BitsPerWord& (BitsPerWord - 1)) == 0, "BitsPerWord must be power of two");

    static constexpr size_t WordShift = [] {
        size_t s = 0;
        size_t v = BitsPerWord;
        while (v > 1)
        {
            v >>= 1;
            ++s;
        }
        return s;
        }();

    static constexpr size_t WordMask = BitsPerWord - 1;
    static constexpr size_t WordCount =
        (Bits + BitsPerWord - 1) >> WordShift;

    std::array<std::atomic<Word>, WordCount> data{};

    static inline constexpr size_t wordIndex(size_t bit) noexcept
    {
        return bit >> WordShift;
    }

    static inline constexpr size_t bitOffset(size_t bit) noexcept
    {
        return bit & WordMask;
    }

public:
    AtomicBitset() noexcept
    {
        reset();
    }

    AtomicBitset(const AtomicBitset& other)
    {
        for (size_t i = 0; i < WordCount; i++)
        {
            data[i].store(other.data[i].load(std::memory_order_acquire), std::memory_order_release);
		}
    }

    AtomicBitset& operator=(const AtomicBitset& other)
    {
        if (this != &other)
        {
            for (size_t i = 0; i < WordCount; i++)
            {
                data[i].store(other.data[i].load(std::memory_order_acquire), std::memory_order_release);
            }
        }
        return *this;
	}

    void reset() noexcept
    {
        for (auto& w : data)
        {
            w.store(0, std::memory_order_release);
        }
    }

    void set(size_t index, bool value) noexcept
    {
        const size_t w = wordIndex(index);
        const Word mask = Word(1) << bitOffset(index);

        if (value)
        {
            data[w].fetch_or(mask, std::memory_order_acq_rel);
        }
        else
        {
            data[w].fetch_and(~mask, std::memory_order_acq_rel);
        }
    }

    [[nodiscard]] bool read(size_t index) const noexcept
    {
        const size_t w = wordIndex(index);
        const Word mask = Word(1) << bitOffset(index);

        return (data[w].load(std::memory_order_acquire) & mask) != 0;
    }

    [[nodiscard]] bool readAndSet(size_t index, bool value) noexcept
    {
        const size_t w = wordIndex(index);
        const Word mask = Word(1) << bitOffset(index);

        if (value)
        {
            return (data[w].fetch_or(mask, std::memory_order_acq_rel) & mask) != 0;
        }
        else
        {
            return (data[w].fetch_and(~mask, std::memory_order_acq_rel) & mask) != 0;
        }
    }

    [[nodiscard]] bool any() const noexcept
    {
        for (const auto& w : data)
        {
            if (w.load(std::memory_order_acquire) != 0)
            {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] bool none() const noexcept
    {
        return !any();
	}
};