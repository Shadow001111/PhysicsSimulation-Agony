#pragma once
#include <bit>
#include <array>
#include <cstddef>
#include <type_traits>

template <size_t Bits, typename Word = unsigned int>
    requires (std::is_unsigned_v<Word>)
class Bitset
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

    static inline constexpr size_t wordIndex(size_t bit) noexcept
    {
        return bit >> WordShift;
    }

    static inline constexpr size_t bitOffset(size_t bit) noexcept
    {
        return bit & WordMask;
    }

    static inline int countTrailingZeros(Word w) noexcept
    {
        return std::countr_zero(w);
    }

    std::array<Word, WordCount> words{};
public:
    Bitset() noexcept
    {
        reset();
    }

    Bitset(const Bitset& other)
    {
        std::memcpy(words.data(), other.words.data(), WordCount * sizeof(Word));
    }

    Bitset& operator=(const Bitset& other)
    {
        if (this != &other)
        {
            std::memcpy(words.data(), other.words.data(), WordCount * sizeof(Word));
        }
        return *this;
    }

    void reset() noexcept
    {
        std::memset(words.data(), 0, WordCount * sizeof(Word));
    }

    void set(size_t index, bool value) noexcept
    {
        const size_t w = wordIndex(index);
        const Word mask = Word(1) << bitOffset(index);

        if (value)
        {
            words[w] |= mask;
        }
        else
        {
            words[w] &= ~mask;
        }
    }

    void fill(bool value) noexcept
    {
        Word fillValue = value ? ~Word(0) : Word(0);
        for (auto& w : words)
            w = fillValue;
	}

    [[nodiscard]] bool read(size_t index) const noexcept
    {
        const size_t w = wordIndex(index);
        const Word mask = Word(1) << bitOffset(index);

        return (words[w] & mask) != 0;
    }

    [[nodiscard]] bool operator[](size_t index) const noexcept { return read(index); }

    [[nodiscard]] bool readAndSet(size_t index, bool value) noexcept
    {
        const size_t w = wordIndex(index);
        const Word mask = Word(1) << bitOffset(index);

        bool cond = (words[w] & mask) != 0;
        if (value)
        {
            words[w] |= mask;
        }
        else
        {
            words[w] &= ~mask;
        }
        return cond;
    }

    [[nodiscard]] bool any() const noexcept
    {
        // Check all full words first
        for (size_t i = 0; i < Bits / BitsPerWord; i++)
			if (words[i] != 0) return true;

		// Check remaining bits in the last partial word (if any)
		constexpr size_t remainder = Bits % BitsPerWord;
        if constexpr (remainder != 0)
        {
            constexpr Word mask = (Word(1) << remainder) - 1;
			if ((words[WordCount - 1] & mask) != 0) return true;
        }
        return false;
    }

    [[nodiscard]] bool none() const noexcept
    {
        return !any();
    }

    [[nodiscard]] bool all() const noexcept
    {
        // Check all full words first
        for (size_t i = 0; i < Bits / BitsPerWord; i++)
            if (words[i] != ~Word(0)) return false;
 
        // Check remaining bits in the last partial word (if any)
        constexpr size_t remainder = Bits % BitsPerWord;
        if constexpr (remainder != 0)
        {
            constexpr Word mask = (Word(1) << remainder) - 1;
            if ((words[WordCount - 1] & mask) != mask) return false;
        }
 
        return true;
    }

    [[nodiscard]] size_t count() const noexcept
    {
        size_t total = 0;
        for (const auto& w : words)
            total += std::popcount(w);
        return total;
	}

    // Returns the index of the first set bit, or Bits if none
    [[nodiscard]] size_t findFirst() const noexcept
    {
        for (size_t i = 0; i < WordCount; i++)
        {
            if (words[i] != 0)
                return i * BitsPerWord + countTrailingZeros(words[i]);
        }
        return Bits;
    }

    // Returns the index of the next set bit strictly after pos, or Bits if none
    [[nodiscard]] size_t findNext(size_t pos) const noexcept
    {
        const size_t next = pos + 1;
        if (next >= Bits)
            return Bits;

        size_t wi = wordIndex(next);
        const size_t offset = bitOffset(next);

        // Mask off bits below 'next' within the starting word
        const Word masked = words[wi] & (~Word(0) << offset);
        if (masked != 0)
            return wi * BitsPerWord + countTrailingZeros(masked);

        // Scan remaining words
        wi++; // Move to next
        for (; wi < WordCount; wi++)
        {
            if (words[wi] != 0)
                return wi * BitsPerWord + countTrailingZeros(words[wi]);
        }
        return Bits;
    }

    [[nodiscard]] Bitset operator~() const noexcept
    {
        Bitset result;
        for (size_t i = 0; i < WordCount; ++i)
            result.words[i] = ~words[i];
        return result;
    }

    [[nodiscard]] Bitset operator&(const Bitset& rhs) const noexcept
    {
        Bitset result;
        for (size_t i = 0; i < WordCount; ++i)
            result.words[i] = words[i] & rhs.words[i];
        return result;
    }

    [[nodiscard]] Bitset operator|(const Bitset& rhs) const noexcept
    {
        Bitset result;
        for (size_t i = 0; i < WordCount; ++i)
            result.words[i] = words[i] | rhs.words[i];
        return result;
    }

    [[nodiscard]] Bitset operator^(const Bitset& rhs) const noexcept
    {
        Bitset result;
        for (size_t i = 0; i < WordCount; ++i)
            result.words[i] = words[i] ^ rhs.words[i];
        return result;
    }

    [[nodiscard]] Bitset operator!() const noexcept
    {
        return ~(*this);
	}

    Bitset& operator&=(const Bitset& rhs) noexcept
    {
        for (size_t i = 0; i < WordCount; ++i)
            words[i] &= rhs.words[i];
        return *this;
    }

    Bitset& operator|=(const Bitset& rhs) noexcept
    {
        for (size_t i = 0; i < WordCount; ++i)
            words[i] |= rhs.words[i];
        return *this;
    }

    Bitset& operator^=(const Bitset& rhs) noexcept
    {
        for (size_t i = 0; i < WordCount; ++i)
            words[i] ^= rhs.words[i];
        return *this;
    }
};