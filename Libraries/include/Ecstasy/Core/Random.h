#pragma once
#include <bit>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <limits>
#include <random>
#include <utility>

namespace Ecstasy::Core::Random
{
    // Deterministic across compilers/standard libraries: only std::mt19937_64
    // (fully specified by the standard) is used; all distributions are hand-written.
    class Generator
    {
        std::mt19937_64 generator;

        // Uniform in [0, 1), never exactly 1.
        template<std::floating_point T>
        [[nodiscard]] T unit()
        {
            constexpr int bits = std::numeric_limits<T>::digits < 53 ? std::numeric_limits<T>::digits : 53;
            constexpr T scale = static_cast<T>(1) / static_cast<T>(1ull << bits);
            return static_cast<T>(generator() >> (64 - bits)) * scale;
        }

    public:
        Generator() = default;
        ~Generator() = default;
        Generator(const Generator&) = delete;
        Generator& operator=(const Generator&) = delete;
        Generator(Generator&&) = delete;
        Generator& operator=(Generator&&) = delete;

        static Generator& getGlobalInstance()
        {
            static Generator instance;
            return instance;
        }

        void setSeed(uint64_t seed)
        {
            generator.seed(seed);
        }

        void setRandomSeed()
        {
            std::random_device rd;
            generator.seed((static_cast<uint64_t>(rd()) << 32) | rd());
        }

        // Uniform in [min, max] (inclusive).
        template<std::integral T>
        [[nodiscard]] T integer(
            T min = std::numeric_limits<T>::min(),
            T max = std::numeric_limits<T>::max()
        )
        {
            using U = uint64_t;
            if (min > max) std::swap(min, max);

            const U span = static_cast<U>(max) - static_cast<U>(min); // Modular, exact for signed too.
            if (span == 0) return min;

            U r;
            if (span == std::numeric_limits<U>::max())
            {
                r = generator();
            }
            else
            {
                // Smallest 2^k-1 >= span; rejection keeps the result unbiased (<2 draws on average).
                const U mask = std::numeric_limits<U>::max() >> std::countl_zero(span);
                do { r = generator() & mask; } while (r > span);
            }
            return static_cast<T>(static_cast<U>(min) + r);
        }

        // Uniform in [min, max).
        template<std::floating_point T>
        [[nodiscard]] T real(T min = 0.0, T max = 1.0)
        {
            if (min > max) std::swap(min, max);
            T r = min + unit<T>() * (max - min);
            if (min < max && r >= max) r = std::nextafter(max, min); // guard against rounding up to max
            return r;
        }

        [[nodiscard]] bool boolean(double probability = 0.5)
        {
            return unit<double>() < probability;
        }
    };
}