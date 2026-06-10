#pragma once
#include <random>
#include <concepts>
#include <limits>

namespace Ecstasy::Random
{
    class Generator
    {
        std::mt19937_64 generator;
    public:
        Generator() = default;
        ~Generator() = default;
        Generator(const Generator&) = delete;
        Generator& operator=(const Generator&) = delete;
        Generator(Generator&&) = delete;
        Generator& operator=(Generator&&) = delete;

        Generator& getGlobalInstance()
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
            generator.seed(rd());
        }

        template<std::integral T>
        [[nodiscard]] T integer(
            T min = std::numeric_limits<T>::min(),
            T max = std::numeric_limits<T>::max()
        )
        {
            if (min > max) std::swap(min, max);
            std::uniform_int_distribution<T> dist(min, max);
            return dist(generator);
        }

        template<std::floating_point T>
        [[nodiscard]] T real(T min = 0.0, T max = 1.0)
        {
            if (min > max) std::swap(min, max);
            std::uniform_real_distribution<T> dist(min, max);
            return dist(generator);
        }

        [[nodiscard]] bool boolean(double probability = 0.5)
        {
            std::bernoulli_distribution dist(probability);
            return dist(generator);
        }
    };
}
