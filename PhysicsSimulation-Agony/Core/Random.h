#pragma once
#include <random>
#include <concepts>
#include <limits>

class Random
{
private:
    // Delete constructor to make class static
    Random() = delete;

    // Initialize generator with proper seed
    static std::mt19937_64 initGenerator()
    {
        std::random_device rd;
        return std::mt19937_64(rd());
    }

    // Thread-local random engine for thread safety
    inline static thread_local std::mt19937_64 generator = initGenerator();
public:
    // Set seed for reproducible results
    static void setSeed(uint64_t seed)
    {
        generator.seed(seed);
    }

    // Generate random integer in [min, max] range
    template<std::integral T>
    static T integer(
        T min = std::numeric_limits<T>::min(),
        T max = std::numeric_limits<T>::max()
    )
    {
        if (min > max) std::swap(min, max);
        std::uniform_int_distribution<T> dist(min, max);
        return dist(generator);
    }

    // Generate random floating-point number in [min, max) range
    template<std::floating_point T>
    static T real(T min = 0.0, T max = 1.0)
    {
        if (min > max) std::swap(min, max);
        std::uniform_real_distribution<T> dist(min, max);
        return dist(generator);
    }

    // Generate random boolean
    static bool boolean(double probability = 0.5)
    {
        std::bernoulli_distribution dist(probability);
        return dist(generator);
    }

    // Shuffle elements in a container
    template<typename Container>
    static void shuffle(Container& container)
    {
        std::shuffle(std::begin(container), std::end(container), generator);
    }

    // Select random element from container
    template<typename Container>
    static auto choice(const Container& container) -> typename Container::value_type
    {
        if (container.empty())
        {
            throw std::invalid_argument("Container is empty");
        }
        auto it = std::begin(container);
        std::advance(it, integer<size_t>(0, container.size() - 1));
        return *it;
    }

    // Generate random string
    static std::string string(size_t length, const std::string& charset = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789")
    {
        if (charset.empty())
        {
            throw std::invalid_argument("Character set cannot be empty");
        }

        std::string result;
        result.resize(length);

        std::uniform_int_distribution<size_t> dist(0, charset.size() - 1);
        for (size_t i = 0; i < length; i++)
        {
            result[i] = charset[dist(generator)];
        }

        return result;
    }

    // Generate random bytes
    static std::vector<uint8_t> bytes(size_t count)
    {
        std::vector<uint8_t> result(count);
        std::uniform_int_distribution<uint16_t> dist(0, 255);

        for (size_t i = 0; i < count; i++)
        {
            result[i] = static_cast<uint8_t>(dist(generator));
        }

        return result;
    }

    //// Generate random color (RGB)
    //static std::tuple<uint8_t, uint8_t, uint8_t> color() {
    //    std::uniform_int_distribution<uint8_t> dist(0, 255);
    //    return { dist(generator), dist(generator), dist(generator) };
    //}
    //
    //// Generate random color with alpha (RGBA)
    //static std::tuple<uint8_t, uint8_t, uint8_t, uint8_t> colorWithAlpha() {
    //    std::uniform_int_distribution<uint8_t> dist(0, 255);
    //    return { dist(generator), dist(generator), dist(generator), dist(generator) };
    //}
};