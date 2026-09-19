#pragma once
#include <cstdint>
#include <type_traits>
#include <immintrin.h>

namespace Ecstasy::Core
{
    // Register width.
    #if defined(__AVX2__)
        inline constexpr size_t kSimdBits = 256;
        #define ECSTASY_SIMD_AVX2
    #elif defined(__AVX__)
        inline constexpr size_t kSimdBits = 256;
        #define ECSTASY_SIMD_AVX1
    #else
        #if (defined(__clang__) || defined(__GNUC__)) && !defined(__SSE4_1__)
            #error "Ecstasy SIMD requires at least SSE4.1. Bare SSE2 is not supported."
        #endif
            inline constexpr size_t kSimdBits = 128;
        #define ECSTASY_SIMD_SSE
    #endif

    inline constexpr size_t kSimdBytes = kSimdBits / 8;

    // Features.
    #if defined(__FMA__) || (defined(_MSC_VER) && defined(__AVX2__))
        #define ECSTASY_HAS_FMA 1
    #else
        #define ECSTASY_HAS_FMA 0
    #endif

    // Allowed element types.
    template<typename T>
    concept SimdElement =
        std::is_same_v<T, int32_t> || std::is_same_v<T, int64_t> ||
        std::is_same_v<T, uint32_t> || std::is_same_v<T, uint64_t> ||
        std::is_same_v<T, float> || std::is_same_v<T, double>;

    // Default register width per element type.
    // AVX1 only provides 256-bit float/double ops; 256-bit integer ops require AVX2.
    // So under AVX1-without-AVX2, integer types must default to 128 bits.
    template<typename T>
    inline constexpr size_t kSimdBitsFor =
        #if defined(ECSTASY_SIMD_AVX1)
        std::is_integral_v<T> ? 128 : kSimdBits;
    #else
        kSimdBits;
    #endif

    // Register type trait.
    template<typename T, size_t Bits> struct SimdReg;
    template<> struct SimdReg<int32_t, 128> { using type = __m128i; };
    template<> struct SimdReg<int32_t, 256> { using type = __m256i; };

    template<> struct SimdReg<int64_t, 128> { using type = __m128i; };
    template<> struct SimdReg<int64_t, 256> { using type = __m256i; };

    template<> struct SimdReg<uint32_t, 128> { using type = __m128i; };
    template<> struct SimdReg<uint32_t, 256> { using type = __m256i; };

    template<> struct SimdReg<uint64_t, 128> { using type = __m128i; };
    template<> struct SimdReg<uint64_t, 256> { using type = __m256i; };

    template<> struct SimdReg<float, 128> { using type = __m128; };
    template<> struct SimdReg<float, 256> { using type = __m256; };

    template<> struct SimdReg<double, 128> { using type = __m128d; };
    template<> struct SimdReg<double, 256> { using type = __m256d; };

    // Simd class.
    template<SimdElement T, size_t Bits = kSimdBitsFor<T>>
    struct Simd
    {
        static_assert(Bits == 128 || Bits == 256, "Invalid bit count");
        #if defined(ECSTASY_SIMD_SSE)
        static_assert(Bits < 256, "256-bit SIMD requires AVX2");
        #endif
        #if defined(ECSTASY_SIMD_AVX1)
        static_assert(!(Bits == 256 && std::is_integral_v<T>),
            "256-bit integer SIMD requires AVX2 (AVX1 only supports 256-bit float/double)");
        #endif

        using ValueType = T;
        using RegType = typename SimdReg<T, Bits>::type;

        static constexpr size_t bytes = Bits / 8;
        static constexpr size_t lanes = bytes / sizeof(T);

        static constexpr bool IS_INT32 = std::is_same_v<T, int32_t>;
        static constexpr bool IS_INT64 = std::is_same_v<T, int64_t>;
        static constexpr bool IS_UINT32 = std::is_same_v<T, uint32_t>;
        static constexpr bool IS_UINT64 = std::is_same_v<T, uint64_t>;
        static constexpr bool IS_FLOAT = std::is_same_v<T, float>;
        static constexpr bool IS_DOUBLE = std::is_same_v<T, double>;

        static constexpr bool IS_INTEGER = std::is_integral_v<T>;
        static constexpr bool IS_UNSIGNED_INTEGER = std::is_unsigned_v<T>;
        static constexpr bool IS_32BIT_INTEGER = IS_INT32 || IS_UINT32;
        static constexpr bool IS_64BIT_INTEGER = IS_INT64 || IS_UINT64;

        static constexpr bool IS_REAL = IS_FLOAT || IS_DOUBLE;

        RegType reg;

        #include "SimdImpl/Simd_Construct.inl"
        #include "SimdImpl/Simd_LoadStore.inl"
        #include "SimdImpl/Simd_Gather.inl"
        #include "SimdImpl/Simd_Get.inl"
        #include "SimdImpl/Simd_Bitwise.inl"
        #include "SimdImpl/Simd_Arithmetic.inl"
        #include "SimdImpl/Simd_Rounding.inl"
        #include "SimdImpl/Simd_Compare.inl"
        #include "SimdImpl/Simd_MovemaskBlend.inl"
        #include "SimdImpl/Simd_MinMaxClamp.inl"
        #include "SimdImpl/Simd_ExtractShuffle.inl"
        #include "SimdImpl/Simd_Horizontal.inl"
        #include "SimdImpl/Simd_Cast.inl"
        #include "SimdImpl/Simd_Reinterpret.inl"
    };
}