#pragma once
#include <cstdint>
#include <type_traits>
#include <immintrin.h>

namespace Ecstasy
{
    // Compile-time register width
    #if defined(__AVX2__)
    inline constexpr size_t kSimdBits = 256;
    #define SIMD_AVX2
    #else // SSE by default
    inline constexpr size_t kSimdBits = 128;
    #define SIMD_SSE
    #endif

    inline constexpr size_t kSimdBytes = kSimdBits / 8;

    // Allowed element types
    template<typename T>
    concept SimdElement =
        std::is_same_v<T, int32_t> || std::is_same_v<T, int64_t> ||
        std::is_same_v<T, uint32_t> || std::is_same_v<T, uint64_t> ||
        std::is_same_v<T, float> || std::is_same_v<T, double>;

    // Register type trait
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

    // Simd class
    template<SimdElement T, size_t Bits = kSimdBits>
    struct Simd
    {
        static_assert(Bits == 128 || Bits == 256);
        #if !defined(SIMD_AVX2)
        static_assert(Bits < 256, "256-bit SIMD requires AVX2");
        #endif

        using value_type = T;
        using reg_type = typename SimdReg<T, Bits>::type;

        static constexpr size_t bytes = Bits / 8;
        static constexpr size_t lanes = bytes / sizeof(T);

        reg_type reg;

        // Construction

        [[nodiscard]] static Simd fill_lanes_with_value(const T& val) noexcept
        {
            Simd s;
            if constexpr (std::is_same_v<T, int32_t>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_set1_epi32(static_cast<int32_t>(val));
                else                       s.reg = _mm_set1_epi32(static_cast<int32_t>(val));
            }
            else if constexpr (std::is_same_v<T, int64_t>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_set1_epi64x(static_cast<int64_t>(val));
                else                       s.reg = _mm_set1_epi64x(static_cast<int64_t>(val));
            }
            else if constexpr (std::is_same_v<T, uint32_t>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_set1_epi32(static_cast<int32_t>(val));
                else                       s.reg = _mm_set1_epi32(static_cast<int32_t>(val));
            }
            else if constexpr (std::is_same_v<T, uint64_t>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_set1_epi64x(static_cast<int64_t>(val));
                else                       s.reg = _mm_set1_epi64x(static_cast<int64_t>(val));
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_set1_ps(static_cast<float>(val));
                else                       s.reg = _mm_set1_ps(static_cast<float>(val));
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_set1_pd(static_cast<double>(val));
                else                       s.reg = _mm_set1_pd(static_cast<double>(val));
            }
            return s;
        }

        [[nodiscard]] static Simd fill_lanes_with_zero() noexcept
        {
            Simd s;
            if constexpr (std::is_integral_v<T>) // Integers only
            {
                if constexpr (Bits == 256) s.reg = _mm256_setzero_si256();
                else                       s.reg = _mm_setzero_si128();
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_setzero_ps();
                else                       s.reg = _mm_setzero_ps();
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_setzero_pd();
                else                       s.reg = _mm_setzero_pd();
            }
            return s;
        }

        [[nodiscard]] static Simd fill_lanes_with_full_value() noexcept
        {
            if constexpr (std::is_integral_v<T>)
                return fill_lanes_with_value(static_cast<T>(-1));
            else if constexpr (std::is_same_v<T, float>)
                return Simd<int32_t, Bits>::fill_lanes_with_value(-1).as_float();
            else
                return Simd<int64_t, Bits>::fill_lanes_with_value(-1).as_double();
        }

        Simd() = default;

        Simd(const T& val) noexcept
        {
            *this = fill_lanes_with_value(val);
        }

        // Loads in reverse order (first argument becomes highest lane)
        template<typename... Args>
        [[nodiscard]] static Simd set(Args... vals) noexcept
            requires std::is_integral_v<T>
        {
            static_assert(sizeof...(vals) == lanes);
            static_assert((std::is_convertible_v<Args, T> && ...));

            Simd s;
            if constexpr (std::is_same_v<T, int32_t> || std::is_same_v<T, uint32_t>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_set_epi32(static_cast<int32_t>(vals)...);
                else                       s.reg = _mm_set_epi32(static_cast<int32_t>(vals)...);
            }
            else // int64_t or uint64_t
            {
                if constexpr (Bits == 256) s.reg = _mm256_set_epi64x(static_cast<int64_t>(vals)...);
                else                       s.reg = _mm_set_epi64x(static_cast<int64_t>(vals)...);
            }
            return s;
        }

        template<typename... Args>
        [[nodiscard]] static Simd set(Args... vals) noexcept
            requires (std::is_same_v<T, float>)
        {
            static_assert(sizeof...(vals) == lanes);
            static_assert((std::is_convertible_v<Args, T> && ...));

            Simd s;
            if constexpr (Bits == 256) s.reg = _mm256_set_ps(static_cast<T>(vals)...);
            else                       s.reg = _mm_set_ps(static_cast<T>(vals)...);
            return s;
        }

        template<typename... Args>
        [[nodiscard]] static Simd set(Args... vals) noexcept
            requires (std::is_same_v<T, double>)
        {
            static_assert(sizeof...(vals) == lanes);
            static_assert((std::is_convertible_v<Args, T> && ...));

            Simd s;
            if constexpr (Bits == 256) s.reg = _mm256_set_pd(static_cast<T>(vals)...);
            else                       s.reg = _mm_set_pd(static_cast<T>(vals)...);
            return s;
        }

        // Load

        [[nodiscard]] static Simd load(const T* ptr) noexcept
        {
            Simd s;
            if constexpr (std::is_integral_v<T>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_load_si256(reinterpret_cast<const __m256i*>(ptr));
                else                       s.reg = _mm_load_si128(reinterpret_cast<const __m128i*>(ptr));
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_load_ps(ptr);
                else                       s.reg = _mm_load_ps(ptr);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_load_pd(ptr);
                else                       s.reg = _mm_load_pd(ptr);
            }
            return s;
        }

        [[nodiscard]] static Simd loadu(const T* ptr) noexcept
        {
            Simd s;
            if constexpr (std::is_integral_v<T>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(ptr));
                else                       s.reg = _mm_loadu_si128(reinterpret_cast<const __m128i*>(ptr));
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_loadu_ps(ptr);
                else                       s.reg = _mm_loadu_ps(ptr);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_loadu_pd(ptr);
                else                       s.reg = _mm_loadu_pd(ptr);
            }
            return s;
        }

        // Store

        void store(T* ptr) const noexcept
        {
            if constexpr (std::is_integral_v<T>)
            {
                if constexpr (Bits == 256) _mm256_store_si256(reinterpret_cast<__m256i*>(ptr), reg);
                else                       _mm_store_si128(reinterpret_cast<__m128i*>(ptr), reg);
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) _mm256_store_ps(ptr, reg);
                else                       _mm_store_ps(ptr, reg);
            }
            else
            {
                if constexpr (Bits == 256) _mm256_store_pd(ptr, reg);
                else                       _mm_store_pd(ptr, reg);
            }
        }

        void storeu(T* ptr) const noexcept
        {
            if constexpr (std::is_integral_v<T>)
            {
                if constexpr (Bits == 256) _mm256_storeu_si256(reinterpret_cast<__m256i*>(ptr), reg);
                else                       _mm_storeu_si128(reinterpret_cast<__m128i*>(ptr), reg);
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) _mm256_storeu_ps(ptr, reg);
                else                       _mm_storeu_ps(ptr, reg);
            }
            else
            {
                if constexpr (Bits == 256) _mm256_storeu_pd(ptr, reg);
                else                       _mm_storeu_pd(ptr, reg);
            }
        }

        void store_lower_int_64(T* ptr) const noexcept
            requires (std::is_same_v<T, int32_t>&& Bits == 128)
        {
            _mm_storel_epi64(reinterpret_cast<__m128i*>(ptr), reg);
        }

        // Get
        int32_t get_least_significant_int_32() const noexcept
            requires (std::is_same_v<T, int32_t>&& Bits == 128)
        {
            return _mm_cvtsi128_si32(reg);
        }

        // Bitwise operations

        [[nodiscard]] static Simd bitwise_and(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (std::is_integral_v<T>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_and_si256(a.reg, b.reg);
                else                       s.reg = _mm_and_si128(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_and_ps(a.reg, b.reg);
                else                       s.reg = _mm_and_ps(a.reg, b.reg);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_and_pd(a.reg, b.reg);
                else                       s.reg = _mm_and_pd(a.reg, b.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd bitwise_or(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (std::is_integral_v<T>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_or_si256(a.reg, b.reg);
                else                       s.reg = _mm_or_si128(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_or_ps(a.reg, b.reg);
                else                       s.reg = _mm_or_ps(a.reg, b.reg);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_or_pd(a.reg, b.reg);
                else                       s.reg = _mm_or_pd(a.reg, b.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd bitwise_xor(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (std::is_integral_v<T>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_xor_si256(a.reg, b.reg);
                else                       s.reg = _mm_xor_si128(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_xor_ps(a.reg, b.reg);
                else                       s.reg = _mm_xor_ps(a.reg, b.reg);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_xor_pd(a.reg, b.reg);
                else                       s.reg = _mm_xor_pd(a.reg, b.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd bitwise_not(const Simd& a) noexcept
        {
            return bitwise_xor(a, fill_lanes_with_full_value());
        }

        // andnot(a, b) = (~a) & b
        [[nodiscard]] static Simd bitwise_andnot(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (std::is_integral_v<T>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_andnot_si256(a.reg, b.reg);
                else                       s.reg = _mm_andnot_si128(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_andnot_ps(a.reg, b.reg);
                else                       s.reg = _mm_andnot_ps(a.reg, b.reg);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_andnot_pd(a.reg, b.reg);
                else                       s.reg = _mm_andnot_pd(a.reg, b.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd logical_shift_left(const Simd& a, int32_t count) noexcept
            requires (std::is_integral_v<T>)
        {
            Simd s;
            if constexpr (std::is_same_v<T, int32_t> || std::is_same_v<T, uint32_t>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_slli_epi32(a.reg, count);
                else                       s.reg = _mm_slli_epi32(a.reg, count);
            }
            else // int64_t or uint64_t
            {
                if constexpr (Bits == 256) s.reg = _mm256_slli_epi64(a.reg, count);
                else                       s.reg = _mm_slli_epi64(a.reg, count);
            }
            return s;
        }

        [[nodiscard]] static Simd logical_shift_right(const Simd& a, int32_t count) noexcept
            requires (std::is_integral_v<T>)
        {
            Simd s;
            if constexpr (std::is_same_v<T, int32_t> || std::is_same_v<T, uint32_t>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_srli_epi32(a.reg, count);
                else                       s.reg = _mm_srli_epi32(a.reg, count);
            }
            else // int64_t
            {
                if constexpr (Bits == 256) s.reg = _mm256_srli_epi64(a.reg, count);
                else                       s.reg = _mm_srli_epi64(a.reg, count);
            }
            return s;
        }

        [[nodiscard]] static Simd arithmetic_shift_right(const Simd& a, int32_t count) noexcept
            requires (std::is_integral_v<T>)
        {
            if constexpr (std::is_unsigned_v<T>)
            {
                // For unsigned, arithmetic right shift is same as logical shift
                return logical_shift_right(a, count);
            }
            else // Signed
            {
                if constexpr (std::is_same_v<T, int32_t>)
                {
                    Simd s;
                    if constexpr (Bits == 256) s.reg = _mm256_srai_epi32(a.reg, count);
                    else                       s.reg = _mm_srai_epi32(a.reg, count);
                    return s;
                }
                else // int64_t
                {
                    // No native 64-bit arithmetic shift; emulate via sign + logical shift
                    if (count == 0) return a;
                    Simd sign = logical_shift_right(a, 63); // All ones if negative, else 0
                    Simd logical = logical_shift_right(a, count);
                    Simd high_mask = logical_shift_left(sign, 64 - count);
                    return bitwise_or(logical, high_mask);
                }
            }
        }

        [[nodiscard]] Simd operator&(const Simd& other) const noexcept { return bitwise_and(*this, other); }
        [[nodiscard]] Simd operator|(const Simd& other) const noexcept { return bitwise_or(*this, other); }
        [[nodiscard]] Simd operator^(const Simd& other) const noexcept { return bitwise_xor(*this, other); }
        [[nodiscard]] Simd operator~()                  const noexcept { return bitwise_not(*this); }
        [[nodiscard]] Simd operator<<(int32_t count) const noexcept
            requires (std::is_integral_v<T>)
        {
            return logical_shift_left(*this, count);
        }
        [[nodiscard]] Simd operator>>(int32_t count) const noexcept
            requires (std::is_integral_v<T>)
        {
            return logical_shift_right(*this, count);
        }

        Simd& operator&=(const Simd& other) noexcept { *this = bitwise_and(*this, other); return *this; }
        Simd& operator|=(const Simd& other) noexcept { *this = bitwise_or(*this, other);  return *this; }
        Simd& operator^=(const Simd& other) noexcept { *this = bitwise_xor(*this, other); return *this; }
        Simd& operator<<=(int32_t count) noexcept
            requires (std::is_integral_v<T>)
        {
            *this = logical_shift_left(*this, count);  return *this;
        }
        Simd& operator>>=(int32_t count) noexcept
            requires (std::is_integral_v<T>)
        {
            *this = logical_shift_right(*this, count); return *this;
        }

        // Arithmetic

        [[nodiscard]] static Simd add(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (std::is_same_v<T, int32_t> || std::is_same_v<T, uint32_t>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_add_epi32(a.reg, b.reg);
                else                       s.reg = _mm_add_epi32(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, int64_t> || std::is_same_v<T, uint64_t>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_add_epi64(a.reg, b.reg);
                else                       s.reg = _mm_add_epi64(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_add_ps(a.reg, b.reg);
                else                       s.reg = _mm_add_ps(a.reg, b.reg);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_add_pd(a.reg, b.reg);
                else                       s.reg = _mm_add_pd(a.reg, b.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd sub(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (std::is_same_v<T, int32_t> || std::is_same_v<T, uint32_t>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_sub_epi32(a.reg, b.reg);
                else                       s.reg = _mm_sub_epi32(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, int64_t> || std::is_same_v<T, uint64_t>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_sub_epi64(a.reg, b.reg);
                else                       s.reg = _mm_sub_epi64(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_sub_ps(a.reg, b.reg);
                else                       s.reg = _mm_sub_ps(a.reg, b.reg);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_sub_pd(a.reg, b.reg);
                else                       s.reg = _mm_sub_pd(a.reg, b.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd mul(const Simd& a, const Simd& b) noexcept
            requires (!std::is_same_v<T, int64_t> && !std::is_same_v<T, uint64_t>)   // no 64-bit integer multiply in SSE/AVX2
        {
            Simd s;
            if constexpr (std::is_same_v<T, int32_t> || std::is_same_v<T, uint32_t>)
            {
                // mullo: low 32 Bits of each 32x32->64 product (wrapping)
                if constexpr (Bits == 256) s.reg = _mm256_mullo_epi32(a.reg, b.reg);
                else                       s.reg = _mm_mullo_epi32(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_mul_ps(a.reg, b.reg);
                else                       s.reg = _mm_mul_ps(a.reg, b.reg);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_mul_pd(a.reg, b.reg);
                else                       s.reg = _mm_mul_pd(a.reg, b.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd div(const Simd& a, const Simd& b) noexcept
            requires (std::is_same_v<T, float> || std::is_same_v<T, double>)
        {
            Simd s;
            if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_div_ps(a.reg, b.reg);
                else                       s.reg = _mm_div_ps(a.reg, b.reg);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_div_pd(a.reg, b.reg);
                else                       s.reg = _mm_div_pd(a.reg, b.reg);
            }
            return s;
        }

        // a * b + c
        [[nodiscard]] static Simd mul_add(const Simd& a, const Simd& b, const Simd& c) noexcept
            requires (std::is_same_v<T, float> || std::is_same_v<T, double>)
        {
            Simd s;
            if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_fmadd_ps(a.reg, b.reg, c.reg);
                else                       s.reg = _mm_fmadd_ps(a.reg, b.reg, c.reg);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_fmadd_pd(a.reg, b.reg, c.reg);
                else                       s.reg = _mm_fmadd_pd(a.reg, b.reg, c.reg);
            }
            return s;
        }

        // a * b - c
        [[nodiscard]] static Simd mul_sub(const Simd& a, const Simd& b, const Simd& c) noexcept
            requires (std::is_same_v<T, float> || std::is_same_v<T, double>)
        {
            Simd s;
            if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_fmsub_ps(a.reg, b.reg, c.reg);
                else                       s.reg = _mm_fmsub_ps(a.reg, b.reg, c.reg);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_fmsub_pd(a.reg, b.reg, c.reg);
                else                       s.reg = _mm_fmsub_pd(a.reg, b.reg, c.reg);
            }
            return s;
        }

        // -(a * b) + c
        [[nodiscard]] static Simd neg_mul_add(const Simd& a, const Simd& b, const Simd& c) noexcept
            requires (std::is_same_v<T, float> || std::is_same_v<T, double>)
        {
            Simd s;
            if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_fnmadd_ps(a.reg, b.reg, c.reg);
                else                       s.reg = _mm_fnmadd_ps(a.reg, b.reg, c.reg);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_fnmadd_pd(a.reg, b.reg, c.reg);
                else                       s.reg = _mm_fnmadd_pd(a.reg, b.reg, c.reg);
            }
            return s;
        }

        // -(a * b) - c
        [[nodiscard]] static Simd neg_mul_sub(const Simd& a, const Simd& b, const Simd& c) noexcept
            requires (std::is_same_v<T, float> || std::is_same_v<T, double>)
        {
            Simd s;
            if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_fnmsub_ps(a.reg, b.reg, c.reg);
                else                       s.reg = _mm_fnmsub_ps(a.reg, b.reg, c.reg);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_fnmsub_pd(a.reg, b.reg, c.reg);
                else                       s.reg = _mm_fnmsub_pd(a.reg, b.reg, c.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd negate(const Simd& a) noexcept
        {
            if constexpr (std::is_integral_v<T>)
                return sub(fill_lanes_with_zero(), a);
            else if constexpr (std::is_same_v<T, float>)
            {
                const Simd sign_mask = fill_lanes_with_value(-0.0);
                return bitwise_xor(a, sign_mask);
            }
            else
            {
                const Simd sign_mask = fill_lanes_with_value(-0.0);
                return bitwise_xor(a, sign_mask);
            }
        }

        [[nodiscard]] static Simd sqrt(const Simd& a) noexcept
            requires (std::is_same_v<T, float> || std::is_same_v<T, double>)
        {
            Simd s;
            if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_sqrt_ps(a.reg);
                else                       s.reg = _mm_sqrt_ps(a.reg);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_sqrt_pd(a.reg);
                else                       s.reg = _mm_sqrt_pd(a.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd get_abs_mask() noexcept
            requires (std::is_same_v<T, float> || std::is_same_v<T, double>)
        {
            Simd s;
            if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_castsi256_ps(_mm256_set1_epi32(0x7FFFFFFF));
                else                       s.reg = _mm_castsi128_ps(_mm_set1_epi32(0x7FFFFFFF));
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_castsi256_pd(_mm256_set1_epi64x(0x7FFFFFFFFFFFFFFFll));
                else                       s.reg = _mm_castsi128_pd(_mm_set1_epi64x(0x7FFFFFFFFFFFFFFFll));
            }
            return s;
        }

        [[nodiscard]] Simd operator+(const Simd& other) const noexcept { return add(*this, other); }
        [[nodiscard]] Simd operator-(const Simd& other) const noexcept { return sub(*this, other); }
        [[nodiscard]] Simd operator*(const Simd& other) const noexcept { return mul(*this, other); }
        [[nodiscard]] Simd operator/(const Simd& other) const noexcept
            requires (std::is_same_v<T, float> || std::is_same_v<T, double>)
        {
            return div(*this, other);
        }
        [[nodiscard]] Simd operator-() const noexcept { return negate(*this); }

        Simd& operator+=(const Simd& other) noexcept { *this = add(*this, other); return *this; }
        Simd& operator-=(const Simd& other) noexcept { *this = sub(*this, other); return *this; }
        Simd& operator*=(const Simd& other) noexcept { *this = mul(*this, other); return *this; }
        Simd& operator/=(const Simd& other) noexcept
            requires (std::is_same_v<T, float> || std::is_same_v<T, double>)
        {
            *this = div(*this, other); return *this;
        }

        // Rounding

        [[nodiscard]] static Simd round(const Simd& a) noexcept
            requires (std::is_same_v<T, float> || std::is_same_v<T, double>)
        {
            Simd s;
            constexpr int kNearest = _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC; // round to nearest, but don't raise exceptions on invalid input (e.g. NaN)
            if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_round_ps(a.reg, kNearest);
                else                       s.reg = _mm_round_ps(a.reg, kNearest);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_round_pd(a.reg, kNearest);
                else                       s.reg = _mm_round_pd(a.reg, kNearest);
            }
            return s;
        }

        [[nodiscard]] static Simd floor(const Simd& a) noexcept
            requires (std::is_same_v<T, float> || std::is_same_v<T, double>)
        {
            Simd s;
            if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_floor_ps(a.reg);
                else                       s.reg = _mm_floor_ps(a.reg);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_floor_pd(a.reg);
                else                       s.reg = _mm_floor_pd(a.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd ceil(const Simd& a) noexcept
            requires (std::is_same_v<T, float> || std::is_same_v<T, double>)
        {
            Simd s;
            if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_ceil_ps(a.reg);
                else                       s.reg = _mm_ceil_ps(a.reg);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_ceil_pd(a.reg);
                else                       s.reg = _mm_ceil_pd(a.reg);
            }
            return s;
        }

        // Comparisons - return a lane mask (0xFFFFFFFF... or 0x0 per lane)

        [[nodiscard]] static Simd compare_equal(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (std::is_same_v<T, int32_t> || std::is_same_v<T, uint32_t>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmpeq_epi32(a.reg, b.reg);
                else                       s.reg = _mm_cmpeq_epi32(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, int64_t> || std::is_same_v<T, uint64_t>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmpeq_epi64(a.reg, b.reg);
                else                       s.reg = _mm_cmpeq_epi64(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmp_ps(a.reg, b.reg, _CMP_EQ_OQ);
                else                       s.reg = _mm_cmpeq_ps(a.reg, b.reg);
            }
            else
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmp_pd(a.reg, b.reg, _CMP_EQ_OQ);
                else                       s.reg = _mm_cmpeq_pd(a.reg, b.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd compare_not_equal(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmp_ps(a.reg, b.reg, _CMP_NEQ_OQ);
                else                       s.reg = _mm_cmpneq_ps(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, double>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmp_pd(a.reg, b.reg, _CMP_NEQ_OQ);
                else                       s.reg = _mm_cmpneq_pd(a.reg, b.reg);
            }
            else // Integers
            {
                Simd eq = compare_equal(a, b);
                Simd ones = fill_lanes_with_full_value();
                if constexpr (Bits == 256) s.reg = _mm256_xor_si256(eq.reg, ones.reg);
                else                       s.reg = _mm_xor_si128(eq.reg, ones.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd compare_less(const Simd& a, const Simd& b) noexcept
        {
            Simd s;

            if constexpr (std::is_unsigned_v<T>)
            {
                // a < b  ->  (a ^ signBit) < (b ^ signBit) as signed
                constexpr T signBit = (T(1) << (sizeof(T) * 8 - 1));
                Simd a_xor = Simd::bitwise_xor(a, Simd(signBit));
                Simd b_xor = Simd::bitwise_xor(b, Simd(signBit));
                if constexpr (std::is_same_v<T, uint32_t>)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_cmpgt_epi32(b_xor.reg, a_xor.reg);
                    else                       s.reg = _mm_cmpgt_epi32(b_xor.reg, a_xor.reg);
                }
                else // uint64_t
                {
                    if constexpr (Bits == 256) s.reg = _mm256_cmpgt_epi64(b_xor.reg, a_xor.reg);
                    else                       s.reg = _mm_cmpgt_epi64(b_xor.reg, a_xor.reg);
                }
            }
            else
            {
                if constexpr (std::is_same_v<T, int32_t>)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_cmpgt_epi32(b.reg, a.reg);
                    else                       s.reg = _mm_cmpgt_epi32(b.reg, a.reg);
                }
                else if constexpr (std::is_same_v<T, int64_t>)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_cmpgt_epi64(b.reg, a.reg);
                    else                       s.reg = _mm_cmpgt_epi64(b.reg, a.reg);
                }
                else if constexpr (std::is_same_v<T, float>)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_cmp_ps(a.reg, b.reg, _CMP_LT_OQ);
                    else                       s.reg = _mm_cmplt_ps(a.reg, b.reg);
                }
                else
                {
                    if constexpr (Bits == 256) s.reg = _mm256_cmp_pd(a.reg, b.reg, _CMP_LT_OQ);
                    else                       s.reg = _mm_cmplt_pd(a.reg, b.reg);
                }
            }
            return s;
        }

        [[nodiscard]] static Simd compare_greater(const Simd& a, const Simd& b) noexcept
        {
            return compare_less(b, a);
        }

        [[nodiscard]] static Simd compare_less_equal(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmp_ps(a.reg, b.reg, _CMP_LE_OQ);
                else                       s.reg = _mm_cmple_ps(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, double>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmp_pd(a.reg, b.reg, _CMP_LE_OQ);
                else                       s.reg = _mm_cmple_pd(a.reg, b.reg);
            }
            else // Integers
            {
                Simd gt = compare_greater(a, b);
                Simd ones = fill_lanes_with_full_value();
                if constexpr (Bits == 256) s.reg = _mm256_xor_si256(gt.reg, ones.reg);
                else                       s.reg = _mm_xor_si128(gt.reg, ones.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd compare_greater_equal(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmp_ps(a.reg, b.reg, _CMP_GE_OQ);
                else                       s.reg = _mm_cmpge_ps(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, double>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmp_pd(a.reg, b.reg, _CMP_GE_OQ);
                else                       s.reg = _mm_cmpge_pd(a.reg, b.reg);
            }
            else // integers (int32_t, int64_t)
            {
                Simd lt = compare_less(a, b);
                Simd ones = fill_lanes_with_full_value();
                if constexpr (Bits == 256) s.reg = _mm256_xor_si256(lt.reg, ones.reg);
                else                       s.reg = _mm_xor_si128(lt.reg, ones.reg);
            }
            return s;
        }

        [[nodiscard]] Simd operator==(const Simd& other) const noexcept { return compare_equal(*this, other); }
        [[nodiscard]] Simd operator!=(const Simd& other) const noexcept { return compare_not_equal(*this, other); }
        [[nodiscard]] Simd operator< (const Simd& other) const noexcept { return compare_less(*this, other); }
        [[nodiscard]] Simd operator<=(const Simd& other) const noexcept { return compare_less_equal(*this, other); }
        [[nodiscard]] Simd operator> (const Simd& other) const noexcept { return compare_greater(*this, other); }
        [[nodiscard]] Simd operator>=(const Simd& other) const noexcept { return compare_greater_equal(*this, other); }

        // Movemask
        [[nodiscard]] int movemask() const noexcept
            requires (std::is_same_v<T, float> || std::is_same_v<T, double>)
        {
            if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256)
                    return _mm256_movemask_ps(reg);
                else
                    return _mm_movemask_ps(reg);
            }
            else
            {
                if constexpr (Bits == 256)
                    return _mm256_movemask_pd(reg);
                else
                    return _mm_movemask_pd(reg);
            }
        }

        // Blend
        [[nodiscard]] static Simd blendv(const Simd& a, const Simd& b, const Simd& mask) noexcept
        {
            Simd s;
            if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_blendv_ps(a.reg, b.reg, mask.reg);
                else                       s.reg = _mm_blendv_ps(a.reg, b.reg, mask.reg);
            }
            else if constexpr (std::is_same_v<T, double>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_blendv_pd(a.reg, b.reg, mask.reg);
                else                       s.reg = _mm_blendv_pd(a.reg, b.reg, mask.reg);
            }
            else // Integers
            {
                if constexpr (Bits == 256) s.reg = _mm256_blendv_epi8(a.reg, b.reg, mask.reg);
                else                       s.reg = _mm_blendv_epi8(a.reg, b.reg, mask.reg);
            }
            return s;
        }

        // Min / max / clamp

        [[nodiscard]] static Simd min(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (std::is_same_v<T, int32_t>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_min_epi32(a.reg, b.reg);
                else                       s.reg = _mm_min_epi32(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, uint32_t>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_min_epu32(a.reg, b.reg);
                else                       s.reg = _mm_min_epu32(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_min_ps(a.reg, b.reg);
                else                       s.reg = _mm_min_ps(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, double>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_min_pd(a.reg, b.reg);
                else                       s.reg = _mm_min_pd(a.reg, b.reg);
            }
            else // int64_t, uint64_t
            {
                Simd mask = compare_less(a, b);
                return blendv(a, b, mask);
            }
            return s;
        }

        [[nodiscard]] static Simd max(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (std::is_same_v<T, int32_t>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_max_epi32(a.reg, b.reg);
                else                       s.reg = _mm_max_epi32(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, uint32_t>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_max_epu32(a.reg, b.reg);
                else                       s.reg = _mm_max_epu32(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_max_ps(a.reg, b.reg);
                else                       s.reg = _mm_max_ps(a.reg, b.reg);
            }
            else if constexpr (std::is_same_v<T, double>)
            {
                if constexpr (Bits == 256) s.reg = _mm256_max_pd(a.reg, b.reg);
                else                       s.reg = _mm_max_pd(a.reg, b.reg);
            }
            else // int64_t, uint64_t
            {
                Simd mask = compare_greater(a, b);
                return blendv(a, b, mask);
            }
            return s;
        }

        [[nodiscard]] static T horizontal_min(const Simd& a) noexcept
            requires (std::is_same_v<T, float> || std::is_same_v<T, double>)
        {
            if constexpr (Bits == 128)
            {
                if constexpr (std::is_same_v<T, float>)
                {
                    __m128 v = a.reg;
                    v = _mm_min_ps(v, _mm_shuffle_ps(v, v, 0x4E));
                    v = _mm_min_ps(v, _mm_shuffle_ps(v, v, 0xB1));
                    return _mm_cvtss_f32(v);
                }
                else
                { // double
                    __m128d v = a.reg;
                    v = _mm_min_pd(v, _mm_shuffle_pd(v, v, 1));
                    return _mm_cvtsd_f64(v);
                }
            }
            else if constexpr (Bits == 256)
            {
                if constexpr (std::is_same_v<T, float>)
                {
                    __m256 v = a.reg;
                    __m128 lo = _mm256_castps256_ps128(v);
                    __m128 hi = _mm256_extractf128_ps(v, 1);
                    __m128 min128 = _mm_min_ps(lo, hi);
                    min128 = _mm_min_ps(min128, _mm_shuffle_ps(min128, min128, 0x4E));
                    min128 = _mm_min_ps(min128, _mm_shuffle_ps(min128, min128, 0xB1));
                    return _mm_cvtss_f32(min128);
                }
                else
                { // double
                    __m256d v = a.reg;
                    __m128d lo = _mm256_castpd256_pd128(v);
                    __m128d hi = _mm256_extractf128_pd(v, 1);
                    __m128d min128 = _mm_min_pd(lo, hi);
                    min128 = _mm_min_pd(min128, _mm_shuffle_pd(min128, min128, 1));
                    return _mm_cvtsd_f64(min128);
                }
            }
        }

        [[nodiscard]] static T horizontal_max(const Simd& a) noexcept
            requires (std::is_same_v<T, float> || std::is_same_v<T, double>)
        {
            if constexpr (Bits == 128)
            {
                if constexpr (std::is_same_v<T, float>)
                {
                    __m128 v = a.reg;
                    v = _mm_max_ps(v, _mm_shuffle_ps(v, v, 0x4E));
                    v = _mm_max_ps(v, _mm_shuffle_ps(v, v, 0xB1));
                    return _mm_cvtss_f32(v);
                }
                else { // double
                    __m128d v = a.reg;
                    v = _mm_max_pd(v, _mm_shuffle_pd(v, v, 1));
                    return _mm_cvtsd_f64(v);
                }
            }
            else if constexpr (Bits == 256)
            {
                if constexpr (std::is_same_v<T, float>)
                {
                    __m256 v = a.reg;
                    __m128 lo = _mm256_castps256_ps128(v);
                    __m128 hi = _mm256_extractf128_ps(v, 1);
                    __m128 max128 = _mm_max_ps(lo, hi);
                    max128 = _mm_max_ps(max128, _mm_shuffle_ps(max128, max128, 0x4E));
                    max128 = _mm_max_ps(max128, _mm_shuffle_ps(max128, max128, 0xB1));
                    return _mm_cvtss_f32(max128);
                }
                else
                { // double
                    __m256d v = a.reg;
                    __m128d lo = _mm256_castpd256_pd128(v);
                    __m128d hi = _mm256_extractf128_pd(v, 1);
                    __m128d max128 = _mm_max_pd(lo, hi);
                    max128 = _mm_max_pd(max128, _mm_shuffle_pd(max128, max128, 1));
                    return _mm_cvtsd_f64(max128);
                }
            }
        }

        [[nodiscard]] static Simd clamp(const Simd& value, const Simd& minBoundary, const Simd& maxBoundary)
        {
            return min(maxBoundary, max(minBoundary, value));
        }

        // Extract

        template<int Index>
        [[nodiscard]] static Simd<T, 128> extract_int_128(const Simd& a) noexcept
            requires (std::is_integral_v<T>&& Bits == 256)
        {
            Simd<T, 128> s;
            s.reg = _mm256_extracti128_si256(a.reg, Index);
            return s;
        }

        // Narrow-saturate (int32_t, 128-bit only)

        [[nodiscard]] static Simd<T, 128> narrow_saturate_16_to_8(const Simd& low, const Simd& high) noexcept
            requires (std::is_same_v<T, int32_t>&& Bits == 128)
        {
            Simd<T, 128> s;
            s.reg = _mm_packs_epi16(low.reg, high.reg);
            return s;
        }

        [[nodiscard]] static Simd<T, 128> narrow_saturate_32_to_16(const Simd& low, const Simd& high) noexcept
            requires (std::is_same_v<T, int32_t>&& Bits == 128)
        {
            Simd<T, 128> s;
            s.reg = _mm_packs_epi32(low.reg, high.reg);
            return s;
        }

        // Type casts

        [[nodiscard]] Simd<int32_t> to_int32() const noexcept
            requires std::is_same_v<T, float>
        {
            Simd<int32_t> s;
            if constexpr (Bits == 256) s.reg = _mm256_cvttps_epi32(reg);
            else                       s.reg = _mm_cvttps_epi32(reg);
            return s;
        }

        [[nodiscard]] Simd<int32_t, 128> to_int32() const noexcept
            requires std::is_same_v<T, double>
        {
            Simd<int32_t, 128> s;
            if constexpr (Bits == 256) s.reg = _mm256_cvttpd_epi32(reg);
            else                       s.reg = _mm_cvttpd_epi32(reg);
            return s;
        }

        [[nodiscard]] Simd<uint32_t, Bits> to_uint32() const noexcept
            requires std::is_same_v<T, float>
        {
            Simd<uint32_t, Bits> s;

            if constexpr (Bits == 128)
            {
                const __m128  bias_f = _mm_set1_ps(2147483648.0f); // 2^31
                const __m128i bias_i = _mm_set1_epi32(0x80000000u);

                const __m128 mask = _mm_cmpge_ps(reg, bias_f);
                const __m128 adjusted = _mm_sub_ps(reg, _mm_and_ps(mask, bias_f));
                const __m128i signed_i = _mm_cvttps_epi32(adjusted);

                s.reg = _mm_xor_si128(signed_i, _mm_and_si128(_mm_castps_si128(mask), bias_i));
            }
            else
            {
                const __m256  bias_f = _mm256_set1_ps(2147483648.0f); // 2^31
                const __m256i bias_i = _mm256_set1_epi32(0x80000000u);

                // mask = (reg >= 2^31)
                const __m256 mask = _mm256_cmp_ps(reg, bias_f, _CMP_GE_OQ);

                // subtract 2^31 only for lanes that need it
                const __m256 adjusted = _mm256_sub_ps(reg, _mm256_and_ps(mask, bias_f));

                // signed truncation of the adjusted value
                const __m256i signed_i = _mm256_cvttps_epi32(adjusted);

                // restore unsigned bit pattern
                s.reg = _mm256_xor_si256(signed_i, _mm256_and_si256(_mm256_castps_si256(mask), bias_i));
            }

            return s;
        }

        [[nodiscard]] Simd<float> to_float() const noexcept
            requires (std::is_same_v<T, int32_t>)
        {
            Simd<float> s;
            if constexpr (Bits == 256) s.reg = _mm256_cvtepi32_ps(reg);
            else                       s.reg = _mm_cvtepi32_ps(reg);
            return s;
        }

        [[nodiscard]] Simd<float> to_float() const noexcept
            requires (std::is_same_v<T, uint32_t>)
        {
            // Convert unsigned 32-bit to float using bias method:
            // (float)(a ^ 0x80000000) + 2147483648.0f
            Simd<float> s;
            Simd<uint32_t> biased = bitwise_xor(*this, Simd<uint32_t>(0x80000000U));
            Simd<float> as_signed_float;
            if constexpr (Bits == 256) as_signed_float.reg = _mm256_cvtepi32_ps(biased.reg);
            else                       as_signed_float.reg = _mm_cvtepi32_ps(biased.reg);
            s.reg = as_signed_float.reg + Simd<float>(2147483648.0f).reg;
            return s;
        }

        // Reinterpret casts – zero cost, no instruction emitted

        [[nodiscard]] Simd<int32_t> as_int32() const noexcept
            requires std::is_same_v<T, float>
        {
            Simd<int32_t> s;
            if constexpr (Bits == 256) s.reg = _mm256_castps_si256(reg);
            else                       s.reg = _mm_castps_si128(reg);
            return s;
        }

        [[nodiscard]] Simd<int32_t> as_int32() const noexcept
            requires std::is_same_v<T, double>
        {
            Simd<int32_t> s;
            if constexpr (Bits == 256) s.reg = _mm256_castpd_si256(reg);
            else                       s.reg = _mm_castpd_si128(reg);
            return s;
        }

        [[nodiscard]] Simd<uint32_t> as_uint32() const noexcept
            requires std::is_same_v<T, float>
        {
            Simd<uint32_t> s;
            if constexpr (Bits == 256) s.reg = _mm256_castps_si256(reg);
            else                       s.reg = _mm_castps_si128(reg);
            return s;
        }

        [[nodiscard]] Simd<uint32_t> as_uint32() const noexcept
            requires std::is_same_v<T, double>
        {
            Simd<uint32_t> s;
            if constexpr (Bits == 256) s.reg = _mm256_castpd_si256(reg);
            else                       s.reg = _mm_castpd_si128(reg);
            return s;
        }

        [[nodiscard]] Simd<int64_t> as_int64() const noexcept
            requires std::is_same_v<T, double>
        {
            Simd<int64_t> s;
            if constexpr (Bits == 256) s.reg = _mm256_castpd_si256(reg);
            else                       s.reg = _mm_castpd_si128(reg);
            return s;
        }

        [[nodiscard]] Simd<uint64_t> as_uint64() const noexcept
            requires std::is_same_v<T, double>
        {
            Simd<uint64_t> s;
            if constexpr (Bits == 256) s.reg = _mm256_castpd_si256(reg);
            else                       s.reg = _mm_castpd_si128(reg);
            return s;
        }

        [[nodiscard]] Simd<float> as_float() const noexcept
            requires (std::is_integral_v<T>)
        {
            Simd<float> s;
            if constexpr (Bits == 256) s.reg = _mm256_castsi256_ps(reg);
            else                       s.reg = _mm_castsi128_ps(reg);
            return s;
        }

        [[nodiscard]] Simd<double> as_double() const noexcept
            requires (std::is_integral_v<T>)
        {
            Simd<double> s;
            if constexpr (Bits == 256) s.reg = _mm256_castsi256_pd(reg);
            else                       s.reg = _mm_castsi128_pd(reg);
            return s;
        }
    };

    using SimdI = Simd<int32_t>;
    using SimdL = Simd<int64_t>;
    using SimdU = Simd<uint32_t>;
    using SimdUL = Simd<uint64_t>;
    using SimdF = Simd<float>;
    using SimdD = Simd<double>;

    using Simd128I = Simd<int32_t, 128>;
    using Simd128L = Simd<int64_t, 128>;
    using Simd128U = Simd<uint32_t, 128>;
    using Simd128UL = Simd<uint64_t, 128>;
    using Simd128F = Simd<float, 128>;
    using Simd128D = Simd<double, 128>;

    using Simd256I = Simd<int32_t, 256>;
    using Simd256L = Simd<int64_t, 256>;
    using Simd256U = Simd<uint32_t, 256>;
    using Simd256UL = Simd<uint64_t, 256>;
    using Simd256F = Simd<float, 256>;
    using Simd256D = Simd<double, 256>;
}
