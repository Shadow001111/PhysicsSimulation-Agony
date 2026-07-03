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

    // Allowed element types.
    template<typename T>
    concept SimdElement =
        std::is_same_v<T, int32_t>  || std::is_same_v<T, int64_t>  ||
        std::is_same_v<T, uint32_t> || std::is_same_v<T, uint64_t> ||
        std::is_same_v<T, float>    || std::is_same_v<T, double>;

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

    // Simd class
    template<SimdElement T, size_t Bits = kSimdBits>
    struct Simd
    {
        static_assert(Bits == 128 || Bits == 256, "Invalid bit count");
        #if !defined(SIMD_AVX2)
        static_assert(Bits < 256, "256-bit SIMD requires AVX2");
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

        // Construction.

        [[nodiscard]] static Simd fillLanesWith(const T& val) noexcept
        {
            Simd s;
            if constexpr (IS_INT32)
            {
                if constexpr (Bits == 256) s.reg = _mm256_set1_epi32(static_cast<int32_t>(val));
                else                       s.reg = _mm_set1_epi32(static_cast<int32_t>(val));
            }
            else if constexpr (IS_INT64)
            {
                if constexpr (Bits == 256) s.reg = _mm256_set1_epi64x(static_cast<int64_t>(val));
                else                       s.reg = _mm_set1_epi64x(static_cast<int64_t>(val));
            }
            else if constexpr (IS_UINT32)
            {
                if constexpr (Bits == 256) s.reg = _mm256_set1_epi32(static_cast<int32_t>(val));
                else                       s.reg = _mm_set1_epi32(static_cast<int32_t>(val));
            }
            else if constexpr (IS_UINT64)
            {
                if constexpr (Bits == 256) s.reg = _mm256_set1_epi64x(static_cast<int64_t>(val));
                else                       s.reg = _mm_set1_epi64x(static_cast<int64_t>(val));
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_set1_ps(static_cast<float>(val));
                else                       s.reg = _mm_set1_ps(static_cast<float>(val));
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_set1_pd(static_cast<double>(val));
                else                       s.reg = _mm_set1_pd(static_cast<double>(val));
            }
            return s;
        }

        [[nodiscard]] static Simd fillLanesWithZero() noexcept
        {
            Simd s;
            if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_setzero_si256();
                else                       s.reg = _mm_setzero_si128();
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_setzero_ps();
                else                       s.reg = _mm_setzero_ps();
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_setzero_pd();
                else                       s.reg = _mm_setzero_pd();
            }
            return s;
        }

        [[nodiscard]] static Simd fillLanesWithFullValue() noexcept
        {
            if constexpr (IS_INTEGER)
            {
                return fillLanesWith(static_cast<T>(-1));
            }
            else if constexpr (IS_FLOAT)
            {
                return Simd<int32_t, Bits>::fillLanesWith(static_cast<int32_t>(-1)).template as<Simd<float, Bits>>();
            }
            else if constexpr (IS_DOUBLE)
            {
                return Simd<int64_t, Bits>::fillLanesWith(static_cast<int64_t>(-1)).template as<Simd<double, Bits>>();
            }
        }

        Simd() = default;

        explicit Simd(const T& val) noexcept
        {
            *this = fillLanesWith(val);
        }

        // Loads in reverse order (first argument becomes highest lane).
        template<typename... Args>
        [[nodiscard]] static Simd set(Args... vals) noexcept
        {
            static_assert(sizeof...(vals) == lanes);
            static_assert((std::is_convertible_v<Args, T> && ...));

            Simd s;
            if constexpr (IS_32BIT_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_set_epi32(static_cast<int32_t>(vals)...);
                else                       s.reg = _mm_set_epi32(static_cast<int32_t>(vals)...);
            }
            else if constexpr (IS_64BIT_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_set_epi64x(static_cast<int64_t>(vals)...);
                else                       s.reg = _mm_set_epi64x(static_cast<int64_t>(vals)...);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_set_ps(static_cast<float>(vals)...);
                else                       s.reg = _mm_set_ps(static_cast<float>(vals)...);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_set_pd(static_cast<double>(vals)...);
                else                       s.reg = _mm_set_pd(static_cast<double>(vals)...);
            }
            return s;
        }

        // Load.

        [[nodiscard]] static Simd load(const T* ptr) noexcept
        {
            Simd s;
            if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_load_si256(reinterpret_cast<const __m256i*>(ptr));
                else                       s.reg = _mm_load_si128(reinterpret_cast<const __m128i*>(ptr));
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_load_ps(ptr);
                else                       s.reg = _mm_load_ps(ptr);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_load_pd(ptr);
                else                       s.reg = _mm_load_pd(ptr);
            }
            return s;
        }

        [[nodiscard]] static Simd loadu(const T* ptr) noexcept
        {
            Simd s;
            if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(ptr));
                else                       s.reg = _mm_loadu_si128(reinterpret_cast<const __m128i*>(ptr));
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_loadu_ps(ptr);
                else                       s.reg = _mm_loadu_ps(ptr);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_loadu_pd(ptr);
                else                       s.reg = _mm_loadu_pd(ptr);
            }
            return s;
        }

        // Store.

        void store(T* ptr) const noexcept
        {
            if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) _mm256_store_si256(reinterpret_cast<__m256i*>(ptr), reg);
                else                       _mm_store_si128(reinterpret_cast<__m128i*>(ptr), reg);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) _mm256_store_ps(ptr, reg);
                else                       _mm_store_ps(ptr, reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) _mm256_store_pd(ptr, reg);
                else                       _mm_store_pd(ptr, reg);
            }
        }

        void storeu(T* ptr) const noexcept
        {
            if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) _mm256_storeu_si256(reinterpret_cast<__m256i*>(ptr), reg);
                else                       _mm_storeu_si128(reinterpret_cast<__m128i*>(ptr), reg);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) _mm256_storeu_ps(ptr, reg);
                else                       _mm_storeu_ps(ptr, reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) _mm256_storeu_pd(ptr, reg);
                else                       _mm_storeu_pd(ptr, reg);
            }
        }

        // Non-temporal streaming store (requires aligned pointer; must be 16-byte aligned for 128-bit, 32-byte for 256-bit).
        void streamStore(T* ptr) const noexcept
        {
            if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) _mm256_stream_si256(reinterpret_cast<__m256i*>(ptr), reg);
                else                       _mm_stream_si128(reinterpret_cast<__m128i*>(ptr), reg);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) _mm256_stream_ps(ptr, reg);
                else                       _mm_stream_ps(ptr, reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) _mm256_stream_pd(ptr, reg);
                else                       _mm_stream_pd(ptr, reg);
            }
        }

        // Non-temporal streaming load (requires aligned pointer; must be 16-byte aligned for 128-bit, 32-byte for 256-bit).
        [[nodiscard]] static Simd streamLoad(const T* ptr) noexcept
        {
            Simd s;
            if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_stream_load_si256(reinterpret_cast<const __m256i*>(ptr));
                else                       s.reg = _mm_stream_load_si128(reinterpret_cast<const __m128i*>(ptr));
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_castsi256_ps(_mm256_stream_load_si256(reinterpret_cast<const __m256i*>(ptr)));
                else                       s.reg = _mm_castsi128_ps(_mm_stream_load_si128(reinterpret_cast<const __m128i*>(ptr)));
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_castsi256_pd(_mm256_stream_load_si256(reinterpret_cast<const __m256i*>(ptr)));
                else                       s.reg = _mm_castsi128_pd(_mm_stream_load_si128(reinterpret_cast<const __m128i*>(ptr)));
            }
            return s;
        }

        void storeLowerInt64(T* ptr) const noexcept
            requires (IS_INT32 && Bits == 128)
        {
            _mm_storel_epi64(reinterpret_cast<__m128i*>(ptr), reg);
        }

        // Gather.

        [[nodiscard]] static constexpr bool isGatherAvailable() noexcept
        {
            #ifdef SIMD_AVX2
            return true;
            #else
            return false;
            #endif
        }

        template<size_t IdxBits>
        [[nodiscard]] static Simd gather(const T* ptr, const Simd<int32_t, IdxBits>& indices) noexcept
        {
            using IdxT = int32_t;

            static_assert(Simd<IdxT, IdxBits>::lanes >= lanes, "Index vector does not contain enough lanes.");

            if constexpr (isGatherAvailable())
            {
                // AVX2 gather intrinsics expect a 128-bit index register for 64-bit element gathers 
                // (double/int64) or 128-bit widths, and a 256-bit index register for 256-bit 32-bit element gathers.
                const auto idxReg = [&]() {
                    if constexpr (Bits == 256 && sizeof(T) == 4)
                    {
                        static_assert(IdxBits == 256, "256-bit float/int32 gather needs 256-bit indices.");
                        return indices.reg; // __m256i
                    }
                    else
                    {
                        if constexpr (IdxBits == 256)
                            return _mm256_castsi256_si128(indices.reg); // __m128i
                        else
                            return indices.reg; // __m128i
                    }
                }();

                Simd s;
                if constexpr (IS_FLOAT)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_i32gather_ps(ptr, idxReg, sizeof(T));
                    else                       s.reg = _mm_i32gather_ps(ptr, idxReg, sizeof(T));
                }
                else if constexpr (IS_DOUBLE)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_i32gather_pd(ptr, idxReg, sizeof(T));
                    else                       s.reg = _mm_i32gather_pd(ptr, idxReg, sizeof(T));
                }
                else if constexpr (IS_32BIT_INTEGER)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_i32gather_epi32(reinterpret_cast<const int*>(ptr), idxReg, sizeof(T));
                    else                       s.reg = _mm_i32gather_epi32(reinterpret_cast<const int*>(ptr), idxReg, sizeof(T));
                }
                else if constexpr (IS_64BIT_INTEGER)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_i32gather_epi64(reinterpret_cast<const long long*>(ptr), idxReg, sizeof(T));
                    else                       s.reg = _mm_i32gather_epi64(reinterpret_cast<const long long*>(ptr), idxReg, sizeof(T));
                }
                return s;
            }
            else
            {
                // Fallback.
                alignas(Simd<IdxT, IdxBits>::bytes) IdxT idxBuffer[Simd<IdxT, IdxBits>::lanes];
                indices.store(idxBuffer);

                alignas(bytes) T buffer[lanes];
                for (size_t j = 0; j < lanes; j++)
                {
                    buffer[j] = ptr[idxBuffer[j]];
                }
                return load(buffer);
            }
        }

        // Get.

        [[nodiscard]] T getLowest() const noexcept
        {
            if constexpr (IS_32BIT_INTEGER)
            {
                if constexpr (Bits == 256) return _mm256_cvtsi256_si32(reg);
                else                       return _mm_cvtsi128_si32(reg);
            }
            else if constexpr (IS_64BIT_INTEGER)
            {
                if constexpr (Bits == 256) return _mm256_cvtsi256_si64(reg);
                else                       return _mm_cvtsi128_si64(reg);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) return _mm256_cvtss_f32(reg);
                else                       return _mm_cvtss_f32(reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) return _mm256_cvtsd_f64(reg);
                else                       return _mm_cvtsd_f64(reg);
            }
        }

        // Bitwise operations.

        [[nodiscard]] static Simd bitwiseAnd(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_and_si256(a.reg, b.reg);
                else                       s.reg = _mm_and_si128(a.reg, b.reg);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_and_ps(a.reg, b.reg);
                else                       s.reg = _mm_and_ps(a.reg, b.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_and_pd(a.reg, b.reg);
                else                       s.reg = _mm_and_pd(a.reg, b.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd bitwiseOr(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_or_si256(a.reg, b.reg);
                else                       s.reg = _mm_or_si128(a.reg, b.reg);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_or_ps(a.reg, b.reg);
                else                       s.reg = _mm_or_ps(a.reg, b.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_or_pd(a.reg, b.reg);
                else                       s.reg = _mm_or_pd(a.reg, b.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd bitwiseXor(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_xor_si256(a.reg, b.reg);
                else                       s.reg = _mm_xor_si128(a.reg, b.reg);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_xor_ps(a.reg, b.reg);
                else                       s.reg = _mm_xor_ps(a.reg, b.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_xor_pd(a.reg, b.reg);
                else                       s.reg = _mm_xor_pd(a.reg, b.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd bitwiseNot(const Simd& a) noexcept
        {
            return bitwiseXor(a, fillLanesWithFullValue());
        }

        // andnot(a, b) = (~a) & b
        [[nodiscard]] static Simd bitwiseAndnot(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_andnot_si256(a.reg, b.reg);
                else                       s.reg = _mm_andnot_si128(a.reg, b.reg);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_andnot_ps(a.reg, b.reg);
                else                       s.reg = _mm_andnot_ps(a.reg, b.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_andnot_pd(a.reg, b.reg);
                else                       s.reg = _mm_andnot_pd(a.reg, b.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd logicalShiftLeft(const Simd& a, int count) noexcept
            requires IS_INTEGER
        {
            Simd s;
            if constexpr (IS_32BIT_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_slli_epi32(a.reg, count);
                else                       s.reg = _mm_slli_epi32(a.reg, count);
            }
            else if constexpr (IS_64BIT_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_slli_epi64(a.reg, count);
                else                       s.reg = _mm_slli_epi64(a.reg, count);
            }
            return s;
        }

        [[nodiscard]] static Simd logicalShiftRight(const Simd& a, int count) noexcept
            requires IS_INTEGER
        {
            Simd s;
            if constexpr (IS_32BIT_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_srli_epi32(a.reg, count);
                else                       s.reg = _mm_srli_epi32(a.reg, count);
            }
            else if constexpr (IS_64BIT_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_srli_epi64(a.reg, count);
                else                       s.reg = _mm_srli_epi64(a.reg, count);
            }
            return s;
        }

        [[nodiscard]] static Simd arithmeticShiftRight(const Simd& a, int count) noexcept
            requires IS_INTEGER
        {
            if constexpr (IS_UNSIGNED_INTEGER)
            {
                // For unsigned, arithmetic right shift is same as logical shift.
                return logicalShiftRight(a, count);
            }
            else // Signed.
            {
                if constexpr (IS_INT32)
                {
                    Simd s;
                    if constexpr (Bits == 256) s.reg = _mm256_srai_epi32(a.reg, count);
                    else                       s.reg = _mm_srai_epi32(a.reg, count);
                    return s;
                }
                else // int64_t.
                {
                    // No native 64-bit arithmetic shift; emulate via sign + logical shift. (Appears only in avx512).
                    if (count == 0) return a;
                    Simd sign = logicalShiftRight(a, 63);
                    Simd logical = logicalShiftRight(a, count);
                    Simd high_mask = logicalShiftLeft(sign, 64 - count);
                    return bitwiseOr(logical, high_mask);
                }
            }
        }

        [[nodiscard]] Simd operator&(const Simd& other) const noexcept { return bitwiseAnd(*this, other); }
        [[nodiscard]] Simd operator|(const Simd& other) const noexcept { return bitwiseOr(*this, other); }
        [[nodiscard]] Simd operator^(const Simd& other) const noexcept { return bitwiseXor(*this, other); }
        [[nodiscard]] Simd operator~()                  const noexcept { return bitwiseNot(*this); }
        [[nodiscard]] Simd operator<<(int count) const noexcept
            requires IS_INTEGER
        {
            return logicalShiftLeft(*this, count);
        }
        [[nodiscard]] Simd operator>>(int count) const noexcept
            requires IS_INTEGER
        {
            return logicalShiftRight(*this, count);
        }

        Simd& operator&=(const Simd& other) noexcept { *this = bitwiseAnd(*this, other); return *this; }
        Simd& operator|=(const Simd& other) noexcept { *this = bitwiseOr(*this, other);  return *this; }
        Simd& operator^=(const Simd& other) noexcept { *this = bitwiseXor(*this, other); return *this; }
        Simd& operator<<=(int count) noexcept
            requires IS_INTEGER
        {
            *this = logicalShiftLeft(*this, count);  return *this;
        }
        Simd& operator>>=(int32_t count) noexcept
            requires IS_INTEGER
        {
            *this = logicalShiftRight(*this, count); return *this;
        }

        // Arithmetic.

        [[nodiscard]] static Simd add(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (IS_32BIT_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_add_epi32(a.reg, b.reg);
                else                       s.reg = _mm_add_epi32(a.reg, b.reg);
            }
            else if constexpr (IS_64BIT_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_add_epi64(a.reg, b.reg);
                else                       s.reg = _mm_add_epi64(a.reg, b.reg);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_add_ps(a.reg, b.reg);
                else                       s.reg = _mm_add_ps(a.reg, b.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_add_pd(a.reg, b.reg);
                else                       s.reg = _mm_add_pd(a.reg, b.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd sub(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (IS_32BIT_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_sub_epi32(a.reg, b.reg);
                else                       s.reg = _mm_sub_epi32(a.reg, b.reg);
            }
            else if constexpr (IS_64BIT_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_sub_epi64(a.reg, b.reg);
                else                       s.reg = _mm_sub_epi64(a.reg, b.reg);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_sub_ps(a.reg, b.reg);
                else                       s.reg = _mm_sub_ps(a.reg, b.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_sub_pd(a.reg, b.reg);
                else                       s.reg = _mm_sub_pd(a.reg, b.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd mul(const Simd& a, const Simd& b) noexcept
            requires (!IS_64BIT_INTEGER) // No 64-bit integer multiply in AVX2. Appears only in AVX512.
        {
            Simd s;
            if constexpr (IS_32BIT_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_mullo_epi32(a.reg, b.reg);
                else                       s.reg = _mm_mullo_epi32(a.reg, b.reg);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_mul_ps(a.reg, b.reg);
                else                       s.reg = _mm_mul_ps(a.reg, b.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_mul_pd(a.reg, b.reg);
                else                       s.reg = _mm_mul_pd(a.reg, b.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd div(const Simd& a, const Simd& b) noexcept
            requires IS_REAL
        {
            Simd s;
            if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_div_ps(a.reg, b.reg);
                else                       s.reg = _mm_div_ps(a.reg, b.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_div_pd(a.reg, b.reg);
                else                       s.reg = _mm_div_pd(a.reg, b.reg);
            }
            return s;
        }

        // a * b + c
        [[nodiscard]] static Simd mulAdd(const Simd& a, const Simd& b, const Simd& c) noexcept
            requires IS_REAL
        {
            Simd s;
            if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_fmadd_ps(a.reg, b.reg, c.reg);
                else                       s.reg = _mm_fmadd_ps(a.reg, b.reg, c.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_fmadd_pd(a.reg, b.reg, c.reg);
                else                       s.reg = _mm_fmadd_pd(a.reg, b.reg, c.reg);
            }
            return s;
        }

        // a * b - c
        [[nodiscard]] static Simd mulSub(const Simd& a, const Simd& b, const Simd& c) noexcept
            requires IS_REAL
        {
            Simd s;
            if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_fmsub_ps(a.reg, b.reg, c.reg);
                else                       s.reg = _mm_fmsub_ps(a.reg, b.reg, c.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_fmsub_pd(a.reg, b.reg, c.reg);
                else                       s.reg = _mm_fmsub_pd(a.reg, b.reg, c.reg);
            }
            return s;
        }

        // -(a * b) + c
        [[nodiscard]] static Simd negMulAdd(const Simd& a, const Simd& b, const Simd& c) noexcept
            requires IS_REAL
        {
            Simd s;
            if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_fnmadd_ps(a.reg, b.reg, c.reg);
                else                       s.reg = _mm_fnmadd_ps(a.reg, b.reg, c.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_fnmadd_pd(a.reg, b.reg, c.reg);
                else                       s.reg = _mm_fnmadd_pd(a.reg, b.reg, c.reg);
            }
            return s;
        }

        // -(a * b) - c
        [[nodiscard]] static Simd negMulSub(const Simd& a, const Simd& b, const Simd& c) noexcept
            requires IS_REAL
        {
            Simd s;
            if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_fnmsub_ps(a.reg, b.reg, c.reg);
                else                       s.reg = _mm_fnmsub_ps(a.reg, b.reg, c.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_fnmsub_pd(a.reg, b.reg, c.reg);
                else                       s.reg = _mm_fnmsub_pd(a.reg, b.reg, c.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd negate(const Simd& a) noexcept
        {
            if constexpr (IS_INTEGER)
            {
                return sub(fillLanesWithZero(), a);
            }
            else if constexpr (IS_FLOAT)
            {
                const Simd signMask = fillLanesWith(-0.0);
                return bitwiseXor(a, signMask);
            }
            else if constexpr (IS_DOUBLE)
            {
                const Simd signMask = fillLanesWith(-0.0);
                return bitwiseXor(a, signMask);
            }
        }

        [[nodiscard]] static Simd sqrt(const Simd& a) noexcept
            requires IS_REAL
        {
            Simd s;
            if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_sqrt_ps(a.reg);
                else                       s.reg = _mm_sqrt_ps(a.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_sqrt_pd(a.reg);
                else                       s.reg = _mm_sqrt_pd(a.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd getAbsMask() noexcept
            requires IS_REAL
        {
            Simd s;
            if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_castsi256_ps(_mm256_set1_epi32(0x7FFFFFFF));
                else                       s.reg = _mm_castsi128_ps(_mm_set1_epi32(0x7FFFFFFF));
            }
            else if constexpr (IS_DOUBLE)
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
            requires IS_REAL
        {
            return div(*this, other);
        }
        [[nodiscard]] Simd operator-() const noexcept { return negate(*this); }

        Simd& operator+=(const Simd& other) noexcept { *this = add(*this, other); return *this; }
        Simd& operator-=(const Simd& other) noexcept { *this = sub(*this, other); return *this; }
        Simd& operator*=(const Simd& other) noexcept { *this = mul(*this, other); return *this; }
        Simd& operator/=(const Simd& other) noexcept
            requires IS_REAL
        {
            *this = div(*this, other); return *this;
        }

        // Rounding.

        [[nodiscard]] static Simd roundToNearest(const Simd& a) noexcept
            requires IS_REAL
        {
            Simd s;
            constexpr int kNearest = _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC; // Round to nearest, but don't raise exceptions on invalid input (e.g. NaN).
            if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_round_ps(a.reg, kNearest);
                else                       s.reg = _mm_round_ps(a.reg, kNearest);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_round_pd(a.reg, kNearest);
                else                       s.reg = _mm_round_pd(a.reg, kNearest);
            }
            return s;
        }

        [[nodiscard]] static Simd roundTowardsZero(const Simd& a) noexcept
            requires IS_REAL
        {
            Simd s;
            constexpr int kNearest = _MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC; // Round towards zero, but don't raise exceptions on invalid input (e.g. NaN).
            if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_round_ps(a.reg, kNearest);
                else                       s.reg = _mm_round_ps(a.reg, kNearest);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_round_pd(a.reg, kNearest);
                else                       s.reg = _mm_round_pd(a.reg, kNearest);
            }
            return s;
        }

        [[nodiscard]] static Simd floor(const Simd& a) noexcept
            requires IS_REAL
        {
            Simd s;
            if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_floor_ps(a.reg);
                else                       s.reg = _mm_floor_ps(a.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_floor_pd(a.reg);
                else                       s.reg = _mm_floor_pd(a.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd ceil(const Simd& a) noexcept
            requires IS_REAL
        {
            Simd s;
            if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_ceil_ps(a.reg);
                else                       s.reg = _mm_ceil_ps(a.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_ceil_pd(a.reg);
                else                       s.reg = _mm_ceil_pd(a.reg);
            }
            return s;
        }

        // Comparisons - return a lane mask (0xFFFFFFFF... or 0x0 per lane).

        [[nodiscard]] static Simd compareEqual(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (IS_32BIT_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmpeq_epi32(a.reg, b.reg);
                else                       s.reg = _mm_cmpeq_epi32(a.reg, b.reg);
            }
            else if constexpr (IS_64BIT_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmpeq_epi64(a.reg, b.reg);
                else                       s.reg = _mm_cmpeq_epi64(a.reg, b.reg);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmp_ps(a.reg, b.reg, _CMP_EQ_OQ);
                else                       s.reg = _mm_cmpeq_ps(a.reg, b.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmp_pd(a.reg, b.reg, _CMP_EQ_OQ);
                else                       s.reg = _mm_cmpeq_pd(a.reg, b.reg);
            }
            return s;
        }

        [[nodiscard]] static Simd compareNotEqual(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmp_ps(a.reg, b.reg, _CMP_NEQ_OQ);
                else                       s.reg = _mm_cmpneq_ps(a.reg, b.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmp_pd(a.reg, b.reg, _CMP_NEQ_OQ);
                else                       s.reg = _mm_cmpneq_pd(a.reg, b.reg);
            }
            else if constexpr (IS_INTEGER)
            {
                Simd eq = compareEqual(a, b);
                Simd full = fillLanesWithFullValue();
                s = bitwiseXor(eq, full);
            }
            return s;
        }

        [[nodiscard]] static Simd compareLess(const Simd& a, const Simd& b) noexcept
        {
            Simd s;

            if constexpr (IS_UNSIGNED_INTEGER)
            {
                // a < b  ->  (a ^ signBit) < (b ^ signBit) as signed.
                constexpr T signBit = (T(1) << (sizeof(T) * 8 - 1));
                Simd a_xor = Simd::bitwiseXor(a, Simd(signBit));
                Simd b_xor = Simd::bitwiseXor(b, Simd(signBit));
                if constexpr (IS_UINT32)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_cmpgt_epi32(b_xor.reg, a_xor.reg);
                    else                       s.reg = _mm_cmpgt_epi32(b_xor.reg, a_xor.reg);
                }
                else if constexpr (IS_UINT64)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_cmpgt_epi64(b_xor.reg, a_xor.reg);
                    else                       s.reg = _mm_cmpgt_epi64(b_xor.reg, a_xor.reg);
                }
            }
            else
            {
                if constexpr (IS_INT32)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_cmpgt_epi32(b.reg, a.reg);
                    else                       s.reg = _mm_cmpgt_epi32(b.reg, a.reg);
                }
                else if constexpr (IS_INT64)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_cmpgt_epi64(b.reg, a.reg);
                    else                       s.reg = _mm_cmpgt_epi64(b.reg, a.reg);
                }
                else if constexpr (IS_FLOAT)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_cmp_ps(a.reg, b.reg, _CMP_LT_OQ);
                    else                       s.reg = _mm_cmplt_ps(a.reg, b.reg);
                }
                else if constexpr (IS_DOUBLE)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_cmp_pd(a.reg, b.reg, _CMP_LT_OQ);
                    else                       s.reg = _mm_cmplt_pd(a.reg, b.reg);
                }
            }
            return s;
        }

        [[nodiscard]] static Simd compareGreater(const Simd& a, const Simd& b) noexcept
        {
            return compareLess(b, a);
        }

        [[nodiscard]] static Simd compareLessEqual(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmp_ps(a.reg, b.reg, _CMP_LE_OQ);
                else                       s.reg = _mm_cmple_ps(a.reg, b.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmp_pd(a.reg, b.reg, _CMP_LE_OQ);
                else                       s.reg = _mm_cmple_pd(a.reg, b.reg);
            }
            else if constexpr (IS_INTEGER)
            {
                Simd gt = compareGreater(a, b);
                Simd full = fillLanesWithFullValue();
                s = bitwiseXor(gt, full);
            }
            return s;
        }

        [[nodiscard]] static Simd compareGreaterEqual(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmp_ps(a.reg, b.reg, _CMP_GE_OQ);
                else                       s.reg = _mm_cmpge_ps(a.reg, b.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_cmp_pd(a.reg, b.reg, _CMP_GE_OQ);
                else                       s.reg = _mm_cmpge_pd(a.reg, b.reg);
            }
            else if constexpr (IS_INTEGER)
            {
                Simd lt = compareLess(a, b);
                Simd full = fillLanesWithFullValue();
                s = bitwiseXor(lt, full);
            }
            return s;
        }

        [[nodiscard]] Simd operator==(const Simd& other) const noexcept { return compareEqual(*this, other); }
        [[nodiscard]] Simd operator!=(const Simd& other) const noexcept { return compareNotEqual(*this, other); }
        [[nodiscard]] Simd operator< (const Simd& other) const noexcept { return compareLess(*this, other); }
        [[nodiscard]] Simd operator<=(const Simd& other) const noexcept { return compareLessEqual(*this, other); }
        [[nodiscard]] Simd operator> (const Simd& other) const noexcept { return compareGreater(*this, other); }
        [[nodiscard]] Simd operator>=(const Simd& other) const noexcept { return compareGreaterEqual(*this, other); }

        // Movemask. Returns most significant bits, packed in integer.
        [[nodiscard]] int movemask() const noexcept
            requires IS_REAL
        {
            if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) return _mm256_movemask_ps(reg);
                else                       return _mm_movemask_ps(reg);
            }
            if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) return _mm256_movemask_pd(reg);
                else                       return _mm_movemask_pd(reg);
            }
        }

        // Blend. Value = mask ? b : a.
        [[nodiscard]] static Simd blendv(const Simd& a, const Simd& b, const Simd& mask) noexcept
        {
            Simd s;
            if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_blendv_ps(a.reg, b.reg, mask.reg);
                else                       s.reg = _mm_blendv_ps(a.reg, b.reg, mask.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_blendv_pd(a.reg, b.reg, mask.reg);
                else                       s.reg = _mm_blendv_pd(a.reg, b.reg, mask.reg);
            }
            else if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_blendv_epi8(a.reg, b.reg, mask.reg);
                else                       s.reg = _mm_blendv_epi8(a.reg, b.reg, mask.reg);
            }
            return s;
        }

        // Min / max / clamp.

        [[nodiscard]] static Simd min(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (IS_INT32)
            {
                if constexpr (Bits == 256) s.reg = _mm256_min_epi32(a.reg, b.reg);
                else                       s.reg = _mm_min_epi32(a.reg, b.reg);
            }
            else if constexpr (IS_UINT32)
            {
                if constexpr (Bits == 256) s.reg = _mm256_min_epu32(a.reg, b.reg);
                else                       s.reg = _mm_min_epu32(a.reg, b.reg);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_min_ps(a.reg, b.reg);
                else                       s.reg = _mm_min_ps(a.reg, b.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_min_pd(a.reg, b.reg);
                else                       s.reg = _mm_min_pd(a.reg, b.reg);
            }
            else if constexpr (IS_64BIT_INTEGER) // Real instructions appear only in AVX512.
            {
                return blendv(b, a, a < b);
            }
            return s;
        }

        [[nodiscard]] static Simd max(const Simd& a, const Simd& b) noexcept
        {
            Simd s;
            if constexpr (IS_INT32)
            {
                if constexpr (Bits == 256) s.reg = _mm256_max_epi32(a.reg, b.reg);
                else                       s.reg = _mm_max_epi32(a.reg, b.reg);
            }
            else if constexpr (IS_UINT32)
            {
                if constexpr (Bits == 256) s.reg = _mm256_max_epu32(a.reg, b.reg);
                else                       s.reg = _mm_max_epu32(a.reg, b.reg);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_max_ps(a.reg, b.reg);
                else                       s.reg = _mm_max_ps(a.reg, b.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_max_pd(a.reg, b.reg);
                else                       s.reg = _mm_max_pd(a.reg, b.reg);
            }
            else if constexpr (IS_64BIT_INTEGER) // Real instructions appear only in AVX512.
            {
                return blendv(b, a, a > b);
            }
            return s;
        }

        [[nodiscard]] static Simd clamp(const Simd& value, const Simd& minBoundary, const Simd& maxBoundary)
        {
            return min(maxBoundary, max(minBoundary, value));
        }

        // Extract.

        template<int Index>
        [[nodiscard]] static Simd<T, 128> extract128From256(const Simd& a) noexcept
            requires (Bits == 256)
        {
            static_assert(Index == 0 || Index == 1, "Index must be 0 or 1.");


            if constexpr (Index == 0)
            {
                return castFrom256to128(a);
            }

            Simd<T, 128> s;
            if constexpr (IS_FLOAT)
            {
                s.reg = _mm256_extractf128_ps(a.reg, Index);
            }
            else if constexpr (IS_DOUBLE)
            {
                s.reg = _mm256_extractf128_pd(a.reg, Index);
            }
            else
            {
                s.reg = _mm256_extracti128_si256(a.reg, Index);
            }

            return s;
        }

        template<int Imm>
        [[nodiscard]] static Simd shuffle(const Simd& a) noexcept
            requires (Bits == 128)
        {
            Simd s;

            if constexpr (IS_FLOAT)
            {
                s.reg = _mm_shuffle_ps(a.reg, a.reg, Imm);
            }
            else if constexpr (IS_DOUBLE)
            {
                s.reg = _mm_shuffle_pd(a.reg, a.reg, Imm);
            }
            else
            {
                s.reg = _mm_shuffle_epi32(a.reg, Imm);
            }

            return s;
        }

        template<int Imm>
        [[nodiscard]] static Simd shuffle(const Simd& a) noexcept
            requires (Bits == 256)
        {
            Simd s;

            if constexpr (IS_FLOAT)
            {
                s.reg = _mm256_shuffle_ps(a.reg, a.reg, Imm);
            }
            else if constexpr (IS_DOUBLE)
            {
                s.reg = _mm256_shuffle_pd(a.reg, a.reg, Imm);
            }
            else if constexpr (IS_32BIT_INTEGER)
            {
                s.reg = _mm256_shuffle_epi32(a.reg, Imm);
            }
            else if constexpr (IS_64BIT_INTEGER)
            {
                s.reg = _mm256_permute4x64_epi64(a.reg, Imm);
            }

            return s;
        }

        // Narrow-saturate (int32_t, 128-bit only).

        [[nodiscard]] static Simd<T, 128> narrowSaturate16To8(const Simd& low, const Simd& high) noexcept
            requires (IS_INT32 && Bits == 128)
        {
            Simd<T, 128> s;
            s.reg = _mm_packs_epi16(low.reg, high.reg);
            return s;
        }

        [[nodiscard]] static Simd<T, 128> narrowSaturate32To16(const Simd& low, const Simd& high) noexcept
            requires (IS_INT32 && Bits == 128)
        {
            Simd<T, 128> s;
            s.reg = _mm_packs_epi32(low.reg, high.reg);
            return s;
        }

        // Horizontal methods.
    private:
        template<typename Op>
        [[nodiscard]] static T reduce128ToScalar(const Simd<T, 128>& x, Op op) noexcept
        {
            Simd<T, 128> v = x;
            if constexpr (sizeof(T) == 4)
            {
                v = op(v, Simd<T, 128>::template shuffle<0xB1>(v));
                v = op(v, Simd<T, 128>::template shuffle<0x4E>(v));
            }
            else
            {
                v = op(v, Simd<T, 128>::template shuffle<0x1>(v));
            }
            return v.getLowest();
        }

        template<typename Op>
        [[nodiscard]] static T horizontalReduce(const Simd& a, Op op) noexcept
            requires std::is_invocable_r_v<Simd<T, 128>, Op, const Simd<T, 128>&, const Simd<T, 128>&>
        {
            if constexpr (Bits == 256)
            {
                // Split 256 on two 128.
                Simd<T, 128> lo = extract128From256<0>(a);
                Simd<T, 128> hi = extract128From256<1>(a);

                // Perform operation on two halves.
                Simd<T, 128> v = op(lo, hi);

                // Reduce 128 to scalar.
                return reduce128ToScalar(v, op);
            }
            else
            {
                // Reduce 128 to scalar.
                return reduce128ToScalar(a, op);
            }
        }
    public:
        [[nodiscard]] static T horizontalAdd(const Simd& a) noexcept
        {
            return horizontalReduce(a, &Simd<T, 128>::add);
        }

        [[nodiscard]] static T horizontalMin(const Simd& a) noexcept
        {
            return horizontalReduce(a, &Simd<T, 128>::min);
        }

        [[nodiscard]] static T horizontalMax(const Simd& a) noexcept
        {
            return horizontalReduce(a, &Simd<T, 128>::max);
        }

        // Type casts.

        template<typename Target>
        [[nodiscard]] Target to() const noexcept
            requires std::is_same_v<Target, Simd<int32_t, Bits>> && IS_FLOAT
        {
            Target s;
            if constexpr (Bits == 256) s.reg = _mm256_cvttps_epi32(reg);
            else                       s.reg = _mm_cvttps_epi32(reg);
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target to() const noexcept
            requires (std::is_same_v<Target, Simd<int32_t, 128>> && IS_DOUBLE && Bits == 256)
        {
            Target s;
            s.reg = _mm256_cvttpd_epi32(reg);
            //if constexpr (Bits == 256) s.reg = _mm256_cvttpd_epi32(reg);
            //else                       s.reg = _mm_cvttpd_epi32(reg);
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target to() const noexcept
            requires std::is_same_v<Target, Simd<uint32_t, Bits>> && IS_FLOAT
        {
            Target s;
            if constexpr (Bits == 128)
            {
                const __m128  bias_f = _mm_set1_ps(2147483648.0f);
                const __m128i bias_i = _mm_set1_epi32(0x80000000u);

                const __m128  mask = _mm_cmpge_ps(reg, bias_f);
                const __m128  adjusted = _mm_sub_ps(reg, _mm_and_ps(mask, bias_f));
                const __m128i signed_i = _mm_cvttps_epi32(adjusted);
                s.reg = _mm_xor_si128(signed_i, _mm_and_si128(_mm_castps_si128(mask), bias_i));
            }
            else
            {
                const __m256  bias_f = _mm256_set1_ps(2147483648.0f);
                const __m256i bias_i = _mm256_set1_epi32(0x80000000u);

                const __m256  mask = _mm256_cmp_ps(reg, bias_f, _CMP_GE_OQ);
                const __m256  adjusted = _mm256_sub_ps(reg, _mm256_and_ps(mask, bias_f));
                const __m256i signed_i = _mm256_cvttps_epi32(adjusted);
                s.reg = _mm256_xor_si256(signed_i, _mm256_and_si256(_mm256_castps_si256(mask), bias_i));
            }
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target to() const noexcept
            requires std::is_same_v<Target, Simd<float, Bits>> && IS_INT32
        {
            Target s;
            if constexpr (Bits == 256) s.reg = _mm256_cvtepi32_ps(reg);
            else                       s.reg = _mm_cvtepi32_ps(reg);
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target to() const noexcept
            requires std::is_same_v<Target, Simd<float, Bits>> && IS_UINT32
        {
            Simd<uint32_t, Bits> biased = bitwiseXor(*this, Simd<uint32_t, Bits>(0x80000000u));
            Target as_signed_float;
            if constexpr (Bits == 256) as_signed_float.reg = _mm256_cvtepi32_ps(biased.reg);
            else                       as_signed_float.reg = _mm_cvtepi32_ps(biased.reg);
            Target s;
            s.reg = as_signed_float.reg + Simd<float, Bits>(2147483648.0f).reg;
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target to() const noexcept
            requires std::is_same_v<Target, Simd<double, Bits>> && IS_INT64
        {
            const Target magic(0x0018000000000000);
            const Simd biased = (*this) + magic.template as<Simd>();
            return biased.as<Target>() - magic;
        }

        template<typename Target>
        [[nodiscard]] Target to() const noexcept
            requires std::is_same_v<Target, Simd<double, Bits>> && IS_UINT64
        {
            const Target magic(0x0010000000000000);
            const Simd united = (*this) | magic.template as<Simd>();
            return united.as<Target>() - magic;
        }

        [[nodiscard]] Simd<int64_t, Bits> realToSmallNonNegativeInteger() const noexcept
            requires IS_DOUBLE
        {
            using Target = Simd<int64_t, Bits>;

            const Target magicBits = Target::fillLanesWith(0x4330000000000000LL);
            const Simd magic(4503599627370496.0);
            
            const Simd truncated = Simd::roundTowardsZero(*this);
            const Target biased = (truncated + magic).template as<Target>();
            return biased - magicBits;
        }

        // This one uses built-in conversion.
        [[nodiscard]] Simd<int32_t, Bits> realToSmallNonNegativeInteger() const noexcept
            requires IS_FLOAT
        {
            using Target = Simd<int32_t, Bits>;
            return this->to<Target>();
        }

        // Reinterpret casts – zero cost, no instruction emitted.

        template<typename Target>
        [[nodiscard]] Target as() const noexcept
            requires std::is_same_v<Target, Simd<int32_t, Bits>> && IS_REAL
        {
            Target s;
            if constexpr (Bits == 256)
            {
                if constexpr (IS_FLOAT) s.reg = _mm256_castps_si256(reg);
                else                    s.reg = _mm256_castpd_si256(reg);
            }
            else
            {
                if constexpr (IS_FLOAT) s.reg = _mm_castps_si128(reg);
                else                    s.reg = _mm_castpd_si128(reg);
            }
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target as() const noexcept
            requires std::is_same_v<Target, Simd<uint32_t, Bits>> && IS_REAL
        {
            Target s;
            if constexpr (Bits == 256)
            {
                if constexpr (IS_FLOAT) s.reg = _mm256_castps_si256(reg);
                else                                    s.reg = _mm256_castpd_si256(reg);
            }
            else
            {
                if constexpr (IS_FLOAT) s.reg = _mm_castps_si128(reg);
                else                                    s.reg = _mm_castpd_si128(reg);
            }
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target as() const noexcept
            requires std::is_same_v<Target, Simd<int64_t, Bits>> && IS_DOUBLE
        {
            Target s;
            if constexpr (Bits == 256) s.reg = _mm256_castpd_si256(reg);
            else                       s.reg = _mm_castpd_si128(reg);
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target as() const noexcept
            requires std::is_same_v<Target, Simd<uint64_t, Bits>> && IS_DOUBLE
        {
            Target s;
            if constexpr (Bits == 256) s.reg = _mm256_castpd_si256(reg);
            else                       s.reg = _mm_castpd_si128(reg);
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target as() const noexcept
            requires std::is_same_v<Target, Simd<float, Bits>> && IS_INTEGER
        {
            Target s;
            if constexpr (Bits == 256) s.reg = _mm256_castsi256_ps(reg);
            else                       s.reg = _mm_castsi128_ps(reg);
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target as() const noexcept
            requires std::is_same_v<Target, Simd<double, Bits>> && IS_INTEGER
        {
            Target s;
            if constexpr (Bits == 256) s.reg = _mm256_castsi256_pd(reg);
            else                       s.reg = _mm_castsi128_pd(reg);
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target as() const noexcept
            requires (
                (std::is_same_v<Target, Simd<uint32_t, Bits>> && IS_INT32) ||
                (std::is_same_v<Target, Simd<int32_t, Bits>> && IS_UINT32) ||
                (std::is_same_v<Target, Simd<uint64_t, Bits>> && IS_INT64) ||
                (std::is_same_v<Target, Simd<int64_t, Bits>> && IS_UINT64)
            )
        {
            // Dummy function. Simd doesn't care if int is signed or unsigned.
            return *this;
        }

        [[nodiscard]] static Simd<T, 128> castFrom256to128(const Simd& a) noexcept
            requires (Bits == 256)
        {
            Simd<T, 128> s;

            if constexpr (IS_FLOAT)
            {
                s.reg = _mm256_castps256_ps128(a.reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                s.reg = _mm256_castpd256_pd128(a.reg);
            }
            else
            {
                s.reg = _mm256_castsi256_si128(a.reg);
            }

            return s;
        }
    };

    //using SimdI = Simd<int32_t>;
    //using SimdL = Simd<int64_t>;
    //using SimdU = Simd<uint32_t>;
    //using SimdUL = Simd<uint64_t>;
    //using SimdF = Simd<float>;
    //using SimdD = Simd<double>;
    //
    //using Simd128I = Simd<int32_t, 128>;
    //using Simd128L = Simd<int64_t, 128>;
    //using Simd128U = Simd<uint32_t, 128>;
    //using Simd128UL = Simd<uint64_t, 128>;
    //using Simd128F = Simd<float, 128>;
    //using Simd128D = Simd<double, 128>;
    //
    //using Simd256I = Simd<int32_t, 256>;
    //using Simd256L = Simd<int64_t, 256>;
    //using Simd256U = Simd<uint32_t, 256>;
    //using Simd256UL = Simd<uint64_t, 256>;
    //using Simd256F = Simd<float, 256>;
    //using Simd256D = Simd<double, 256>;
}
