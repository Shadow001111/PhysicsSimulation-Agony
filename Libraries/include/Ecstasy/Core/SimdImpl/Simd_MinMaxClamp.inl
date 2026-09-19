// Part of Ecstasy::Core::Simd<T, Bits> — included inside the class body by Simd.h.
// Not a standalone header: do not #include this file directly.

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

