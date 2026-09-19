// Part of Ecstasy::Core::Simd<T, Bits> — included inside the class body by Simd.h.
// Not a standalone header: do not #include this file directly.

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

