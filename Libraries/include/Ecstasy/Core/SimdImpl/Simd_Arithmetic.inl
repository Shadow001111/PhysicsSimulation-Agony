// Part of Ecstasy::Core::Simd<T, Bits> — included inside the class body by Simd.h.
// Not a standalone header: do not #include this file directly.

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
            #if ECSTASY_HAS_FMA
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
            #else
            return a * b + c;
            #endif
        }

        // a * b - c
        [[nodiscard]] static Simd mulSub(const Simd& a, const Simd& b, const Simd& c) noexcept
            requires IS_REAL
        {
            #if ECSTASY_HAS_FMA
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
            #else
            return a * b - c;
            #endif
        }

        // -(a * b) + c
        // c - a * b
        [[nodiscard]] static Simd negMulAdd(const Simd& a, const Simd& b, const Simd& c) noexcept
            requires IS_REAL
        {
            #if ECSTASY_HAS_FMA
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
            #else
            return c - a * b;
            #endif
        }

        // -(a * b) - c
        // -c - a * b
        [[nodiscard]] static Simd negMulSub(const Simd& a, const Simd& b, const Simd& c) noexcept
            requires IS_REAL
        {
            #if ECSTASY_HAS_FMA
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
            #else
            return -c - a * b;
            #endif
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

