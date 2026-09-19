// Part of Ecstasy::Core::Simd<T, Bits> — included inside the class body by Simd.h.
// Not a standalone header: do not #include this file directly.

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

