// Part of Ecstasy::Core::Simd<T, Bits> — included inside the class body by Simd.h.
// Not a standalone header: do not #include this file directly.

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

