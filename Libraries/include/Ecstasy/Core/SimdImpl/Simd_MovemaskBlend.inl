// Part of Ecstasy::Core::Simd<T, Bits> — included inside the class body by Simd.h.
// Not a standalone header: do not #include this file directly.

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

