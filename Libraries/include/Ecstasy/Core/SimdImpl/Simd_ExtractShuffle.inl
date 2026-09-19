// Part of Ecstasy::Core::Simd<T, Bits> — included inside the class body by Simd.h.
// Not a standalone header: do not #include this file directly.

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
            requires (IS_INT32&& Bits == 128)
        {
            Simd<T, 128> s;
            s.reg = _mm_packs_epi16(low.reg, high.reg);
            return s;
        }

        [[nodiscard]] static Simd<T, 128> narrowSaturate32To16(const Simd& low, const Simd& high) noexcept
            requires (IS_INT32&& Bits == 128)
        {
            Simd<T, 128> s;
            s.reg = _mm_packs_epi32(low.reg, high.reg);
            return s;
        }

