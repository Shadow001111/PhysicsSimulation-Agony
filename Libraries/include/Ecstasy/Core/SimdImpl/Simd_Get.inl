// Part of Ecstasy::Core::Simd<T, Bits> — included inside the class body by Simd.h.
// Not a standalone header: do not #include this file directly.

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

