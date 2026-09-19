// Part of Ecstasy::Core::Simd<T, Bits> — included inside the class body by Simd.h.
// Not a standalone header: do not #include this file directly.

        // Reinterpret casts - zero cost, no instruction emitted.

        template<typename Target>
        [[nodiscard]] Target as() const noexcept
            requires std::is_same_v<Target, Simd<int32_t, Bits>>&& IS_REAL
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
            requires std::is_same_v<Target, Simd<uint32_t, Bits>>&& IS_REAL
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
            requires std::is_same_v<Target, Simd<int64_t, Bits>>&& IS_DOUBLE
        {
            Target s;
            if constexpr (Bits == 256) s.reg = _mm256_castpd_si256(reg);
            else                       s.reg = _mm_castpd_si128(reg);
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target as() const noexcept
            requires std::is_same_v<Target, Simd<uint64_t, Bits>>&& IS_DOUBLE
        {
            Target s;
            if constexpr (Bits == 256) s.reg = _mm256_castpd_si256(reg);
            else                       s.reg = _mm_castpd_si128(reg);
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target as() const noexcept
            requires std::is_same_v<Target, Simd<float, Bits>>&& IS_INTEGER
        {
            Target s;
            if constexpr (Bits == 256) s.reg = _mm256_castsi256_ps(reg);
            else                       s.reg = _mm_castsi128_ps(reg);
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target as() const noexcept
            requires std::is_same_v<Target, Simd<double, Bits>>&& IS_INTEGER
        {
            Target s;
            if constexpr (Bits == 256) s.reg = _mm256_castsi256_pd(reg);
            else                       s.reg = _mm_castsi128_pd(reg);
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target as() const noexcept
            requires (
        (std::is_same_v<Target, Simd<uint32_t, Bits>>&& IS_INT32) ||
            (std::is_same_v<Target, Simd<int32_t, Bits>> && IS_UINT32) ||
            (std::is_same_v<Target, Simd<uint64_t, Bits>> && IS_INT64) ||
            (std::is_same_v<Target, Simd<int64_t, Bits>> && IS_UINT64)
            )
        {
            Target s;
            s.reg = reg;
            return s;
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
