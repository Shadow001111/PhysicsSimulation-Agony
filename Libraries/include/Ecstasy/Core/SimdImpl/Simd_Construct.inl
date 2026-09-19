// Part of Ecstasy::Core::Simd<T, Bits> — included inside the class body by Simd.h.
// Not a standalone header: do not #include this file directly.

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

