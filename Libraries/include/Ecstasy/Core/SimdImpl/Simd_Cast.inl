// Part of Ecstasy::Core::Simd<T, Bits> — included inside the class body by Simd.h.
// Not a standalone header: do not #include this file directly.

        // Type casts.

        template<typename Target>
        [[nodiscard]] Target to() const noexcept
            requires std::is_same_v<Target, Simd<int32_t, Bits>>&& IS_FLOAT
        {
            Target s;
            if constexpr (Bits == 256) s.reg = _mm256_cvttps_epi32(reg);
            else                       s.reg = _mm_cvttps_epi32(reg);
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target to() const noexcept
            requires (std::is_same_v<Target, Simd<int32_t, 128>>&& IS_DOUBLE&& Bits == 256)
        {
            Target s;
            s.reg = _mm256_cvttpd_epi32(reg);
            //if constexpr (Bits == 256) s.reg = _mm256_cvttpd_epi32(reg);
            //else                       s.reg = _mm_cvttpd_epi32(reg);
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target to() const noexcept
            requires std::is_same_v<Target, Simd<uint32_t, Bits>>&& IS_FLOAT
        {
            Target s;
            if constexpr (Bits == 128)
            {
                const __m128  bias_f = _mm_set1_ps(2147483648.0f);
                const __m128i bias_i = _mm_set1_epi32(0x80000000u);

                const __m128  mask = _mm_cmpge_ps(reg, bias_f);
                const __m128  adjusted = _mm_sub_ps(reg, _mm_and_ps(mask, bias_f));
                const __m128i signed_i = _mm_cvttps_epi32(adjusted);
                s.reg = _mm_xor_si128(signed_i, _mm_and_si128(_mm_castps_si128(mask), bias_i));
            }
            else
            {
                const __m256  bias_f = _mm256_set1_ps(2147483648.0f);
                const __m256i bias_i = _mm256_set1_epi32(0x80000000u);

                const __m256  mask = _mm256_cmp_ps(reg, bias_f, _CMP_GE_OQ);
                const __m256  adjusted = _mm256_sub_ps(reg, _mm256_and_ps(mask, bias_f));
                const __m256i signed_i = _mm256_cvttps_epi32(adjusted);
                s.reg = _mm256_xor_si256(signed_i, _mm256_and_si256(_mm256_castps_si256(mask), bias_i));
            }
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target to() const noexcept
            requires std::is_same_v<Target, Simd<float, Bits>>&& IS_INT32
        {
            Target s;
            if constexpr (Bits == 256) s.reg = _mm256_cvtepi32_ps(reg);
            else                       s.reg = _mm_cvtepi32_ps(reg);
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target to() const noexcept
            requires std::is_same_v<Target, Simd<float, Bits>>&& IS_UINT32
        {
            Simd<uint32_t, Bits> biased = bitwiseXor(*this, Simd<uint32_t, Bits>(0x80000000u));
            Target as_signed_float;
            if constexpr (Bits == 256) as_signed_float.reg = _mm256_cvtepi32_ps(biased.reg);
            else                       as_signed_float.reg = _mm_cvtepi32_ps(biased.reg);
            Target s;
            s.reg = as_signed_float.reg + Simd<float, Bits>(2147483648.0f).reg;
            return s;
        }

        template<typename Target>
        [[nodiscard]] Target to() const noexcept
            requires std::is_same_v<Target, Simd<double, Bits>>&& IS_INT64
        {
            const Target magic(0x0018000000000000);
            const Simd biased = (*this) + magic.template as<Simd>();
            return biased.as<Target>() - magic;
        }

        template<typename Target>
        [[nodiscard]] Target to() const noexcept
            requires std::is_same_v<Target, Simd<double, Bits>>&& IS_UINT64
        {
            const Target magic(0x0010000000000000);
            const Simd united = (*this) | magic.template as<Simd>();
            return united.as<Target>() - magic;
        }

        [[nodiscard]] Simd<int64_t, Bits> realToSmallNonNegativeInteger() const noexcept
            requires IS_DOUBLE
        {
            using Target = Simd<int64_t, Bits>;

            const Target magicBits = Target::fillLanesWith(0x4330000000000000LL);
            const Simd magic(4503599627370496.0);

            const Simd truncated = Simd::roundTowardsZero(*this);
            const Target biased = (truncated + magic).template as<Target>();
            return biased - magicBits;
        }

        // This one uses built-in conversion.
        [[nodiscard]] Simd<int32_t, Bits> realToSmallNonNegativeInteger() const noexcept
            requires IS_FLOAT
        {
            using Target = Simd<int32_t, Bits>;
            return this->to<Target>();
        }

