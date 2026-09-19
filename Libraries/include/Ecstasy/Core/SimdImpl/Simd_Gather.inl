// Part of Ecstasy::Core::Simd<T, Bits> — included inside the class body by Simd.h.
// Not a standalone header: do not #include this file directly.

        // Gather.

        [[nodiscard]] static constexpr bool isGatherAvailable() noexcept
        {
            #ifdef ECSTASY_SIMD_AVX2
            return true;
            #else
            return false;
            #endif
        }

        template<size_t IdxBits>
        [[nodiscard]] static Simd gather(const T* ptr, const Simd<int32_t, IdxBits>& indices) noexcept
        {
            using IdxT = int32_t;

            static_assert(Simd<IdxT, IdxBits>::lanes >= lanes, "Index vector does not contain enough lanes.");

            if constexpr (isGatherAvailable())
            {
                // AVX2 gather intrinsics expect a 128-bit index register for 64-bit element gathers 
                // (double/int64) or 128-bit widths, and a 256-bit index register for 256-bit 32-bit element gathers.
                const auto idxReg = [&]() {
                    if constexpr (Bits == 256 && sizeof(T) == 4)
                    {
                        static_assert(IdxBits == 256, "256-bit float/int32 gather needs 256-bit indices.");
                        return indices.reg; // __m256i
                    }
                    else
                    {
                        if constexpr (IdxBits == 256)
                            return _mm256_castsi256_si128(indices.reg); // __m128i
                        else
                            return indices.reg; // __m128i
                    }
                    }();

                Simd s;
                if constexpr (IS_FLOAT)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_i32gather_ps(ptr, idxReg, sizeof(T));
                    else                       s.reg = _mm_i32gather_ps(ptr, idxReg, sizeof(T));
                }
                else if constexpr (IS_DOUBLE)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_i32gather_pd(ptr, idxReg, sizeof(T));
                    else                       s.reg = _mm_i32gather_pd(ptr, idxReg, sizeof(T));
                }
                else if constexpr (IS_32BIT_INTEGER)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_i32gather_epi32(reinterpret_cast<const int*>(ptr), idxReg, sizeof(T));
                    else                       s.reg = _mm_i32gather_epi32(reinterpret_cast<const int*>(ptr), idxReg, sizeof(T));
                }
                else if constexpr (IS_64BIT_INTEGER)
                {
                    if constexpr (Bits == 256) s.reg = _mm256_i32gather_epi64(reinterpret_cast<const long long*>(ptr), idxReg, sizeof(T));
                    else                       s.reg = _mm_i32gather_epi64(reinterpret_cast<const long long*>(ptr), idxReg, sizeof(T));
                }
                return s;
            }
            else
            {
                // Fallback.
                alignas(Simd<IdxT, IdxBits>::bytes) IdxT idxBuffer[Simd<IdxT, IdxBits>::lanes];
                indices.store(idxBuffer);

                alignas(bytes) T buffer[lanes];
                for (size_t j = 0; j < lanes; j++)
                {
                    buffer[j] = ptr[idxBuffer[j]];
                }
                return load(buffer);
            }
        }

