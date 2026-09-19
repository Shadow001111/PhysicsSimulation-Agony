// Part of Ecstasy::Core::Simd<T, Bits> — included inside the class body by Simd.h.
// Not a standalone header: do not #include this file directly.

        // Load.

        [[nodiscard]] static Simd load(const T* ptr) noexcept
        {
            Simd s;
            if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_load_si256(reinterpret_cast<const __m256i*>(ptr));
                else                       s.reg = _mm_load_si128(reinterpret_cast<const __m128i*>(ptr));
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_load_ps(ptr);
                else                       s.reg = _mm_load_ps(ptr);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_load_pd(ptr);
                else                       s.reg = _mm_load_pd(ptr);
            }
            return s;
        }

        [[nodiscard]] static Simd loadu(const T* ptr) noexcept
        {
            Simd s;
            if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(ptr));
                else                       s.reg = _mm_loadu_si128(reinterpret_cast<const __m128i*>(ptr));
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_loadu_ps(ptr);
                else                       s.reg = _mm_loadu_ps(ptr);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_loadu_pd(ptr);
                else                       s.reg = _mm_loadu_pd(ptr);
            }
            return s;
        }

        // Store.

        void store(T* ptr) const noexcept
        {
            if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) _mm256_store_si256(reinterpret_cast<__m256i*>(ptr), reg);
                else                       _mm_store_si128(reinterpret_cast<__m128i*>(ptr), reg);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) _mm256_store_ps(ptr, reg);
                else                       _mm_store_ps(ptr, reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) _mm256_store_pd(ptr, reg);
                else                       _mm_store_pd(ptr, reg);
            }
        }

        void storeu(T* ptr) const noexcept
        {
            if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) _mm256_storeu_si256(reinterpret_cast<__m256i*>(ptr), reg);
                else                       _mm_storeu_si128(reinterpret_cast<__m128i*>(ptr), reg);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) _mm256_storeu_ps(ptr, reg);
                else                       _mm_storeu_ps(ptr, reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) _mm256_storeu_pd(ptr, reg);
                else                       _mm_storeu_pd(ptr, reg);
            }
        }

        // Non-temporal streaming store (requires aligned pointer; must be 16-byte aligned for 128-bit, 32-byte for 256-bit).
        void streamStore(T* ptr) const noexcept
        {
            if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) _mm256_stream_si256(reinterpret_cast<__m256i*>(ptr), reg);
                else                       _mm_stream_si128(reinterpret_cast<__m128i*>(ptr), reg);
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) _mm256_stream_ps(ptr, reg);
                else                       _mm_stream_ps(ptr, reg);
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) _mm256_stream_pd(ptr, reg);
                else                       _mm_stream_pd(ptr, reg);
            }
        }

        // Non-temporal streaming load (requires aligned pointer; must be 16-byte aligned for 128-bit, 32-byte for 256-bit).
        [[nodiscard]] static Simd streamLoad(const T* ptr) noexcept
        {
            Simd s;
            if constexpr (IS_INTEGER)
            {
                if constexpr (Bits == 256) s.reg = _mm256_stream_load_si256(reinterpret_cast<const __m256i*>(ptr));
                else                       s.reg = _mm_stream_load_si128(reinterpret_cast<const __m128i*>(ptr));
            }
            else if constexpr (IS_FLOAT)
            {
                if constexpr (Bits == 256) s.reg = _mm256_castsi256_ps(_mm256_stream_load_si256(reinterpret_cast<const __m256i*>(ptr)));
                else                       s.reg = _mm_castsi128_ps(_mm_stream_load_si128(reinterpret_cast<const __m128i*>(ptr)));
            }
            else if constexpr (IS_DOUBLE)
            {
                if constexpr (Bits == 256) s.reg = _mm256_castsi256_pd(_mm256_stream_load_si256(reinterpret_cast<const __m256i*>(ptr)));
                else                       s.reg = _mm_castsi128_pd(_mm_stream_load_si128(reinterpret_cast<const __m128i*>(ptr)));
            }
            return s;
        }

        void storeLowerInt64(T* ptr) const noexcept
            requires (IS_INT32&& Bits == 128)
        {
            _mm_storel_epi64(reinterpret_cast<__m128i*>(ptr), reg);
        }

