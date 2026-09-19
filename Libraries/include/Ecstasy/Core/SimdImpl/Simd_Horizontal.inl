// Part of Ecstasy::Core::Simd<T, Bits> — included inside the class body by Simd.h.
// Not a standalone header: do not #include this file directly.

        // Horizontal methods.
    private:
        template<typename Op>
        [[nodiscard]] static T reduce128ToScalar(const Simd<T, 128>& x, Op op) noexcept
        {
            Simd<T, 128> v = x;
            if constexpr (sizeof(T) == 4)
            {
                v = op(v, Simd<T, 128>::template shuffle<0xB1>(v));
                v = op(v, Simd<T, 128>::template shuffle<0x4E>(v));
            }
            else
            {
                v = op(v, Simd<T, 128>::template shuffle<0x1>(v));
            }
            return v.getLowest();
        }

        template<typename Op>
        [[nodiscard]] static T horizontalReduce(const Simd& a, Op op) noexcept
            requires std::is_invocable_r_v<Simd<T, 128>, Op, const Simd<T, 128>&, const Simd<T, 128>&>
        {
            if constexpr (Bits == 256)
            {
                // Split 256 on two 128.
                Simd<T, 128> lo = extract128From256<0>(a);
                Simd<T, 128> hi = extract128From256<1>(a);

                // Perform operation on two halves.
                Simd<T, 128> v = op(lo, hi);

                // Reduce 128 to scalar.
                return reduce128ToScalar(v, op);
            }
            else
            {
                // Reduce 128 to scalar.
                return reduce128ToScalar(a, op);
            }
        }
    public:
        [[nodiscard]] static T horizontalAdd(const Simd& a) noexcept
        {
            return horizontalReduce(a, &Simd<T, 128>::add);
        }

        [[nodiscard]] static T horizontalMin(const Simd& a) noexcept
        {
            return horizontalReduce(a, &Simd<T, 128>::min);
        }

        [[nodiscard]] static T horizontalMax(const Simd& a) noexcept
        {
            return horizontalReduce(a, &Simd<T, 128>::max);
        }

