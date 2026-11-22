#pragma once

#include <bit>
#include <concepts>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace PS_AGONY2D::ConstexprMath
{
    namespace detail
    {
        template <typename T>
        concept real = std::floating_point<T> && std::numeric_limits<T>::is_iec559 && (sizeof(T) == 4 || sizeof(T) == 8);

        template <real T>
        inline constexpr bool is_single = sizeof(T) == 4;

        template <real T>
        constexpr T kernel_sin(T r) noexcept
        {
            const T z = r * r;
            if constexpr (is_single<T>)
            {
                return r + r * z * (T(-1.6666654611e-1) + z * (T(8.3321608736e-3) + z * T(-1.9515295891e-4)));
            }
            else
            {
                const T z2 = z * z;
                const T a = T(-1.66666666666666324348e-01) + z * T(8.33333333332248946124e-03);
                const T b = T(-1.98412698298579493134e-04) + z * T(2.75573137070700676789e-06);
                const T c = T(-2.50507602534068634195e-08) + z * T(1.58969099521155010221e-10);
                return r + r * z * (a + z2 * (b + z2 * c));
            }
        }

        template <real T>
        constexpr T kernel_cos(T r) noexcept
        {
            const T z = r * r;
            if constexpr (is_single<T>)
            {
                const T p = T(4.166664568298827e-2) + z * (T(-1.388731625493765e-3) + z * T(2.443315711809948e-5));
                return T(1) - T(0.5) * z + z * z * p;
            }
            else
            {
                const T z2 = z * z;
                const T a = T(4.16666666666666019037e-02) + z * T(-1.38888888888741095749e-03);
                const T b = T(2.48015872894767294178e-05) + z * T(-2.75573143513906633035e-07);
                const T c = T(2.08757232129817482790e-09) + z * T(-1.13596475577881948265e-11);
                return T(1) - T(0.5) * z + z2 * (a + z2 * (b + z2 * c));
            }
        }

        template <real T>
        constexpr T clamp_abs(T v, T limit) noexcept
        {
            return v > limit ? limit : (v < -limit ? -limit : v);
        }

        template <real T>
        constexpr T reduce(T x, std::int64_t& quadrant) noexcept
        {
            constexpr T inv_half_pi = T(0.63661977236758134308);
            constexpr T limit = T(4.0e18);

            constexpr T p1 = is_single<T> ? T(1.5703125) : T(1.57079632673412561417e+00);
            constexpr T p2 = is_single<T> ? T(4.837512969970703125e-4) : T(6.07710050630396597660e-11);
            constexpr T p3 = is_single<T> ? T(7.54978995489188216e-8) : T(2.02226624879595063154e-21);

            const T t = clamp_abs(x * inv_half_pi, limit);
            const std::int64_t n = static_cast<std::int64_t>(t < T(0) ? t - T(0.5) : t + T(0.5));
            const T nf = static_cast<T>(n);

            quadrant = n;
            return ((x - nf * p1) - nf * p2) - nf * p3;
        }

        template <real T>
        constexpr T trig(T x, std::int64_t shift) noexcept
        {
            std::int64_t n;
            const T r = reduce(x, n);
            n += shift;
            const T v = (n & 1) ? kernel_cos(r) : kernel_sin(r);
            return (n & 2) ? -v : v;
        }
    }

    template <detail::real T>
    constexpr T sqrt(T x) noexcept
    {
        using U = std::conditional_t<detail::is_single<T>, std::uint32_t, std::uint64_t>;
        constexpr U magic = detail::is_single<T> ? U(0x1fbd1df5u) : U(0x1ff7a3bea91d9b1bull);
        constexpr int iterations = detail::is_single<T> ? 3 : 4;

        if (x <= T(0))
        {
            return T(0);
        }

        T y = std::bit_cast<T>(static_cast<U>((std::bit_cast<U>(x) >> 1) + magic));
        for (int i = 0; i < iterations; ++i)
        {
            y = T(0.5) * (y + x / y);
        }
        return y;
    }

    template <detail::real T>
    constexpr T to_radians(T degrees) noexcept
    {
        return degrees * T(0.017453292519943295769236907684886);
    }

    template <detail::real T>
    constexpr T to_degrees(T radians) noexcept
    {
        return radians * T(57.295779513082320876798154814105);
    }

    template <detail::real T>
    constexpr T sin(T x) noexcept
    {
        return detail::trig(x, 0);
    }

    template <detail::real T>
    constexpr T cos(T x) noexcept
    {
        return detail::trig(x, 1);
    }
}