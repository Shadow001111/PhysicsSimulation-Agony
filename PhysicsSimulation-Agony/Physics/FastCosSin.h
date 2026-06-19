#pragma once
#include "Types.h"
#include "Constants.h"

#include "EcstasyCore/Portablity.h"
#include "EcstasyCore/Simd.h"

#include <cmath>

namespace PS_AGONY::FastCosSin
{
	// Requirments to use these functions:
	// 1) Input pointers must be RealSimd aligned.
	// 2) Angles/rotations must be in range [0; 2pi)

	void bhaskaraCosSinSimd(
		const Real* ECSTASY_RESTRICT anglePtr,
		Real* ECSTASY_RESTRICT cosPtr,
		Real* ECSTASY_RESTRICT sinPtr,
		size_t size
	)
	{
		using RealSimd = Ecstasy::Simd<Real>;
		using IntSimd = Ecstasy::Simd<int32_t>;

		// Math contants.
		constexpr Real PI_SQUARED = Constants::PI * Constants::PI;

		const RealSimd piSquaredV(PI_SQUARED);

		const RealSimd halfPiV(Constants::HALF_PI);
		const RealSimd invHalfPiV(Constants::INV_HALF_PI);

		// Other contants.
		constexpr size_t LANES = RealSimd::lanes;

		size_t i = 0;
		for (; i + LANES <= size; i += LANES)
		{
			RealSimd x = RealSimd::load(anglePtr + i);

			IntSimd q = (x * invHalfPiV).to_int32();

			RealSimd u = x - q.to_float() * halfPiV;

			RealSimd mirror = (q & IntSimd(1)).to_float();

			RealSimd a = RealSimd::mul_add(
				mirror,
				RealSimd::neg_mul_add(RealSimd(2), u, halfPiV),
				u
			);
			RealSimd aSquared = a * a;

			RealSimd sinMask = ((q & IntSimd(2)) << 30).as_float();
			RealSimd cosMask = (((q + IntSimd(1)) & IntSimd(2)) << 30).as_float();

			RealSimd cosMag = RealSimd::neg_mul_add(aSquared, RealSimd(4), piSquaredV) / (piSquaredV + aSquared);
			RealSimd sinMag = RealSimd::sqrt(RealSimd::neg_mul_add(cosMag, cosMag, RealSimd(1)));

			RealSimd sin = sinMag ^ sinMask;
			RealSimd cos = cosMag ^ cosMask;

			cos.store(cosPtr + i);
			sin.store(sinPtr + i);
		}
		for (; i < size; i++)
		{
			Real x = anglePtr[i];

			int q = (int)(x * Constants::INV_HALF_PI);
			Real u = x - q * Constants::HALF_PI;

			Real mirror = Real(q & 1);

			Real a = u + mirror * (Constants::HALF_PI - Real(2) * u);
			Real aSquared = a * a;

			Real sin_sign = Real(1) - Real(2) * Real(q >> 1);
			Real cos_sign = Real(1) - Real(2) * (Real)(((q + 1) >> 1) & 1);


			Real cos = cos_sign * (PI_SQUARED - Real(4) * aSquared) / (PI_SQUARED + aSquared);
			Real sin = sin_sign * std::sqrt(Real(1) - cos * cos);

			cosPtr[i] = cos;
			sinPtr[i] = sin;
		}
	}

	void order4CosSinSimd(
		const Real* ECSTASY_RESTRICT anglePtr,
		Real* ECSTASY_RESTRICT cosPtr,
		Real* ECSTASY_RESTRICT sinPtr,
		size_t size)
	{
		using RealSimd = Ecstasy::Simd<Real>;
		using IntSimd = Ecstasy::Simd<int32_t>;

		// Math contants.
		constexpr Real coeff2 =  0.012237999245583372;
		constexpr Real coeff3 = -0.19938700643358559;
		constexpr Real coeff4 =  0.02821727698224993;

		const RealSimd coeff2V(coeff2);
		const RealSimd coeff3V(coeff3);
		const RealSimd coeff4V(coeff4);

		const RealSimd halfPiV(Constants::HALF_PI);
		const RealSimd invHalfPiV(Constants::INV_HALF_PI);

		// Other contants.
		constexpr size_t LANES = RealSimd::lanes;

		size_t i = 0;
		for (; i + LANES <= size; i += LANES)
		{
			RealSimd x = RealSimd::loadu(anglePtr + i);

			IntSimd q = (x * invHalfPiV).to_int32();

			RealSimd u = x - q.to_float() * halfPiV;

			RealSimd mirror = (q & IntSimd(1)).to_float();

			RealSimd a = RealSimd::mul_add(
				mirror,
				RealSimd::neg_mul_add(RealSimd(2), u, halfPiV),
				u
			);

			RealSimd sinMask = ((q & IntSimd(2)) << 30).as_float();
			RealSimd cosMask = (((q + IntSimd(1)) & IntSimd(2)) << 30).as_float();

			RealSimd sinMag;
			sinMag = RealSimd::mul_add(a, coeff4V, coeff3V);
			sinMag = RealSimd::mul_add(sinMag, a, coeff2);
			sinMag = RealSimd::mul_add(sinMag, a, RealSimd(1));
			sinMag = a * sinMag;

			RealSimd cosMag = RealSimd::sqrt(RealSimd::neg_mul_add(sinMag, sinMag, RealSimd(1)));

			RealSimd sin = sinMag ^ sinMask;
			RealSimd cos = cosMag ^ cosMask;

			sin.storeu(sinPtr + i);
			cos.storeu(cosPtr + i);
		}
		for (; i < size; i++)
		{
			Real x = anglePtr[i];

			int q = (int)(x * Constants::INV_HALF_PI);
			Real u = x - q * Constants::HALF_PI;

			Real mirror = Real(q & 1);
			Real a = u + mirror * (Constants::HALF_PI - Real(2) * u);

			Real sin_sign = Real(1) - Real(2) * Real(q >> 1);
			Real cos_sign = Real(1) - Real(2) * (Real)(((q + 1) >> 1) & 1);

			Real sin = sin_sign * a * (Real(1) + a * (coeff2 + a * (coeff3 + a * coeff4)));
			Real cos = cos_sign * std::sqrt(Real(1) - sin * sin);

			sinPtr[i] = sin;
			cosPtr[i] = cos;
		}
	}
}
