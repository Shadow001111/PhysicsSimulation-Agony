#include "FastCosSin.h"
#include "Constants.h"

#include <cmath>

namespace PS_AGONY::FastCosSin
{
	// Constants.

	constexpr Real coeff0 = 9.9999999999953215e-01;
	constexpr Real coeff1 = -1.6666666534403832e-01;
	constexpr Real coeff2 = 8.3332302391869520e-03;
	constexpr Real coeff3 = -1.9818200679665358e-04;
	constexpr Real coeff4 = 2.6182346015248291e-06;

	constexpr int signMaskShift = sizeof(Real) * 8 - 2;

	// Other.

	using IntSimd = std::conditional_t<sizeof(Real) == 8,
		Ecstasy::Core::Simd<int64_t>,
		Ecstasy::Core::Simd<int32_t>>;

	// Functions.

	void order5Array(
		const Real* ECSTASY_RESTRICT inAngleArray,
		Real* ECSTASY_RESTRICT outCosArray,
		Real* ECSTASY_RESTRICT outSinArray,
		size_t size
	)
	{
		// Constants.
		const RealSimd coeff0V(coeff0);
		const RealSimd coeff1V(coeff1);
		const RealSimd coeff2V(coeff2);
		const RealSimd coeff3V(coeff3);
		const RealSimd coeff4V(coeff4);

		const RealSimd halfPiV(Constants::HALF_PI);
		const RealSimd invHalfPiV(Constants::INV_HALF_PI);

		// Main loop.
		constexpr size_t LANES = RealSimd::lanes;

		size_t i = 0;
		for (; i + LANES <= size; i += LANES)
		{
			RealSimd x = RealSimd::load(inAngleArray + i);

			IntSimd quadrantIndex = (x * invHalfPiV).realToSmallNonNegativeInteger();

			RealSimd u = RealSimd::negMulAdd(quadrantIndex.to<RealSimd>(), halfPiV, x);

			RealSimd mirror = (quadrantIndex & IntSimd(1)).to<RealSimd>();

			RealSimd a = RealSimd::mulAdd(
				mirror,
				RealSimd::negMulAdd(RealSimd(2), u, halfPiV),
				u
			);

			RealSimd sinMask = ((quadrantIndex & IntSimd(2)) << signMaskShift).as<RealSimd>();
			RealSimd cosMask = (((quadrantIndex + IntSimd(1)) & IntSimd(2)) << signMaskShift).as<RealSimd>();

			RealSimd a2 = a * a;
			RealSimd sinMag = coeff4V;
			sinMag = RealSimd::mulAdd(sinMag, a2, coeff3V);
			sinMag = RealSimd::mulAdd(sinMag, a2, coeff2V);
			sinMag = RealSimd::mulAdd(sinMag, a2, coeff1V);
			sinMag = RealSimd::mulAdd(sinMag, a2, coeff0V);
			sinMag = sinMag * a;
			sinMag = RealSimd::min(sinMag, RealSimd(1));

			RealSimd cosMag = RealSimd::sqrt(RealSimd::negMulAdd(sinMag, sinMag, RealSimd(1)));

			RealSimd sin = sinMag ^ sinMask;
			RealSimd cos = cosMag ^ cosMask;

			sin.store(outSinArray + i);
			cos.store(outCosArray + i);
		}
		for (; i < size; i++)
		{
			Real x = inAngleArray[i];

			int quadrantIndex = (int)(x * Real(Constants::INV_HALF_PI));
			Real u = x - quadrantIndex * Real(Constants::HALF_PI);

			Real mirror = Real(quadrantIndex & 1);
			Real a = u + mirror * (Real(Constants::HALF_PI) - Real(2) * u);

			Real sinSign = Real(1) - Real(2) * Real(quadrantIndex >> 1);
			Real cosSign = Real(1) - Real(2) * Real(((quadrantIndex + 1) >> 1) & 1);

			Real a2 = a * a;
			Real sinMag = coeff4;
			sinMag = sinMag * a2 + coeff3;
			sinMag = sinMag * a2 + coeff2;
			sinMag = sinMag * a2 + coeff1;
			sinMag = sinMag * a2 + coeff0;
			sinMag = sinMag * a;
			sinMag = std::fmin(sinMag, Real(1));

			Real cosMag = std::sqrt(Real(1) - sinMag * sinMag);

			Real sin = sinSign * sinMag;
			Real cos = cosSign * cosMag;

			outSinArray[i] = sin;
			outCosArray[i] = cos;
		}
	}

	void order5Simd(const RealSimd& inAngle, RealSimd& outCos, RealSimd& outSin)
	{
		// Constants.
		const RealSimd coeff0V(coeff0);
		const RealSimd coeff1V(coeff1);
		const RealSimd coeff2V(coeff2);
		const RealSimd coeff3V(coeff3);
		const RealSimd coeff4V(coeff4);

		const RealSimd halfPiV(Constants::HALF_PI);
		const RealSimd invHalfPiV(Constants::INV_HALF_PI);

		{
			const RealSimd& x = inAngle;

			IntSimd quadrantIndex = (x * invHalfPiV).realToSmallNonNegativeInteger();

			RealSimd u = RealSimd::negMulAdd(quadrantIndex.to<RealSimd>(), halfPiV, x);

			RealSimd mirror = (quadrantIndex & IntSimd(1)).to<RealSimd>();

			RealSimd a = RealSimd::mulAdd(
				mirror,
				RealSimd::negMulAdd(RealSimd(2), u, halfPiV),
				u
			);

			RealSimd sinMask = ((quadrantIndex & IntSimd(2)) << signMaskShift).as<RealSimd>();
			RealSimd cosMask = (((quadrantIndex + IntSimd(1)) & IntSimd(2)) << signMaskShift).as<RealSimd>();

			RealSimd a2 = a * a;
			RealSimd sinMag = coeff4V;
			sinMag = RealSimd::mulAdd(sinMag, a2, coeff3V);
			sinMag = RealSimd::mulAdd(sinMag, a2, coeff2V);
			sinMag = RealSimd::mulAdd(sinMag, a2, coeff1V);
			sinMag = RealSimd::mulAdd(sinMag, a2, coeff0V);
			sinMag = sinMag * a;
			sinMag = RealSimd::min(sinMag, RealSimd(1));

			RealSimd cosMag = RealSimd::sqrt(RealSimd::negMulAdd(sinMag, sinMag, RealSimd(1)));

			outSin = sinMag ^ sinMask;
			outCos = cosMag ^ cosMask;
		}
	}

	std::pair<Real, Real> order5Scalar(const Real x)
	{
		int quadrantIndex = (int)(x * Real(Constants::INV_HALF_PI));
		Real u = x - quadrantIndex * Real(Constants::HALF_PI);

		Real mirror = Real(quadrantIndex & 1);
		Real a = u + mirror * (Real(Constants::HALF_PI) - Real(2) * u);

		Real sinSign = Real(1) - Real(2) * Real(quadrantIndex >> 1);
		Real cosSign = Real(1) - Real(2) * Real(((quadrantIndex + 1) >> 1) & 1);

		Real a2 = a * a;
		Real sinMag = coeff4;
		sinMag = sinMag * a2 + coeff3;
		sinMag = sinMag * a2 + coeff2;
		sinMag = sinMag * a2 + coeff1;
		sinMag = sinMag * a2 + coeff0;
		sinMag = sinMag * a;
		sinMag = std::fmin(sinMag, Real(1));

		Real cosMag = std::sqrt(Real(1) - sinMag * sinMag);

		Real sin = sinSign * sinMag;
		Real cos = cosSign * cosMag;

		return { cos, sin };
	}
}