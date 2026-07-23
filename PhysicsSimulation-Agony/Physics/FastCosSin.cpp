#include "FastCosSin.h"
#include "Constants.h"

#include "Ecstasy/Core/Simd.h"

#include <cmath>

namespace PS_AGONY::FastCosSin
{
	using RealSimd = Ecstasy::Core::Simd<Real>;
	using IntSimd = std::conditional_t<sizeof(Real) == 8,
		Ecstasy::Core::Simd<int64_t>,
		Ecstasy::Core::Simd<int32_t>>;

	void order4CosSin(
		const Real* ECSTASY_RESTRICT inAngleArray,
		Real* ECSTASY_RESTRICT outCosArray,
		Real* ECSTASY_RESTRICT outSinArray,
		size_t size
	)
	{
		// Constants.
		constexpr Real coeff2 = 0.012238f;
		constexpr Real coeff3 = -0.199387f;
		constexpr Real coeff4 = 0.0282173f;

		constexpr int signMaskShift = sizeof(Real) * 8 - 2;

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

			RealSimd sinMag = coeff4V;
			sinMag = RealSimd::mulAdd(sinMag, a, coeff3V);
			sinMag = RealSimd::mulAdd(sinMag, a, coeff2V);
			sinMag = RealSimd::mulAdd(sinMag, a, RealSimd(1));
			sinMag = a * sinMag;
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

			Real sinMag = coeff4;
			sinMag = sinMag * a + coeff3;
			sinMag = sinMag * a + coeff2;
			sinMag = sinMag * a + Real(1);
			sinMag = a * sinMag;
			sinMag = std::fmin(sinMag, Real(1));

			Real cosMag = std::sqrt(Real(1) - sinMag * sinMag);

			Real sin = sinSign * sinMag;
			Real cos = cosSign * cosMag;

			outSinArray[i] = sin;
			outCosArray[i] = cos;
		}
	}
}
