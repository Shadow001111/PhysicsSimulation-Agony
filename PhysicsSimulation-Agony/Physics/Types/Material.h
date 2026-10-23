#pragma once
#include "Types.h"

namespace PS_AGONY2D
{
	struct Material
	{
		Real elasticity = 1.0;
		Real staticFriction = 0.0;
		Real dynamicFriction = 0.0;
	};
}
