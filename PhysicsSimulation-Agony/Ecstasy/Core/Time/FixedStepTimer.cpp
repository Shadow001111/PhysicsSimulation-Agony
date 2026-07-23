#include "FixedStepTimer.h"

namespace Ecstasy::Core::Time
{
	FixedStepTimer::FixedStepTimer(double updatesPerSecond) :
		accumulatedTime(0.0), updateInterval(updatesPerSecond > 0.0 ? 1.0 / updatesPerSecond : 0.0)
	{
	}

	void FixedStepTimer::addTime(double deltaTime)
	{
		accumulatedTime += deltaTime;
	}

	void FixedStepTimer::setUpdateToTrue()
	{
		accumulatedTime = updateInterval;
	}

	bool FixedStepTimer::shouldUpdate()
	{
		if (accumulatedTime >= updateInterval)
		{
			accumulatedTime -= updateInterval;
			return true;
		}
		return false;
	}

	int FixedStepTimer::howManyTimesShouldUpdate()
	{
		int count = 0;
		if (accumulatedTime >= updateInterval)
		{
			accumulatedTime -= updateInterval;
			count++;
		}
		return count;
	}
}
