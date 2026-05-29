#include "UpdateTimer.h"

UpdateTimer::UpdateTimer(double updatesPerSecond) :
	accumulatedTime(0.0), updateInterval(updatesPerSecond > 0.0 ? 1.0 / updatesPerSecond : 0.0)
{
}

void UpdateTimer::addTime(double deltaTime)
{
	accumulatedTime += deltaTime;
}

void UpdateTimer::setUpdateToTrue()
{
	accumulatedTime = updateInterval;
}

bool UpdateTimer::shouldUpdate()
{
	if (accumulatedTime >= updateInterval)
	{
		accumulatedTime -= updateInterval;
		return true;
	}
	return false;
}

int UpdateTimer::howManyTimesShouldUpdate()
{
	int count = 0;
	if (accumulatedTime >= updateInterval)
	{
		accumulatedTime -= updateInterval;
		count++;
	}
	return count;
}
