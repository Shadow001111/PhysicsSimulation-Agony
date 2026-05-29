#pragma once
class UpdateTimer
{
	double accumulatedTime = 0.0f;
	double updateInterval = 0.0f;
public:
	UpdateTimer(double updatesPerSecond);

	void addTime(double deltaTime);
	void setUpdateToTrue();

	bool shouldUpdate();
	int howManyTimesShouldUpdate();
	bool peek() const { return accumulatedTime >= updateInterval; };

	double getAccumulatedTime() const { return accumulatedTime; };
	double getAccumulatedTimeInPercent() const { return updateInterval > 0.0f ? (accumulatedTime / updateInterval) : 0.0f; };
	double getUpdateInterval() const { return updateInterval; };
};

