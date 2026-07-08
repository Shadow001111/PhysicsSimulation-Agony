#pragma once
#include "SolvingPlannerBase.h"

namespace PS_AGONY
{
	// (Fixed-capacity sequential) solving planner.
	class FCSSolvingPlanner final : public SolvingPlannerBase
	{
    public:
        static constexpr size_t MAX_INDICES_PER_PASS = 256;

        FCSSolvingPlanner() = default;
        ~FCSSolvingPlanner() override = default;

        FCSSolvingPlanner(const FCSSolvingPlanner&) = delete;
        FCSSolvingPlanner& operator=(const FCSSolvingPlanner&) = delete;
        FCSSolvingPlanner(FCSSolvingPlanner&&) = delete;
        FCSSolvingPlanner& operator=(FCSSolvingPlanner&&) = delete;

        static size_t chooseWorkerCount(size_t collisionCount, size_t availableWorkerCount);

        void planExecution(const std::vector<BodyPair>& collisions, size_t bodyCount) override;
        double computeEfficiency() const override;
        bool validateNoCrossing(const std::vector<BodyPair>& collisions, size_t bodyCount) const override;
        size_t getMemoryUsage() const override;
	};
}

