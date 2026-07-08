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

        void planExecution(const std::vector<BodyPair>& collisions, size_t bodyCount) override;
        [[nodiscard]] double computeEfficiency() const override;
        [[nodiscard]] bool validateNoCrossing(const std::vector<BodyPair>& collisions, size_t bodyCount) const override;
        [[nodiscard]] size_t getMemoryUsage() const override;
	};
}

