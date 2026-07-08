#pragma once
#include "SolvingPlannerBase.h"

namespace PS_AGONY
{
    // (Least-loaded) solving planner.
    class LLSolvingPlanner final : public SolvingPlannerBase
    {
    public:
        LLSolvingPlanner() = default;
        ~LLSolvingPlanner() override = default;

        LLSolvingPlanner(const LLSolvingPlanner&) = delete;
        LLSolvingPlanner& operator=(const LLSolvingPlanner&) = delete;
        LLSolvingPlanner(LLSolvingPlanner&&) = delete;
        LLSolvingPlanner& operator=(LLSolvingPlanner&&) = delete;

        void planExecution(const std::vector<BodyPair>& collisions, size_t bodyCount) override;
        [[nodiscard]] double computeEfficiency() const override;
        [[nodiscard]] bool validateNoCrossing(const std::vector<BodyPair>& collisions, size_t bodyCount) const override;
        [[nodiscard]] size_t getMemoryUsage() const override;
    };
}

