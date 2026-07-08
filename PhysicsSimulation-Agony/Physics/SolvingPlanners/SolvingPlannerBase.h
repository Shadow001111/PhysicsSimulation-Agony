#pragma once
#include "../NarrowPhaseCollisionDetector.h"
#include "../Types.h"

#include <cstdint>
#include <vector>

namespace PS_AGONY
{
    class SolvingPlannerBase
    {
    protected:
        struct Pass
        {
            uint32_t start = 0;
            uint32_t size = 0;
        };

        size_t workerCount = 0;

        std::vector<size_t> flatIndices;
        std::vector<Pass> passOffsets;

        std::vector<size_t> remainingIndices;
        std::vector<size_t> nextRemainingIndices;
        std::vector<uint8_t> usedBodies;
    public:
        SolvingPlannerBase() = default;
        virtual ~SolvingPlannerBase() = default;

        SolvingPlannerBase(const SolvingPlannerBase&) = delete;
        SolvingPlannerBase& operator=(const SolvingPlannerBase&) = delete;
        SolvingPlannerBase(SolvingPlannerBase&&) = delete;
        SolvingPlannerBase& operator=(SolvingPlannerBase&&) = delete;

        void setWorkerCount(size_t workerCount) noexcept;
        virtual void planExecution(const std::vector<BodyPair>& collisions, size_t bodyCount) = 0;
        [[nodiscard]] virtual double computeEfficiency() const = 0;
        [[nodiscard]] virtual bool validateNoCrossing(const std::vector<BodyPair>& collisions, size_t bodyCount) const = 0;
        [[nodiscard]] virtual size_t getMemoryUsage() const = 0;
        [[nodiscard]] size_t getWorkerCount() const noexcept { return workerCount; };
        [[nodiscard]] const auto& getFlatIndices() const noexcept { return flatIndices; }
        [[nodiscard]] const auto& getPassOffsets() const noexcept { return passOffsets; }
    };
}

