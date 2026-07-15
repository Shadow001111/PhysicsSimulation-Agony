#pragma once
#include "../Types.h"

#include <cstdint>
#include <vector>

namespace PS_AGONY
{
    class SolvingPlanner
    {
        struct Pass
        {
            uint32_t start = 0;
            uint32_t size = 0;
        };

        size_t workerCount = 0;

        std::vector<size_t> flatIndices;
        std::vector<Pass> passOffsets;

        std::vector<size_t> remainingIndices;
        std::vector<size_t> delayedIndices;
        std::vector<size_t> nextRemainingIndices;
        std::vector<uint8_t> usedBodies;
    public:
        SolvingPlanner() = default;
        ~SolvingPlanner() = default;

        SolvingPlanner(const SolvingPlanner&) = delete;
        SolvingPlanner& operator=(const SolvingPlanner&) = delete;
        SolvingPlanner(SolvingPlanner&&) = delete;
        SolvingPlanner& operator=(SolvingPlanner&&) = delete;

        void setWorkerCount(size_t workerCount) noexcept { this->workerCount = workerCount; };

        void planStandardExecution(const std::vector<ObjectPair>& collisions, size_t bodyCount);
        void planSimdAwareExecution(const std::vector<ObjectPair>& collisions, size_t bodyCount, size_t chunkSize);
        void planCacheLineAwareExecution(
            const std::vector<ObjectPair>& collisions,
            size_t bodyCount,
            size_t bodyStrideBytes,
            size_t cacheLineBytes);

        void printExecutionPlan();

        [[nodiscard]] double computeEfficiency() const;
        [[nodiscard]] bool validateNoCrossing(const std::vector<ObjectPair>& collisions, size_t bodyCount) const;
        [[nodiscard]] size_t getMemoryUsage() const;
        [[nodiscard]] size_t getWorkerCount() const noexcept { return workerCount; };
        [[nodiscard]] const auto& getFlatIndices() const noexcept { return flatIndices; }
        [[nodiscard]] const auto& getPassOffsets() const noexcept { return passOffsets; }
    };
}
