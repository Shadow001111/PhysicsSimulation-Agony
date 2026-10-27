#pragma once
#include "PS_AGONY2D/Types/Types.h"

#include <cstdint>
#include <queue>
#include <span>
#include <vector>

namespace PS_AGONY2D
{
    // Wave and worker schedule for one solver's constraint set.
    struct SolvingPlan
    {
        struct Pass
        {
            uint32_t start = 0;
            uint32_t size = 0;
        };

        std::vector<size_t> flatIndices;
        std::vector<Pass> passOffsets;
        size_t workerCount = 0;

        void printExecutionPlan() const;

        [[nodiscard]] double computeEfficiency() const;
        [[nodiscard]] bool validateNoCrossing(std::span<const ObjectPair> collisions, size_t bodyCount, const uint8_t* isStatic) const;
        [[nodiscard]] size_t getMemoryUsage() const;
        [[nodiscard]] size_t getWorkerCount() const noexcept { return workerCount; }
        [[nodiscard]] const auto& getFlatIndices() const noexcept { return flatIndices; }
        [[nodiscard]] const auto& getPassOffsets() const noexcept { return passOffsets; }
    };

    // Planning engine with reusable scratch buffers. Not reentrant on the same instance.
    // The bodyCount parameter is the largest body index. The isStatic array must cover it.
    // The hasStaticStaticPairs parameter allows skipping the static-static filtering when it is known to be unneeded.
    class SolvingPlanner
    {
    private:
        struct GroupPair
        {
            uint32_t a;
            uint32_t b;
        };

        std::vector<size_t> pending;
        std::vector<size_t> halfClaimed;
        std::vector<size_t> nextPending;
        std::vector<uint8_t> claimOwner;
        std::vector<std::vector<size_t>> workerQueues;

        std::vector<size_t> workerLoad;
        std::vector<size_t> wavePool;
        std::vector<size_t> simdRetry;

        std::vector<GroupPair> groupPairs;
        std::vector<std::vector<size_t>> adjacency;
        std::vector<uint8_t> groupWorker;
        std::vector<size_t> workerGroupCount;
        std::vector<size_t> seeds;
        std::queue<size_t> bfsQueue;
        std::vector<uint8_t> waveClaim;
        std::vector<size_t> interiorPending;
        std::vector<size_t> boundaryPending;
        std::vector<ObjectPair> singlePairs;

        void buildSchedulable(std::span<const ObjectPair> collisions, const uint8_t* isStatic, bool hasStaticStaticPairs, uint32_t groupShift);
        template <bool UseShift>
        void planGreedy(SolvingPlan& outPlan, std::span<const ObjectPair> collisions, size_t bodyCount, const uint8_t* isStatic, bool hasStaticStaticPairs, size_t workerCount, uint32_t groupShift);
        void partitionGroups(size_t groupCount, size_t workerCount);

    public:
        SolvingPlanner() = default;
        ~SolvingPlanner() = default;

        SolvingPlanner(const SolvingPlanner&) = delete;
        SolvingPlanner& operator=(const SolvingPlanner&) = delete;
        SolvingPlanner(SolvingPlanner&&) = delete;
        SolvingPlanner& operator=(SolvingPlanner&&) = delete;

        // Per-thread shared instance.
        static SolvingPlanner& threadLocal()
        {
            thread_local SolvingPlanner instance;
            return instance;
        }

        void planStandardExecution(
            SolvingPlan& outPlan,
            std::span<const ObjectPair> collisions,
            size_t bodyCount,
            const uint8_t* isStatic,
            bool hasStaticStaticPairs,
            size_t workerCount);

        void planSimdAwareExecution(
            SolvingPlan& outPlan,
            std::span<const ObjectPair> collisions,
            size_t bodyCount,
            const uint8_t* isStatic,
            bool hasStaticStaticPairs,
            size_t workerCount,
            size_t chunkSize);

        void planCacheLineAwareExecution(
            SolvingPlan& outPlan,
            std::span<const ObjectPair> collisions,
            size_t bodyCount,
            const uint8_t* isStatic,
            bool hasStaticStaticPairs,
            size_t workerCount,
            size_t bodyStrideBytes);

        void planCacheSharingAwareExecution(
            SolvingPlan& outPlan,
            std::span<const ObjectPair> collisions,
            size_t bodyCount,
            const uint8_t* isStatic,
            bool hasStaticStaticPairs,
            size_t workerCount,
            size_t bodyStrideBytes);

        void planSingleStandardExecution(
            SolvingPlan& outPlan,
            const std::vector<ObjectDenseIndex>& objects,
            size_t bodyCount,
            const uint8_t* isStatic,
            bool hasStaticStaticPairs,
            size_t workerCount);

        [[nodiscard]] size_t getMemoryUsage() const;
    };
}