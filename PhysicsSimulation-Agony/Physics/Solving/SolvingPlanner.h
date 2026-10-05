#pragma once
#include "Physics/Types.h"

#include <cstdint>
#include <queue>
#include <vector>

namespace PS_AGONY
{
    // Result of a single planning call: the wave/worker schedule for one solver's
    // constraint set. Owned by the solver (not the planner) so its buffers persist
    // and keep their capacity across frames without being shared/clobbered by
    // other solvers using the same SolvingPlanner.
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
        [[nodiscard]] bool validateNoCrossing(const std::vector<ObjectPair>& collisions, size_t bodyCount) const;
        [[nodiscard]] size_t getMemoryUsage() const;
        [[nodiscard]] size_t getWorkerCount() const noexcept { return workerCount; };
        [[nodiscard]] const auto& getFlatIndices() const noexcept { return flatIndices; }
        [[nodiscard]] const auto& getPassOffsets() const noexcept { return passOffsets; }
    };

    // Stateless (besides scratch buffers) planning engine. Holds no results of its
    // own: every plan* call writes into the caller-supplied SolvingPlan. Because of
    // that, a single instance's scratch buffers can safely be reused across every
    // solver in a simulation step; use threadLocal() to get one per thread rather
    // than each solver owning its own copy of these buffers.
    class SolvingPlanner
    {
    private:
        std::vector<size_t> remainingIndices;
        std::vector<size_t> delayedIndices;
        std::vector<size_t> nextRemainingIndices;
        std::vector<uint8_t> usedBodies;
        std::vector<std::vector<size_t>> waveAssignments;

        // Scratch shared across plan* variants (each variant only touches the subset it needs).
        std::vector<size_t> workerLoad;
        std::vector<size_t> wavePool;
        std::vector<size_t> simdRetry;

        std::vector<std::vector<size_t>> adjacency;
        std::vector<uint8_t> groupOwner;
        std::vector<size_t> groupSizes;
        std::vector<size_t> seeds;
        std::queue<size_t> bfsQueue;
        std::vector<uint8_t> waveOwner;
        std::vector<size_t> interiorIndices;
        std::vector<size_t> boundaryIndices;
        std::vector<ObjectPair> singlePairs;
    public:
        SolvingPlanner() = default;
        ~SolvingPlanner() = default;

        SolvingPlanner(const SolvingPlanner&) = delete;
        SolvingPlanner& operator=(const SolvingPlanner&) = delete;
        SolvingPlanner(SolvingPlanner&&) = delete;
        SolvingPlanner& operator=(SolvingPlanner&&) = delete;

        // Per-thread shared instance. Planning is not reentrant on the same
        // instance, but is otherwise stateless between calls, so solvers on the
        // same thread can plan one after another through this without paying for
        // their own copy of the scratch buffers.
        static SolvingPlanner& threadLocal()
        {
            thread_local SolvingPlanner instance;
            return instance;
        }

        void planStandardExecution(const std::vector<ObjectPair>& collisions, size_t bodyCount, size_t workerCount, SolvingPlan& outPlan);
        void planSimdAwareExecution(const std::vector<ObjectPair>& collisions, size_t bodyCount, size_t workerCount, size_t chunkSize, SolvingPlan& outPlan);
        void planCacheLineAwareExecution(
            const std::vector<ObjectPair>& collisions,
            size_t bodyCount,
            size_t workerCount,
            size_t bodyStrideBytes,
            size_t cacheLineBytes,
            SolvingPlan& outPlan);
        void planCacheSharingAwareExecution(
            const std::vector<ObjectPair>& collisions,
            size_t bodyCount,
            size_t workerCount,
            size_t bodyStrideBytes,
            size_t cacheLineBytes,
            SolvingPlan& outPlan,
            const uint8_t* isStatic = nullptr);
        void planSingleStandardExecution(const std::vector<ObjectIndex>& objects, size_t bodyCount, size_t workerCount, SolvingPlan& outPlan, const uint8_t* isStatic = nullptr);

        [[nodiscard]] size_t getMemoryUsage() const;
    };
}