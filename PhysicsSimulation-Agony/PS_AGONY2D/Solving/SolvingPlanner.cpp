#include "SolvingPlanner.h"

#include "Ecstasy/Core/Portablity.h"

#include "PS_AGONY2D/ContainerUtilities.h"
#include "PS_AGONY2D/Profiling/TracyProfiler.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <new>
#include <numeric>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace PS_AGONY2D
{
    using Pass = SolvingPlan::Pass;

    namespace
    {
        constexpr uint8_t kUnowned = -1;
        constexpr size_t kMaxWorkers = 255;

        uint64_t hashGroupPair(uint32_t lo, uint32_t hi)
        {
            uint64_t key = (static_cast<uint64_t>(lo) << 32) | static_cast<uint64_t>(hi);
            key = (~key) + (key << 21);
            key = key ^ (key >> 24);
            key = (key + (key << 3)) + (key << 8);
            key = key ^ (key >> 14);
            key = (key + (key << 2)) + (key << 4);
            key = key ^ (key >> 28);
            return key;
        }

        void beginPlan(SolvingPlan& outPlan, size_t workerCount, size_t collisionCount)
        {
            if (workerCount > kMaxWorkers)
                throw std::invalid_argument("SolvingPlanner: workerCount must not exceed 255.");

            outPlan.workerCount = workerCount;
            outPlan.flatIndices.clear();
            outPlan.flatIndices.reserve(collisionCount);
            outPlan.passOffsets.clear();
        }

        void appendWave(SolvingPlan& outPlan, const std::vector<std::vector<size_t>>& queues, size_t workerCount, size_t waveStartPass)
        {
            TRACY_SCOPE_NC("Append to flatIndices", PS_AGONY2D::Profiling::Color::Magenta);
            for (size_t w = 0; w < workerCount; w++)
            {
                Pass& pass = outPlan.passOffsets[waveStartPass + w];
                pass.start = static_cast<uint32_t>(outPlan.flatIndices.size());
                pass.size = static_cast<uint32_t>(queues[w].size());
                outPlan.flatIndices.insert(outPlan.flatIndices.end(), queues[w].begin(), queues[w].end());
            }
        }

        size_t leastLoadedWorker(const size_t* ECSTASY_RESTRICT load, size_t count)
        {
            size_t best = 0;
            size_t bestLoad = load[0];
            for (size_t i = 1; i < count; ++i)
            {
                const size_t current = load[i];
                if (current < bestLoad)
                {
                    bestLoad = current;
                    best = i;
                }
            }
            return best;
        }

        uint32_t cacheGroupShift(size_t bodyStrideBytes, size_t cacheLineBytes)
        {
            const size_t bodiesPerLine = std::max<size_t>(1, cacheLineBytes / bodyStrideBytes);
            return static_cast<uint32_t>(std::bit_width(bodiesPerLine - 1));
        }
    }

    void SolvingPlanner::buildSchedulable(std::span<const ObjectPair> collisions, const uint8_t* isStatic, bool hasStaticStaticPairs, uint32_t groupShift)
    {
        const size_t collisionCount = collisions.size();
        groupPairs.resize(collisionCount);
        pending.clear();
        pending.reserve(collisionCount);

        const ObjectPair* ECSTASY_RESTRICT collisionsPtr = collisions.data();
        const uint8_t* ECSTASY_RESTRICT staticPtr = isStatic;
        GroupPair* ECSTASY_RESTRICT pairsPtr = groupPairs.data();

        for (size_t i = 0; i < collisionCount; i++)
        {
            const ObjectPair& c = collisionsPtr[i];
            const bool aStatic = staticPtr[c.a] != 0;
            const bool bStatic = staticPtr[c.b] != 0;

            if (hasStaticStaticPairs && aStatic && bStatic)
                continue;

            const size_t ownerA = static_cast<size_t>(aStatic ? c.b : c.a);
            const size_t ownerB = static_cast<size_t>(bStatic ? c.a : c.b);
            pairsPtr[i] = { static_cast<uint32_t>(ownerA >> groupShift), static_cast<uint32_t>(ownerB >> groupShift) };
            pending.push_back(i);
        }
    }

    template <bool UseShift>
    void SolvingPlanner::planGreedy(
        SolvingPlan& outPlan,
        std::span<const ObjectPair> collisions,
        size_t bodyCount,
        const uint8_t* ECSTASY_RESTRICT isStatic,
        bool hasStaticStaticPairs,
        size_t workerCount,
        uint32_t groupShift)
    {
        beginPlan(outPlan, workerCount, collisions.size());

        if (collisions.empty() || workerCount == 0)
            return;

        const size_t collisionCount = collisions.size();
        const size_t groupCount = (UseShift ? (bodyCount >> groupShift) : bodyCount) + 1;

        groupPairs.resize(collisionCount);
        claimOwner.assign(groupCount, kUnowned);
        workerQueues.resize(workerCount);
        workerLoad.assign(workerCount, 0);
        pending.clear();

        const ObjectPair* ECSTASY_RESTRICT collisionsPtr = collisions.data();
        GroupPair* ECSTASY_RESTRICT pairsPtr = groupPairs.data();
        uint8_t* ECSTASY_RESTRICT owners = claimOwner.data();
        size_t* ECSTASY_RESTRICT loads = workerLoad.data();

        const auto toGroup = [groupShift](size_t body) -> uint32_t
            {
                if constexpr (UseShift)
                    return static_cast<uint32_t>(body >> groupShift);
                else
                    return static_cast<uint32_t>(body);
            };

        const auto pass1Item = [&](auto first, size_t idx, uint32_t ga, uint32_t gb)
            {
                const uint8_t ownerA = owners[ga];
                const uint8_t ownerB = owners[gb];

                if (ownerA == ownerB)
                {
                    size_t worker;
                    if (ownerA == kUnowned)
                    {
                        worker = leastLoadedWorker(loads, workerCount);
                        owners[ga] = static_cast<uint8_t>(worker);
                        owners[gb] = static_cast<uint8_t>(worker);
                    }
                    else
                    {
                        worker = ownerA;
                    }

                    workerQueues[worker].push_back(idx);
                    loads[worker]++;
                }
                else
                {
                    if constexpr (decltype(first)::value)
                        pairsPtr[idx] = GroupPair{ ga, gb };

                    auto& vec = (ownerA == kUnowned || ownerB == kUnowned) ? halfClaimed : nextPending;
                    vec.push_back(idx);
                }
            };

        for (bool first = true; first || !pending.empty(); first = false)
        {
            const size_t waveStartPass = outPlan.passOffsets.size();
            outPlan.passOffsets.resize(waveStartPass + workerCount, Pass{ 0, 0 });

            std::fill_n(loads, workerCount, size_t(0));
            std::fill_n(owners, groupCount, kUnowned);
            halfClaimed.clear();
            nextPending.clear();

            for (auto& queue : workerQueues)
                queue.clear();

            {
                TRACY_SCOPE_NC("Pass1", PS_AGONY2D::Profiling::Color::Cyan);
                if (first)
                {
                    for (size_t i = 0; i < collisionCount; ++i)
                    {
                        const ObjectPair& c = collisionsPtr[i];
                        const bool aStatic = isStatic[c.a] != 0;
                        const bool bStatic = isStatic[c.b] != 0;

                        if (hasStaticStaticPairs && aStatic && bStatic)
                            continue;

                        const size_t bodyA = static_cast<size_t>(aStatic ? c.b : c.a);
                        const size_t bodyB = static_cast<size_t>(bStatic ? c.a : c.b);
                        pass1Item(std::true_type{}, i, toGroup(bodyA), toGroup(bodyB));
                    }
                }
                else
                {
                    for (size_t idx : pending)
                    {
                        const GroupPair g = pairsPtr[idx];
                        pass1Item(std::false_type{}, idx, g.a, g.b);
                    }
                }
            }

            {
                TRACY_SCOPE_NC("Pass2", PS_AGONY2D::Profiling::Color::Yellow);
                for (size_t idx : halfClaimed)
                {
                    const GroupPair g = pairsPtr[idx];
                    const uint8_t ownerA = owners[g.a];
                    const uint8_t ownerB = owners[g.b];

                    if (ownerA != kUnowned && ownerB != kUnowned && ownerA != ownerB)
                    {
                        nextPending.push_back(idx);
                        continue;
                    }

                    const uint8_t worker = std::min(ownerA, ownerB);
                    workerQueues[worker].push_back(idx);
                    loads[worker]++;
                    owners[g.a] = worker;
                    owners[g.b] = worker;
                }
            }

            appendWave(outPlan, workerQueues, workerCount, waveStartPass);
            pending.swap(nextPending);

            if (first && outPlan.flatIndices.empty())
                outPlan.passOffsets.clear();
        }
    }

    void SolvingPlanner::planStandardExecution(
        SolvingPlan& outPlan,
        std::span<const ObjectPair> collisions,
        size_t bodyCount,
        const uint8_t* isStatic,
        bool hasStaticStaticPairs,
        size_t workerCount)
    {
        planGreedy<false>(outPlan, collisions, bodyCount, isStatic, hasStaticStaticPairs, workerCount, 0);
    }

    void SolvingPlanner::planCacheLineAwareExecution(
        SolvingPlan& outPlan,
        std::span<const ObjectPair> collisions,
        size_t bodyCount,
        const uint8_t* isStatic,
        bool hasStaticStaticPairs,
        size_t workerCount,
        size_t bodyStrideBytes)
    {
        if (bodyStrideBytes == 0)
        {
            beginPlan(outPlan, workerCount, 0);
            return;
        }

        planGreedy<true>(outPlan, collisions, bodyCount, isStatic, hasStaticStaticPairs, workerCount,
            cacheGroupShift(bodyStrideBytes, std::hardware_destructive_interference_size));
    }

    void SolvingPlanner::planSimdAwareExecution(
        SolvingPlan& outPlan,
        std::span<const ObjectPair> collisions,
        size_t bodyCount,
        const uint8_t* ECSTASY_RESTRICT isStatic,
        bool hasStaticStaticPairs,
        size_t workerCount,
        size_t chunkSize)
    {
        beginPlan(outPlan, workerCount, collisions.size());

        if (chunkSize == 0)
            throw std::invalid_argument("SolvingPlanner: chunkSize must not be zero.");

        if (collisions.empty() || workerCount == 0)
            return;

        buildSchedulable(collisions, isStatic, hasStaticStaticPairs, 0);

        claimOwner.assign(bodyCount + 1, kUnowned);
        workerQueues.resize(workerCount);
        workerLoad.assign(workerCount, 0);

        const size_t bodySlots = bodyCount + 1;
        const GroupPair* ECSTASY_RESTRICT pairsPtr = groupPairs.data();
        uint8_t* ECSTASY_RESTRICT owners = claimOwner.data();
        size_t* ECSTASY_RESTRICT loads = workerLoad.data();

        const auto tryPlace = [&](size_t idx, size_t worker)
            {
                const GroupPair g = pairsPtr[idx];
                auto& queue = workerQueues[worker];
                const size_t size = queue.size();

                for (size_t i = size - size % chunkSize; i < size; ++i)
                {
                    const GroupPair p = pairsPtr[queue[i]];
                    if (g.a == p.a || g.a == p.b || g.b == p.a || g.b == p.b)
                        return false;
                }

                queue.push_back(idx);
                loads[worker]++;
                owners[g.a] = static_cast<uint8_t>(worker);
                owners[g.b] = static_cast<uint8_t>(worker);
                return true;
            };

        while (!pending.empty())
        {
            const size_t waveStartPass = outPlan.passOffsets.size();
            outPlan.passOffsets.resize(waveStartPass + workerCount, Pass{ 0, 0 });

            std::fill_n(loads, workerCount, size_t(0));
            std::fill_n(owners, bodySlots, kUnowned);
            nextPending.clear();

            for (auto& queue : workerQueues)
                queue.clear();

            wavePool.assign(pending.begin(), pending.end());

            while (true)
            {
                bool placedAny = false;
                halfClaimed.clear();
                simdRetry.clear();

                {
                    TRACY_SCOPE_NC("Pass1", PS_AGONY2D::Profiling::Color::Cyan);
                    for (size_t idx : wavePool)
                    {
                        const GroupPair g = pairsPtr[idx];
                        const uint8_t ownerA = owners[g.a];
                        const uint8_t ownerB = owners[g.b];

                        if (ownerA == ownerB)
                        {
                            const size_t worker = ownerA == kUnowned ? leastLoadedWorker(loads, workerCount) : ownerA;
                            if (tryPlace(idx, worker))
                                placedAny = true;
                            else
                                simdRetry.push_back(idx);
                        }
                        else
                        {
                            auto& vec = (ownerA == kUnowned || ownerB == kUnowned) ? halfClaimed : nextPending;
                            vec.push_back(idx);
                        }
                    }
                }

                {
                    TRACY_SCOPE_NC("Pass2", PS_AGONY2D::Profiling::Color::Yellow);
                    for (size_t idx : halfClaimed)
                    {
                        const GroupPair g = pairsPtr[idx];
                        const uint8_t ownerA = owners[g.a];
                        const uint8_t ownerB = owners[g.b];

                        if (ownerA != kUnowned && ownerB != kUnowned && ownerA != ownerB)
                        {
                            nextPending.push_back(idx);
                            continue;
                        }

                        if (tryPlace(idx, std::min(ownerA, ownerB)))
                            placedAny = true;
                        else
                            simdRetry.push_back(idx);
                    }
                }

                if (!placedAny || simdRetry.empty())
                {
                    nextPending.insert(nextPending.end(), simdRetry.begin(), simdRetry.end());
                    break;
                }

                wavePool.swap(simdRetry);
            }

            appendWave(outPlan, workerQueues, workerCount, waveStartPass);
            pending.swap(nextPending);
        }
    }

    void SolvingPlanner::partitionGroups(size_t groupCount, size_t workerCount)
    {
        groupWorker.assign(groupCount, kUnowned);
        workerGroupCount.assign(workerCount, 0);

        if (workerCount == 1)
        {
            std::fill(groupWorker.begin(), groupWorker.end(), uint8_t(0));
            return;
        }

        seeds.resize(groupCount);
        std::iota(seeds.begin(), seeds.end(), size_t(0));

        uint32_t seedRng = 1234;
        for (size_t i = seeds.size() - 1; i > 0; --i)
        {
            seedRng = seedRng * 1664525u + 1013904223u;
            std::swap(seeds[i], seeds[seedRng % (i + 1)]);
        }

        while (!bfsQueue.empty())
            bfsQueue.pop();

        const size_t targetSize = std::max<size_t>(1, static_cast<size_t>(std::round(static_cast<double>(groupCount) / static_cast<double>(workerCount) * 1.15)));

        for (size_t w = 0; w < workerCount && w < groupCount; ++w)
        {
            const size_t seedGroup = seeds[w];
            groupWorker[seedGroup] = static_cast<uint8_t>(w);
            workerGroupCount[w]++;
            bfsQueue.push(seedGroup);
        }

        while (!bfsQueue.empty())
        {
            const size_t node = bfsQueue.front();
            bfsQueue.pop();

            const uint8_t w = groupWorker[node];
            if (workerGroupCount[w] >= targetSize)
                continue;

            for (size_t neighbor : adjacency[node])
            {
                if (groupWorker[neighbor] == kUnowned)
                {
                    groupWorker[neighbor] = w;
                    workerGroupCount[w]++;
                    bfsQueue.push(neighbor);
                }
            }
        }

        for (size_t g = 0; g < groupCount; ++g)
        {
            if (groupWorker[g] != kUnowned)
                continue;

            const size_t smallest = leastLoadedWorker(workerGroupCount.data(), workerCount);
            groupWorker[g] = static_cast<uint8_t>(smallest);
            workerGroupCount[smallest]++;
        }
    }

    void SolvingPlanner::planCacheSharingAwareExecution(
        SolvingPlan& outPlan,
        std::span<const ObjectPair> collisions,
        size_t bodyCount,
        const uint8_t* isStatic,
        bool hasStaticStaticPairs,
        size_t workerCount,
        size_t bodyStrideBytes)
    {
        beginPlan(outPlan, workerCount, collisions.size());

        if (collisions.empty() || workerCount == 0 || bodyStrideBytes == 0)
            return;

        const uint32_t groupShift = cacheGroupShift(bodyStrideBytes, std::hardware_destructive_interference_size);
        const size_t groupCount = (bodyCount >> groupShift) + 1;

        buildSchedulable(collisions, isStatic, hasStaticStaticPairs, groupShift);

        adjacency.resize(groupCount);
        for (auto& v : adjacency)
            v.clear();

        for (size_t idx : pending)
        {
            const GroupPair g = groupPairs[idx];
            if (g.a != g.b)
            {
                adjacency[g.a].push_back(g.b);
                adjacency[g.b].push_back(g.a);
            }
        }

        for (auto& v : adjacency)
        {
            std::sort(v.begin(), v.end());
            v.erase(std::unique(v.begin(), v.end()), v.end());
        }

        partitionGroups(groupCount, workerCount);

        waveClaim.assign(groupCount, kUnowned);
        workerQueues.resize(workerCount);

        const GroupPair* ECSTASY_RESTRICT pairsPtr = groupPairs.data();
        const uint8_t* ECSTASY_RESTRICT partition = groupWorker.data();
        uint8_t* ECSTASY_RESTRICT claims = waveClaim.data();

        while (!pending.empty())
        {
            const size_t waveStartPass = outPlan.passOffsets.size();
            outPlan.passOffsets.resize(waveStartPass + workerCount, Pass{ 0, 0 });

            nextPending.clear();
            halfClaimed.clear();
            interiorPending.clear();
            boundaryPending.clear();
            std::fill_n(claims, groupCount, kUnowned);

            for (auto& queue : workerQueues)
                queue.clear();

            for (size_t idx : pending)
            {
                const GroupPair g = pairsPtr[idx];
                auto& vec = partition[g.a] == partition[g.b] ? interiorPending : boundaryPending;
                vec.push_back(idx);
            }

            for (size_t idx : interiorPending)
            {
                const GroupPair g = pairsPtr[idx];
                const uint8_t chosen = partition[g.a];

                workerQueues[chosen].push_back(idx);
                claims[g.a] = chosen;
                claims[g.b] = chosen;
            }

            for (size_t idx : boundaryPending)
            {
                const GroupPair g = pairsPtr[idx];
                const uint8_t ownerA = claims[g.a];
                const uint8_t ownerB = claims[g.b];

                uint8_t chosen;
                if (ownerA == ownerB && ownerA != kUnowned)
                {
                    chosen = ownerA;
                }
                else if (ownerA == kUnowned && ownerB == kUnowned)
                {
                    const uint32_t lo = std::min(g.a, g.b);
                    const uint32_t hi = std::max(g.a, g.b);
                    chosen = (hashGroupPair(lo, hi) & 1) == 0 ? partition[lo] : partition[hi];
                }
                else
                {
                    halfClaimed.push_back(idx);
                    continue;
                }

                workerQueues[chosen].push_back(idx);
                claims[g.a] = chosen;
                claims[g.b] = chosen;
            }

            for (size_t idx : halfClaimed)
            {
                const GroupPair g = pairsPtr[idx];
                const uint8_t ownerA = claims[g.a];
                const uint8_t ownerB = claims[g.b];

                if (ownerA != kUnowned && ownerB != kUnowned && ownerA != ownerB)
                {
                    nextPending.push_back(idx);
                    continue;
                }

                const uint8_t chosen = std::min(ownerA, ownerB);
                workerQueues[chosen].push_back(idx);
                claims[g.a] = chosen;
                claims[g.b] = chosen;
            }

            appendWave(outPlan, workerQueues, workerCount, waveStartPass);
            pending.swap(nextPending);
        }
    }

    void SolvingPlanner::planSingleStandardExecution(
        SolvingPlan& outPlan,
        const std::vector<ObjectDenseIndex>& objects,
        size_t bodyCount,
        const uint8_t* isStatic,
        bool hasStaticStaticPairs,
        size_t workerCount)
    {
        singlePairs.resize(objects.size());
        for (size_t i = 0; i < objects.size(); i++)
            singlePairs[i] = { objects[i], objects[i] };

        planCacheSharingAwareExecution(
            outPlan, singlePairs, bodyCount, isStatic, hasStaticStaticPairs, workerCount,
            sizeof(Real));
    }

    void SolvingPlan::printExecutionPlan() const
    {
        const size_t numWaves = (passOffsets.empty() ? 0 : passOffsets.size() / workerCount);
        const int colWidth = 6;

        std::cout << std::setw(colWidth) << " ";
        for (size_t wave = 0; wave < numWaves; ++wave)
            std::cout << std::setw(colWidth) << ("Wv" + std::to_string(wave));
        std::cout << "\n";

        for (size_t worker = 0; worker < workerCount; ++worker)
        {
            std::cout << std::setw(colWidth - 1) << ("Wk" + std::to_string(worker)) << " ";
            for (size_t wave = 0; wave < numWaves; ++wave)
            {
                const size_t idx = wave * workerCount + worker;
                if (idx < passOffsets.size())
                    std::cout << std::setw(colWidth) << passOffsets[idx].size;
                else
                    std::cout << std::setw(colWidth) << "-";
            }
            std::cout << "\n";
        }
    }

    double SolvingPlan::computeEfficiency() const
    {
        if (workerCount == 0 || flatIndices.empty())
            return 0.0;

        const size_t numWaves = passOffsets.size() / workerCount;
        size_t capacity = 0;

        for (size_t wave = 0; wave < numWaves; ++wave)
        {
            size_t waveMax = 0;
            for (size_t w = 0; w < workerCount; ++w)
                waveMax = std::max<size_t>(waveMax, passOffsets[wave * workerCount + w].size);
            capacity += waveMax * workerCount;
        }

        if (capacity == 0)
            return 0.0;

        return static_cast<double>(flatIndices.size()) / static_cast<double>(capacity) * 100.0;
    }

    bool SolvingPlan::validateNoCrossing(std::span<const ObjectPair> collisions, size_t bodyCount, const uint8_t* isStatic) const
    {
        if (workerCount == 0)
            return passOffsets.empty();

        const size_t numWaves = passOffsets.size() / workerCount;
        std::vector<int32_t> bodyOwner(bodyCount + 1, -1);
        std::vector<uint8_t> scheduled(collisions.size(), 0);

        for (size_t wave = 0; wave < numWaves; ++wave)
        {
            std::fill(bodyOwner.begin(), bodyOwner.end(), -1);

            for (size_t worker = 0; worker < workerCount; ++worker)
            {
                const int32_t workerId = static_cast<int32_t>(worker);
                const Pass& pass = passOffsets[wave * workerCount + worker];

                const auto claim = [&](ObjectDenseIndex body)
                    {
                        if (isStatic[body] != 0)
                            return true;

                        int32_t& owner = bodyOwner[body];
                        if (owner != -1 && owner != workerId)
                            return false;

                        owner = workerId;
                        return true;
                    };

                for (size_t i = 0; i < pass.size; ++i)
                {
                    const size_t idx = flatIndices[static_cast<size_t>(pass.start) + i];
                    if (idx >= collisions.size() || scheduled[idx] != 0)
                        return false;
                    scheduled[idx] = 1;

                    const ObjectPair& c = collisions[idx];
                    if (isStatic[c.a] != 0 && isStatic[c.b] != 0)
                        return false;

                    if (!claim(c.a) || !claim(c.b))
                        return false;
                }
            }
        }

        for (size_t i = 0; i < collisions.size(); ++i)
        {
            const ObjectPair& c = collisions[i];
            if (scheduled[i] == 0 && !(isStatic[c.a] != 0 && isStatic[c.b] != 0))
                return false;
        }

        return true;
    }

    size_t SolvingPlan::getMemoryUsage() const
    {
        size_t total = 0;
        total += getVectorMemoryUsage(flatIndices);
        total += getVectorMemoryUsage(passOffsets);
        return total;
    }

    size_t SolvingPlanner::getMemoryUsage() const
    {
        size_t total = 0;
        total += getVectorMemoryUsage(pending);
        total += getVectorMemoryUsage(halfClaimed);
        total += getVectorMemoryUsage(nextPending);
        total += getVectorMemoryUsage(claimOwner);

        total += getVectorMemoryUsage(workerQueues);
        for (const auto& queue : workerQueues)
            total += getVectorMemoryUsage(queue);

        total += getVectorMemoryUsage(workerLoad);
        total += getVectorMemoryUsage(wavePool);
        total += getVectorMemoryUsage(simdRetry);
        total += getVectorMemoryUsage(groupPairs);

        total += getVectorMemoryUsage(adjacency);
        for (const auto& adj : adjacency)
            total += getVectorMemoryUsage(adj);

        total += getVectorMemoryUsage(groupWorker);
        total += getVectorMemoryUsage(workerGroupCount);
        total += getVectorMemoryUsage(seeds);
        total += getVectorMemoryUsage(waveClaim);
        total += getVectorMemoryUsage(interiorPending);
        total += getVectorMemoryUsage(boundaryPending);
        total += getVectorMemoryUsage(singlePairs);

        return total;
    }
}