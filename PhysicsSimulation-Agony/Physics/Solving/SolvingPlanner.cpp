#include "SolvingPlanner.h"

#include "Physics/ContainerUtilities.h"

#include "Physics/Profiling/TracyProfiler.h"

#include <numeric>
#include <new>
#include <iostream>
#include <iomanip>
#include <string>
#include <algorithm>

namespace PS_AGONY2D
{
    using Pass = SolvingPlan::Pass;

    static inline size_t hashGroupPair(size_t lo, size_t hi)
    {
        uint64_t key = (static_cast<uint64_t>(lo) << 32) | static_cast<uint64_t>(hi);
        key = (~key) + (key << 21);
        key = key ^ (key >> 24);
        key = (key + (key << 3)) + (key << 8);
        key = key ^ (key >> 14);
        key = (key + (key << 2)) + (key << 4);
        key = key ^ (key >> 28);
        return static_cast<size_t>(key);
    }

    void SolvingPlanner::planStandardExecution(
        std::span<const ObjectPair> collisions,
        size_t bodyCount,
        size_t workerCount,
        SolvingPlan& outPlan
    )
    {
        const ObjectPair* collisionsPtr = collisions.data();
        const size_t collisionCount = collisions.size();

        outPlan.workerCount = workerCount;
        outPlan.flatIndices.clear();
        outPlan.flatIndices.reserve(collisionCount);
        outPlan.passOffsets.clear();

        if (collisionCount == 0 || workerCount == 0)
            return;

        remainingIndices.resize(collisionCount);
        std::iota(remainingIndices.begin(), remainingIndices.end(), size_t(0));

        delayedIndices.clear();
        nextRemainingIndices.clear();

        usedBodies.assign(bodyCount, uint8_t(-1));

        waveAssignments.resize(workerCount);

        workerLoad.assign(workerCount, 0);

        while (!remainingIndices.empty())
        {
            const size_t waveStartPass = outPlan.passOffsets.size();
            outPlan.passOffsets.resize(waveStartPass + workerCount, Pass{ 0, 0 });

            std::fill(workerLoad.begin(), workerLoad.end(), 0);

            delayedIndices.clear();
            nextRemainingIndices.clear();

            std::fill(usedBodies.begin(), usedBodies.end(), uint8_t(-1));

            for (auto& vec : waveAssignments)
                vec.clear();

            // Pass 1: Process unallocated or fully matching pairs.
            {
                TRACY_SCOPE_NC("Pass1", PS_AGONY2D::Profiling::Color::Cyan);
                for (size_t idx : remainingIndices)
                {
                    const ObjectPair& c = collisionsPtr[idx];
                    const uint8_t ownerA = usedBodies[c.a];
                    const uint8_t ownerB = usedBodies[c.b];

                    size_t chosenWorker = -1;
                    if (ownerA == ownerB)
                    {
                        if (ownerA == uint8_t(-1))
                        {
                            size_t minLoad = workerLoad[0];
                            chosenWorker = 0;
                            for (size_t i = 1; i < workerCount; i++)
                            {
                                size_t load = workerLoad[i];
                                if (load < minLoad)
                                {
                                    minLoad = load;
                                    chosenWorker = i;
                                }
                            }

                            const uint8_t choosenWorkerU8 = static_cast<uint8_t>(chosenWorker);
                            usedBodies[c.a] = choosenWorkerU8;
                            usedBodies[c.b] = choosenWorkerU8;
                        }
                        else
                        {
                            chosenWorker = static_cast<size_t>(ownerA);
                        }

                        waveAssignments[chosenWorker].push_back(idx);
                        workerLoad[chosenWorker]++;
                    }
                    else
                    {
                        const bool eitherIsUnassigned = ownerA == uint8_t(-1) || ownerB == uint8_t(-1);
                        auto& vec = eitherIsUnassigned ? delayedIndices : nextRemainingIndices;
                        vec.push_back(idx);
                    }
                }
            }

            // Pass 2: Process the partially claimed pairs.
            {
                TRACY_SCOPE_NC("Pass2", PS_AGONY2D::Profiling::Color::Yellow);
                for (size_t idx : delayedIndices)
                {
                    const ObjectPair& c = collisionsPtr[idx];
                    const uint8_t ownerA = usedBodies[c.a];
                    const uint8_t ownerB = usedBodies[c.b];

                    if (ownerA != uint8_t(-1) && ownerB != uint8_t(-1) && ownerA != ownerB)
                    {
                        nextRemainingIndices.push_back(idx);
                        continue;
                    }

                    const uint8_t chosenWorkerU8 = std::min(ownerA, ownerB);
                    const size_t chosenWorker = static_cast<size_t>(chosenWorkerU8);

                    waveAssignments[chosenWorker].push_back(idx);
                    workerLoad[chosenWorker]++;

                    usedBodies[c.a] = chosenWorkerU8;
                    usedBodies[c.b] = chosenWorkerU8;
                }
            }

            {
                TRACY_SCOPE_NC("Append to flatIndices", PS_AGONY2D::Profiling::Color::Magenta);
                for (size_t w = 0; w < workerCount; w++)
                {
                    const size_t passIndex = waveStartPass + w;
                    outPlan.passOffsets[passIndex].start = static_cast<uint32_t>(outPlan.flatIndices.size());
                    outPlan.passOffsets[passIndex].size = static_cast<uint32_t>(waveAssignments[w].size());

                    outPlan.flatIndices.insert(outPlan.flatIndices.end(),
                        waveAssignments[w].begin(),
                        waveAssignments[w].end());
                }
            }

            remainingIndices.swap(nextRemainingIndices);
        }
    }

    void SolvingPlanner::planSimdAwareExecution(
        std::span<const ObjectPair> collisions,
        size_t bodyCount,
        size_t workerCount,
        size_t chunkSize,
        SolvingPlan& outPlan
    )
    {
        const ObjectPair* collisionsPtr = collisions.data();
        const size_t collisionCount = collisions.size();

        outPlan.workerCount = workerCount;
        outPlan.flatIndices.clear();
        outPlan.flatIndices.reserve(collisionCount);
        outPlan.passOffsets.clear();

        if (collisionCount == 0 || workerCount == 0)
            return;

        remainingIndices.resize(collisionCount);
        std::iota(remainingIndices.begin(), remainingIndices.end(), size_t(0));

        delayedIndices.clear();
        nextRemainingIndices.clear();

        usedBodies.assign(bodyCount, uint8_t(-1));

        waveAssignments.resize(workerCount);

        workerLoad.assign(workerCount, 0);

        wavePool.clear();
        simdRetry.clear();

        while (!remainingIndices.empty())
        {
            const size_t waveStartPass = outPlan.passOffsets.size();
            outPlan.passOffsets.resize(waveStartPass + workerCount, Pass{ 0, 0 });

            std::fill(workerLoad.begin(), workerLoad.end(), 0);

            nextRemainingIndices.clear();

            std::fill(usedBodies.begin(), usedBodies.end(), uint8_t(-1));

            for (auto& vec : waveAssignments)
                vec.clear();

            // The pool of items currently being evaluated for this wave.
            wavePool = remainingIndices;

            while (true)
            {
                bool placedAny = false;
                delayedIndices.clear();
                simdRetry.clear();

                // Pass 1: Process unallocated or fully matching pairs first.
                {
                    TRACY_SCOPE_NC("Pass1", PS_AGONY2D::Profiling::Color::Cyan);
                    for (size_t idx : wavePool)
                    {
                        const ObjectPair& c = collisionsPtr[idx];
                        const uint8_t ownerA = usedBodies[c.a];
                        const uint8_t ownerB = usedBodies[c.b];

                        size_t chosenWorker = -1;
                        if (ownerA == ownerB)
                        {
                            if (ownerA == uint8_t(-1))
                            {
                                size_t minLoad = workerLoad[0];
                                chosenWorker = 0;
                                for (size_t i = 1; i < workerCount; i++)
                                {
                                    size_t load = workerLoad[i];
                                    if (load < minLoad)
                                    {
                                        minLoad = load;
                                        chosenWorker = i;
                                    }
                                }
                            }
                            else
                            {
                                chosenWorker = static_cast<size_t>(ownerA);
                            }

                            waveAssignments[chosenWorker].push_back(idx);
                            workerLoad[chosenWorker]++;
                        }
                        else
                        {
                            const bool eitherIsUnassigned = ownerA == uint8_t(-1) || ownerB == uint8_t(-1);
                            auto& vec = eitherIsUnassigned ? delayedIndices : nextRemainingIndices;
                            vec.push_back(idx);
                            continue;
                        }

                        // Evaluate intra-chunk SIMD safety constraints
                        const size_t currSize = waveAssignments[chosenWorker].size();
                        const size_t chunkRem = currSize % chunkSize;
                        bool simdConflict = false;
                        if (chunkRem > 0)
                        {
                            for (size_t i = currSize - chunkRem; i < currSize; ++i)
                            {
                                const size_t prevIdx = waveAssignments[chosenWorker][i];
                                const ObjectPair& pc = collisionsPtr[prevIdx];
                                if (c.a == pc.a || c.a == pc.b || c.b == pc.a || c.b == pc.b)
                                {
                                    simdConflict = true;
                                    break;
                                }
                            }
                        }

                        if (simdConflict)
                        {
                            simdRetry.push_back(idx);
                        }
                        else
                        {
                            waveAssignments[chosenWorker].push_back(idx);
                            workerLoad[chosenWorker]++;

                            usedBodies[c.a] = static_cast<uint8_t>(chosenWorker);
                            usedBodies[c.b] = static_cast<uint8_t>(chosenWorker);
                            placedAny = true;
                        }
                    }
                }

                // Pass 2: Process the partially claimed pairs later in the loop.
                {
                    TRACY_SCOPE_NC("Pass2", PS_AGONY2D::Profiling::Color::Yellow);
                    for (size_t idx : delayedIndices)
                    {
                        const ObjectPair& c = collisionsPtr[idx];
                        const uint8_t ownerA = usedBodies[c.a];
                        const uint8_t ownerB = usedBodies[c.b];

                        if (ownerA != uint8_t(-1) && ownerB != uint8_t(-1) && ownerA != ownerB)
                        {
                            nextRemainingIndices.push_back(idx);
                            continue;
                        }

                        const uint8_t chosenWorkerU8 = std::min(ownerA, ownerB);
                        const size_t chosenWorker = static_cast<size_t>(chosenWorkerU8);

                        // Evaluate intra-chunk SIMD safety constraints.
                        const size_t currSize = waveAssignments[chosenWorker].size();
                        const size_t chunkRem = currSize % chunkSize;
                        bool simdConflict = false;
                        if (chunkRem > 0)
                        {
                            for (size_t i = currSize - chunkRem; i < currSize; ++i)
                            {
                                const size_t prevIdx = waveAssignments[chosenWorker][i];
                                const ObjectPair& pc = collisionsPtr[prevIdx];
                                if (c.a == pc.a || c.a == pc.b || c.b == pc.a || c.b == pc.b)
                                {
                                    simdConflict = true;
                                    break;
                                }
                            }
                        }

                        if (simdConflict)
                        {
                            simdRetry.push_back(idx);
                        }
                        else
                        {
                            waveAssignments[chosenWorker].push_back(idx);
                            workerLoad[chosenWorker]++;

                            usedBodies[c.a] = chosenWorkerU8;
                            usedBodies[c.b] = chosenWorkerU8;
                            placedAny = true;
                        }
                    }
                }

                // Convergence check: if we couldn't place anything new this loop iteration,
                // any remaining simdRetry elements must wait for the next execution wave.
                if (!placedAny)
                {
                    nextRemainingIndices.insert(nextRemainingIndices.end(), simdRetry.begin(), simdRetry.end());
                    break;
                }
                else
                {
                    // Retry the SIMD-deferred items because chunk boundaries have shifted!
                    wavePool = simdRetry;
                }
            }

            {
                TRACY_SCOPE_NC("Append to flatIndices", PS_AGONY2D::Profiling::Color::Magenta);
                for (size_t w = 0; w < workerCount; w++)
                {
                    const size_t passIndex = waveStartPass + w;
                    outPlan.passOffsets[passIndex].start = static_cast<uint32_t>(outPlan.flatIndices.size());
                    outPlan.passOffsets[passIndex].size = static_cast<uint32_t>(waveAssignments[w].size());

                    outPlan.flatIndices.insert(outPlan.flatIndices.end(),
                        waveAssignments[w].begin(),
                        waveAssignments[w].end());
                }
            }

            remainingIndices.swap(nextRemainingIndices);
        }
    }

    void SolvingPlanner::planCacheLineAwareExecution(
        std::span<const ObjectPair> collisions,
        size_t bodyCount,
        size_t workerCount,
        size_t bodyStrideBytes,
        size_t cacheLineBytes,
        SolvingPlan& outPlan)
    {
        const ObjectPair* collisionsPtr = collisions.data();
        const size_t collisionCount = collisions.size();

        outPlan.workerCount = workerCount;
        outPlan.flatIndices.clear();
        outPlan.flatIndices.reserve(collisionCount);
        outPlan.passOffsets.clear();

        if (collisionCount == 0 || workerCount == 0 || bodyStrideBytes == 0 || cacheLineBytes == 0)
            return;

        // How many bodies share a single cache line; workers must own whole groups
        // of these so two workers never write adjacent bodies on the same line.
        const size_t bodiesPerCacheLine = std::max<size_t>(1, cacheLineBytes / bodyStrideBytes);
        const size_t groupCount = (bodyCount + bodiesPerCacheLine - 1) / bodiesPerCacheLine;

        remainingIndices.resize(collisionCount);
        std::iota(remainingIndices.begin(), remainingIndices.end(), size_t(0));

        delayedIndices.clear();
        nextRemainingIndices.clear();

        // Ownership tracked per cache-line group, not per body.
        usedBodies.assign(groupCount, uint8_t(-1));

        waveAssignments.resize(workerCount);

        workerLoad.assign(workerCount, 0);

        while (!remainingIndices.empty())
        {
            const size_t waveStartPass = outPlan.passOffsets.size();
            outPlan.passOffsets.resize(waveStartPass + workerCount, Pass{ 0, 0 });

            std::fill(workerLoad.begin(), workerLoad.end(), 0);

            delayedIndices.clear();
            nextRemainingIndices.clear();

            std::fill(usedBodies.begin(), usedBodies.end(), uint8_t(-1));

            for (auto& vec : waveAssignments)
                vec.clear();

            // Pass 1: Process unallocated or fully matching cache-line groups.
            {
                TRACY_SCOPE_NC("Pass1", PS_AGONY2D::Profiling::Color::Cyan);
                for (size_t idx : remainingIndices)
                {
                    const ObjectPair& c = collisionsPtr[idx];
                    const size_t groupA = c.a / bodiesPerCacheLine;
                    const size_t groupB = c.b / bodiesPerCacheLine;

                    const uint8_t ownerA = usedBodies[groupA];
                    const uint8_t ownerB = usedBodies[groupB];

                    size_t chosenWorker = -1;
                    if (ownerA == ownerB)
                    {
                        if (ownerA == uint8_t(-1))
                        {
                            size_t minLoad = workerLoad[0];
                            chosenWorker = 0;
                            for (size_t i = 1; i < workerCount; i++)
                            {
                                size_t load = workerLoad[i];
                                if (load < minLoad)
                                {
                                    minLoad = load;
                                    chosenWorker = i;
                                }
                            }

                            const uint8_t choosenWorkerU8 = static_cast<uint8_t>(chosenWorker);
                            usedBodies[c.a] = choosenWorkerU8;
                            usedBodies[c.b] = choosenWorkerU8;
                        }
                        else
                        {
                            chosenWorker = static_cast<size_t>(ownerA);
                        }

                        waveAssignments[chosenWorker].push_back(idx);
                        workerLoad[chosenWorker]++;
                    }
                    else
                    {
                        const bool eitherIsUnassigned = ownerA == uint8_t(-1) || ownerB == uint8_t(-1);
                        auto& vec = eitherIsUnassigned ? delayedIndices : nextRemainingIndices;
                        vec.push_back(idx);
                    }
                }
            }

            // Pass 2: Process the partially claimed groups.
            {
                TRACY_SCOPE_NC("Pass2", PS_AGONY2D::Profiling::Color::Yellow);
                for (size_t idx : delayedIndices)
                {
                    const ObjectPair& c = collisionsPtr[idx];
                    const uint8_t ownerA = usedBodies[c.a];
                    const uint8_t ownerB = usedBodies[c.b];

                    if (ownerA != uint8_t(-1) && ownerB != uint8_t(-1) && ownerA != ownerB)
                    {
                        nextRemainingIndices.push_back(idx);
                        continue;
                    }

                    const uint8_t chosenWorkerU8 = std::min(ownerA, ownerB);
                    const size_t chosenWorker = static_cast<size_t>(chosenWorkerU8);

                    waveAssignments[chosenWorker].push_back(idx);
                    workerLoad[chosenWorker]++;

                    usedBodies[c.a] = chosenWorkerU8;
                    usedBodies[c.b] = chosenWorkerU8;
                }
            }

            {
                TRACY_SCOPE_NC("Append to flatIndices", PS_AGONY2D::Profiling::Color::Magenta);
                for (size_t w = 0; w < workerCount; w++)
                {
                    const size_t passIndex = waveStartPass + w;
                    outPlan.passOffsets[passIndex].start = static_cast<uint32_t>(outPlan.flatIndices.size());
                    outPlan.passOffsets[passIndex].size = static_cast<uint32_t>(waveAssignments[w].size());

                    outPlan.flatIndices.insert(outPlan.flatIndices.end(),
                        waveAssignments[w].begin(),
                        waveAssignments[w].end());
                }
            }

            remainingIndices.swap(nextRemainingIndices);
        }
    }

    void SolvingPlanner::planCacheSharingAwareExecution(
        std::span<const ObjectPair> collisions,
        size_t bodyCount,
        size_t workerCount,
        size_t bodyStrideBytes,
        size_t cacheLineBytes,
        SolvingPlan& outPlan,
        const uint8_t* isStatic)
    {
        const ObjectPair* collisionsPtr = collisions.data();
        const size_t collisionCount = collisions.size();

        outPlan.workerCount = workerCount;
        outPlan.flatIndices.clear();
        outPlan.flatIndices.reserve(collisionCount);
        outPlan.passOffsets.clear();

        if (collisionCount == 0 || workerCount == 0 || bodyStrideBytes == 0 || cacheLineBytes == 0)
            return;

        const size_t bodiesPerCacheLine = std::max<size_t>(1, cacheLineBytes / bodyStrideBytes);
        const size_t groupCount = (bodyCount + bodiesPerCacheLine - 1) / bodiesPerCacheLine;

        // Static bodies are never written, so they neither own nor conflict: a constraint maps to the
        // group(s) of its dynamic bodies only, and constraints with no dynamic body are not scheduled.
        const auto isStaticBody = [isStatic](ObjectIndex b) { return isStatic != nullptr && isStatic[b] != 0; };
        const auto groupsOf = [&](const ObjectPair& c, size_t& ga, size_t& gb)
            {
                const ObjectIndex da = isStaticBody(c.a) ? c.b : c.a;
                const ObjectIndex db = isStaticBody(c.b) ? c.a : c.b;
                ga = da / bodiesPerCacheLine;
                gb = db / bodiesPerCacheLine;
            };

        // Build cache-line group adjacency graph.
        adjacency.resize(groupCount);
        for (auto& v : adjacency)
            v.clear();

        for (const auto& c : collisions)
        {
            if (isStaticBody(c.a) && isStaticBody(c.b)) continue;
            size_t ga, gb;
            groupsOf(c, ga, gb);
            if (ga != gb)
            {
                adjacency[ga].push_back(gb);
                adjacency[gb].push_back(ga);
            }
        }

        // Deduplicate adjacency lists.
        for (size_t g = 0; g < groupCount; g++)
        {
            std::sort(adjacency[g].begin(), adjacency[g].end());
            adjacency[g].erase(std::unique(adjacency[g].begin(), adjacency[g].end()), adjacency[g].end());
        }

        // Persistent partitioning via Multi-Source BFS.
        groupOwner.assign(groupCount, uint8_t(-1));
        groupSizes.assign(workerCount, 0);

        if (workerCount == 1)
        {
            std::fill(groupOwner.begin(), groupOwner.end(), static_cast<uint8_t>(0));
        }
        else if (groupCount > 0)
        {
            seeds.resize(groupCount);
            std::iota(seeds.begin(), seeds.end(), 0);

            // Deterministic LCG shuffle for seed candidates.
            uint32_t seedRng = 1234;
            for (size_t i = seeds.size() - 1; i > 0; --i)
            {
                seedRng = seedRng * 1664525u + 1013904223u;
                size_t j = seedRng % (i + 1);
                std::swap(seeds[i], seeds[j]);
            }

            while (!bfsQueue.empty())
                bfsQueue.pop();

            const size_t targetSize = std::max<size_t>(1, static_cast<size_t>(std::round(static_cast<double>(groupCount) / workerCount * 1.15)));

            for (size_t w = 0; w < workerCount && w < groupCount; ++w)
            {
                size_t seedGroup = seeds[w];
                if (groupOwner[seedGroup] == uint8_t(-1))
                {
                    groupOwner[seedGroup] = static_cast<uint8_t>(w);
                    groupSizes[w]++;
                    bfsQueue.push(seedGroup);
                }
            }

            while (!bfsQueue.empty())
            {
                size_t node = bfsQueue.front();
                bfsQueue.pop();

                uint8_t w = groupOwner[node];
                if (groupSizes[w] >= targetSize)
                    continue;

                for (size_t neighbor : adjacency[node])
                {
                    if (groupOwner[neighbor] == uint8_t(-1))
                    {
                        groupOwner[neighbor] = w;
                        groupSizes[w]++;
                        bfsQueue.push(neighbor);
                    }
                }
            }

            // Assign remaining unreached groups to the worker with the smallest region.
            for (size_t g = 0; g < groupCount; ++g)
            {
                if (groupOwner[g] == uint8_t(-1))
                {
                    size_t minWorker = 0;
                    size_t minSize = groupSizes[0];
                    for (size_t w = 1; w < workerCount; ++w)
                    {
                        if (groupSizes[w] < minSize)
                        {
                            minSize = groupSizes[w];
                            minWorker = w;
                        }
                    }
                    groupOwner[g] = static_cast<uint8_t>(minWorker);
                    groupSizes[minWorker]++;
                }
            }
        }

        // Multi-wave execution planner.
        remainingIndices.clear();
        for (size_t idx = 0; idx < collisionCount; idx++)
        {
            const ObjectPair& c = collisionsPtr[idx];
            if (isStaticBody(c.a) && isStaticBody(c.b)) continue;
            remainingIndices.push_back(idx);
        }

        waveOwner.assign(groupCount, uint8_t(-1));
        waveAssignments.resize(workerCount);

        interiorIndices.clear();
        boundaryIndices.clear();

        while (!remainingIndices.empty())
        {
            const size_t waveStartPass = outPlan.passOffsets.size();
            outPlan.passOffsets.resize(waveStartPass + workerCount, Pass{ 0, 0 });

            nextRemainingIndices.clear();
            std::fill(waveOwner.begin(), waveOwner.end(), uint8_t(-1));

            for (auto& vec : waveAssignments)
                vec.clear();

            interiorIndices.clear();
            boundaryIndices.clear();

            for (size_t idx : remainingIndices)
            {
                const ObjectPair& c = collisionsPtr[idx];
                size_t ga, gb;
                groupsOf(c, ga, gb);

                auto& vec = groupOwner[ga] == groupOwner[gb] ? interiorIndices : boundaryIndices;
                vec.push_back(idx);
            }

            // Phase 1: Interior collisions assigned to persistent owners.
            for (size_t idx : interiorIndices)
            {
                const ObjectPair& c = collisionsPtr[idx];
                size_t ga, gb;
                groupsOf(c, ga, gb);
                const uint8_t chosen = groupOwner[ga];

                waveAssignments[chosen].push_back(idx);
                waveOwner[ga] = chosen;
                waveOwner[gb] = chosen;
            }

            // Phase 2 (Pass A): Boundary collisions (unclaimed groups resolved deterministically).
            delayedIndices.clear();
            for (size_t idx : boundaryIndices)
            {
                const ObjectPair& c = collisionsPtr[idx];
                size_t ga, gb;
                groupsOf(c, ga, gb);

                const uint8_t ownerA = waveOwner[ga];
                const uint8_t ownerB = waveOwner[gb];

                uint8_t chosen = uint8_t(-1);

                if (ownerA == ownerB && ownerA != uint8_t(-1))
                {
                    chosen = ownerA;
                }
                else if (ownerA == uint8_t(-1) && ownerB == uint8_t(-1))
                {
                    size_t lo = std::min(ga, gb);
                    size_t hi = std::max(ga, gb);
                    chosen = ((hashGroupPair(lo, hi) & 1) == 0) ? groupOwner[lo] : groupOwner[hi];
                }
                else
                {
                    delayedIndices.push_back(idx);
                    continue;
                }

                waveAssignments[chosen].push_back(idx);
                waveOwner[ga] = chosen;
                waveOwner[gb] = chosen;
            }

            // Phase 2 (Pass B): Boundary collisions (partially claimed groups inherit active worker).
            for (size_t idx : delayedIndices)
            {
                const ObjectPair& c = collisionsPtr[idx];
                size_t ga, gb;
                groupsOf(c, ga, gb);

                const uint8_t ownerA = waveOwner[ga];
                const uint8_t ownerB = waveOwner[gb];

                if ((ownerA == uint8_t(-1) && ownerB == uint8_t(-1)) ||
                    (ownerA != uint8_t(-1) && ownerB != uint8_t(-1) && ownerA != ownerB))
                {
                    nextRemainingIndices.push_back(idx);
                    continue;
                }

                // Branchless worker selection: min() picks whichever is a valid worker ID (< 255)
                const uint8_t chosen = std::min(ownerA, ownerB);

                waveAssignments[chosen].push_back(idx);
                waveOwner[ga] = chosen;
                waveOwner[gb] = chosen;
            }

            {
                TRACY_SCOPE_NC("Append to flatIndices", PS_AGONY2D::Profiling::Color::Magenta);
                for (size_t w = 0; w < workerCount; w++)
                {
                    const size_t passIndex = waveStartPass + w;
                    outPlan.passOffsets[passIndex].start = static_cast<uint32_t>(outPlan.flatIndices.size());
                    outPlan.passOffsets[passIndex].size = static_cast<uint32_t>(waveAssignments[w].size());

                    outPlan.flatIndices.insert(outPlan.flatIndices.end(),
                        waveAssignments[w].begin(),
                        waveAssignments[w].end());
                }
            }

            remainingIndices.swap(nextRemainingIndices);
        }
    }

    void SolvingPlanner::planSingleStandardExecution(const std::vector<ObjectIndex>& objects, size_t bodyCount, size_t workerCount, SolvingPlan& outPlan, const uint8_t* isStatic)
    {
        // A single-body constraint is a pair (b, b): same ownership/conflict rules apply.
        singlePairs.resize(objects.size());
        for (size_t i = 0; i < objects.size(); i++)
        {
            singlePairs[i] = { objects[i], objects[i] };
        }

        planCacheSharingAwareExecution(
            singlePairs, bodyCount, workerCount,
            sizeof(Real), std::hardware_destructive_interference_size,
            outPlan, isStatic);
    }

    void SolvingPlan::printExecutionPlan() const
    {
        const size_t numWaves = (passOffsets.empty() ? 0 : passOffsets.size() / workerCount);
        const int colWidth = 6;

        // Header: wave IDs
        std::cout << std::setw(colWidth) << " ";
        for (size_t wave = 0; wave < numWaves; ++wave)
            std::cout << std::setw(colWidth) << ("Wv" + std::to_string(wave));
        std::cout << "\n";

        // Rows: worker ID + pass sizes
        for (size_t worker = 0; worker < workerCount; ++worker)
        {
            std::cout << std::setw(colWidth - 1) << ("Wk" + std::to_string(worker)) << " ";
            for (size_t wave = 0; wave < numWaves; ++wave)
            {
                size_t idx = wave * workerCount + worker;
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

        std::vector<size_t> totalLoad(workerCount, 0);
        for (size_t w = 0; w < workerCount; ++w)
        {
            for (size_t wave = 0; wave < passOffsets.size() / workerCount; ++wave)
            {
                const Pass& p = passOffsets[wave * workerCount + w];
                totalLoad[w] += p.size;
            }
        }

        size_t maxLoad = *std::max_element(totalLoad.begin(), totalLoad.end());
        size_t scheduled = flatIndices.size();

        if (maxLoad == 0) return 0.0;
        return static_cast<double>(scheduled) / (static_cast<double>(workerCount) * maxLoad) * 100.0;
    }

    bool SolvingPlan::validateNoCrossing(std::span<const ObjectPair> collisions, size_t bodyCount) const
    {
        if (workerCount == 0) return passOffsets.empty();

        const size_t numWaves = passOffsets.size() / workerCount;
        std::vector<int32_t> bodyOwner(bodyCount, -1);

        const ObjectPair* collisionsPtr = collisions.data();

        for (size_t wave = 0; wave < numWaves; ++wave)
        {
            std::fill(bodyOwner.begin(), bodyOwner.end(), -1);

            for (size_t worker = 0; worker < workerCount; ++worker)
            {
                const size_t passIdx = wave * workerCount + worker;
                const Pass& pass = passOffsets[passIdx];

                for (size_t i = 0; i < static_cast<size_t>(pass.size); ++i)
                {
                    size_t collisionIdx = flatIndices[static_cast<size_t>(pass.start) + i];
                    const ObjectPair& c = collisionsPtr[collisionIdx];

                    const int32_t workerId = static_cast<int32_t>(worker);
                    if (bodyOwner[c.a] != -1 && bodyOwner[c.a] != workerId)
                        return false;
                    if (bodyOwner[c.b] != -1 && bodyOwner[c.b] != workerId)
                        return false;

                    bodyOwner[c.a] = workerId;
                    bodyOwner[c.b] = workerId;
                }
            }
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
        total += getVectorMemoryUsage(remainingIndices);
        total += getVectorMemoryUsage(delayedIndices);
        total += getVectorMemoryUsage(nextRemainingIndices);
        total += getVectorMemoryUsage(usedBodies);

        total += getVectorMemoryUsage(waveAssignments);
        for (const auto& assignment : waveAssignments)
        {
            total += getVectorMemoryUsage(assignment);
        }

        total += getVectorMemoryUsage(workerLoad);
        total += getVectorMemoryUsage(wavePool);
        total += getVectorMemoryUsage(simdRetry);

        total += getVectorMemoryUsage(adjacency);
        for (const auto& adj : adjacency)
        {
            total += getVectorMemoryUsage(adj);
        }

        total += getVectorMemoryUsage(groupOwner);
        total += getVectorMemoryUsage(groupSizes);
        total += getVectorMemoryUsage(seeds);
        total += getVectorMemoryUsage(waveOwner);
        total += getVectorMemoryUsage(interiorIndices);
        total += getVectorMemoryUsage(boundaryIndices);
        total += getVectorMemoryUsage(singlePairs);
        // bfsQueue intentionally excluded: std::queue<size_t> exposes no way to
        // inspect its backing container's capacity.

        return total;
    }
}