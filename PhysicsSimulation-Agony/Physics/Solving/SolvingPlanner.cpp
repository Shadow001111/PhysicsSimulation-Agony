#include "SolvingPlanner.h"
#include "../ContainerUtilities.h"

#include "Ecstasy/Core/TracyProfiler.h"

#include <numeric>
#include <iostream>
#include <iomanip>
#include <string>

namespace PS_AGONY
{
    void SolvingPlanner::planStandardExecution(const std::vector<ObjectPair>& collisions, size_t bodyCount)
    {
        const size_t collisionCount = collisions.size();

        flatIndices.clear();
        flatIndices.reserve(collisionCount);
        passOffsets.clear();

        if (collisionCount == 0 || workerCount == 0)
            return;

        remainingIndices.resize(collisionCount);
        std::iota(remainingIndices.begin(), remainingIndices.end(), size_t(0));

        delayedIndices.clear();
        nextRemainingIndices.clear();

        usedBodies.assign(bodyCount, uint8_t(-1));

        std::vector<std::vector<size_t>> waveAssignments(workerCount);

        std::vector<size_t> workerLoad(workerCount);

        while (!remainingIndices.empty())
        {
            const size_t waveStartPass = passOffsets.size();
            passOffsets.resize(waveStartPass + workerCount, Pass{ 0, 0 });

            std::fill(workerLoad.begin(), workerLoad.end(), 0);

            delayedIndices.clear();
            nextRemainingIndices.clear();

            std::fill(usedBodies.begin(), usedBodies.end(), uint8_t(-1));

            for (auto& vec : waveAssignments)
                vec.clear();

            // Pass 1: Process unallocated or fully matching pairs.
            {
                TRACY_SCOPE_NC("Pass1", Ecstasy::Core::Color::Cyan);
                for (size_t idx : remainingIndices)
                {
                    const ObjectPair& c = collisions[idx];
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
                    }
                    else if (ownerA == uint8_t(-1) || ownerB == uint8_t(-1))
                    {
                        // Delay on second pass.
                        delayedIndices.push_back(idx);
                        continue;
                    }
                    else
                    {
                        nextRemainingIndices.push_back(idx);
                        continue;
                    }

                    waveAssignments[chosenWorker].push_back(idx);
                    workerLoad[chosenWorker]++;

                    usedBodies[c.a] = static_cast<uint8_t>(chosenWorker);
                    usedBodies[c.b] = static_cast<uint8_t>(chosenWorker);
                }
            }

            // Pass 2: Process the partially claimed pairs.
            {
                TRACY_SCOPE_NC("Pass2", Ecstasy::Core::Color::Yellow);
                for (size_t idx : delayedIndices)
                {
                    const ObjectPair& c = collisions[idx];
                    const uint8_t ownerA = usedBodies[c.a];
                    const uint8_t ownerB = usedBodies[c.b];

                    size_t chosenWorker;

                    if (ownerA == ownerB)
                    {
                        chosenWorker = static_cast<size_t>(ownerA);
                    }
                    else if (ownerA == uint8_t(-1))
                    {
                        chosenWorker = static_cast<size_t>(ownerB);
                    }
                    else if (ownerB == uint8_t(-1))
                    {
                        chosenWorker = static_cast<size_t>(ownerA);
                    }
                    else
                    {
                        nextRemainingIndices.push_back(idx);
                        continue;
                    }

                    waveAssignments[chosenWorker].push_back(idx);
                    workerLoad[chosenWorker]++;

                    usedBodies[c.a] = static_cast<uint8_t>(chosenWorker);
                    usedBodies[c.b] = static_cast<uint8_t>(chosenWorker);
                }
            }

            {
                TRACY_SCOPE_NC("Append to flatIndices", Ecstasy::Core::Color::Magenta);
                for (size_t w = 0; w < workerCount; w++)
                {
                    const size_t passIndex = waveStartPass + w;
                    passOffsets[passIndex].start = static_cast<uint32_t>(flatIndices.size());
                    passOffsets[passIndex].size = static_cast<uint32_t>(waveAssignments[w].size());

                    flatIndices.insert(flatIndices.end(),
                        waveAssignments[w].begin(),
                        waveAssignments[w].end());
                }
            }

            remainingIndices.swap(nextRemainingIndices);
        }
    }

    void SolvingPlanner::planSimdAwareExecution(const std::vector<ObjectPair>& collisions, size_t bodyCount, size_t chunkSize)
    {
        const size_t collisionCount = collisions.size();

        flatIndices.clear();
        flatIndices.reserve(collisionCount);
        passOffsets.clear();

        if (collisionCount == 0 || workerCount == 0)
            return;

        remainingIndices.resize(collisionCount);
        std::iota(remainingIndices.begin(), remainingIndices.end(), size_t(0));

        delayedIndices.clear();
        nextRemainingIndices.clear();

        usedBodies.assign(bodyCount, uint8_t(-1));

        std::vector<std::vector<size_t>> waveAssignments(workerCount);

        std::vector<size_t> workerLoad(workerCount);

        std::vector<size_t> wavePool;
        std::vector<size_t> simdRetry;

        while (!remainingIndices.empty())
        {
            const size_t waveStartPass = passOffsets.size();
            passOffsets.resize(waveStartPass + workerCount, Pass{ 0, 0 });

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
                    TRACY_SCOPE_NC("Pass1", Ecstasy::Core::Color::Cyan);
                    for (size_t idx : wavePool)
                    {
                        const ObjectPair& c = collisions[idx];
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
                        }
                        else if (ownerA == uint8_t(-1) || ownerB == uint8_t(-1))
                        {
                            // Defer partially claimed pairs to Pass 2
                            delayedIndices.push_back(idx);
                            continue;
                        }
                        else
                        {
                            // Direct wave structural conflict (different workers)
                            nextRemainingIndices.push_back(idx);
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
                                const ObjectPair& pc = collisions[prevIdx];
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
                    TRACY_SCOPE_NC("Pass2", Ecstasy::Core::Color::Yellow);
                    for (size_t idx : delayedIndices)
                    {
                        const ObjectPair& c = collisions[idx];
                        const uint8_t ownerA = usedBodies[c.a];
                        const uint8_t ownerB = usedBodies[c.b];

                        size_t chosenWorker = -1;
                        if (ownerA == ownerB)
                        {
                            chosenWorker = static_cast<size_t>(ownerA);
                        }
                        else if (ownerA == uint8_t(-1))
                        {
                            chosenWorker = static_cast<size_t>(ownerB);
                        }
                        else if (ownerB == uint8_t(-1))
                        {
                            chosenWorker = static_cast<size_t>(ownerA);
                        }
                        else
                        {
                            nextRemainingIndices.push_back(idx);
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
                                const ObjectPair& pc = collisions[prevIdx];
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
                TRACY_SCOPE_NC("Append to flatIndices", Ecstasy::Core::Color::Magenta);
                for (size_t w = 0; w < workerCount; w++)
                {
                    const size_t passIndex = waveStartPass + w;
                    passOffsets[passIndex].start = static_cast<uint32_t>(flatIndices.size());
                    passOffsets[passIndex].size = static_cast<uint32_t>(waveAssignments[w].size());

                    flatIndices.insert(flatIndices.end(),
                        waveAssignments[w].begin(),
                        waveAssignments[w].end());
                }
            }

            remainingIndices.swap(nextRemainingIndices);
        }
    }

    void SolvingPlanner::planCacheLineAwareExecution(
        const std::vector<ObjectPair>& collisions,
        size_t bodyCount,
        size_t bodyStrideBytes,
        size_t cacheLineBytes)
    {
        const size_t collisionCount = collisions.size();

        flatIndices.clear();
        flatIndices.reserve(collisionCount);
        passOffsets.clear();

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

        std::vector<std::vector<size_t>> waveAssignments(workerCount);
        std::vector<size_t> workerLoad(workerCount);

        while (!remainingIndices.empty())
        {
            const size_t waveStartPass = passOffsets.size();
            passOffsets.resize(waveStartPass + workerCount, Pass{ 0, 0 });

            std::fill(workerLoad.begin(), workerLoad.end(), 0);

            delayedIndices.clear();
            nextRemainingIndices.clear();

            std::fill(usedBodies.begin(), usedBodies.end(), uint8_t(-1));

            for (auto& vec : waveAssignments)
                vec.clear();

            // Pass 1: Process unallocated or fully matching cache-line groups.
            {
                TRACY_SCOPE_NC("Pass1", Ecstasy::Core::Color::Cyan);
                for (size_t idx : remainingIndices)
                {
                    const ObjectPair& c = collisions[idx];
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
                        }
                        else
                        {
                            chosenWorker = static_cast<size_t>(ownerA);
                        }
                    }
                    else if (ownerA == uint8_t(-1) || ownerB == uint8_t(-1))
                    {
                        // Delay on second pass.
                        delayedIndices.push_back(idx);
                        continue;
                    }
                    else
                    {
                        nextRemainingIndices.push_back(idx);
                        continue;
                    }

                    waveAssignments[chosenWorker].push_back(idx);
                    workerLoad[chosenWorker]++;

                    usedBodies[groupA] = static_cast<uint8_t>(chosenWorker);
                    usedBodies[groupB] = static_cast<uint8_t>(chosenWorker);
                }
            }

            // Pass 2: Process the partially claimed groups.
            {
                TRACY_SCOPE_NC("Pass2", Ecstasy::Core::Color::Yellow);
                for (size_t idx : delayedIndices)
                {
                    const ObjectPair& c = collisions[idx];
                    const size_t groupA = c.a / bodiesPerCacheLine;
                    const size_t groupB = c.b / bodiesPerCacheLine;

                    const uint8_t ownerA = usedBodies[groupA];
                    const uint8_t ownerB = usedBodies[groupB];

                    size_t chosenWorker;

                    if (ownerA == ownerB)
                    {
                        chosenWorker = static_cast<size_t>(ownerA);
                    }
                    else if (ownerA == uint8_t(-1))
                    {
                        chosenWorker = static_cast<size_t>(ownerB);
                    }
                    else if (ownerB == uint8_t(-1))
                    {
                        chosenWorker = static_cast<size_t>(ownerA);
                    }
                    else
                    {
                        nextRemainingIndices.push_back(idx);
                        continue;
                    }

                    waveAssignments[chosenWorker].push_back(idx);
                    workerLoad[chosenWorker]++;

                    usedBodies[groupA] = static_cast<uint8_t>(chosenWorker);
                    usedBodies[groupB] = static_cast<uint8_t>(chosenWorker);
                }
            }

            {
                TRACY_SCOPE_NC("Append to flatIndices", Ecstasy::Core::Color::Magenta);
                for (size_t w = 0; w < workerCount; w++)
                {
                    const size_t passIndex = waveStartPass + w;
                    passOffsets[passIndex].start = static_cast<uint32_t>(flatIndices.size());
                    passOffsets[passIndex].size = static_cast<uint32_t>(waveAssignments[w].size());

                    flatIndices.insert(flatIndices.end(),
                        waveAssignments[w].begin(),
                        waveAssignments[w].end());
                }
            }

            remainingIndices.swap(nextRemainingIndices);
        }
    }

    void SolvingPlanner::printExecutionPlan()
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

    double SolvingPlanner::computeEfficiency() const
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

    bool SolvingPlanner::validateNoCrossing(const std::vector<ObjectPair>& collisions, size_t bodyCount) const
    {
        if (workerCount == 0) return passOffsets.empty();

        const size_t numWaves = passOffsets.size() / workerCount;
        std::vector<int32_t> bodyOwner(bodyCount, -1);

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
                    const ObjectPair& c = collisions[collisionIdx];

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

    size_t SolvingPlanner::getMemoryUsage() const
    {
        size_t total = 0;
        total += getVectorMemoryUsage(flatIndices);
        total += getVectorMemoryUsage(passOffsets);
        total += getVectorMemoryUsage(remainingIndices);
        total += getVectorMemoryUsage(delayedIndices);
        total += getVectorMemoryUsage(nextRemainingIndices);
        total += getVectorMemoryUsage(usedBodies);
        return total;
    }
}