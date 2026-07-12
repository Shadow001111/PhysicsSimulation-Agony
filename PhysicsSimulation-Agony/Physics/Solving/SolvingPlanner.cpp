#include "SolvingPlanner.h"
#include "../ContainerUtilities.h"

#include <numeric>
#include <iostream>
#include <iomanip>
#include <string>

namespace PS_AGONY
{
    void SolvingPlanner::planExecution(const std::vector<BodyPair>& collisions, size_t bodyCount)
    {
        const size_t collisionCount = collisions.size();

        flatIndices.clear();
        flatIndices.reserve(collisionCount);
        passOffsets.clear();

        if (collisionCount == 0 || workerCount == 0)
            return;

        remainingIndices.resize(collisionCount);
        std::iota(remainingIndices.begin(), remainingIndices.end(), size_t(0));

        usedBodies.assign(bodyCount, uint8_t(-1));
        nextRemainingIndices.clear();

        std::vector<std::vector<size_t>> waveAssignments(workerCount);

        while (!remainingIndices.empty())
        {
            const size_t waveStartPass = passOffsets.size();
            passOffsets.resize(waveStartPass + workerCount, Pass{ 0, 0 });

            std::vector<size_t> workerLoad(workerCount, 0);
            std::fill(usedBodies.begin(), usedBodies.end(), uint8_t(-1));
            nextRemainingIndices.clear();

            for (auto& vec : waveAssignments)
                vec.clear();

            for (size_t idx : remainingIndices)
            {
                const BodyPair& c = collisions[idx];
                const uint8_t ownerA = usedBodies[c.a];
                const uint8_t ownerB = usedBodies[c.b];

                size_t chosenWorker;

                if (ownerA == ownerB)
                {
                    if (ownerA == uint8_t(-1))
                    {
                        auto it = std::min_element(workerLoad.begin(), workerLoad.end());
                        chosenWorker = static_cast<size_t>(it - workerLoad.begin());
                    }
                    else
                    {
                        chosenWorker = static_cast<size_t>(ownerA);
                    }
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

            for (size_t w = 0; w < workerCount; ++w)
            {
                const size_t passIndex = waveStartPass + w;
                passOffsets[passIndex].start = static_cast<uint32_t>(flatIndices.size());
                passOffsets[passIndex].size = static_cast<uint32_t>(waveAssignments[w].size());

                flatIndices.insert(flatIndices.end(),
                    waveAssignments[w].begin(),
                    waveAssignments[w].end());
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

    bool SolvingPlanner::validateNoCrossing(const std::vector<BodyPair>& collisions, size_t bodyCount) const
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
                    const BodyPair& c = collisions[collisionIdx];

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
        total += getVectorMemoryUsage(nextRemainingIndices);
        total += getVectorMemoryUsage(usedBodies);
        return total;
    }
}