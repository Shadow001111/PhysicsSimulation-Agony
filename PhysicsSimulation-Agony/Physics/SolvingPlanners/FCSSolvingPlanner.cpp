#include "FCSSolvingPlanner.h"

#include <numeric>

namespace PS_AGONY
{
    void SolvingPlannerBase::setWorkerCount(size_t workerCount) noexcept
    {
        this->workerCount = workerCount;
    }

    void FCSSolvingPlanner::planExecution(const std::vector<BodyPair>& collisions, size_t bodyCount)
    {
        const size_t collisionCount = collisions.size();

        nextRemainingIndices.clear();
        flatIndices.clear();
        flatIndices.reserve(collisionCount);
        passOffsets.clear();
        usedBodies.resize(bodyCount);

        remainingIndices.resize(collisionCount);
        std::iota(remainingIndices.begin(), remainingIndices.end(), size_t(0));

        while (true)
        {
            std::fill(usedBodies.begin(), usedBodies.end(), uint8_t(-1));

            const size_t waveStartPassIndex = passOffsets.size();
            passOffsets.resize(waveStartPassIndex + workerCount, Pass{ 0, 0 });

            const size_t readSize = remainingIndices.size();
            size_t currentPass = 0;
            for (size_t readPos = 0; readPos < readSize; readPos++)
            {
                const size_t collisionIndex = remainingIndices[readPos];
                const BodyPair& collision = collisions[collisionIndex];

                const auto bodyIndexA = collision.a;
                const auto bodyIndexB = collision.b;

                if (usedBodies[bodyIndexA] < currentPass || usedBodies[bodyIndexB] < currentPass)
                {
                    nextRemainingIndices.push_back(collisionIndex);
                    continue;
                }

                usedBodies[bodyIndexA] = static_cast<uint8_t>(currentPass);
                usedBodies[bodyIndexB] = static_cast<uint8_t>(currentPass);

                const size_t passGlobalIndex = waveStartPassIndex + currentPass;
                if (passOffsets[passGlobalIndex].size == 0)
                    passOffsets[passGlobalIndex].start = static_cast<uint32_t>(flatIndices.size());

                flatIndices.push_back(collisionIndex);
                passOffsets[passGlobalIndex].size++;

                if (passOffsets[passGlobalIndex].size >= MAX_INDICES_PER_PASS)
                {
                    currentPass++;
                    if (currentPass >= workerCount)
                    {
                        nextRemainingIndices.insert(
                            nextRemainingIndices.end(),
                            remainingIndices.begin() + (readPos + 1),
                            remainingIndices.end()
                        );
                        break;
                    }
                }
            }

            remainingIndices.clear();
            remainingIndices.swap(nextRemainingIndices);

            if (passOffsets[waveStartPassIndex].size == 0)
            {
                passOffsets.resize(waveStartPassIndex);
                break;
            }

            if (remainingIndices.empty())
                break;
        }
    }

    double FCSSolvingPlanner::computeEfficiency() const
    {
        if (workerCount == 0 || passOffsets.empty())
            return 0.0;

        const size_t waveCount = passOffsets.size() / workerCount;
        const size_t maxPossible = waveCount * workerCount * MAX_INDICES_PER_PASS;

        size_t actual = 0;
        for (const auto& pass : passOffsets)
            actual += pass.size;

        return static_cast<double>(actual) / static_cast<double>(maxPossible) * 100.0;
    }

    bool FCSSolvingPlanner::validateNoCrossing(const std::vector<BodyPair>& collisions, size_t bodyCount) const
    {
        if (workerCount == 0)
            return passOffsets.empty();

        std::vector<int32_t> bodyOwner(bodyCount, -1);

        for (size_t worker = 0; worker < workerCount; ++worker)
        {
            const Pass& pass = passOffsets[worker];

            for (size_t i = 0; i < static_cast<size_t>(pass.size); ++i)
            {
                const size_t collisionIndex = flatIndices[static_cast<size_t>(pass.start) + i];
                const BodyPair& c = collisions[collisionIndex];

                const int32_t workerId = static_cast<int32_t>(worker);

                if (bodyOwner[c.a] != -1 && bodyOwner[c.a] != workerId)
                    return false;
                if (bodyOwner[c.b] != -1 && bodyOwner[c.b] != workerId)
                    return false;

                bodyOwner[c.a] = workerId;
                bodyOwner[c.b] = workerId;
            }
        }

        return true;
    }

    size_t FCSSolvingPlanner::getMemoryUsage() const
    {
        size_t total = 0;
        total += getVectorMemoryUsage(remainingIndices);
        total += getVectorMemoryUsage(nextRemainingIndices);
        total += getVectorMemoryUsage(usedBodies);
        total += getVectorMemoryUsage(flatIndices);
        total += getVectorMemoryUsage(passOffsets);
        return total;
    }
}