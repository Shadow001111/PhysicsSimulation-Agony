#include "IslandManager.h"

#include <numeric>

namespace PS_AGONY2D
{
    void IslandManager::build(BodySoA& bodies, std::span<const ObjectPair> edges)
    {
        const uint32_t dynamicBodyCount = bodies.groupCounts[0];
        if (edges.empty() || dynamicBodyCount == 0)
        {
            buildSingletons(bodies);
            return;
        }

        const size_t total = bodies.getCount();
        uint32_t* islandId = bodies.islandId.data();

        // Fill other body groups.
        if (dynamicBodyCount < total)
        {
            std::fill(islandId + dynamicBodyCount, islandId + total, NO_ISLAND);
        }

        // parentArray[i] >= 0: parent index. parentArray[i] < 0: root, -parentArray[i] is the set size.
        parentArray.assign(dynamicBodyCount, -1);

        for (const ObjectPair& edge : edges)
        {
            if (edge.a >= dynamicBodyCount || edge.b >= dynamicBodyCount)
                continue;
            unite(edge.a, edge.b);
        }

        // Pass 1: number roots, store their sizes in startArray, mark root as -(id + 1).
        startArray.resize(dynamicBodyCount + 1);
        uint32_t k = 0;
        for (uint32_t i = 0; i < dynamicBodyCount; i++)
        {
            if (parentArray[i] < 0)
            {
                startArray[k] = static_cast<uint32_t>(-parentArray[i]);
                parentArray[i] = -static_cast<int32_t>(k) - 1;
                k++;
            }
        }

        // Exclusive prefix sum.
        uint32_t sum = 0;
        for (uint32_t j = 0; j < k; j++)
        {
            const uint32_t s = startArray[j];
            startArray[j] = sum;
            sum += s;
        }
        startArray[k] = sum;

        // Pass 2: scatter (counting sort). startArray[id] is used as a cursor.
        membersArray.resize(dynamicBodyCount);
        for (uint32_t i = 0; i < dynamicBodyCount; i++)
        {
            const uint32_t root = find(i);
            const uint32_t id = static_cast<uint32_t>(-parentArray[root] - 1);
            islandId[i] = id;
            membersArray[startArray[id]++] = i;
        }

        // Cursors now hold island ends; shift back to starts.
        for (uint32_t j = k; j > 0; --j)
            startArray[j] = startArray[j - 1];
        startArray[0] = 0;

        islandCount = k;
        valid = true;
    }

    void IslandManager::buildSingletons(BodySoA& bodies)
    {
        const uint32_t dynamicBodyCount = bodies.groupCounts[0];
        const size_t total = bodies.getCount();
        uint32_t* islandId = bodies.islandId.data();

        if (dynamicBodyCount < total)
            std::fill(islandId + dynamicBodyCount, islandId + total, NO_ISLAND);
        std::iota(islandId, islandId + dynamicBodyCount, 0u);

        startArray.resize(dynamicBodyCount + 1);
        std::iota(startArray.begin(), startArray.end(), 0u);
        membersArray.resize(dynamicBodyCount);
        std::iota(membersArray.begin(), membersArray.end(), uint32_t(0));

        islandCount = dynamicBodyCount;
        valid = true;
    }

    uint32_t IslandManager::find(uint32_t x) noexcept
    {
        int32_t* parentArrayPtr = parentArray.data();
        while (parentArrayPtr[x] >= 0)
        {
            const uint32_t parent = static_cast<uint32_t>(parentArrayPtr[x]);
            const int32_t grandparent = parentArrayPtr[parent];
            if (grandparent < 0) return parent;
            parentArrayPtr[x] = grandparent;
            x = static_cast<uint32_t>(grandparent);
        }
        return x;
    }

    void IslandManager::unite(uint32_t a, uint32_t b) noexcept
    {
        uint32_t rootA = find(a);
        uint32_t rootB = find(b);
        if (rootA == rootB) return;
        // More negative = larger set. Attach smaller under larger.
        if (parentArray[rootA] > parentArray[rootB])
            std::swap(rootA, rootB);
        parentArray[rootA] += parentArray[rootB];
        parentArray[rootB] = static_cast<int32_t>(rootA);
    }
}
