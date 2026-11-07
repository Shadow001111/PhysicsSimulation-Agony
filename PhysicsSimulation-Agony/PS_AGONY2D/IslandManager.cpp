#include "IslandManager.h"

#include <algorithm>

namespace PS_AGONY2D
{
    void IslandManager::reset(BodySoA& bodies)
    {
        const auto range = bodies.range(BodyGroup::Dynamic);
        dynBegin = static_cast<uint32_t>(range.begin);
        dynEnd = static_cast<uint32_t>(range.end);

        const size_t total = bodies.getCount();
        uint32_t* islandId = bodies.islandId.data();
        std::fill(islandId, islandId + dynBegin, NO_ISLAND);
        std::fill(islandId + dynEnd, islandId + total, NO_ISLAND);

        // parentArray[i] >= 0: parent index. parentArray[i] < 0: root, -parentArray[i] is the set size.
        parentArray.assign(dynEnd - dynBegin, -1);
        valid = false;
    }

    void IslandManager::step(std::span<const ObjectPair> edges) noexcept
    {
        const uint32_t count = dynEnd - dynBegin;
        for (const ObjectPair& edge : edges)
        {
            const uint32_t a = edge.a - dynBegin;
            const uint32_t b = edge.b - dynBegin;
            // Unsigned wrap rejects indices below dynBegin as well.
            if (a >= count || b >= count)
                continue;
            unite(a, b);
        }
        valid = false;
    }

    void IslandManager::finalize(BodySoA& bodies)
    {
        const uint32_t count = dynEnd - dynBegin;
        uint32_t* islandId = bodies.islandId.data() + dynBegin;

        // Pass 1: number roots in index order, store their sizes in startArray.
        startArray.resize(count + 1);
        rootToId.resize(count);
        uint32_t k = 0;
        for (uint32_t i = 0; i < count; i++)
        {
            if (parentArray[i] < 0)
            {
                startArray[k] = static_cast<uint32_t>(-parentArray[i]);
                rootToId[i] = k;
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
        membersArray.resize(count);
        for (uint32_t i = 0; i < count; i++)
        {
            const uint32_t id = rootToId[find(i)];
            islandId[i] = id;
            membersArray[startArray[id]++] = i + dynBegin;
        }

        // Cursors now hold island ends; shift back to starts.
        for (uint32_t j = k; j > 0; --j)
            startArray[j] = startArray[j - 1];
        startArray[0] = 0;

        islandCount = k;
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