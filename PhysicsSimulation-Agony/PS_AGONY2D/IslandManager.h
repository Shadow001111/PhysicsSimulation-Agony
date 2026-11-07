#pragma once
#include "PS_AGONY2D/SoA/BodySoA.h"
#include "PS_AGONY2D/Types/Types.h"
#include "PS_AGONY2D/ContainerUtilities.h"

#include <cstdint>
#include <span>
#include <vector>

namespace PS_AGONY2D
{
    // Builds islands of Dynamic bodies.
    // Usage per physics step: reset() once, step(edges) any number of times, finalize() when islands are needed.
    class IslandManager
    {
        CacheLineAlignedVector<int32_t> parentArray;  // Local DSU (index - dynBegin).
        CacheLineAlignedVector<uint32_t> rootToId;    // Local root -> island id (finalize scratch).
        CacheLineAlignedVector<uint32_t> startArray;
        CacheLineAlignedVector<uint32_t> membersArray; // Global body indices.
        uint32_t dynBegin = 0;
        uint32_t dynEnd = 0;
        uint32_t islandCount = 0;
        bool valid = false;
    public:
        static constexpr uint32_t NO_ISLAND = -1;

        // Preparation before the first step: every Dynamic body is its own set.
        // Sets islandId of non-Dynamic bodies to NO_ISLAND. Islands are stale until finalize().
        void reset(BodySoA& bodies);

        // Unites bodies of the given edges. Incremental: can be called repeatedly after reset().
        // Edges with a non-Dynamic body are ignored.
        void step(std::span<const ObjectPair> edges) noexcept;

        // O(n). Fills bodies.islandId (0..K-1 for Dynamic) and the island member lists.
        // Island ids are ordered by DSU root index (deterministic, not stable between steps).
        // Does not modify the DSU: step() may continue afterwards.
        void finalize(BodySoA& bodies);

        // Call at the start of a physics step. Islands are stale until finalize() runs.
        void invalidate() noexcept { valid = false; }
        bool isValid() const noexcept { return valid; }

        uint32_t getIslandCount() const noexcept { return islandCount; }

        // Bodies of island i (global indices), ascending uint32_t.
        std::span<const uint32_t> island(uint32_t i) const noexcept
        {
            return { membersArray.data() + startArray[i], startArray[i + 1] - startArray[i] };
        }
    private:
        // Iterative path halving. Valid for both roots and non-roots.
        uint32_t find(uint32_t x) noexcept;

        void unite(uint32_t a, uint32_t b) noexcept;
    };
}