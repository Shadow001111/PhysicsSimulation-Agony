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
    class IslandManager
    {
        CacheLineAlignedVector<int32_t> parentArray;
        CacheLineAlignedVector<uint32_t> startArray;
        CacheLineAlignedVector<uint32_t> membersArray;
        uint32_t islandCount = 0;
        bool valid = false;
    public:
        static constexpr uint32_t NO_ISLAND = -1;

        // Fills bodies.islandId for every body: 0..K-1 for Dynamic, NoIsland for the rest.
        // Island ids are ordered by lowest member index (deterministic).
        // Dynamic bodies without edges become singleton islands.
        // Edges with a non-Dynamic endpoint are ignored.
        void build(BodySoA& bodies, std::span<const ObjectPair> edges);

        // Every Dynamic body is its own island (id = index). Used when there are no edges.
        void buildSingletons(BodySoA& bodies);

        // Call at the start of a physics step. Islands are stale until build/buildSingletons runs.
        void invalidate() noexcept { valid = false; }
        bool isValid() const noexcept { return valid; }

        uint32_t getIslandCount() const noexcept { return islandCount; }

        // Bodies of island i, ascending uint32_t.
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