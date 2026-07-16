#pragma once
#include <cstdint>
#include <filesystem>
#include <vector>

namespace PS_AGONY::GraphUtilities
{
    struct GraphLink
    {
        uint32_t a;
        uint32_t b;
    };

    struct Graph
    {
        std::vector<uint32_t> nodes; // Unique, sorted node ids.
        std::vector<GraphLink> links;
    };

    // Derives the set of nodes that actually exist from the links, since
    // a node "exists" iff it appears in at least one link.
    Graph buildGraph(const std::vector<GraphLink>& links);

    // Writes the graph to `filepath`, creating any missing parent
    // directories first. Returns false on failure.
    bool exportGraph(const Graph& graph, const std::filesystem::path& filepath);
}
