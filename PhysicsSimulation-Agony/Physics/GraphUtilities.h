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

    // Reads the graph from `filepath` into the provided `graph` reference.
    // Returns false if the file cannot be read or is corrupted.
    bool importGraph(Graph& graph, const std::filesystem::path& filepath);

    // Writes the graph links to a CSV file optimized for Cosmograph.
    // Creates missing parent directories first. Returns false on failure.
    bool exportGraphToCSV(const Graph& graph, const std::filesystem::path& filepath);
}