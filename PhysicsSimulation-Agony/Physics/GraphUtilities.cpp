#include "GraphUtilities.h"

#include <fstream>
#include <iostream>
#include <set>

namespace PS_AGONY::GraphUtilities
{
    Graph buildGraph(const std::vector<GraphLink>& links)
    {
        Graph graph;
        graph.links = links;

        std::set<uint32_t> nodeSet;
        for (const auto& link : links)
        {
            nodeSet.insert(link.a);
            nodeSet.insert(link.b);
        }
        graph.nodes.assign(nodeSet.begin(), nodeSet.end());
        return graph;
    }

    bool exportGraph(const Graph& graph, const std::filesystem::path& filepath)
    {
        const auto parent = filepath.parent_path();
        if (!parent.empty())
        {
            std::error_code ec;
            std::filesystem::create_directories(parent, ec);
            if (ec)
            {
                std::cerr << "Failed to create directories \"" << parent << "\": " << ec.message() << "\n";
                return false;
            }
        }

        std::ofstream out(filepath, std::ios::binary | std::ios::trunc);
        if (!out)
        {
            std::cerr << "Failed to open \"" << filepath << "\" for writing.\n";
            return false;
        }

        const uint32_t numNodes = static_cast<uint32_t>(graph.nodes.size());
        out.write(reinterpret_cast<const char*>(&numNodes), sizeof(numNodes));
        out.write(reinterpret_cast<const char*>(graph.nodes.data()),
            static_cast<std::streamsize>(numNodes * sizeof(uint32_t)));

        const uint32_t numLinks = static_cast<uint32_t>(graph.links.size());
        out.write(reinterpret_cast<const char*>(&numLinks), sizeof(numLinks));
        out.write(reinterpret_cast<const char*>(graph.links.data()),
            static_cast<std::streamsize>(numLinks * sizeof(GraphLink)));

        return static_cast<bool>(out);
    }

    bool importGraph(Graph& graph, const std::filesystem::path& filepath)
    {
        std::ifstream in(filepath, std::ios::binary);
        if (!in)
        {
            std::cerr << "Failed to open \"" << filepath << "\" for reading.\n";
            return false;
        }

        // Read the number of nodes.
        uint32_t numNodes = 0;
        if (!in.read(reinterpret_cast<char*>(&numNodes), sizeof(numNodes)))
        {
            std::cerr << "Failed to read node count from \"" << filepath << "\".\n";
            return false;
        }

        // Read the nodes block.
        graph.nodes.resize(numNodes);
        if (numNodes > 0)
        {
            if (!in.read(reinterpret_cast<char*>(graph.nodes.data()),
                static_cast<std::streamsize>(numNodes * sizeof(uint32_t))))
            {
                std::cerr << "Failed to read nodes from \"" << filepath << "\".\n";
                return false;
            }
        }

        // Read the number of links.
        uint32_t numLinks = 0;
        if (!in.read(reinterpret_cast<char*>(&numLinks), sizeof(numLinks)))
        {
            std::cerr << "Failed to read link count from \"" << filepath << "\".\n";
            return false;
        }

        // Read the links block.
        graph.links.resize(numLinks);
        if (numLinks > 0)
        {
            if (!in.read(reinterpret_cast<char*>(graph.links.data()),
                static_cast<std::streamsize>(numLinks * sizeof(GraphLink))))
            {
                std::cerr << "Failed to read links from \"" << filepath << "\".\n";
                return false;
            }
        }

        return true;
    }

    bool exportGraphToCSV(const Graph& graph, const std::filesystem::path& filepath)
    {
        const auto parent = filepath.parent_path();
        if (!parent.empty())
        {
            std::error_code ec;
            std::filesystem::create_directories(parent, ec);
            if (ec)
            {
                std::cerr << "Failed to create directories \"" << parent << "\": " << ec.message() << "\n";
                return false;
            }
        }

        std::ofstream out(filepath, std::ios::trunc);
        if (!out)
        {
            std::cerr << "Failed to open \"" << filepath << "\" for writing CSV.\n";
            return false;
        }

        out << "source,target\n";
        for (const auto& link : graph.links)
        {
            out << link.a << "," << link.b << "\n";
        }

        return static_cast<bool>(out);
    }
}