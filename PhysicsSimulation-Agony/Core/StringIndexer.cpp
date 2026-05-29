#include "StringIndexer.h"

size_t StringIndexer::registerAndGetId(const std::string& str)
{
    auto it = nameToID.find(str);
    if (it == nameToID.end())
    {
        size_t id = nameToID.size();
        nameToID.emplace(str, id);
        return id;
    }
    return it->second;
}

bool StringIndexer::isRegistered(const std::string& str) const
{
    return nameToID.find(str) != nameToID.end();
}

std::optional<size_t> StringIndexer::getId(const std::string& str) const
{
    auto it = nameToID.find(str);
    if (it == nameToID.end())
    {
        return std::nullopt;
    }
    return it->second;
}

void StringIndexer::clear()
{
    nameToID.clear();
}

bool StringIndexer::swapIds(const std::string& a, const std::string& b)
{
    auto itA = nameToID.find(a);
    auto itB = nameToID.find(b);
    auto end = nameToID.end();
    if (itA == end || itB == end)
    {
        return false;
    }

    std::swap(itA->second, itB->second);
    return true;
}
