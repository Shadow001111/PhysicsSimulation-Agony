#pragma once
#include <string>
#include <optional>

#include "robin_hood.h"

class StringIndexer
{
	robin_hood::unordered_flat_map<std::string, size_t> nameToID;
public:
	StringIndexer() = default;
	~StringIndexer() = default;

	StringIndexer(const StringIndexer& other) = delete;
	StringIndexer& operator=(const StringIndexer& other) = delete;
	StringIndexer(StringIndexer&& other) = delete;
	StringIndexer& operator=(StringIndexer&& other) = delete;

	size_t registerAndGetId(const std::string& str);
	bool isRegistered(const std::string& str) const;
	std::optional<size_t> getId(const std::string& str) const;

	const robin_hood::unordered_flat_map<std::string, size_t>& getNameToIDMap() const { return nameToID; };

	void clear();

	bool swapIds(const std::string& a, const std::string& b);
};