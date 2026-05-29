#pragma once
#include <fstream>
#include <filesystem>
#include <mutex>

namespace fs = std::filesystem;

class FileLogger
{
    std::ofstream file;
    std::mutex mtx;
    fs::path filepath;

    void construct(const fs::path& fpath);
public:
    explicit FileLogger(const fs::path& fpath);

    void add(std::string_view line);
};