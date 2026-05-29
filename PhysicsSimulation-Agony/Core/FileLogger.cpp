#include "FileLogger.h"

#include <chrono>
#include <iostream>
#include <string>

void FileLogger::construct(const fs::path& fpath)
{
    filepath = fpath;
    try
    {
        // Create parent directories if they don't exist
        if (filepath.has_parent_path())
        {
            fs::create_directories(filepath.parent_path());
        }

        // Open the file
        file.open(filepath, std::ios::app);
        if (!file.is_open())
        {
            std::cerr << "[FileLogger]: Cannot open log file\n";
        }
    }
    catch (const fs::filesystem_error& e)
    {
        std::cerr << "[FileLogger]: " << e.what() << " (code: " << e.code() << ")" << "\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "[FileLogger]: " << e.what() << "\n";
    }
}

FileLogger::FileLogger(const fs::path& fpath)
{
    construct(fpath);
}

void FileLogger::add(std::string_view line)
{
    std::lock_guard lock(mtx);

    if (!file.is_open())
    {
        std::cerr << "[FileLogger]: Log file is not open: " << filepath << "\n";
        return;
    }

    // Modify line to include timestamp
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::current_zone()->to_local(now);
    auto date = std::format("{:%Y-%m-%d %H:%M}", time);

    std::string lineWithTimestamp = std::format("[{}] {}", date, line);

    try
    {
        file << lineWithTimestamp << "\n";
        file.flush();  // Ensure it's written immediately
    }
    catch (const std::exception& e)
    {
        std::cerr << "[FileLogger]: Write error: " << e.what() << "\n";
    }
}
