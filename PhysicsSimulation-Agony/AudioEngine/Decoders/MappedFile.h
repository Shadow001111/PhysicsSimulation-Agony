#pragma once
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

class MappedFile final
{
public:
    enum class Access
    {
        ReadOnly,
        ReadWrite
    };

    enum class Creation
    {
        OpenExisting,
        OpenOrCreate,
        CreateAlways,
        TruncateExisting
    };
private:
    void* fileHandle = nullptr;
    void* mappingHandle = nullptr;
    std::byte* dataPtr = nullptr;
    std::size_t sizeValue = 0;
    bool writableFlag = false;
public:
    MappedFile() noexcept = default;
    ~MappedFile() { close(); }

    MappedFile(const MappedFile&) = delete;
    MappedFile& operator=(const MappedFile&) = delete;

    MappedFile(MappedFile&& other) noexcept;
    MappedFile& operator=(MappedFile&& other) noexcept;

    bool open(const char* path,
        Access access = Access::ReadOnly,
        Creation creation = Creation::OpenExisting,
        std::size_t mappingSize = 0,
        std::string* error = nullptr);

    bool open(const std::filesystem::path& path,
        Access access = Access::ReadOnly,
        Creation creation = Creation::OpenExisting,
        std::size_t mappingSize = 0,
        std::string* error = nullptr);

    void close() noexcept;

    [[nodiscard]] bool getIsOpen() const noexcept { return dataPtr != nullptr; };
    [[nodiscard]] bool getIsWritable() const noexcept { return writableFlag; };
    [[nodiscard]] std::size_t getSize() const noexcept { return sizeValue; };

    [[nodiscard]] std::byte* getData() noexcept { return dataPtr; };
    [[nodiscard]] const std::byte* getData() const noexcept { return dataPtr; };
};