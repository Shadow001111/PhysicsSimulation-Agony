#pragma once
#include <filesystem>
#include <fstream>
#include <string>
#include <type_traits>

class FileStream
{
public:
    enum class Mode { Closed, Read, Write };

    FileStream() = default;

    // Open with explicit mode
    explicit FileStream(const std::filesystem::path& filepath, Mode mode);
    bool open(const std::filesystem::path& filepath, Mode mode);
    void close();

    // Rule of 5 - movable, not copyable
    FileStream(const FileStream&) = delete;
    FileStream& operator=(const FileStream&) = delete;
    FileStream(FileStream&& other) noexcept = default;
    FileStream& operator=(FileStream&& other) noexcept = default;

    // State checking
    explicit operator bool() const noexcept { return m_stream.good(); }
    bool isOpen() const noexcept { return m_stream.is_open(); }
    bool isReading() const noexcept { return m_mode == Mode::Read; }
    bool isWriting() const noexcept { return m_mode == Mode::Write; }
    bool isEOF() const noexcept { return m_stream.eof(); }

    // Position management (works for both read and write)
    size_t tell();
    bool seek(size_t position);
    bool skip(size_t bytes);
    bool rewind() { return seek(0); }

    // ---- Reading methods (only valid in Read mode) ----
    template<typename T>
    bool readObjects(T* dest, size_t count = 1);
    bool readBytes(void* dest, size_t bytes);
    bool readString(std::string& out, size_t maxLength = 1024);
    bool readLine(std::string& out, char delimiter = '\n');

    // ---- Writing methods (only valid in Write mode) ----
    template<typename T>
    bool writeObjects(const T* src, size_t count = 1);
    bool writeBytes(const void* src, size_t bytes);
    bool writeString(const std::string& str);
    bool writeString(const char* str);
    bool writeLine(const std::string& line, char delimiter = '\n');
    bool flush();

    // Access underlying stream (for advanced usage)
    std::fstream& stream() noexcept { return m_stream; }
    const std::fstream& stream() const noexcept { return m_stream; }

    void clearErrors() { m_stream.clear(); }

private:
    std::fstream m_stream;
    Mode m_mode = Mode::Closed;

    // Helper to set stream position based on mode
    std::streampos getPos() const;
    bool setPos(std::streamoff offset, std::ios::seekdir dir = std::ios::beg);
};

// ---- Template implementations ----
template<typename T>
bool FileStream::readObjects(T* dest, size_t count) {
    static_assert(std::is_trivially_copyable_v<T>,
        "Type must be trivially copyable for binary read");
    if (m_mode != Mode::Read || !dest || !m_stream)
        return false;
    return readBytes(dest, sizeof(T) * count);
}

template<typename T>
bool FileStream::writeObjects(const T* src, size_t count) {
    static_assert(std::is_trivially_copyable_v<T>,
        "Type must be trivially copyable for binary write");
    if (m_mode != Mode::Write || !src || !m_stream)
        return false;
    return writeBytes(src, sizeof(T) * count);
}