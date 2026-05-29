#include "FileStream.h"
#include <limits>
#include <cstring>

FileStream::FileStream(const std::filesystem::path& filepath, Mode mode)
{
    open(filepath, mode);
}

bool FileStream::open(const std::filesystem::path& filepath, Mode mode)
{
    if (m_stream.is_open())
        close();

    std::ios::openmode flags = std::ios::binary;
    if (mode == Mode::Read)
        flags |= std::ios::in;
    else if (mode == Mode::Write)
        flags |= std::ios::out;
    else
        return false; // Mode::Closed not allowed for open

    m_stream.open(filepath, flags);
    if (m_stream.is_open())
        m_mode = mode;
    else
        m_mode = Mode::Closed;

    return m_stream.is_open();
}

void FileStream::close()
{
    m_stream.close();
    m_mode = Mode::Closed;
    m_stream.clear(); // Reset any error flags
}

size_t FileStream::tell()
{
    if (!m_stream)
        return 0;

    std::streampos pos;
    if (m_mode == Mode::Read)
        pos = m_stream.tellg();
    else if (m_mode == Mode::Write)
        pos = m_stream.tellp();
    else
        return 0;

    if (pos < 0)
        return 0;

    return static_cast<size_t>(pos);
}

bool FileStream::seek(size_t position)
{
    return setPos(static_cast<std::streamoff>(position), std::ios::beg);
}

bool FileStream::skip(size_t bytes)
{
    return setPos(static_cast<std::streamoff>(bytes), std::ios::cur);
}

bool FileStream::setPos(std::streamoff offset, std::ios::seekdir dir)
{
    if (!m_stream)
        return false;

    if (m_mode == Mode::Read)
        m_stream.seekg(offset, dir);
    else if (m_mode == Mode::Write)
        m_stream.seekp(offset, dir);
    else
        return false;

    return m_stream.good();
}

bool FileStream::readBytes(void* dest, size_t bytes)
{
    if (m_mode != Mode::Read || !dest || !m_stream || bytes == 0)
        return false;

    m_stream.read(static_cast<char*>(dest), static_cast<std::streamsize>(bytes));
    return m_stream.good();
}

bool FileStream::readString(std::string& out, size_t maxLength)
{
    out.clear();
    if (m_mode != Mode::Read || !m_stream)
        return false;

    char ch;
    while (out.size() < maxLength && m_stream.get(ch))
    {
        out.push_back(ch);
        if (ch == '\0')
            break;
    }
    return m_stream.good() || m_stream.eof();
}

bool FileStream::readLine(std::string& out, char delimiter)
{
    out.clear();
    if (m_mode != Mode::Read || !m_stream)
        return false;
    return std::getline(m_stream, out, delimiter).good();
}

bool FileStream::writeBytes(const void* src, size_t bytes)
{
    if (m_mode != Mode::Write || !src || !m_stream || bytes == 0)
        return false;

    m_stream.write(static_cast<const char*>(src), static_cast<std::streamsize>(bytes));
    return m_stream.good();
}

bool FileStream::writeString(const std::string& str)
{
    return writeBytes(str.data(), str.size());
}

bool FileStream::writeString(const char* str)
{
    if (!str) return false;
    return writeBytes(str, std::strlen(str));
}

bool FileStream::writeLine(const std::string& line, char delimiter)
{
    if (m_mode != Mode::Write || !m_stream)
        return false;
    m_stream << line << delimiter;
    return m_stream.good();
}

bool FileStream::flush()
{
    if (m_mode != Mode::Write || !m_stream)
        return false;
    m_stream.flush();
    return m_stream.good();
}