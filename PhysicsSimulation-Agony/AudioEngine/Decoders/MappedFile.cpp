#include "MappedFile.h"

#include <cerrno>
#include <cstring>
#include <utility>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOGDI
#define NOUSER
#define NOMINMAX
#define NOSOCKET
#define NOCRYPT
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace
{
    void setError(std::string* error, const char* message)
    {
        if (error)
        {
            *error = message ? message : "unknown error";
        }
    }

    #if defined(_WIN32)

    std::string lastSystemErrorMessage()
    {
        const DWORD code = GetLastError();

        LPSTR buffer = nullptr;
        const DWORD flags =
            FORMAT_MESSAGE_ALLOCATE_BUFFER |
            FORMAT_MESSAGE_FROM_SYSTEM |
            FORMAT_MESSAGE_IGNORE_INSERTS;

        const DWORD len = FormatMessageA(
            flags,
            nullptr,
            code,
            MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
            reinterpret_cast<LPSTR>(&buffer),
            0,
            nullptr);

        std::string result = "Windows error ";
        result += std::to_string(code);

        if (len != 0 && buffer != nullptr)
        {
            result += ": ";
            result.append(buffer, buffer + len);
            LocalFree(buffer);

            while (!result.empty() &&
                (result.back() == '\r' || result.back() == '\n' || result.back() == ' '))
            {
                result.pop_back();
            }
        }

        return result;
    }

    bool getFileSize(HANDLE file, std::uint64_t& size, std::string* error)
    {
        LARGE_INTEGER li{};

        if (!GetFileSizeEx(file, &li))
        {
            setError(error, lastSystemErrorMessage().c_str());
            return false;
        }

        if (li.QuadPart < 0)
        {
            setError(error, "negative file size");
            return false;
        }

        size = static_cast<std::uint64_t>(li.QuadPart);
        return true;
    }

    bool resizeFile(HANDLE file, std::uint64_t size, std::string* error)
    {
        LARGE_INTEGER li{};
        li.QuadPart = static_cast<LONGLONG>(size);

        if (!SetFilePointerEx(file, li, nullptr, FILE_BEGIN))
        {
            setError(error, lastSystemErrorMessage().c_str());
            return false;
        }

        if (!SetEndOfFile(file))
        {
            setError(error, lastSystemErrorMessage().c_str());
            return false;
        }

        return true;
    }

    #else

    std::string lastSystemErrorMessage()
    {
        return std::strerror(errno);
    }

    bool getFileSize(int file, std::size_t& size, std::string* error)
    {
        struct stat st {};

        if (fstat(file, &st) != 0)
        {
            setError(error, lastSystemErrorMessage().c_str());
            return false;
        }

        if (st.st_size < 0)
        {
            setError(error, "negative file size");
            return false;
        }

        size = static_cast<std::size_t>(st.st_size);
        return true;
    }

    bool resizeFile(int file, std::size_t size, std::string* error)
    {
        if (ftruncate(file, static_cast<off_t>(size)) != 0)
        {
            setError(error, lastSystemErrorMessage().c_str());
            return false;
        }

        return true;
    }

    #endif
}

MappedFile::MappedFile(MappedFile&& other) noexcept
    : fileHandle(other.fileHandle)
    , mappingHandle(other.mappingHandle)
    , dataPtr(other.dataPtr)
    , sizeValue(other.sizeValue)
    , writableFlag(other.writableFlag)
{
    other.fileHandle = nullptr;
    other.mappingHandle = nullptr;
    other.dataPtr = nullptr;
    other.sizeValue = 0;
    other.writableFlag = false;
}

MappedFile& MappedFile::operator=(MappedFile&& other) noexcept
{
    if (this != &other)
    {
        close();

        fileHandle = other.fileHandle;
        mappingHandle = other.mappingHandle;
        dataPtr = other.dataPtr;
        sizeValue = other.sizeValue;
        writableFlag = other.writableFlag;

        other.fileHandle = nullptr;
        other.mappingHandle = nullptr;
        other.dataPtr = nullptr;
        other.sizeValue = 0;
        other.writableFlag = false;
    }

    return *this;
}

bool MappedFile::open(const char* path,
    Access access,
    Creation creation,
    std::size_t mappingSize,
    std::string* error)
{
    if (path == nullptr || *path == '\0')
    {
        setError(error, "path is null or empty");
        return false;
    }

    close();

#if defined(_WIN32)
    const bool writable = (access == Access::ReadWrite);
    writableFlag = writable;

    DWORD desiredAccess = GENERIC_READ;
    DWORD shareMode = FILE_SHARE_READ;
    DWORD creationDisposition = OPEN_EXISTING;
    DWORD protect = PAGE_READONLY;
    DWORD mapAccess = FILE_MAP_READ;

    if (writable)
    {
        desiredAccess |= GENERIC_WRITE;
        shareMode |= FILE_SHARE_WRITE;
        protect = PAGE_READWRITE;
        mapAccess = FILE_MAP_READ | FILE_MAP_WRITE;
    }

    switch (creation)
    {
    case Creation::OpenExisting:
        creationDisposition = OPEN_EXISTING;
        break;
    case Creation::OpenOrCreate:
        creationDisposition = OPEN_ALWAYS;
        break;
    case Creation::CreateAlways:
        creationDisposition = CREATE_ALWAYS;
        break;
    case Creation::TruncateExisting:
        creationDisposition = TRUNCATE_EXISTING;
        break;
    }

    const std::filesystem::path filePath(path);
    const std::wstring widePath = filePath.wstring();

    HANDLE file = CreateFileW(
        widePath.c_str(),
        desiredAccess,
        shareMode,
        nullptr,
        creationDisposition,
        FILE_ATTRIBUTE_NORMAL,
        nullptr);

    if (file == INVALID_HANDLE_VALUE)
    {
        setError(error, lastSystemErrorMessage().c_str());
        return false;
    }

    std::uint64_t currentSize = 0;
    if (!getFileSize(file, currentSize, error))
    {
        CloseHandle(file);
        return false;
    }

    std::uint64_t targetSize = currentSize;
    if (writable && mappingSize > targetSize)
    {
        targetSize = static_cast<std::uint64_t>(mappingSize);
        if (!resizeFile(file, targetSize, error))
        {
            CloseHandle(file);
            return false;
        }
    }

    if (targetSize == 0)
    {
        setError(error, "cannot map an empty file");
        CloseHandle(file);
        return false;
    }

    const DWORD sizeHigh = static_cast<DWORD>((targetSize >> 32) & 0xFFFFFFFFu);
    const DWORD sizeLow = static_cast<DWORD>(targetSize & 0xFFFFFFFFu);

    HANDLE mapping = CreateFileMappingW(
        file,
        nullptr,
        protect,
        sizeHigh,
        sizeLow,
        nullptr);

    if (mapping == nullptr)
    {
        setError(error, lastSystemErrorMessage().c_str());
        CloseHandle(file);
        return false;
    }

    LPVOID view = MapViewOfFile(mapping, mapAccess, 0, 0, 0);
    if (view == nullptr)
    {
        setError(error, lastSystemErrorMessage().c_str());
        CloseHandle(mapping);
        CloseHandle(file);
        return false;
    }

    fileHandle = file;
    mappingHandle = mapping;
    dataPtr = static_cast<std::byte*>(view);
    sizeValue = static_cast<std::size_t>(targetSize);
    return true;
#else
    const bool writable = (access == Access::ReadWrite);
    writableFlag = writable;

    int flags = writable ? O_RDWR : O_RDONLY;

    switch (creation)
    {
    case Creation::OpenExisting:
        break;
    case Creation::OpenOrCreate:
        flags |= O_CREAT;
        break;
    case Creation::CreateAlways:
        flags |= (O_CREAT | O_TRUNC);
        break;
    case Creation::TruncateExisting:
        flags |= O_TRUNC;
        break;
    }

    int file = ::open(path, flags, 0666);
    if (file < 0)
    {
        setError(error, lastSystemErrorMessage().c_str());
        return false;
    }

    std::size_t currentSize = 0;
    if (!getFileSize(file, currentSize, error))
    {
        ::close(file);
        return false;
    }

    std::size_t targetSize = currentSize;
    if (writable && mappingSize > targetSize)
    {
        targetSize = mappingSize;
        if (!resizeFile(file, targetSize, error))
        {
            ::close(file);
            return false;
        }
    }

    if (targetSize == 0)
    {
        setError(error, "cannot map an empty file");
        ::close(file);
        return false;
    }

    int prot = PROT_READ;
    if (writable)
    {
        prot |= PROT_WRITE;
    }

    void* view = mmap(nullptr, targetSize, prot, MAP_SHARED, file, 0);
    if (view == MAP_FAILED)
    {
        setError(error, lastSystemErrorMessage().c_str());
        ::close(file);
        return false;
    }

    fileHandle = reinterpret_cast<void*>(static_cast<std::intptr_t>(file));
    mappingHandle = nullptr;
    dataPtr = static_cast<std::byte*>(view);
    sizeValue = targetSize;
    return true;
 #endif
}

bool MappedFile::open(const std::filesystem::path& path,
    Access access,
    Creation creation,
    std::size_t mappingSize,
    std::string* error)
{
    return open(path.string().c_str(), access, creation, mappingSize, error);
}

void MappedFile::close() noexcept
{
#if defined(_WIN32)
    if (dataPtr != nullptr)
    {
        UnmapViewOfFile(dataPtr);
        dataPtr = nullptr;
    }

    if (mappingHandle != nullptr)
    {
        CloseHandle(static_cast<HANDLE>(mappingHandle));
        mappingHandle = nullptr;
    }

    if (fileHandle != nullptr)
    {
        CloseHandle(static_cast<HANDLE>(fileHandle));
        fileHandle = nullptr;
    }
#else
    if (dataPtr != nullptr)
    {
        munmap(dataPtr, sizeValue);
        dataPtr = nullptr;
    }

    if (fileHandle != nullptr)
    {
        ::close(static_cast<int>(reinterpret_cast<std::intptr_t>(fileHandle)));
        fileHandle = nullptr;
    }

    mappingHandle = nullptr;
#endif

    sizeValue = 0;
    writableFlag = false;
}
