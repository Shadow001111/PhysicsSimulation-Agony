#pragma once
#include <cstddef> /* For size_t. */
#include <cstdint>

typedef int8_t   ma_int8;
typedef uint8_t  ma_uint8;
typedef int16_t  ma_int16;
typedef uint16_t ma_uint16;
typedef int32_t  ma_int32;
typedef uint32_t ma_uint32;
typedef int64_t  ma_int64;
typedef uint64_t ma_uint64;

#if defined(__LP64__) || defined(_WIN64) || (defined(__x86_64__) && !defined(__ILP32__)) || defined(_M_X64) || defined(__ia64) || defined(_M_IA64) || defined(__aarch64__) || defined(_M_ARM64) || defined(__powerpc64__) || defined(__ppc64__)
inline constexpr size_t MA_SIZEOF_PTR = 8;
typedef ma_uint64 ma_uintptr;
#else
inline constexpr size_t MA_SIZEOF_PTR = 4;
typedef ma_uint32 ma_uintptr;
#endif

typedef ma_uint8  ma_bool8;
typedef ma_uint32 ma_bool32;

inline constexpr bool MA_TRUE = true;
inline constexpr bool MA_FALSE = false;

/* These float types are not used universally by miniaudio. It's to simplify some macro expansion for atomic types. */
typedef float  ma_float;
typedef double ma_double;

typedef void* ma_handle;
typedef void* ma_ptr;

/*
ma_proc is annoying because when compiling with GCC we get pedantic warnings about converting
between `void*` and `void (*)()`. We can't use `void (*)()` with MSVC however, because we'll get
warning C4191 about "type cast between incompatible function types". To work around this I'm going
to use a different data type depending on the compiler.
*/
#if defined(__GNUC__)
typedef void (*ma_proc)(void);
#else
typedef void* ma_proc;
#endif

#if defined(_MSC_VER) && !defined(_WCHAR_T_DEFINED)
typedef ma_uint16 wchar_t;
#endif

/* Define NULL for some compilers. */
#ifndef NULL
#define NULL 0
#endif

#if defined(SIZE_MAX)
inline constexpr size_t MA_SIZE_MAX = SIZE_MAX;
#else
/* When SIZE_MAX is not defined by the standard library just default to the maximum 32-bit unsigned integer. */
inline constexpr size_t MA_SIZE_MAX = 0xFFFFFFFF;
#endif

//inline constexpr size_t MA_UINT64_MAX = ma_uint64(-1);

/*
Special wchar_t type to ensure any structures in the public sections that
reference it have a consistent size across all platforms.

On Windows, wchar_t is 2 bytes, whereas everywhere else it's 4 bytes. Since
Windows likes to use wchar_t for its IDs, we need a special explicitly sized
wchar type that is always 2 bytes on all platforms.
*/
#if !defined(MA_POSIX) && defined(MA_WIN32)
typedef wchar_t ma_wchar_win32;
#else
typedef ma_uint16 ma_wchar_win32;
#endif