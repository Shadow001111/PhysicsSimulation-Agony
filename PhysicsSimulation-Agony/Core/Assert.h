#pragma once
#include <iostream>
#include <sstream>

#define ENABLE_ASSERTS 1

#if defined(_MSC_VER)
#define DEBUG_BREAK() __debugbreak()
#elif defined(__GNUC__) || defined(__clang__)
#define DEBUG_BREAK() __builtin_trap()
#else
#define DEBUG_BREAK() std::abort()
#endif

#if ENABLE_ASSERTS
#define ASSERT(expr)                                                             \
    do {                                                                         \
        if (!(expr)) [[unlikely]] {                                              \
            std::ostringstream __ASSERT_stream;                                  \
            __ASSERT_stream << "Assertion failed!\n"                             \
                            << "Expression: " << #expr << "\n"                   \
                            << "File: " << __FILE__ << "\n"                      \
                            << "Line: " << __LINE__ << "\n";                     \
            std::cerr << __ASSERT_stream.str();                                  \
            DEBUG_BREAK();                                                       \
        }                                                                        \
    } while (0)
#else
// In release builds, disable assertions
// But still evaluate the expression to avoid potential bugs, if someone will put important function calls inside ASSERT (which is not recommended, but just in case)
#define ASSERT(expr) ((void)(expr))
#endif