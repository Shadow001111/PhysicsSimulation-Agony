#pragma once
#ifdef ECSTASY_ENABLE_ASSERTS
    #include <iostream>
    #include <sstream>

    #if defined(_MSC_VER)
        #define ECSTASY_DEBUG_BREAK() __debugbreak()
    #elif defined(__GNUC__) || defined(__clang__)
        #define ECSTASY_DEBUG_BREAK() __builtin_trap()
    #else
        #define ECSTASY_DEBUG_BREAK() std::abort()
    #endif
        #define ECSTASY_ASSERT(expr)                                                     \
            do {                                                                         \
                if (!(expr)) [[unlikely]] {                                              \
                    std::ostringstream __ASSERT_stream;                                  \
                    __ASSERT_stream << "Assertion failed!\n"                             \
                                    << "Expression: " << #expr << "\n"                   \
                                    << "File: " << __FILE__ << "\n"                      \
                                    << "Line: " << __LINE__ << "\n";                     \
                    std::cerr << __ASSERT_stream.str();                                  \
                    ECSTASY_DEBUG_BREAK();                                               \
                }                                                                        \
            } while (0)
#else
    #define ECSTASY_ASSERT(expr) ((void)(expr))
#endif