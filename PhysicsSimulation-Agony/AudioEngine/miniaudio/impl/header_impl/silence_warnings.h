#pragma once

#if defined(_MSC_VER) && !defined(__clang__)
#pragma warning(push)
#pragma warning(disable:4201)   /* nonstandard extension used: nameless struct/union */
#pragma warning(disable:4214)   /* nonstandard extension used: bit field types other than int */
#pragma warning(disable:4324)   /* structure was padded due to alignment specifier */
#elif defined(__clang__) || (defined(__GNUC__) && (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 8)))
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic" /* For ISO C99 doesn't support unnamed structs/unions [-Wpedantic] */
#if defined(__clang__)
#pragma GCC diagnostic ignored "-Wc11-extensions"   /* anonymous unions are a C11 extension */
#endif
#endif