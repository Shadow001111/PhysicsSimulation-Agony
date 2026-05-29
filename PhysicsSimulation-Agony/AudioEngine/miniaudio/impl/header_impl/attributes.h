#pragma once

// FALLTHROUGH
#if !defined(MA_FALLTHROUGH) && defined(__cplusplus) && __cplusplus >= 201703L
	#define MA_FALLTHROUGH [[fallthrough]]
#endif
#if !defined(MA_FALLTHROUGH) && defined(__STDC_VERSION__) && __STDC_VERSION__ >= 202000L
	#define MA_FALLTHROUGH [[fallthrough]]
#endif
#if !defined(MA_FALLTHROUGH) && defined(__has_attribute)
	#if __has_attribute(fallthrough)
		#define MA_FALLTHROUGH __attribute__((fallthrough))
	#endif
#endif
#if !defined(MA_FALLTHROUGH)
	#define MA_FALLTHROUGH ((void)0)
#endif

// INLINE
#ifdef _MSC_VER
	#define MA_INLINE __forceinline

	/* noinline was introduced in Visual Studio 2005. */
	#if _MSC_VER >= 1400
		#define MA_NO_INLINE __declspec(noinline)
	#else
		#define MA_NO_INLINE
	#endif
#elif defined(__GNUC__)
	/*
	I've had a bug report where GCC is emitting warnings about functions
	possibly not being inlineable. This warning happens when the
	__attribute__((always_inline)) attribute is defined without an "inline"
	statement. I think therefore there must be some case where "__inline__" is
	not always defined, thus the compiler emitting these warnings. When using
	-std=c89 or -ansi on the command line, we cannot use the "inline" keyword
	and instead need to use "__inline__". In an attempt to work around this
	issue I am using
	"__inline__" only when we're compiling in strict ANSI mode.
	*/
	#if defined(__STRICT_ANSI__)
		#define MA_GNUC_INLINE_HINT __inline__
	#else
		#define MA_GNUC_INLINE_HINT inline
	#endif

	#if (__GNUC__ > 3 || (__GNUC__ == 3 && __GNUC_MINOR__ >= 2)) || defined(__clang__)
		#define MA_INLINE MA_GNUC_INLINE_HINT __attribute__((always_inline))
		#define MA_NO_INLINE __attribute__((noinline))
	#else
		#define MA_INLINE MA_GNUC_INLINE_HINT
		#define MA_NO_INLINE
	#endif
#elif defined(__WATCOMC__)
	#define MA_INLINE __inline
	#define MA_NO_INLINE
#else
	#define MA_INLINE
	#define MA_NO_INLINE
#endif