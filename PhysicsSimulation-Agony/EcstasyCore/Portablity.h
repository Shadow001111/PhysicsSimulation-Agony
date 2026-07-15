#pragma once

#ifndef ECSTASY_RESTRICT
	#if defined(_MSC_VER)
		#define ECSTASY_RESTRICT __restrict
	#elif defined(__clang__) || defined(__GNUC__)
		#define ECSTASY_RESTRICT __restrict__
	#else
		#define ECSTASY_RESTRICT
	#endif
#endif

#ifndef ECSTASY_SPIN_PAUSE
	#if defined(_MSC_VER) || defined(__x86_64__) || defined(__i386__)
		#include <immintrin.h>
		#define ECSTASY_SPIN_PAUSE() _mm_pause()
	#else
		#include <thread>
		#define ECSTASY_SPIN_PAUSE() std::this_thread::yield()
	#endif
#endif