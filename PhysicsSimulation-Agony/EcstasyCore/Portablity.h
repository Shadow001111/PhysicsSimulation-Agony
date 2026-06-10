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