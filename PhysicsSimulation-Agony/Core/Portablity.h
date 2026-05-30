#pragma once

#ifndef CORE_RESTRICT
	#if defined(_MSC_VER)
		#define CORE_RESTRICT __restrict
	#elif defined(__clang__) || defined(__GNUC__)
		#define CORE_RESTRICT __restrict__
	#else
		#define CORE_RESTRICT
	#endif
#endif