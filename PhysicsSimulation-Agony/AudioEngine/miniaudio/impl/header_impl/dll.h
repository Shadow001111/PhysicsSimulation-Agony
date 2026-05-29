#pragma once

#if defined(_WIN32)
	#define MA_DLL_IMPORT __declspec(dllimport)
	#define MA_DLL_EXPORT __declspec(dllexport)
	#define MA_DLL_PRIVATE static
#else
	#if defined(__GNUC__) && __GNUC__ >= 4
		#define MA_DLL_IMPORT __attribute__((visibility("default")))
		#define MA_DLL_EXPORT __attribute__((visibility("default")))
		#define MA_DLL_PRIVATE __attribute__((visibility("hidden")))
	#else
		#define MA_DLL_IMPORT
		#define MA_DLL_EXPORT
		#define MA_DLL_PRIVATE static
	#endif
#endif