#pragma once

#if !defined(MA_API)
	#if defined(MA_DLL)
		#if defined(MINIAUDIO_IMPLEMENTATION) || defined(MA_IMPLEMENTATION)
			#define MA_API MA_DLL_EXPORT
		#else
			#define MA_API MA_DLL_IMPORT
		#endif
	#else
		#define MA_API extern
	#endif
#endif

#if !defined(MA_STATIC)
	#if defined(MA_DLL)
		#define MA_PRIVATE MA_DLL_PRIVATE
	#else
		#define MA_PRIVATE static
	#endif
#endif