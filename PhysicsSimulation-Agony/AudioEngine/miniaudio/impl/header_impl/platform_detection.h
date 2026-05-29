#pragma once

// WINDOWS / XBOX
#if defined(_WIN32)
	#define MA_WIN32
	#if defined(MA_FORCE_UWP) || (defined(WINAPI_FAMILY) && ((defined(WINAPI_FAMILY_PC_APP) && WINAPI_FAMILY == WINAPI_FAMILY_PC_APP) || (defined(WINAPI_FAMILY_PHONE_APP) && WINAPI_FAMILY == WINAPI_FAMILY_PHONE_APP)))
		#define MA_WIN32_UWP
	#elif defined(WINAPI_FAMILY) && (defined(WINAPI_FAMILY_GAMES) && WINAPI_FAMILY == WINAPI_FAMILY_GAMES)
		#define MA_WIN32_GDK
	#elif defined(NXDK)
		#define MA_WIN32_NXDK
	#else
		#define MA_WIN32_DESKTOP
	#endif

	/* The original Xbox. */
	#if defined(NXDK) /* <-- Add other Xbox compiler toolchains here, and then add a toolchain-specific define in case we need to discriminate between them later. */
		#define MA_XBOX

		#if defined(NXDK)
			#define MA_XBOX_NXDK
		#endif
	#endif
#endif

// DOS
#if defined(__MSDOS__) || defined(MSDOS) || defined(_MSDOS) || defined(__DOS__)
	#define MA_DOS

	/* No threading allowed on DOS. */
	#ifndef MA_NO_THREADING
		#define MA_NO_THREADING
	#endif

	/* No runtime linking allowed on DOS. */
	#ifndef MA_NO_RUNTIME_LINKING
		#define MA_NO_RUNTIME_LINKING
	#endif
#endif

// POSIX
#if !defined(MA_WIN32) && !defined(MA_DOS) /* If it's not Win32, assume POSIX. */
	#define MA_POSIX

	#if !defined(MA_NO_THREADING)
		/*
		Use the MA_NO_PTHREAD_IN_HEADER option at your own risk. This is
		intentionally undocumented. You can use this to avoid including
		pthread.h in the header section. The downside is that it results in some
		fixed sized structures being declared for the various types that are
		used in miniaudio. The risk here is that these types might be too small
		for a given platform. This risk is yours to take and no support will be
		offered if you enable this option.
		*/
		#ifndef MA_NO_PTHREAD_IN_HEADER
			#include <pthread.h> /* Unfortunate #include, but needed for pthread_t, pthread_mutex_t and pthread_cond_t types. */
typedef pthread_t ma_pthread_t;
typedef pthread_mutex_t ma_pthread_mutex_t;
typedef pthread_cond_t ma_pthread_cond_t;
		#else
typedef ma_uintptr ma_pthread_t;
typedef union ma_pthread_mutex_t {
	char __data[40];
	ma_uint64 __alignment;
} ma_pthread_mutex_t;
typedef union ma_pthread_cond_t {
	char __data[48];
	ma_uint64 __alignment;
} ma_pthread_cond_t;
		#endif
	#endif

	#if defined(__unix__)
		#define MA_UNIX
	#endif
	#if defined(__linux__)
		#define MA_LINUX
	#endif
	#if defined(__APPLE__)
		#define MA_APPLE
	#endif
	#if defined(__DragonFly__) || defined(__FreeBSD__) || defined(__NetBSD__) || defined(__OpenBSD__)
		#define MA_BSD
	#endif
	#if defined(__ANDROID__)
		#define MA_ANDROID
	#endif
	#if defined(__EMSCRIPTEN__)
		#define MA_EMSCRIPTEN
	#endif
	#if defined(__ORBIS__)
		#define MA_ORBIS
	#endif
	#if defined(__PROSPERO__)
		#define MA_PROSPERO
	#endif
	#if defined(__3DS__)
		#define MA_3DS
	#endif
	#if defined(__SWITCH__) || defined(__NX__)
		#define MA_SWITCH
	#endif
	#if defined(__BEOS__) || defined(__HAIKU__)
		#define MA_BEOS
	#endif
	#if defined(__HAIKU__)
		#define MA_HAIKU
	#endif
#endif