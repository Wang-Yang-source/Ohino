// MinGW-w64 compatibility shim for building Ohino (and the vendored
// opennurbs) with x86_64-w64-mingw32-g++.
//
// opennurbs assumes either MSVC (Win32 FindFirstFile path, UNICODE TCHARs) or
// a POSIX libc (dirent with readdir_r and NAME_MAX). MinGW-w64 is neither:
// it has the Win32 API but its <dirent.h> has no readdir_r/NAME_MAX, and its
// TCHAR is `char` unless UNICODE is defined. This header is force-included
// with -include so both flavours compile.
#ifndef OHINO_MINGW_COMPAT_H
#define OHINO_MINGW_COMPAT_H

#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0601 /* Windows 7: SHGetKnownFolderPath, KF_FLAG_DEFAULT */
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>
#include <shlobj.h>     /* SHGetKnownFolderPath, CoTaskMemFree */
#include <knownfolders.h>
#include <objbase.h>
#include <limits.h>

#ifndef NAME_MAX
#define NAME_MAX 255
#endif

#include <dirent.h>

// MinGW-w64's <dirent.h> has no readdir_r(); provide the POSIX signature on
// top of readdir(). opennurbs only calls it sequentially with its own storage.
#if defined(__cplusplus)
extern "C++" {
#endif
static __inline int ohino_readdir_r(DIR* dirp, struct dirent* entry, struct dirent** result) {
  struct dirent* p = readdir(dirp);
  if (p == 0) {
    if (result) *result = 0;
    return 0;
  }
  if (entry) *entry = *p;
  if (result) *result = entry;
  return 0;
}
#if defined(__cplusplus)
}
#endif

#ifndef readdir_r
#define readdir_r(dirp, entry, result) ohino_readdir_r((dirp), (entry), (result))
#endif

// MSVCRT (MinGW-w64's default CRT) has only the locale-last _sprintf_l family,
// while opennurbs' validation code probes the POSIX locale-second spellings.
// Provide them; the locale argument is ignored, which matches the invariant
// "C" locale opennurbs asks for anyway.
#if defined(__MINGW32__) && defined(__cplusplus)
#include <stdarg.h>
#include <stdio.h>
static __inline int ohino_sprintf_l(char* buffer, _locale_t locale, const char* format, ...) {
  va_list ap; va_start(ap, format);
  const int rc = vsprintf(buffer, format, ap);
  va_end(ap); (void)locale; return rc;
}
static __inline int ohino_snprintf_l(char* buffer, size_t capacity, _locale_t locale,
                                     const char* format, ...) {
  va_list ap; va_start(ap, format);
  const int rc = vsnprintf(buffer, capacity, format, ap);
  va_end(ap); (void)locale; return rc;
}
static __inline int ohino_sscanf_l(const char* buffer, _locale_t locale, const char* format, ...) {
  va_list ap; va_start(ap, format);
  const int rc = vsscanf(buffer, format, ap);
  va_end(ap); (void)locale; return rc;
}
#define sprintf_l ohino_sprintf_l
#define snprintf_l ohino_snprintf_l
#define sscanf_l ohino_sscanf_l
static __inline int ohino_vsnprintf_l(char* buffer, size_t capacity, _locale_t locale,
                                      const char* format, va_list ap) {
  (void)locale; return vsnprintf(buffer, capacity, format, ap);
}
#define vsnprintf_l ohino_vsnprintf_l

#endif

// opennurbs calls the BSD-style qsort_r(base, nel, width, context, compar) when
// _GNU_SOURCE is in effect; MSVCRT has neither spelling. Emulate it on qsort
// with a per-translation-unit context (ON_qsort calls are not re-entrant).
#if defined(__MINGW32__) && defined(__cplusplus)
#include <stdlib.h>
static int (*ohino_qsort_compare)(void*, const void*, const void*) = 0;
static void* ohino_qsort_context = 0;
static int ohino_qsort_thunk(const void* a, const void* b) {
  return ohino_qsort_compare(ohino_qsort_context, a, b);
}
static __inline void ohino_qsort_r(void* base, size_t nel, size_t width, void* context,
                                   int (*compar)(void*, const void*, const void*)) {
  ohino_qsort_compare = compar;
  ohino_qsort_context = context;
  qsort(base, nel, width, ohino_qsort_thunk);
}
#define qsort_r ohino_qsort_r
#endif

// MSVCRT spells the reentrant time functions localtime_s/gmtime_s with reversed
// argument order; opennurbs uses the POSIX names.
#if defined(__MINGW32__) && defined(__cplusplus)
#include <time.h>
static __inline struct tm* ohino_localtime_r(const time_t* t, struct tm* out) {
  return (out && localtime_s(out, t) == 0) ? out : 0;
}
static __inline struct tm* ohino_gmtime_r(const time_t* t, struct tm* out) {
  return (out && gmtime_s(out, t) == 0) ? out : 0;
}
#define localtime_r ohino_localtime_r
#define gmtime_r ohino_gmtime_r
#endif

#endif // OHINO_MINGW_COMPAT_H
