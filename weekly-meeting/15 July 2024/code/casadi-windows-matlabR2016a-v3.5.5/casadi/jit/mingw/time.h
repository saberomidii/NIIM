/**
 * This file has no copyright assigned and is placed in the Public Domain.
 * This file is part of the mingw-w64 runtime package.
 * No warranty is given; refer to the file DISCLAIMER.PD within this package.
 */
#ifndef _TIME_H_
#define _TIME_H_

#include <crtdefs.h>

#ifndef _WIN32
#error Only Win32 target is supported!
#endif

#pragma pack(push,_CRT_PACKING)

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _CRTIMP
#define _CRTIMP __declspec(dllimport)
#endif

#ifndef _WCHAR_T_DEFINED
#define _WCHAR_T_DEFINED
  typedef unsigned short wchar_t;
#endif

#ifndef _TIME32_T_DEFINED
#define _TIME32_T_DEFINED
  typedef long __time32_t;
#endif

#ifndef _TIME64_T_DEFINED
#define _TIME64_T_DEFINED
  __MINGW_EXTENSION typedef __int64 __time64_t;
#endif

#ifndef _TIME_T_DEFINED
#define _TIME_T_DEFINED
#ifdef _USE_32BIT_TIME_T
  typedef __time32_t time_t;
#else
  typedef __time64_t time_t;
#endif
#endif

#ifndef _CLOCK_T_DEFINED
#define _CLOCK_T_DEFINED
  typedef long clock_t;
#endif

#ifndef _SIZE_T_DEFINED
#define _SIZE_T_DEFINED
#undef size_t
#ifdef _WIN64
  __MINGW_EXTENSION typedef unsigned __int64 size_t;
#else
  typedef unsigned int size_t;
#endif
#endif

#ifndef _SSIZE_T_DEFINED
#define _SSIZE_T_DEFINED
#undef ssize_t
#ifdef _WIN64
  __MINGW_EXTENSION typedef __int64 ssize_t;
#else
  typedef int ssize_t;
#endif
#endif

#ifndef NULL
#ifdef __cplusplus
#ifndef _WIN64
#define NULL 0
#else
#define NULL 0LL
#endif  /* W64 */
#else
#define NULL ((void *)0)
#endif
#endif

#ifndef _TM_DEFINED
#define _TM_DEFINED
  struct tm {
    int tm_sec;
    int tm_min;
    int tm_hour;
    int tm_mday;
    int tm_mon;
    int tm_year;
    int tm_wday;
    int tm_yday;
    int tm_isdst;
  };
#endif

#define CLOCKS_PER_SEC 1000

  __MINGW_IMPORT int _daylight;
  __MINGW_IMPORT long _dstbias;
  __MINGW_IMPORT long _timezone;
  __MINGW_IMPORT char * _tzname[2];

  _CRTIMP errno_t __cdecl _get_daylight(int *_Daylight);
  _CRTIMP errno_t __cdecl _get_dstbias(long *_Daylight_savings_bias);
  _CRTIMP errno_t __cdecl _get_timezone(long *_Timezone);
  _CRTIMP errno_t __cdecl _get_tzname(size_t *_ReturnValue,char *_Buffer,size_t _SizeInBytes,int _Index);
  char *__cdecl asctime(const struct tm *_Tm) __MINGW_ATTRIB_DEPRECATED_SEC_WARN;
  _CRTIMP errno_t __cdecl asctime_s (char *_Buf,size_t _SizeInWords,const struct tm *_Tm);
  char *__cdecl _ctime32(const __time32_t *_Time) __MINGW_ATTRIB_DEPRECATED_SEC_WARN;
  _CRTIMP errno_t __cdecl _ctime32_s (char *_Buf,size_t _SizeInBytes,const __time32_t *_Time);
  clock_t __cdecl clock(void);
  double __cdecl _difftime32(__time32_t _Time1,__time32_t _Time2);
  struct tm *__cdecl _gmtime32(const __time32_t *_Time) __MINGW_ATTRIB_DEPRECATED_SEC_WARN;
  _CRTIMP errno_t __cdecl _gmtime32_s (struct tm *_Tm,const __time32_t *_Time);
  struct tm *__cdecl _localtime32(const __time32_t *_Time) __MINGW_ATTRIB_DEPRECATED_SEC_WARN;
  _CRTIMP errno_t __cdecl _localtime32_s (struct tm *_Tm,const __time32_t *_Time);
  size_t __cdecl strftime(char * __restrict__ _Buf,size_t _SizeInBytes,const char * __restrict__ _Format,const struct tm * __restrict__ _Tm);
  _CRTIMP size_t __cdecl _strftime_l(char * __restrict__ _Buf,size_t _Max_size,const char * __restrict__ _Format,const struct tm * __restrict__ _Tm,_locale_t _Locale);
  _CRTIMP char *__cdecl _strdate(char *_Buffer) __MINGW_ATTRIB_DEPRECATED_SEC_WARN;
  _CRTIMP errno_t __cdecl _strdate_s (char *_Buf,size_t _SizeInBytes);
  _CRTIMP char *__cdecl _strtime(char *_Buffer) __MINGW_ATTRIB_DEPRECATED_SEC_WARN;
  _CRTIMP errno_t __cdecl _strtime_s (char *_Buf ,size_t _SizeInBytes);
  __time32_t __cdecl _time32(__time32_t *_Time);
  __time32_t __cdecl _mktime32(struct tm *_Tm);
  __time32_t __cdecl _mkgmtime32(struct tm *_Tm);

#if defined (_POSIX_) || defined(__GNUC__)
  void __cdecl tzset(void) __MINGW_ATTRIB_DEPRECATED_MSVC2005;
#endif
#if !defined (_POSIX_)
  _CRTIMP void __cdecl _tzset(void);
#endif

  double __cdecl _difftime64(__time64_t _Time1,__time64_t _Time2);
  _CRTIMP char *__cdecl _ctime64(const __time64_t *_Time) __MINGW_ATTRIB_DEPRECATED_SEC_WARN;
  errno_t __cdecl _ctime64_s (char *_Buf,size_t _SizeInBytes,const __time64_t *_Time);
  _CRTIMP struct tm *__cdecl _gmtime64(const __time64_t *_Time) __MINGW_ATTRIB_DEPRECATED_SEC_WARN;
  _CRTIMP errno_t __cdecl _gmtime64_s (struct tm *_Tm,const __time64_t *_Time);
  _CRTIMP struct tm *__cdecl _localtime64(const __time64_t *_Time) __MINGW_ATTRIB_DEPRECATED_SEC_WARN;
  _CRTIMP errno_t __cdecl _localtime64_s (struct tm *_Tm,const __time64_t *_Time);
  _CRTIMP __time64_t __cdecl _mktime64(struct tm *_Tm);
  _CRTIMP __time64_t __cdecl _mkgmtime64(struct tm *_Tm);
  _CRTIMP __time64_t __cdecl _time64(__time64_t *_Time);
  unsigned __cdecl _getsystime(struct tm *_Tm);
  unsigned __cdecl _setsystime(struct tm *_Tm,unsigned _MilliSec);

#ifndef _WTIME_DEFINED
  _CRTIMP wchar_t *__cdecl _wasctime(const struct tm *_Tm);
  _CRTIMP errno_t __cdecl _wasctime_s (wchar_t *_Buf,size_t _SizeInWords,const struct tm *_Tm);
  wchar_t *__cdecl _wctime32(const __time32_t *_Time) __MINGW_ATTRIB_DEPRECATED_SEC_WARN;
  _CRTIMP errno_t __cdecl _wctime32_s (wchar_t *_Buf,size_t _SizeInWords,const __time32_t *_Time);
  size_t __cdecl wcsftime(wchar_t * __restrict__ _Buf,size_t _SizeInWords,const wchar_t * __restrict__ _Format,const struct tm * __restrict__ _Tm);
  _CRTIMP size_t __cdecl _wcsftime_l(wchar_t * __restrict__ _Buf,size_t _SizeInWords,const wchar_t * __restrict__ _Format,const struct tm * __restrict__ _Tm,_locale_t _Locale);
  _CRTIMP wchar_t *__cdecl _wstrdate(wchar_t *_Buffer) __MINGW_ATTRIB_DEPRECATED_SEC_WARN;
  _CRTIMP errno_t __cdecl _wstrdate_s (wchar_t *_Buf,size_t _SizeInWords);
  _CRTIMP wchar_t *__cdecl _wstrtime(wchar_t *_Buffer) __MINGW_ATTRIB_DEPRECATED_SEC_WARN;
  _CRTIMP errno_t __cdecl _wstrtime_s (wchar_t *_Buf,size_t _SizeInWords);
  _CRTIMP wchar_t *__cdecl _wctime64(const __time64_t *_Time) __MINGW_ATTRIB_DEPRECATED_SEC_WARN;
  _CRTIMP errno_t __cdecl _wctime64_s (wchar_t *_Buf,size_t _SizeInWords,const __time64_t *_Time);

#if !defined (RC_INVOKED) && !defined (_INC_WTIME_INL)
#define _INC_WTIME_INL
  wchar_t *__cdecl _wctime(const time_t *) __MINGW_ATTRIB_DEPRECATED_SEC_WARN;
#ifndef __CRT__NO_INLINE
#ifndef _USE_32BIT_TIME_T
  __CRT_INLINE wchar_t *__cdecl _wctime(const time_t *_Time) { return _wctime64(_Time); }
#else
  __CRT_INLINE wchar_t *__cdecl _wctime(const time_t *_Time) { return _wctime32(_Time); }
#endif
#endif /* __CRT__NO_INLINE */
#endif

#if !defined (RC_INVOKED) && !defined (_INC_WTIME_S_INL)
#define _INC_WTIME_S_INL
  errno_t __cdecl _wctime_s(wchar_t *, size_t, const time_t *);
#ifndef __CRT__NO_INLINE
#ifndef _USE_32BIT_TIME_T
  __CRT_INLINE errno_t __cdecl _wctime_s (wchar_t *_Buffer,size_t _SizeInWords,const time_t *_Time) { return _wctime64_s (_Buffer,_SizeInWords,_Time); }
#else
  __CRT_INLINE errno_t __cdecl _wctime_s (wchar_t *_Buffer,size_t _SizeInWords,const time_t *_Time) { return _wctime32_s (_Buffer,_SizeInWords,_Time); }
#endif
#endif  /* __CRT__NO_INLINE */
#endif

#define _WTIME_DEFINED
#endif

#ifndef RC_INVOKED
double __cdecl difftime(time_t _Time1,time_t _Time2);
char *__cdecl ctime(const time_t *_Time) __MINGW_ATTRIB_DEPRECATED_SEC_WARN;
struct tm *__cdecl gmtime(const time_t *_Time) __MINGW_ATTRIB_DEPRECATED_SEC_WARN;
struct tm *__cdecl localtime(const time_t *_Time) __MINGW_ATTRIB_DEPRECATED_SEC_WARN;

time_t __cdecl mktime(struct tm *_Tm);
time_t __cdecl _mkgmtime(struct tm *_Tm);
time_t __cdecl time(time_t *_Time);

#ifndef __CRT__NO_INLINE
#if !defined(_USE_32BIT_TIME_T)
__CRT_INLINE double __cdecl difftime(time_t _Time1,time_t _Time2)
  { return _difftime64(_Time1,_Time2); }
__CRT_INLINE char *__cdecl ctime(const time_t *_Time) { return _ctime64(_Time); }
__CRT_INLINE struct tm *__cdecl gmtime(const time_t *_Time) { return _gmtime64(_Time); }
__CRT_INLINE struct tm *__cdecl localtime(const time_t *_Time) { return _localtime64(_Time); }
__CRT_INLINE time_t __cdecl mktime(struct tm *_Tm) { return _mktime64(_Tm); }
__CRT_INLINE time_t __cdecl _mkgmtime(struct tm *_Tm) { return _mkgmtime64(_Tm); }
__CRT_INLINE time_t __cdecl time(time_t *_Time) { return _time64(_Time); }
#else
__CRT_INLINE double __cdecl difftime(time_t _Time1,time_t _Time2)
  { return _difftime32(_Time1,_Time2); }
__CRT_INLINE char *__cdecl ctime(const time_t *_Time) { return _ctime32(_Time); }
__CRT_INLINE struct tm *__cdecl localtime(const time_t *_Time) { return _localtime32(_Time); }
__CRT_INLINE time_t __cdecl mktime(struct tm *_Tm) { return _mktime32(_Tm); }
__CRT_INLINE struct tm *__cdecl gmtime(const time_t *_Time) { return _gmtime32(_Time); }
__CRT_INLINE time_t __cdecl _mkgmtime(struct tm *_Tm) { return _mkgmtime32(_Tm); }
__CRT_INLINE time_t __cdecl time(time_t *_Time) { return _time32(_Time); }
#endif /* !_USE_32BIT_TIME_T */
#endif /* !__CRT__NO_INLINE */

#ifdef _USE_32BIT_TIME_T
__forceinline errno_t __cdecl localtime_s(struct tm *_Tm,const time_t *_Time) { return _localtime32_s(_Tm,_Time); }
__forceinline errno_t __cdecl gmtime_s(struct tm *_Tm, const time_t *_Time)   { return _gmtime32_s(_Tm, _Time); }
__forceinline errno_t __cdecl ctime_s(char *_Buf,size_t _SizeInBytes,const time_t *_Time) { return _ctime32_s(_Buf,_SizeInBytes,_Time); }

#else
__forceinline errno_t __cdecl localtime_s(struct tm *_Tm,const time_t *_Time) { return _localtime64_s(_Tm,_Time); }
__forceinline errno_t __cdecl gmtime_s(struct tm *_Tm, const time_t *_Time) { return _gmtime64_s(_Tm, _Time); }
__forceinline errno_t __cdecl ctime_s(char *_Buf,size_t _SizeInBytes,const time_t *_Time) { return _ctime64_s(_Buf,_SizeInBytes,_Time); }
#endif
#endif /* !RC_INVOKED */

#if !defined(NO_OLDNAMES) || defined(_POSIX)
#define CLK_TCK CLOCKS_PER_SEC

  _CRTIMP extern int daylight;
  _CRTIMP extern long timezone;
  _CRTIMP extern char *tzname[2];
  void __cdecl tzset(void) __MINGW_ATTRIB_DEPRECATED_MSVC2005;
#endif

#include <_timeval.h>

#ifndef _TIMEZONE_DEFINED /* also in sys/time.h */
#define _TIMEZONE_DEFINED
struct timezone {
  int tz_minuteswest;
  int tz_dsttime;
};

  extern int __cdecl mingw_gettimeofday (struct timeval *p, struct timezone *z);
#endif /* _TIMEZONE_DEFINED */

#ifdef __cplusplus
}
#endif

#pragma pack(pop)

#if defined(_POSIX_C_SOURCE) && !defined(_POSIX_THREAD_SAFE_FUNCTIONS)
#define _POSIX_THREAD_SAFE_FUNCTIONS 200112L
#endif

#ifdef _POSIX_THREAD_SAFE_FUNCTIONS
__forceinline struct tm *__cdecl localtime_r(const time_t *_Time, struct tm *_Tm) {
  return localtime_s(_Tm, _Time) ? NULL : _Tm;
}
__forceinline struct tm *__cdecl gmtime_r(const time_t *_Time, struct tm *_Tm) {
  return gmtime_s(_Tm, _Time) ? NULL : _Tm;
}
__forceinline char *__cdecl ctime_r(const time_t *_Time, char *_Str) {
  return ctime_s(_Str, 0x7fffffff, _Time) ? NULL : _Str;
}
__forceinline char *__cdecl asctime_r(const struct tm *_Tm, char * _Str) {
  return asctime_s(_Str, 0x7fffffff, _Tm) ? NULL : _Str;
}
#endif

/* Adding timespec definition.  */
#include <sys/timeb.h>

/* POSIX 2008 says clock_gettime and timespec are defined in time.h header,
   but other systems - like Linux, Solaris, etc - tend to declare such
   recent extensions only if the following guards are met.  */
#if !defined(IN_WINPTHREAD) && \
	((!defined(_STRICT_STDC) && !defined(__XOPEN_OR_POSIX)) || \
	 (_POSIX_C_SOURCE > 2) || defined(__EXTENSIONS__))
#include <pthread_time.h>
#endif

#endif /* End _TIME_H_ */

