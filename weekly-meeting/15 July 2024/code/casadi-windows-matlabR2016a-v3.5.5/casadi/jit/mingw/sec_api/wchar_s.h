/**
 * This file has no copyright assigned and is placed in the Public Domain.
 * This file is part of the mingw-w64 runtime package.
 * No warranty is given; refer to the file DISCLAIMER.PD within this package.
 */
#ifndef _INC_WCHAR_S
#define _INC_WCHAR_S

#include <wchar.h>

#if defined(MINGW_HAS_SECURE_API)

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _WIO_S_DEFINED
#define _WIO_S_DEFINED
  _CRTIMP errno_t __cdecl _waccess_s (const wchar_t *_Filename,int _AccessMode);
  errno_t __cdecl _wmktemp_s (wchar_t *_TemplateName,size_t _SizeInWords);
#endif

#ifndef _WCONIO_S_DEFINED
#define _WCONIO_S_DEFINED
  errno_t __cdecl _cgetws_s (wchar_t *_Buffer,size_t _SizeInWords,size_t *_SizeRead);
  int __cdecl _cwprintf_s (const wchar_t *_Format,...);
  _CRTIMP int __cdecl _cwscanf_s(const wchar_t *_Format,...);
  _CRTIMP int __cdecl _cwscanf_s_l(const wchar_t *_Format,_locale_t _Locale,...);
  int __cdecl _vcwprintf_s (const wchar_t *_Format,va_list _ArgList);
  int __cdecl _cwprintf_s_l (const wchar_t *_Format,_locale_t _Locale,...);
  int __cdecl _vcwprintf_s_l (const wchar_t *_Format,_locale_t _Locale,va_list _ArgList);
#endif

#ifndef _WSTDIO_S_DEFINED
#define _WSTDIO_S_DEFINED
  _CRTIMP wchar_t *__cdecl _getws_s(wchar_t *_Str,size_t _SizeInWords);
  int __cdecl fwprintf_s(FILE *_File,const wchar_t *_Format,...);
  int __cdecl wprintf_s(const wchar_t *_Format,...);
  int __cdecl vfwprintf_s(FILE *_File,const wchar_t *_Format,va_list _ArgList);
  int __cdecl vwprintf_s(const wchar_t *_Format,va_list _ArgList);

  int __cdecl vswprintf_s(wchar_t *_Dst,size_t _SizeInWords,const wchar_t *_Format,va_list _ArgList);
  __DEFINE_CPP_OVERLOAD_SECURE_FUNC_0_2(int, vswprintf_s, wchar_t, _Dst, const wchar_t*, _Format, va_list, _ArgList)

  int __cdecl swprintf_s(wchar_t *_Dst,size_t _SizeInWords,const wchar_t *_Format,...);
  __DEFINE_CPP_OVERLOAD_SECURE_FUNC_0_1_ARGLIST(int,swprintf_s,vswprintf_s,wchar_t,_Dst,const wchar_t*,_Format)

  _CRTIMP int __cdecl _vsnwprintf_s(wchar_t *_DstBuf,size_t _DstSizeInWords,size_t _MaxCount,const wchar_t *_Format,va_list _ArgList);
  __DEFINE_CPP_OVERLOAD_SECURE_FUNC_0_3(int,_vsnwprintf_s,wchar_t,_DstBuf,size_t,_MaxCount,const wchar_t*,_Format,va_list,_ArgList)

  _CRTIMP int __cdecl _snwprintf_s(wchar_t *_DstBuf,size_t _DstSizeInWords,size_t _MaxCount,const wchar_t *_Format,...);
  __DEFINE_CPP_OVERLOAD_SECURE_FUNC_0_2_ARGLIST(int,_snwprintf_s,_vsnwprintf_s,wchar_t,_DstBuf,size_t,_MaxCount,const char*,_Format)

  _CRTIMP int __cdecl _wprintf_s_l(const wchar_t *_Format,_locale_t _Locale,...);
  _CRTIMP int __cdecl _vwprintf_s_l(const wchar_t *_Format,_locale_t _Locale,va_list _ArgList);
  _CRTIMP int __cdecl _fwprintf_s_l(FILE *_File,const wchar_t *_Format,_locale_t _Locale,...);
  _CRTIMP int __cdecl _vfwprintf_s_l(FILE *_File,const wchar_t *_Format,_locale_t _Locale,va_list _ArgList);
  _CRTIMP int __cdecl _swprintf_s_l(wchar_t *_DstBuf,size_t _DstSize,const wchar_t *_Format,_locale_t _Locale,...);
  _CRTIMP int __cdecl _vswprintf_s_l(wchar_t *_DstBuf,size_t _DstSize,const wchar_t *_Format,_locale_t _Locale,va_list _ArgList);
  _CRTIMP int __cdecl _snwprintf_s_l(wchar_t *_DstBuf,size_t _DstSize,size_t _MaxCount,const wchar_t *_Format,_locale_t _Locale,...);
  _CRTIMP int __cdecl _vsnwprintf_s_l(wchar_t *_DstBuf,size_t _DstSize,size_t _MaxCount,const wchar_t *_Format,_locale_t _Locale,va_list _ArgList);
  _CRTIMP int __cdecl _fwscanf_s_l(FILE *_File,const wchar_t *_Format,_locale_t _Locale,...);
  _CRTIMP int __cdecl _swscanf_s_l(const wchar_t *_Src,const wchar_t *_Format,_locale_t _Locale,...);
  _CRTIMP int __cdecl _snwscanf_s(const wchar_t *_Src,size_t _MaxCount,const wchar_t *_Format,...);
  _CRTIMP int __cdecl _snwscanf_s_l(const wchar_t *_Src,size_t _MaxCount,const wchar_t *_Format,_locale_t _Locale,...);
  _CRTIMP int __cdecl _wscanf_s_l(const wchar_t *_Format,_locale_t _Locale,...);
  _CRTIMP errno_t __cdecl _wfopen_s(FILE **_File,const wchar_t *_Filename,const wchar_t *_Mode);
  _CRTIMP errno_t __cdecl _wfreopen_s(FILE **_File,const wchar_t *_Filename,const wchar_t *_Mode,FILE *_OldFile);
  _CRTIMP errno_t __cdecl _wtmpnam_s(wchar_t *_DstBuf,size_t _SizeInWords);
#endif

#ifndef _WSTDLIB_S_DEFINED
#define _WSTDLIB_S_DEFINED
  _CRTIMP errno_t __cdecl _itow_s (int _Val,wchar_t *_DstBuf,size_t _SizeInWords,int _Radix);
  __DEFINE_CPP_OVERLOAD_SECURE_FUNC_1_1(errno_t,_itow_s,int,_Val,wchar_t,_DstBuf,int,_Radix)

  _CRTIMP errno_t __cdecl _ltow_s (long _Val,wchar_t *_DstBuf,size_t _SizeInWords,int _Radix);
  __DEFINE_CPP_OVERLOAD_SECURE_FUNC_1_1(errno_t,_ltow_s,long,_Val,wchar_t,_DstBuf,int,_Radix)

  _CRTIMP errno_t __cdecl _ultow_s (unsigned long _Val,wchar_t *_DstBuf,size_t _SizeInWords,int _Radix);
  __DEFINE_CPP_OVERLOAD_SECURE_FUNC_1_1(errno_t,_ultow_s,unsigned long,_Val,wchar_t,_DstBuf,int,_Radix)

  _CRTIMP errno_t __cdecl _wgetenv_s(size_t *_ReturnSize,wchar_t *_DstBuf,size_t _DstSizeInWords,const wchar_t *_VarName);
  __DEFINE_CPP_OVERLOAD_SECURE_FUNC_1_1(errno_t,_wgetenv_s,size_t*,_ReturnSize,wchar_t,_DstBuf,const wchar_t*,_VarName)

  _CRTIMP errno_t __cdecl _wdupenv_s(wchar_t **_Buffer,size_t *_BufferSizeInWords,const wchar_t *_VarName);
  _CRTIMP errno_t __cdecl _i64tow_s(__int64 _Val,wchar_t *_DstBuf,size_t _SizeInWords,int _Radix);
  _CRTIMP errno_t __cdecl _ui64tow_s(unsigned __int64 _Val,wchar_t *_DstBuf,size_t _SizeInWords,int _Radix);
#endif

#ifndef _POSIX_
#ifndef _WSTDLIBP_S_DEFINED
#define _WSTDLIBP_S_DEFINED
  _CRTIMP errno_t __cdecl _wmakepath_s(wchar_t *_PathResult,size_t _SizeInWords,const wchar_t *_Drive,const wchar_t *_Dir,const wchar_t *_Filename,const wchar_t *_Ext);
  __DEFINE_CPP_OVERLOAD_SECURE_FUNC_0_4(errno_t,_wmakepath_s,wchar_t,_PathResult,const wchar_t*,_Drive,const wchar_t*,_Dir,const wchar_t*,_Filename,const wchar_t*,_Ext)

  _CRTIMP errno_t __cdecl _wputenv_s(const wchar_t *_Name,const wchar_t *_Value);

  _CRTIMP errno_t __cdecl _wsearchenv_s(const wchar_t *_Filename,const wchar_t *_EnvVar,wchar_t *_ResultPath,size_t _SizeInWords);
  __DEFINE_CPP_OVERLOAD_SECURE_FUNC_2_0(errno_t,_wsearchenv_s,const wchar_t*,_Filename,const wchar_t*,_EnvVar,wchar_t,_ResultPath)

  _CRTIMP errno_t __cdecl _wsplitpath_s(const wchar_t *_FullPath,wchar_t *_Drive,size_t _DriveSizeInWords,wchar_t *_Dir,size_t _DirSizeInWords,wchar_t *_Filename,size_t _FilenameSizeInWords,wchar_t *_Ext,size_t _ExtSizeInWords);
  __DEFINE_CPP_OVERLOAD_SECURE_FUNC_SPLITPATH(errno_t,_wsplitpath_s,wchar_t,_Dest)

#endif
#endif

#ifndef _WSTRING_S_DEFINED
#define _WSTRING_S_DEFINED
  _CRTIMP wchar_t *__cdecl wcstok_s(wchar_t *_Str,const wchar_t *_Delim,wchar_t **_Context);
  _CRTIMP errno_t __cdecl _wcserror_s(wchar_t *_Buf,size_t _SizeInWords,int _ErrNum);
  _CRTIMP errno_t __cdecl __wcserror_s(wchar_t *_Buffer,size_t _SizeInWords,const wchar_t *_ErrMsg);
  _CRTIMP errno_t __cdecl _wcsnset_s(wchar_t *_Dst,size_t _DstSizeInWords,wchar_t _Val,size_t _MaxCount);
  _CRTIMP errno_t __cdecl _wcsset_s(wchar_t *_Str,size_t _SizeInWords,wchar_t _Val);
  _CRTIMP errno_t __cdecl _wcslwr_s(wchar_t *_Str,size_t _SizeInWords);
  _CRTIMP errno_t __cdecl _wcslwr_s_l(wchar_t *_Str,size_t _SizeInWords,_locale_t _Locale);
  _CRTIMP errno_t __cdecl _wcsupr_s(wchar_t *_Str,size_t _Size);
  _CRTIMP errno_t __cdecl _wcsupr_s_l(wchar_t *_Str,size_t _Size,_locale_t _Locale);

  _CRTIMP errno_t __cdecl wcscat_s(wchar_t *_Dst, rsize_t _DstSize, const wchar_t *_Src);
  __DEFINE_CPP_OVERLOAD_SECURE_FUNC_0_1(errno_t, wcscat_s, wchar_t, _Dest, const wchar_t *, _Source)
  _CRTIMP errno_t __cdecl wcscpy_s(wchar_t *_Dst, rsize_t _DstSize, const wchar_t *_Src);
  __DEFINE_CPP_OVERLOAD_SECURE_FUNC_0_1(errno_t, wcscpy_s, wchar_t, _Dest, const wchar_t *, _Source)

  _CRTIMP errno_t __cdecl wcsncat_s(wchar_t *_Dst,size_t _DstSizeInChars,const wchar_t *_Src,size_t _MaxCount);
  _CRTIMP errno_t __cdecl _wcsncat_s_l(wchar_t *_Dst,size_t _DstSizeInChars,const wchar_t *_Src,size_t _MaxCount,_locale_t _Locale);
  _CRTIMP errno_t __cdecl wcsncpy_s(wchar_t *_Dst,size_t _DstSizeInChars,const wchar_t *_Src,size_t _MaxCount);
  _CRTIMP errno_t __cdecl _wcsncpy_s_l(wchar_t *_Dst,size_t _DstSizeInChars,const wchar_t *_Src,size_t _MaxCount,_locale_t _Locale);
  _CRTIMP wchar_t *__cdecl _wcstok_s_l(wchar_t *_Str,const wchar_t *_Delim,wchar_t **_Context,_locale_t _Locale);
  _CRTIMP errno_t __cdecl _wcsset_s_l(wchar_t *_Str,size_t _SizeInChars,unsigned int _Val,_locale_t _Locale);
  _CRTIMP errno_t __cdecl _wcsnset_s_l(wchar_t *_Str,size_t _SizeInChars,unsigned int _Val, size_t _Count,_locale_t _Locale);

  __forceinline size_t __cdecl wcsnlen_s(const wchar_t * _src, size_t _count) {
    return _src ? wcsnlen(_src, _count) : 0;
  }
#endif

#ifndef _WTIME_S_DEFINED
#define _WTIME_S_DEFINED
  errno_t __cdecl _wasctime_s (wchar_t *_Buf,size_t _SizeInWords,const struct tm *_Tm);
  errno_t __cdecl _wctime32_s (wchar_t *_Buf,size_t _SizeInWords,const __time32_t *_Time);
  errno_t __cdecl _wstrdate_s (wchar_t *_Buf,size_t _SizeInWords);
  errno_t __cdecl _wstrtime_s (wchar_t *_Buf,size_t _SizeInWords);
  errno_t __cdecl _wctime64_s (wchar_t *_Buf,size_t _SizeInWords,const __time64_t *_Time);

#if !defined (RC_INVOKED) && !defined (_INC_WTIME_S_INL)
#define _INC_WTIME_S_INL
  errno_t __cdecl _wctime_s(wchar_t *, size_t, const time_t *);
#ifndef _USE_32BIT_TIME_T
__CRT_INLINE errno_t __cdecl _wctime_s(wchar_t *_Buffer,size_t _SizeInWords,const time_t *_Time) { return _wctime64_s(_Buffer,_SizeInWords,_Time); }
#endif
#endif
#endif

  _CRTIMP errno_t __cdecl mbsrtowcs_s(size_t *_Retval,wchar_t *_Dst,size_t _SizeInWords,const char **_PSrc,size_t _N,mbstate_t *_State);
  __DEFINE_CPP_OVERLOAD_SECURE_FUNC_1_3(errno_t,mbsrtowcs_s,size_t*,_Retval,wchar_t,_Dst,const char**,_PSrc,size_t,_N,mbstate_t,_State)

  _CRTIMP errno_t __cdecl wcrtomb_s(size_t *_Retval,char *_Dst,size_t _SizeInBytes,wchar_t _Ch,mbstate_t *_State);
  __DEFINE_CPP_OVERLOAD_SECURE_FUNC_1_2(errno_t,wcrtomb_s,size_t*,_Retval,char,_Dst,wchar_t,_Ch,mbstate_t,_State)

  _CRTIMP errno_t __cdecl wcsrtombs_s(size_t *_Retval,char *_Dst,size_t _SizeInBytes,const wchar_t **_Src,size_t _Size,mbstate_t *_State);
  __DEFINE_CPP_OVERLOAD_SECURE_FUNC_1_3(errno_t,wcsrtombs_s,size_t,_Retval,char,_Dst,const wchar_t**,_Src,size_t,_Size,mbstate_t,_State)

  _CRTIMP errno_t __cdecl wmemcpy_s (wchar_t *_dest,size_t _numberOfElements,const wchar_t *_src,size_t _count);
  _CRTIMP errno_t __cdecl wmemmove_s(wchar_t *_dest,size_t _numberOfElements,const wchar_t *_src,size_t _count);

#ifdef __cplusplus
}
#endif

#endif
#endif
