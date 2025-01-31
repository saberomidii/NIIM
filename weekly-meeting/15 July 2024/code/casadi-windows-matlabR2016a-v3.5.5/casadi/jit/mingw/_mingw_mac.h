/**
 * This file has no copyright assigned and is placed in the Public Domain.
 * This file is part of the mingw-w64 runtime package.
 * No warranty is given; refer to the file DISCLAIMER.PD within this package.
 */

#ifndef _INC_CRTDEFS_MACRO
#define _INC_CRTDEFS_MACRO

#define __STRINGIFY(x) #x
#define __MINGW64_STRINGIFY(x) \
  __STRINGIFY(x)

#define __MINGW64_VERSION_MAJOR 4
#define __MINGW64_VERSION_MINOR 0

/* This macro holds an monotonic increasing value, which indicates
   a specific fix/patch is present on trunk.  This value isn't related to
   minor/major version-macros.  It is increased on demand, if a big
   fix was applied to trunk.  This macro gets just increased on trunk.  For
   other branches its value won't be modified.  */

#define __MINGW64_VERSION_RC 0

#define __MINGW64_VERSION_STR	\
  __MINGW64_STRINGIFY(__MINGW64_VERSION_MAJOR) \
  "." \
  __MINGW64_STRINGIFY(__MINGW64_VERSION_MINOR)

#define __MINGW64_VERSION_STATE "stable"

/* mingw.org's version macros: these make gcc to define
   MINGW32_SUPPORTS_MT_EH and to use the _CRT_MT global
   and the __mingwthr_key_dtor() function from the MinGW
   CRT in its private gthr-win32.h header. */
#define __MINGW32_MAJOR_VERSION 3
#define __MINGW32_MINOR_VERSION 11

#ifdef _WIN64
   /* MS does not prefix symbols by underscores for 64-bit.  */
#  ifndef __MINGW_USE_UNDERSCORE_PREFIX
     /* As we have to support older gcc version, which are using underscores
      as symbol prefix for x64, we have to check here for the user label
      prefix defined by gcc. */
#    ifdef __USER_LABEL_PREFIX__
#      pragma push_macro ("_")
#      undef _
#      define _ 1
#      if (__USER_LABEL_PREFIX__ + 0) != 0
#        define __MINGW_USE_UNDERSCORE_PREFIX 1
#      else
#        define __MINGW_USE_UNDERSCORE_PREFIX 0
#      endif
#      undef _
#      pragma pop_macro ("_")
#    else /* ! __USER_LABEL_PREFIX__ */
#      define __MINGW_USE_UNDERSCORE_PREFIX 0
#    endif /* __USER_LABEL_PREFIX__ */
#  endif
#else /* ! ifdef _WIN64 */
   /* For 32-bits we have always to prefix by underscore.  */
#  undef __MINGW_USE_UNDERSCORE_PREFIX
#  define __MINGW_USE_UNDERSCORE_PREFIX 1
#endif /* ifdef _WIN64 */

#if __MINGW_USE_UNDERSCORE_PREFIX == 0
#  define __MINGW_IMP_SYMBOL(sym) __imp_##sym
#  define __MINGW_IMP_LSYMBOL(sym) __imp_##sym
#  define __MINGW_USYMBOL(sym) sym
#  define __MINGW_LSYMBOL(sym) _##sym
#else /* ! if __MINGW_USE_UNDERSCORE_PREFIX == 0 */
#  define __MINGW_IMP_SYMBOL(sym) _imp__##sym
#  define __MINGW_IMP_LSYMBOL(sym) __imp__##sym
#  define __MINGW_USYMBOL(sym) _##sym
#  define __MINGW_LSYMBOL(sym) sym
#endif /* if __MINGW_USE_UNDERSCORE_PREFIX == 0 */

/* Set VC specific compiler target macros.  */
#if defined(__x86_64) && defined(_X86_)
#  undef _X86_	/* _X86_ is not for __x86_64 */
#endif

#if defined(_X86_) && !defined(_M_IX86) && !defined(_M_IA64) \
   && !defined(_M_AMD64) && !defined(__x86_64)
#  if defined(__i486__)
#    define _M_IX86 400
#  elif defined(__i586__)
#    define _M_IX86 500
#  else
     /* This gives wrong (600 instead of 300) value if -march=i386 is specified
      but we cannot check for__i386__ as it is defined for all 32-bit CPUs. */
#    define _M_IX86 600
#  endif
#endif /* if defined(_X86_) && !defined(_M_IX86) && !defined(_M_IA64) ... */

#if defined(__x86_64) && !defined(_M_IX86) && !defined(_M_IA64) \
   && !defined(_M_AMD64)
#  define _M_AMD64 100
#  define _M_X64 100
#endif

#if defined(__ia64__) && !defined(_M_IX86) && !defined(_M_IA64) \
   && !defined(_M_AMD64) && !defined(_X86_) && !defined(__x86_64)
#  define _M_IA64 100
#endif

#if defined(__arm__) && !defined(_M_ARM)
#  define _M_ARM 100
#  ifndef _ARM_
#    define _ARM_ 1
#  endif
#endif

#ifndef __PTRDIFF_TYPE__
#  ifdef _WIN64
#    define __PTRDIFF_TYPE__ long long int
#  else
#    define __PTRDIFF_TYPE__ long int
#  endif
#endif

#ifndef __SIZE_TYPE__
#  ifdef _WIN64
#    define __SIZE_TYPE__ long long unsigned int
#  else
#    define __SIZE_TYPE__ long unsigned int
#  endif
#endif

#ifndef __WCHAR_TYPE__
#  define __WCHAR_TYPE__ unsigned short
#endif

#ifndef __WINT_TYPE__
#  define __WINT_TYPE__ unsigned short
#endif

#undef __MINGW_EXTENSION

#ifdef __WIDL__
#  define __MINGW_EXTENSION
#else
#  if defined(__GNUC__) || defined(__GNUG__)
#    define __MINGW_EXTENSION __extension__
#  else
#    define __MINGW_EXTENSION
#  endif
#endif /* __WIDL__ */

/* Special case nameless struct/union.  */
#ifndef __C89_NAMELESS
#  define __C89_NAMELESS __MINGW_EXTENSION
#  define __C89_NAMELESSSTRUCTNAME
#  define __C89_NAMELESSSTRUCTNAME1
#  define __C89_NAMELESSSTRUCTNAME2
#  define __C89_NAMELESSSTRUCTNAME3
#  define __C89_NAMELESSSTRUCTNAME4
#  define __C89_NAMELESSSTRUCTNAME5
#  define __C89_NAMELESSUNIONNAME
#  define __C89_NAMELESSUNIONNAME1
#  define __C89_NAMELESSUNIONNAME2
#  define __C89_NAMELESSUNIONNAME3
#  define __C89_NAMELESSUNIONNAME4
#  define __C89_NAMELESSUNIONNAME5
#  define __C89_NAMELESSUNIONNAME6
#  define __C89_NAMELESSUNIONNAME7
#  define __C89_NAMELESSUNIONNAME8
#endif

#ifndef __GNU_EXTENSION
#  define __GNU_EXTENSION __MINGW_EXTENSION
#endif

/* MinGW-w64 has some additional C99 printf/scanf feature support.
   So we add some helper macros to ease recognition of them.  */
#define __MINGW_HAVE_ANSI_C99_PRINTF 1
#define __MINGW_HAVE_WIDE_C99_PRINTF 1
#define __MINGW_HAVE_ANSI_C99_SCANF 1
#define __MINGW_HAVE_WIDE_C99_SCANF 1

#ifdef __MINGW_USE_BROKEN_INTERFACE
#  define __MINGW_POISON_NAME(__IFACE) __IFACE
#else
#  define __MINGW_POISON_NAME(__IFACE) \
     __IFACE##_layout_has_not_been_verified_and_its_declaration_is_most_likely_incorrect
#endif

#ifndef __MSABI_LONG
#  ifndef __LP64__
#    define __MSABI_LONG(x) x ## l
#  else
#    define __MSABI_LONG(x) x
#  endif
#endif

#if __GNUC__
#  define __MINGW_GCC_VERSION	(__GNUC__ * 10000 + \
      __GNUC_MINOR__	* 100	+ \
      __GNUC_PATCHLEVEL__)
#else
#  define __MINGW_GCC_VERSION 0
#endif

#if defined (__GNUC__) && defined (__GNUC_MINOR__)
#  define __MINGW_GNUC_PREREQ(major, minor) \
      (__GNUC__ > (major) \
      || (__GNUC__ == (major) && __GNUC_MINOR__ >= (minor)))
#else
#  define __MINGW_GNUC_PREREQ(major, minor) 0
#endif

#if defined (_MSC_VER)
#  define __MINGW_MSC_PREREQ(major, minor) \
      (_MSC_VER >= (major * 100 + minor * 10))
#else
#  define __MINGW_MSC_PREREQ(major, minor) 0
#endif

#ifdef __MINGW_MSVC_COMPAT_WARNINGS
#  if __MINGW_GNUC_PREREQ (4, 5)
#    define __MINGW_ATTRIB_DEPRECATED_STR(X) \
       __attribute__ ((__deprecated__ (X)))
#  else
#    define __MINGW_ATTRIB_DEPRECATED_STR(X) \
       __MINGW_ATTRIB_DEPRECATED
#  endif
#else
#  define __MINGW_ATTRIB_DEPRECATED_STR(X)
#endif /* ifdef __MINGW_MSVC_COMPAT_WARNINGS */

#define __MINGW_SEC_WARN_STR \
  "This function or variable may be unsafe, use _CRT_SECURE_NO_WARNINGS to disable deprecation"

#define __MINGW_MSVC2005_DEPREC_STR \
  "This POSIX function is deprecated beginning in Visual C++ 2005, use _CRT_NONSTDC_NO_DEPRECATE to disable deprecation"

#if !defined (_CRT_NONSTDC_NO_DEPRECATE)
#  define __MINGW_ATTRIB_DEPRECATED_MSVC2005 \
      __MINGW_ATTRIB_DEPRECATED_STR(__MINGW_MSVC2005_DEPREC_STR)
#else
#  define __MINGW_ATTRIB_DEPRECATED_MSVC2005
#endif

#if !defined (_CRT_SECURE_NO_WARNINGS) || (_CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES == 0)
#  define __MINGW_ATTRIB_DEPRECATED_SEC_WARN \
      __MINGW_ATTRIB_DEPRECATED_STR(__MINGW_SEC_WARN_STR)
#else
#  define __MINGW_ATTRIB_DEPRECATED_SEC_WARN
#endif

#define __MINGW_MS_PRINTF(__format,__args) \
  __attribute__((__format__(ms_printf, __format,__args)))

#define __MINGW_MS_SCANF(__format,__args) \
  __attribute__((__format__(ms_scanf,  __format,__args)))

#define __MINGW_GNU_PRINTF(__format,__args) \
  __attribute__((__format__(gnu_printf,__format,__args)))

#define __MINGW_GNU_SCANF(__format,__args) \
  __attribute__((__format__(gnu_scanf, __format,__args)))

#undef __mingw_ovr

#ifdef __cplusplus
#  define __mingw_ovr  inline __cdecl
#elif defined (__GNUC__)
#  define __mingw_ovr static \
      __attribute__ ((__unused__)) __inline__ __cdecl
#else
#  define __mingw_ovr static __cdecl
#endif /* __cplusplus */

#endif	/* _INC_CRTDEFS_MACRO */
