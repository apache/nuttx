/****************************************************************************
 * include/debug.h
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_DEBUG_H
#define __INCLUDE_DEBUG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Debug macros to runtime filter the opsys debug messages */

#ifdef CONFIG_HAVE_FUNCTIONNAME
# define EXTRA_FMT "%s: "
# define EXTRA_ARG ,__FUNCTION__
#else
# define EXTRA_FMT
# define EXTRA_ARG
#endif

/* Debug macros will differ depending upon if the toolchain supports
 * macros with a variable number of arguments or not.
 */

#ifdef CONFIG_CPP_HAVE_VARARGS

/* Variable argument macros supported */

#ifdef CONFIG_DEBUG
# define dbg(format, arg...) \
  lib_rawprintf(EXTRA_FMT format EXTRA_ARG, ##arg)

# ifdef CONFIG_ARCH_LOWPUTC
#  define lldbg(format, arg...) \
     lib_lowprintf(EXTRA_FMT format EXTRA_ARG, ##arg)
# else
#  define lldbg(x...)
# endif

# ifdef CONFIG_DEBUG_VERBOSE
#  define vdbg(format, arg...) \
     lib_rawprintf(EXTRA_FMT format EXTRA_ARG, ##arg)

#     ifdef CONFIG_ARCH_LOWPUTC
#       define llvdbg(format, arg...) \
          lib_lowprintf(EXTRA_FMT format EXTRA_ARG, ##arg)
#     else
#         define llvdbg(x...)
#     endif

# else
#  define vdbg(x...)
#  define llvdbg(x...)
# endif

#else /* CONFIG_DEBUG */

# define dbg(x...)
# define lldbg(x...)
# define vdbg(x...)
# define llvdbg(x...)

#endif /* CONFIG_DEBUG */

/* Subsystem specific debug */

#ifdef CONFIG_DEBUG_MM
# define mdbg(format, arg...)    dbg(format, ##arg)
# define mlldbg(format, arg...)  lldbg(format, ##arg)
# define mvdbg(format, arg...)   vdbg(format, ##arg)
# define mllvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define mdbg(x...)
# define mlldbg(x...)
# define mvdbg(x...)
# define mllvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_SCHED
# define sdbg(format, arg...)    dbg(format, ##arg)
# define slldbg(format, arg...)  lldbg(format, ##arg)
# define svdbg(format, arg...)   vdbg(format, ##arg)
# define sllvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define sdbg(x...)
# define slldbg(x...)
# define svdbg(x...)
# define sllvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_NET
# define ndbg(format, arg...)    dbg(format, ##arg)
# define nlldbg(format, arg...)  lldbg(format, ##arg)
# define nvdbg(format, arg...)   vdbg(format, ##arg)
# define nllvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define ndbg(x...)
# define nlldbg(x...)
# define nvdbg(x...)
# define nllvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_USB
# define udbg(format, arg...)    dbg(format, ##arg)
# define ulldbg(format, arg...)  lldbg(format, ##arg)
# define uvdbg(format, arg...)   vdbg(format, ##arg)
# define ullvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define udbg(x...)
# define ulldbg(x...)
# define uvdbg(x...)
# define ullvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_FS
# define fdbg(format, arg...)    dbg(format, ##arg)
# define flldbg(format, arg...)  lldbg(format, ##arg)
# define fvdbg(format, arg...)   vdbg(format, ##arg)
# define fllvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define fdbg(x...)
# define flldbg(x...)
# define fvdbg(x...)
# define fllvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS
# define gdbg(format, arg...)    dbg(format, ##arg)
# define glldbg(format, arg...)  lldbg(format, ##arg)
# define gvdbg(format, arg...)   vdbg(format, ##arg)
# define gllvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define gdbg(x...)
# define glldbg(x...)
# define gvdbg(x...)
# define gllvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_BINFMT
# define bdbg(format, arg...)    dbg(format, ##arg)
# define blldbg(format, arg...)  lldbg(format, ##arg)
# define bvdbg(format, arg...)   vdbg(format, ##arg)
# define bllvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define bdbg(x...)
# define blldbg(x...)
# define bvdbg(x...)
# define bllvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_LIB
# define ldbg(format, arg...)    dbg(format, ##arg)
# define llldbg(format, arg...)  lldbg(format, ##arg)
# define lvdbg(format, arg...)   vdbg(format, ##arg)
# define lllvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define ldbg(x...)
# define llldbg(x...)
# define lvdbg(x...)
# define lllvdbg(x...)
#endif

#else /* CONFIG_CPP_HAVE_VARARGS */

/* Variable argument macros NOT supported */

#ifdef CONFIG_DEBUG
# ifndef CONFIG_ARCH_LOWPUTC
#  define lldbg (void)
# endif
# ifndef CONFIG_DEBUG_VERBOSE
#  define vdbg (void)
#  define llvdbg (void)
# else
#  ifndef CONFIG_ARCH_LOWPUTC
#    define llvdbg (void)
#  endif
# endif
#else
# define dbg    (void)
# define lldbg  (void)
# define vdbg   (void)
# define llvdbg (void)
#endif

/* Subsystem specific debug */

#ifdef CONFIG_DEBUG_MM
# define mdbg    dbg
# define mlldbg  lldbg
# define mvdbg   vdbg
# define mllvdbg llvdbg
#else
# define mdbg    (void)
# define mlldbg  (void)
# define mvdbg   (void)
# define mllvdbg (void)
#endif

#ifdef CONFIG_DEBUG_SCHED
# define sdbg    dbg
# define slldbg  lldbg
# define svdbg   vdbg
# define sllvdbg llvdbg
#else
# define sdbg    (void)
# define slldbg  (void)
# define svdbg   (void)
# define sllvdbg (void)
#endif

#ifdef CONFIG_DEBUG_NET
# define ndbg    dbg
# define nlldbg  lldbg
# define nvdbg   vdbg
# define nllvdbg llvdbg
#else
# define ndbg    (void)
# define nlldbg  (void)
# define nvdbg   (void)
# define nllvdbg (void)
#endif

#ifdef CONFIG_DEBUG_USB
# define udbg    dbg
# define ulldbg  lldbg
# define uvdbg   vdbg
# define ullvdbg llvdbg
#else
# define udbg    (void)
# define ulldbg  (void)
# define uvdbg   (void)
# define ullvdbg (void)
#endif

#ifdef CONFIG_DEBUG_FS
# define fdbg    dbg
# define flldbg  lldbg
# define fvdbg   vdbg
# define fllvdbg llvdbg
#else
# define fdbg    (void)
# define flldbg  (void)
# define fvdbg   (void)
# define fllvdbg (void)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS
# define gdbg    dbg
# define glldbg  lldbg
# define gvdbg   vdbg
# define gllvdbg llvdbg
#else
# define gdbg    (void)
# define glldbg  (void)
# define gvdbg   (void)
# define gllvdbg (void)
#endif

#ifdef CONFIG_DEBUG_BINFMT
# define bdbg    dbg
# define blldbg  lldbg
# define bvdbg   vdbg
# define bllvdbg llvdbg
#else
# define bdbg    (void)
# define blldbg  (void)
# define bvdbg   (void)
# define bllvdbg (void)
#endif

#ifdef CONFIG_DEBUG_LIB
# define ldbg    dbg
# define llldbg  lldbg
# define lvdbg   vdbg
# define lllvdbg llvdbg
#else
# define ldbg    (void)
# define llldbg  (void)
# define lvdbg   (void)
# define lllvdbg (void)
#endif

#endif /* CONFIG_CPP_HAVE_VARARGS */

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* These low-level debug APIs are provided by the NuttX library.  If
 * the cross-compiler's pre-processor supports a variable number of
 * macro arguments, then the macros above will map all debug statements
 * on or the other of the following.
 */

EXTERN int lib_rawprintf(const char *format, ...);

#ifdef CONFIG_ARCH_LOWPUTC
EXTERN int lib_lowprintf(const char *format, ...);
#endif

/* If the cross-compiler's pre-processor does not support variable
 * length arguments, then these additional APIs will be built.
 */

#ifndef CONFIG_CPP_HAVE_VARARGS
#ifdef CONFIG_DEBUG
EXTERN int dbg(const char *format, ...);

# ifdef CONFIG_ARCH_LOWPUTC
EXTERN int lldbg(const char *format, ...);
# endif

# ifdef CONFIG_DEBUG_VERBOSE
EXTERN int vdbg(const char *format, ...);

# ifdef CONFIG_ARCH_LOWPUTC
EXTERN int llvdbg(const char *format, ...);
# endif
#endif
#endif /* CONFIG_DEBUG */
#endif /* CONFIG_CPP_HAVE_VARARGS */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_DEBUG_H */
