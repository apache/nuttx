/****************************************************************************
 * include/debug.h
 *
 *   Copyright (C) 2007-2011, 2014, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifdef CONFIG_ARCH_DEBUG_H
# include <arch/debug.h>
#endif

#include <syslog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug macros to runtime filter the debug messages sent to the console.  In
 * general, there are four forms of the debug macros:
 *
 * [a-z]info() -- Outputs messages to the console similar to printf() except
 *    that the output is not buffered.  Output is only generated if
 *    CONFIG_DEBUG_INFO is defined.  The info macros are intended for
 *    verbose "informational" debug output.  If you enable CONFIG_DEBUG_INFO,
 *    then very chatty (and often annoying) output will be generated.
 *
 *    The first character of the macro name indicates the system system
 *    (e.g., n=network, f=filesystm, etc.).  If the first character is
 *    missing (i.e., info()), then it is common.  The common info() macro
 *    is enabled simply with CONFIG_DEBUG_INFO.  Subsystem debug requires an
 *    additional configuration setting to enable it (e.g., CONFIG_DEBUG_NET
 *    for the network, CONFIG_DEBUG_FS for the file system, etc).
 *
 *    In general, error messages and output of importance use [a-z]err().
 *    [a-z]err() is implementation dependent but usually uses file descriptors.
 *    (that is a problem only because the interrupt task may have re-
 *    directed stdout).  Therefore [a-z]err() should not be used in interrupt
 *    handlers.
 *
 * [a-z]warn() -- Identical to [a-z]info() except that it also requires that
 *    CONFIG_DEBUG_WARN be defined.  This is intended for important exception
 *    conditions that are potential errors (or perhaps real errors with non-
 *    fatal consequences).
 *
 * [a-z]err() -- Identical to [a-z]info() except that it also requires that
 *    CONFIG_DEBUG_ERROR be defined.  This is intended for important error-related
 *    information that you probably not want to suppress during normal debug
 *    general debugging.
 *
 * [a-z]llinfo() -- Identical to [a-z]err() except this is uses special
 *    interfaces provided by architecture-specific logic to talk directly
 *    to the underlying console hardware.  If the architecture provides such
 *    logic, it should define CONFIG_ARCH_LOWPUTC.
 *
 *    [a-z]llinfo() should not be used in normal code because the implementation
 *    probably disables interrupts and does things that are not consistent with
 *    good real-time performance.  However, [a-z]llinfo() is particularly useful
 *    in low-level code where it is inappropriate to use file descriptors.  For
 *    example, only [a-z]llinfo() should be used in interrupt handlers.
 *
 * [a-z]llwarn() -- Identical to [a-z]llinfo() except that it also requires that
 *    CONFIG_DEBUG_WARN be defined.  This is intended for important exception
 *    conditions that are potential errors (or perhaps real errors with non-
 *    fatal consequences).
 *
 * [a-z]llerr() -- Identical to [a-z]llinfo() except that it also requires that
 *    CONFIG_DEBUG_ERROR be defined. This is intended for important error-related
 *    information that you probably not want to suppress during normal debug
 *    general debugging.
 */

#ifdef CONFIG_HAVE_FUNCTIONNAME
#  define EXTRA_FMT "%s: "
#  define EXTRA_ARG ,__FUNCTION__
#else
#  define EXTRA_FMT
#  define EXTRA_ARG
#endif

/* The actual logger function may be overridden in arch/debug.h if needed. */

#ifndef __arch_syslog
#  define __arch_syslog syslog
#endif
#ifndef __arch_lowsyslog
#  define __arch_lowsyslog lowsyslog
#endif

/* Debug macros will differ depending upon if the toolchain supports
 * macros with a variable number of arguments or not.
 */

#ifdef CONFIG_CPP_HAVE_VARARGS

/* C-99 style variadic macros are supported */

#ifdef CONFIG_DEBUG_ERROR
#  define err(format, ...) \
   __arch_syslog(LOG_ERR, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)

# ifdef CONFIG_ARCH_LOWPUTC
#  define llerr(format, ...) \
   __arch_lowsyslog(LOG_ERR, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
# else
#  define llerr(x...)
# endif
#else /* CONFIG_DEBUG_ERROR */
#  define err(x...)
#  define llerr(x...)
#endif

#ifdef CONFIG_DEBUG_WARN
#  define warn(format, ...) \
   __arch_syslog(LOG_DEBUG, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)

#  ifdef CONFIG_ARCH_LOWPUTC
#    define llwarn(format, ...) \
     __arch_lowsyslog(LOG_DEBUG, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#  else
#    define llwarn(x...)
#  endif
#else /* CONFIG_DEBUG_INFO */
#  define warn(x...)
#  define llwarn(x...)
#endif /* CONFIG_DEBUG_INFO */

#ifdef CONFIG_DEBUG_INFO
#  define info(format, ...) \
   __arch_syslog(LOG_DEBUG, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)

#  ifdef CONFIG_ARCH_LOWPUTC
#    define llinfo(format, ...) \
     __arch_lowsyslog(LOG_DEBUG, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#  else
#    define llinfo(x...)
#  endif
#else /* CONFIG_DEBUG_INFO */
#  define info(x...)
#  define llinfo(x...)
#endif /* CONFIG_DEBUG_INFO */

/* Subsystem specific debug */

#ifdef CONFIG_DEBUG_MM
#  define merr(format, ...)    err(format, ##__VA_ARGS__)
#  define mllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#  define mwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define mllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#  define minfo(format, ...)   info(format, ##__VA_ARGS__)
#  define mllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define merr(x...)
#  define mllerr(x...)
#  define mwarn(x...)
#  define mllwarn(x...)
#  define minfo(x...)
#  define mllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_SCHED
#  define serr(format, ...)    err(format, ##__VA_ARGS__)
#  define sllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#  define swarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define sllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#  define sinfo(format, ...)   info(format, ##__VA_ARGS__)
#  define sllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define serr(x...)
#  define sllerr(x...)
#  define swarn(x...)
#  define sllwarn(x...)
#  define sinfo(x...)
#  define sllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_PAGING
#  define pgerr(format, ...)    err(format, ##__VA_ARGS__)
#  define pgllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#  define pgwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define pgllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#  define pginfo(format, ...)   info(format, ##__VA_ARGS__)
#  define pgllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define pgerr(x...)
#  define pgllerr(x...)
#  define pgwarn(x...)
#  define pgllwarn(x...)
#  define pginfo(x...)
#  define pgllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_DMA
#  define dmaerr(format, ...)    err(format, ##__VA_ARGS__)
#  define dmallerr(format, ...)  llerr(format, ##__VA_ARGS__)
#  define dmawarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define dmallwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#  define dmainfo(format, ...)   info(format, ##__VA_ARGS__)
#  define dmallinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define dmaerr(x...)
#  define dmallerr(x...)
#  define dmawarn(x...)
#  define dmallwarn(x...)
#  define dmainfo(x...)
#  define dmallinfo(x...)
#endif

#ifdef CONFIG_DEBUG_NET
#  define nerr(format, ...)    err(format, ##__VA_ARGS__)
#  define nllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#  define nwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define nllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#  define ninfo(format, ...)   info(format, ##__VA_ARGS__)
#  define nllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define nerr(x...)
#  define nllerr(x...)
#  define nwarn(x...)
#  define nllwarn(x...)
#  define ninfo(x...)
#  define nllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_USB
#  define uerr(format, ...)    err(format, ##__VA_ARGS__)
#  define ullerr(format, ...)  llerr(format, ##__VA_ARGS__)
#  define uwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define ullwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#  define uinfo(format, ...)   info(format, ##__VA_ARGS__)
#  define ullinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define uerr(x...)
#  define ullerr(x...)
#  define uwarn(x...)
#  define ullwarn(x...)
#  define uinfo(x...)
#  define ullinfo(x...)
#endif

#ifdef CONFIG_DEBUG_FS
#  define ferr(format, ...)    err(format, ##__VA_ARGS__)
#  define fllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#  define fwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define fllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#  define finfo(format, ...)   info(format, ##__VA_ARGS__)
#  define fllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define ferr(x...)
#  define fllerr(x...)
#  define fwarn(x...)
#  define fllwarn(x...)
#  define finfo(x...)
#  define fllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_CRYPTO
#  define crypterr(format, ...)    err(format, ##__VA_ARGS__)
#  define cryptllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#  define cryptwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define cryptllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#  define cryptinfo(format, ...)   info(format, ##__VA_ARGS__)
#  define cryptllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define crypterr(x...)
#  define cryptllerr(x...)
#  define cryptwarn(x...)
#  define cryptllwarn(x...)
#  define cryptinfo(x...)
#  define cryptllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_INPUT
#  define ierr(format, ...)    err(format, ##__VA_ARGS__)
#  define illerr(format, ...)  llerr(format, ##__VA_ARGS__)
#  define iwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define illwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#  define iinfo(format, ...)   info(format, ##__VA_ARGS__)
#  define illinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define ierr(x...)
#  define illerr(x...)
#  define iwarn(x...)
#  define illwarn(x...)
#  define iinfo(x...)
#  define illinfo(x...)
#endif

#ifdef CONFIG_DEBUG_SENSORS
#  define snerr(format, ...)    err(format, ##__VA_ARGS__)
#  define snllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#  define snwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define snllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#  define sninfo(format, ...)   info(format, ##__VA_ARGS__)
#  define snllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define snerr(x...)
#  define snllerr(x...)
#  define snwarn(x...)
#  define snllwarn(x...)
#  define sninfo(x...)
#  define snllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_ANALOG
#  define aerr(format, ...)    err(format, ##__VA_ARGS__)
#  define allerr(format, ...)  llerr(format, ##__VA_ARGS__)
#  define awarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define allwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#  define ainfo(format, ...)   info(format, ##__VA_ARGS__)
#  define allinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define aerr(x...)
#  define allerr(x...)
#  define awarn(x...)
#  define allwarn(x...)
#  define ainfo(x...)
#  define allinfo(x...)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS
#  define gerr(format, ...)    err(format, ##__VA_ARGS__)
#  define gllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#  define gwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define gllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#  define ginfo(format, ...)   info(format, ##__VA_ARGS__)
#  define gllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define gerr(x...)
#  define gllerr(x...)
#  define gwarn(x...)
#  define gllwarn(x...)
#  define ginfo(x...)
#  define gllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_BINFMT
#  define berr(format, ...)    err(format, ##__VA_ARGS__)
#  define bllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#  define bwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define bllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#  define binfo(format, ...)   info(format, ##__VA_ARGS__)
#  define bllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define berr(x...)
#  define bllerr(x...)
#  define bwarn(x...)
#  define bllwarn(x...)
#  define binfo(x...)
#  define bllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_LIB
#  define lerr(format, ...)    err(format, ##__VA_ARGS__)
#  define lllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#  define lwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define lllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#  define linfo(format, ...)   info(format, ##__VA_ARGS__)
#  define lllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define lerr(x...)
#  define lllerr(x...)
#  define lwarn(x...)
#  define lllwarn(x...)
#  define linfo(x...)
#  define lllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_AUDIO
#  define auderr(format, ...)    err(format, ##__VA_ARGS__)
#  define audllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#  define audwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define audllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#  define audinfo(format, ...)   info(format, ##__VA_ARGS__)
#  define audllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define auderr(x...)
#  define audllerr(x...)
#  define audwarn(x...)
#  define audllwarn(x...)
#  define audinfo(x...)
#  define audllinfo(x...)
#endif

#else /* CONFIG_CPP_HAVE_VARARGS */

/* Variadic macros NOT supported */

#ifdef CONFIG_DEBUG_ERROR
#  ifndef CONFIG_ARCH_LOWPUTC
#    define llerr     (void)
#  endif
#else
#  define err         (void)
#  define llerr       (void)
#endif

#ifdef CONFIG_DEBUG_WARN
#  ifndef CONFIG_ARCH_LOWPUTC
#    define llwarn    (void)
#  endif
#else
#  define warn        (void)
#  define llwarn      (void)
#endif

#ifdef CONFIG_DEBUG_INFO
#  ifndef CONFIG_ARCH_LOWPUTC
#    define llinfo    (void)
#  endif
#else
#  define info        (void)
#  define llinfo      (void)
#endif

/* Subsystem specific debug */

#ifdef CONFIG_DEBUG_MM
#  define merr        err
#  define mllerr      llerr
#  define mwarn       warn
#  define mllwarn     llwarn
#  define minfo       info
#  define mllinfo     llinfo
#else
#  define merr        (void)
#  define mllerr      (void)
#  define mwarn       (void)
#  define mllwarn     (void)
#  define minfo       (void)
#  define mllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_SCHED
#  define serr        err
#  define sllerr      llerr
#  define swarn       warn
#  define sllwarn     llwarn
#  define sinfo       info
#  define sllinfo     llinfo
#else
#  define serr        (void)
#  define sllerr      (void)
#  define swarn       (void)
#  define sllwarn     (void)
#  define sinfo       (void)
#  define sllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_PAGING
#  define pgerr       err
#  define pgllerr     llerr
#  define pgwarn      warn
#  define pgllwarn    llwarn
#  define pginfo      info
#  define pgllinfo    llinfo
#else
#  define pgerr       (void)
#  define pgllerr     (void)
#  define pgwarn      (void)
#  define pgllwarn    (void)
#  define pginfo      (void)
#  define pgllinfo    (void)
#endif

#ifdef CONFIG_DEBUG_DMA
#  define dmaerr      err
#  define dmallerr    llerr
#  define dmawarn     warn
#  define dmallwarn   llwarn
#  define dmainfo     info
#  define dmallinfo   llinfo
#else
#  define dmaerr      (void)
#  define dmallerr    (void)
#  define dmawarn     (void)
#  define dmallwarn   (void)
#  define dmainfo     (void)
#  define dmallinfo   (void)
#endif

#ifdef CONFIG_DEBUG_NET
#  define nerr        err
#  define nllerr      llerr
#  define nwarn       warn
#  define nllwarn     llwarn
#  define ninfo       info
#  define nllinfo     llinfo
#else
#  define nerr        (void)
#  define nllerr      (void)
#  define nwarn       (void)
#  define nllwarn     (void)
#  define ninfo       (void)
#  define nllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_USB
#  define uerr        err
#  define ullerr      llerr
#  define uwarn       warn
#  define ullwarn     llwarn
#  define uinfo       info
#  define ullinfo     llinfo
#else
#  define uerr        (void)
#  define ullerr      (void)
#  define uwarn       (void)
#  define ullwarn     (void)
#  define uinfo       (void)
#  define ullinfo     (void)
#endif

#ifdef CONFIG_DEBUG_FS
#  define ferr        err
#  define fllerr      llerr
#  define fwarn       warn
#  define fllwarn     llwarn
#  define finfo       info
#  define fllinfo     llinfo
#else
#  define ferr        (void)
#  define fllerr      (void)
#  define fwarn       (void)
#  define fllwarn     (void)
#  define finfo       (void)
#  define fllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_CRYPTO
#  define crypterr    err
#  define cryptllerr  llerr
#  define cryptwarn   warn
#  define cryptllwarn llwarn
#  define cryptinfo   info
#  define cryptllinfo llinfo
#else
#  define crypterr    (void)
#  define cryptllerr  (void)
#  define cryptwarn   (void)
#  define cryptllwarn (void)
#  define cryptinfo   (void)
#  define cryptllinfo (void)
#endif

#ifdef CONFIG_DEBUG_INPUT
#  define ierr        err
#  define illerr      llerr
#  define iwarn       warn
#  define illwarn     llwarn
#  define iinfo       info
#  define illinfo     llinfo
#else
#  define ierr        (void)
#  define illerr      (void)
#  define iwarn       (void)
#  define illwarn     (void)
#  define iinfo       (void)
#  define illinfo     (void)
#endif

#ifdef CONFIG_DEBUG_SENSORS
#  define snerr       err
#  define snllerr     llerr
#  define snwarn      warn
#  define snllwarn    llwarn
#  define sninfo      info
#  define snllinfo    llinfo
#else
#  define snerr       (void)
#  define snllerr     (void)
#  define snwarn      (void)
#  define snllwarn    (void)
#  define sninfo      (void)
#  define snllinfo    (void)
#endif

#ifdef CONFIG_DEBUG_ANALOG
#  define aerr        err
#  define allerr      llerr
#  define awarn       warn
#  define allwarn     llwarn
#  define ainfo       info
#  define allinfo     llinfo
#else
#  define aerr        (void)
#  define allerr      (void)
#  define awarn       (void)
#  define allwarn     (void)
#  define ainfo       (void)
#  define allinfo     (void)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS
#  define gerr        err
#  define gllerr      llerr
#  define gwarn       warn
#  define gllwarn     llwarn
#  define ginfo       info
#  define gllinfo     llinfo
#else
#  define gerr        (void)
#  define gllerr      (void)
#  define gwarn       (void)
#  define gllwarn     (void)
#  define ginfo       (void)
#  define gllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_BINFMT
#  define berr        err
#  define bllerr      llerr
#  define bwarn       warn
#  define bllwarn     llwarn
#  define binfo       info
#  define bllinfo     llinfo
#else
#  define berr        (void)
#  define bllerr      (void)
#  define bwarn       (void)
#  define bllwarn     (void)
#  define binfo       (void)
#  define bllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_LIB
#  define lerr        err
#  define lllerr      llerr
#  define lwarn       warn
#  define lllwarn     llwarn
#  define linfo       info
#  define lllinfo     llinfo
#else
#  define lerr        (void)
#  define lllerr      (void)
#  define lwarn       (void)
#  define lllwarn     (void)
#  define linfo       (void)
#  define lllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_AUDIO
#  define auderr      err
#  define audllerr    llerr
#  define audwarn     warn
#  define audllwarn   llwarn
#  define audinfo     info
#  define audllinfo   llinfo
#else
#  define auderr      (void)
#  define audllerr    (void)
#  define audwarn     (void)
#  define audllwarn   (void)
#  define audinfo     (void)
#  define audllinfo   (void)
#endif

#endif /* CONFIG_CPP_HAVE_VARARGS */

/* Buffer dumping macros do not depend on varargs */

#ifdef CONFIG_DEBUG_ERROR
#  define errdumpbuffer(m,b,n) lib_dumpbuffer(m,b,n)
#  ifdef CONFIG_DEBUG_INFO
#    define infodumpbuffer(m,b,n) lib_dumpbuffer(m,b,n)
#  else
#   define infodumpbuffer(m,b,n)
#  endif
#else
#  define errdumpbuffer(m,b,n)
#  define infodumpbuffer(m,b,n)
# endif

/* Subsystem specific debug */

#ifdef CONFIG_DEBUG_MM
#  define merrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define minfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define merrdumpbuffer(m,b,n)
#  define minfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_SCHED
#  define serrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define sinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define serrdumpbuffer(m,b,n)
#  define sinfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_PAGING
#  define pgerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define pginfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define pgerrdumpbuffer(m,b,n)
#  define pginfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_DMA
#  define dmaerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define dmainfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define dmaerrdumpbuffer(m,b,n)
#  define dmainfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_NET
#  define nerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define ninfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define nerrdumpbuffer(m,b,n)
#  define ninfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_USB
#  define uerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define uinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define uerrdumpbuffer(m,b,n)
#  define uinfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_FS
#  define ferrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define finfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define ferrdumpbuffer(m,b,n)
#  define finfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_INPUT
#  define ierrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define iinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define ierrdumpbuffer(m,b,n)
#  define iinfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS
#  define gerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define ginfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define gerrdumpbuffer(m,b,n)
#  define ginfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_BINFMT
#  define berrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define binfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define berrdumpbuffer(m,b,n)
#  define binfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_LIB
#  define lerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define linfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define lerrdumpbuffer(m,b,n)
#  define linfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_AUDIO
#  define auderrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define audinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define auderrdumpbuffer(m,b,n)
#  define audinfodumpbuffer(m,b,n)
#endif

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/* Dump a buffer of data */

void lib_dumpbuffer(FAR const char *msg, FAR const uint8_t *buffer,
                    unsigned int buflen);

/* The system logging interfaces are normally accessed via the macros
 * provided above.  If the cross-compiler's C pre-processor supports a
 * variable number of macro arguments, then those macros below will map all
 * debug statements to the logging interfaces declared in syslog.h.
 *
 * If the cross-compiler's pre-processor does not support variable length
 * arguments, then these additional APIs will be built.
 */

#ifndef CONFIG_CPP_HAVE_VARARGS
#ifdef CONFIG_DEBUG_ERROR
int err(const char *format, ...);

# ifdef CONFIG_ARCH_LOWPUTC
int llerr(const char *format, ...);
# endif
#endif /* CONFIG_DEBUG_ERROR */

#ifdef CONFIG_DEBUG_WARN
int warn(const char *format, ...);

# ifdef CONFIG_ARCH_LOWPUTC
int llwarn(const char *format, ...);
# endif
#endif /* CONFIG_DEBUG_WARN */

#ifdef CONFIG_DEBUG_INFO
int info(const char *format, ...);

# ifdef CONFIG_ARCH_LOWPUTC
int llinfo(const char *format, ...);
# endif
#endif /* CONFIG_DEBUG_INFO */
#endif /* CONFIG_CPP_HAVE_VARARGS */

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_DEBUG_H */
