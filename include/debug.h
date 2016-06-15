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
 *
 * alert() - is a special, high-priority, unconditional version that is really
 *    intended only for crash error reporting.
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

#ifdef CONFIG_ARCH_LOWPUTC
#  define alert(format, ...) \
   __arch_lowsyslog(LOG_EMERG, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
# else
#  define alert(x...)
# endif

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
   __arch_syslog(LOG_WARNING, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)

#  ifdef CONFIG_ARCH_LOWPUTC
#    define llwarn(format, ...) \
     __arch_lowsyslog(LOG_WARNING, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#  else
#    define llwarn(x...)
#  endif
#else /* CONFIG_DEBUG_INFO */
#  define warn(x...)
#  define llwarn(x...)
#endif /* CONFIG_DEBUG_INFO */

#ifdef CONFIG_DEBUG_INFO
#  define info(format, ...) \
   __arch_syslog(LOG_INFO, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)

#  ifdef CONFIG_ARCH_LOWPUTC
#    define llinfo(format, ...) \
     __arch_lowsyslog(LOG_INFO, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#  else
#    define llinfo(x...)
#  endif
#else /* CONFIG_DEBUG_INFO */
#  define info(x...)
#  define llinfo(x...)
#endif /* CONFIG_DEBUG_INFO */

/* Subsystem specific debug */

#ifdef CONFIG_DEBUG_MM_ERROR
#  define merr(format, ...)    err(format, ##__VA_ARGS__)
#  define mllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define merr(x...)
#  define mllerr(x...)
#endif

#ifdef CONFIG_DEBUG_MM_WARN
#  define mwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define mllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define mwarn(x...)
#  define mllwarn(x...)
#endif

#ifdef CONFIG_DEBUG_MM_INFO
#  define minfo(format, ...)   info(format, ##__VA_ARGS__)
#  define mllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define minfo(x...)
#  define mllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_SCHED_ERROR
#  define serr(format, ...)    err(format, ##__VA_ARGS__)
#  define sllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define serr(x...)
#  define sllerr(x...)
#endif

#ifdef CONFIG_DEBUG_SCHED_WARN
#  define swarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define sllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define swarn(x...)
#  define sllwarn(x...)
#endif

#ifdef CONFIG_DEBUG_SCHED_INFO
#  define sinfo(format, ...)   info(format, ##__VA_ARGS__)
#  define sllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define sinfo(x...)
#  define sllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_PAGING_ERROR
#  define pgerr(format, ...)    err(format, ##__VA_ARGS__)
#  define pgllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define pgerr(x...)
#  define pgllerr(x...)
#endif

#ifdef CONFIG_DEBUG_PAGING_WARN
#  define pgwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define pgllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define pgwarn(x...)
#  define pgllwarn(x...)
#endif

#ifdef CONFIG_DEBUG_PAGING_INFO
#  define pginfo(format, ...)   info(format, ##__VA_ARGS__)
#  define pgllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define pgerr(x...)
#  define pgllerr(x...)
#endif

#ifdef CONFIG_DEBUG_NET_ERROR
#  define nerr(format, ...)    err(format, ##__VA_ARGS__)
#  define nllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define nerr(x...)
#  define nllerr(x...)
#endif

#ifdef CONFIG_DEBUG_NET_WARN
#  define nwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define nllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define nwarn(x...)
#  define nllwarn(x...)
#endif

#ifdef CONFIG_DEBUG_NET_INFO
#  define ninfo(format, ...)   info(format, ##__VA_ARGS__)
#  define nllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define ninfo(x...)
#  define nllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_USB_ERROR
#  define uerr(format, ...)    err(format, ##__VA_ARGS__)
#  define ullerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define uerr(x...)
#  define ullerr(x...)
#endif

#ifdef CONFIG_DEBUG_USB_WARN
#  define uwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define ullwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define uwarn(x...)
#  define ullwarn(x...)
#endif

#ifdef CONFIG_DEBUG_USB_INFO
#  define uinfo(format, ...)   info(format, ##__VA_ARGS__)
#  define ullinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define uinfo(x...)
#  define ullinfo(x...)
#endif

#ifdef CONFIG_DEBUG_FS_ERROR
#  define ferr(format, ...)    err(format, ##__VA_ARGS__)
#  define fllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define ferr(x...)
#  define fllerr(x...)
#endif

#ifdef CONFIG_DEBUG_FS_WARN
#  define fwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define fllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define fwarn(x...)
#  define fllwarn(x...)
#endif

#ifdef CONFIG_DEBUG_FS_INFO
#  define finfo(format, ...)   info(format, ##__VA_ARGS__)
#  define fllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define finfo(x...)
#  define fllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_ERROR
#  define crypterr(format, ...)    err(format, ##__VA_ARGS__)
#  define cryptllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define crypterr(x...)
#  define cryptllerr(x...)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_WARN
#  define cryptwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define cryptllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define cryptwarn(x...)
#  define cryptllwarn(x...)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_INFO
#  define cryptinfo(format, ...)   info(format, ##__VA_ARGS__)
#  define cryptllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define cryptinfo(x...)
#  define cryptllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_INPUT_ERROR
#  define ierr(format, ...)    err(format, ##__VA_ARGS__)
#  define illerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define ierr(x...)
#  define illerr(x...)
#endif

#ifdef CONFIG_DEBUG_INPUT_WARN
#  define iwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define illwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define iwarn(x...)
#  define illwarn(x...)
#endif

#ifdef CONFIG_DEBUG_INPUT_INFO
#  define iinfo(format, ...)   info(format, ##__VA_ARGS__)
#  define illinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define iinfo(x...)
#  define illinfo(x...)
#endif

#ifdef CONFIG_DEBUG_ANALOG_ERROR
#  define aerr(format, ...)    err(format, ##__VA_ARGS__)
#  define allerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define aerr(x...)
#  define allerr(x...)
#endif

#ifdef CONFIG_DEBUG_ANALOG_WARN
#  define awarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define allwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define awarn(x...)
#  define allwarn(x...)
#endif

#ifdef CONFIG_DEBUG_ANALOG_INFO
#  define ainfo(format, ...)   info(format, ##__VA_ARGS__)
#  define allinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define ainfo(x...)
#  define allinfo(x...)
#endif

#ifdef CONFIG_DEBUG_CAN_ERROR
#  define canerr(format, ...)    err(format, ##__VA_ARGS__)
#  define canllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define canerr(x...)
#  define canllerr(x...)
#endif

#ifdef CONFIG_DEBUG_CAN_WARN
#  define canwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define canllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define canwarn(x...)
#  define canllwarn(x...)
#endif

#ifdef CONFIG_DEBUG_CAN_INFO
#  define caninfo(format, ...)   info(format, ##__VA_ARGS__)
#  define canllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define caninfo(x...)
#  define canllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_ERROR
#  define gerr(format, ...)    err(format, ##__VA_ARGS__)
#  define gllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define gerr(x...)
#  define gllerr(x...)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_WARN
#  define gwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define gllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define gwarn(x...)
#  define gllwarn(x...)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_INFO
#  define ginfo(format, ...)   info(format, ##__VA_ARGS__)
#  define gllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define ginfo(x...)
#  define gllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_BINFMT_ERROR
#  define berr(format, ...)    err(format, ##__VA_ARGS__)
#  define bllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define berr(x...)
#  define bllerr(x...)
#endif

#ifdef CONFIG_DEBUG_BINFMT_WARN
#  define bwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define bllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define bwarn(x...)
#  define bllwarn(x...)
#endif

#ifdef CONFIG_DEBUG_BINFMT_INFO
#  define binfo(format, ...)   info(format, ##__VA_ARGS__)
#  define bllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define binfo(x...)
#  define bllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_LIB_ERROR
#  define lerr(format, ...)    err(format, ##__VA_ARGS__)
#  define lllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define lerr(x...)
#  define lllerr(x...)
#endif

#ifdef CONFIG_DEBUG_LIB_WARN
#  define lwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define lllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define lwarn(x...)
#  define lllwarn(x...)
#endif

#ifdef CONFIG_DEBUG_LIB_INFO
#  define linfo(format, ...)   info(format, ##__VA_ARGS__)
#  define lllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define linfo(x...)
#  define lllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_AUDIO_ERROR
#  define auderr(format, ...)    err(format, ##__VA_ARGS__)
#  define audllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define auderr(x...)
#  define audllerr(x...)
#endif

#ifdef CONFIG_DEBUG_AUDIO_WARN
#  define audwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define audllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define audwarn(x...)
#  define audllwarn(x...)
#endif

#ifdef CONFIG_DEBUG_AUDIO_INFO
#  define audinfo(format, ...)   info(format, ##__VA_ARGS__)
#  define audllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define audinfo(x...)
#  define audllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_DMA_ERROR
#  define dmaerr(format, ...)    err(format, ##__VA_ARGS__)
#  define dmallerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define dmaerr(x...)
#  define dmallerr(x...)
#endif

#ifdef CONFIG_DEBUG_DMA_WARN
#  define dmawarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define dmallwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define dmawarn(x...)
#  define dmallwarn(x...)
#endif

#ifdef CONFIG_DEBUG_DMA_INFO
#  define dmainfo(format, ...)   info(format, ##__VA_ARGS__)
#  define dmallinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define dmainfo(x...)
#  define dmallinfo(x...)
#endif

#ifdef CONFIG_DEBUG_IRQ_ERROR
#  define irqerr(format, ...)    err(format, ##__VA_ARGS__)
#  define irqllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define irqerr(x...)
#  define irqllerr(x...)
#endif

#ifdef CONFIG_DEBUG_IRQ_WARN
#  define irqwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define irqllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define irqwarn(x...)
#  define irqllwarn(x...)
#endif

#ifdef CONFIG_DEBUG_IRQ_INFO
#  define irqinfo(format, ...)   info(format, ##__VA_ARGS__)
#  define irqllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define irqinfo(x...)
#  define irqllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_LCD_ERROR
#  define lcderr(format, ...)    err(format, ##__VA_ARGS__)
#  define lcdllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define lcderr(x...)
#  define lcdllerr(x...)
#endif

#ifdef CONFIG_DEBUG_LCD_WARN
#  define lcdwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define lcdllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define lcdwarn(x...)
#  define lcdllwarn(x...)
#endif

#ifdef CONFIG_DEBUG_LCD_INFO
#  define lcdinfo(format, ...)   info(format, ##__VA_ARGS__)
#  define lcdllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define lcdinfo(x...)
#  define lcdllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_LEDS_ERROR
#  define lederr(format, ...)    err(format, ##__VA_ARGS__)
#  define ledllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define lederr(x...)
#  define ledllerr(x...)
#endif

#ifdef CONFIG_DEBUG_LEDS_WARN
#  define ledwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define ledllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define ledwarn(x...)
#  define ledllwarn(x...)
#endif

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define ledinfo(format, ...)   info(format, ##__VA_ARGS__)
#  define ledllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define ledinfo(x...)
#  define ledllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_GPIO_ERROR
#  define gpioerr(format, ...)   err(format, ##__VA_ARGS__)
#  define gpiollerr(format, ...) llerr(format, ##__VA_ARGS__)
#else
#  define gpioerr(x...)
#  define gpiollerr(x...)
#endif

#ifdef CONFIG_DEBUG_GPIO_WARN
#  define gpiowarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define gpiollwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define gpiowarn(x...)
#  define gpiollwarn(x...)
#endif

#ifdef CONFIG_DEBUG_GPIO_INFO
#  define gpioinfo(format, ...)   info(format, ##__VA_ARGS__)
#  define gpiollinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define gpioinfo(x...)
#  define gpiollinfo(x...)
#endif

#ifdef CONFIG_DEBUG_SENSORS_ERROR
#  define snerr(format, ...)    err(format, ##__VA_ARGS__)
#  define snllerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define snerr(x...)
#  define snllerr(x...)
#endif

#ifdef CONFIG_DEBUG_SENSORS_WARN
#  define snwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define snllwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define snwarn(x...)
#  define snllwarn(x...)
#endif

#ifdef CONFIG_DEBUG_SENSORS_INFO
#  define sninfo(format, ...)   info(format, ##__VA_ARGS__)
#  define snllinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define sninfo(x...)
#  define snllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_SPI_ERROR
#  define spierr(format, ...)    err(format, ##__VA_ARGS__)
#  define spillerr(format, ...)  llerr(format, ##__VA_ARGS__)
#else
#  define spierr(x...)
#  define spillerr(x...)
#endif

#ifdef CONFIG_DEBUG_SPI_WARN
#  define spiwarn(format, ...)   warn(format, ##__VA_ARGS__)
#  define spillwarn(format, ...) llwarn(format, ##__VA_ARGS__)
#else
#  define spiwarn(x...)
#  define spillwarn(x...)
#endif

#ifdef CONFIG_DEBUG_SPI_INFO
#  define spiinfo(format, ...)   info(format, ##__VA_ARGS__)
#  define spillinfo(format, ...) llinfo(format, ##__VA_ARGS__)
#else
#  define spiinfo(x...)
#  define spillinfo(x...)
#endif

#else /* CONFIG_CPP_HAVE_VARARGS */

/* Variadic macros NOT supported */

#ifndef CONFIG_ARCH_LOWPUTC
#  define alert       (void)
# endif

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

#ifdef CONFIG_DEBUG_MM_ERROR
#  define merr        err
#  define mllerr      llerr
#else
#  define merr        (void)
#  define mllerr      (void)
#endif

#ifdef CONFIG_DEBUG_MM_WARN
#  define mwarn       warn
#  define mllwarn     llwarn
#else
#  define mwarn       (void)
#  define mllwarn     (void)
#endif

#ifdef CONFIG_DEBUG_MM_INFO
#  define minfo       info
#  define mllinfo     llinfo
#else
#  define minfo       (void)
#  define mllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_SCHED_ERROR
#  define serr        err
#  define sllerr      llerr
#else
#  define serr        (void)
#  define sllerr      (void)
#endif

#ifdef CONFIG_DEBUG_SCHED_WARN
#  define swarn       warn
#  define sllwarn     llwarn
#else
#  define swarn       (void)
#  define sllwarn     (void)
#endif

#ifdef CONFIG_DEBUG_SCHED_INFO
#  define sinfo       info
#  define sllinfo     llinfo
#else
#  define sinfo       (void)
#  define sllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_PAGING_ERROR
#  define pgerr       err
#  define pgllerr     llerr
#else
#  define pgerr       (void)
#  define pgllerr     (void)
#endif

#ifdef CONFIG_DEBUG_PAGING_WARN
#  define pgwarn      warn
#  define pgllwarn    llwarn
#else
#  define pgwarn      (void)
#  define pgllwarn    (void)
#endif

#ifdef CONFIG_DEBUG_PAGING_INFO
#  define pginfo      info
#  define pgllinfo    llinfo
#else
#  define pginfo      (void)
#  define pgllinfo    (void)
#endif

#ifdef CONFIG_DEBUG_NET_ERROR
#  define nerr        err
#  define nllerr      llerr
#else
#  define nerr        (void)
#  define nllerr      (void)
#endif

#ifdef CONFIG_DEBUG_NET_WARN
#  define nwarn       warn
#  define nllwarn     llwarn
#else
#  define nwarn       (void)
#  define nllwarn     (void)
#endif

#ifdef CONFIG_DEBUG_NET_INFO
#  define ninfo       info
#  define nllinfo     llinfo
#else
#  define ninfo       (void)
#  define nllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_USB_ERROR
#  define uerr        err
#  define ullerr      llerr
#else
#  define uerr        (void)
#  define ullerr      (void)
#endif

#ifdef CONFIG_DEBUG_USB_WARN
#  define uwarn       warn
#  define ullwarn     llwarn
#else
#  define uwarn       (void)
#  define ullwarn     (void)
#endif

#ifdef CONFIG_DEBUG_USB_INFO
#  define uinfo       info
#  define ullinfo     llinfo
#else
#  define uinfo       (void)
#  define ullinfo     (void)
#endif

#ifdef CONFIG_DEBUG_FS_ERROR
#  define ferr        err
#  define fllerr      llerr
#else
#  define ferr        (void)
#  define fllerr      (void)
#endif

#ifdef CONFIG_DEBUG_FS_WARN
#  define fwarn       warn
#  define fllwarn     llwarn
#else
#  define fwarn       (void)
#  define fllwarn     (void)
#endif

#ifdef CONFIG_DEBUG_FS_INFO
#  define finfo       info
#  define fllinfo     llinfo
#else
#  define finfo       (void)
#  define fllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_ERROR
#  define crypterr    err
#  define cryptllerr  llerr
#else
#  define crypterr    (void)
#  define cryptllerr  (void)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_WARN
#  define cryptwarn   warn
#  define cryptllwarn llwarn
#else
#  define cryptwarn   (void)
#  define cryptllwarn (void)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_INFO
#  define cryptinfo   info
#  define cryptllinfo llinfo
#else
#  define cryptinfo(x...)
#  define cryptllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_INPUT_ERROR
#  define ierr        err
#  define illerr      llerr
#else
#  define ierr        (void)
#  define illerr      (void)
#endif

#ifdef CONFIG_DEBUG_INPUT_WARN
#  define iwarn       warn
#  define illwarn     llwarn
#else
#  define iwarn       (void)
#  define illwarn     (void)
#endif

#ifdef CONFIG_DEBUG_INPUT_INFO
#  define iinfo       info
#  define illinfo     llinfo
#else
#  define iinfo       (void)
#  define illinfo     (void)
#endif

#ifdef CONFIG_DEBUG_ANALOG_ERROR
#  define aerr        err
#  define allerr      llerr
#else
#  define aerr        (void)
#  define allerr      (void)
#endif

#ifdef CONFIG_DEBUG_ANALOG_WARN
#  define awarn       warn
#  define allwarn     llwarn
#else
#  define awarn       (void)
#  define allwarn     (void)
#endif

#ifdef CONFIG_DEBUG_ANALOG_INFO
#  define ainfo       info
#  define allinfo     llinfo
#else
#  define ainfo       (void)
#  define allinfo     (void)
#endif

#ifdef CONFIG_DEBUG_CAN_ERROR
#  define canerr        err
#  define canllerr      llerr
#else
#  define canerr        (void)
#  define canllerr      (void)
#endif

#ifdef CONFIG_DEBUG_CAN_WARN
#  define canwarn       warn
#  define canllwarn     llwarn
#else
#  define canwarn       (void)
#  define canllwarn     (void)
#endif

#ifdef CONFIG_DEBUG_CAN_INFO
#  define caninfo       info
#  define canllinfo     llinfo
#else
#  define caninfo       (void)
#  define canllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_ERROR
#  define gerr        err
#  define gllerr      llerr
#else
#  define gerr        (void)
#  define gllerr      (void)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_WARN
#  define gwarn       warn
#  define gllwarn     llwarn
#else
#  define gwarn       (void)
#  define gllwarn     (void)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_INFO
#  define ginfo       info
#  define gllinfo     llinfo
#else
#  define ginfo       (void)
#  define gllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_BINFMT_ERROR
#  define berr        err
#  define bllerr      llerr
#else
#  define berr        (void)
#  define bllerr      (void)
#endif

#ifdef CONFIG_DEBUG_BINFMT_WARN
#  define bwarn       warn
#  define bllwarn     llwarn
#else
#  define bwarn       (void)
#  define bllwarn     (void)
#endif

#ifdef CONFIG_DEBUG_BINFMT_INFO
#  define binfo       info
#  define bllinfo     llinfo
#else
#  define binfo       (void)
#  define bllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_LIB_ERROR
#  define lerr        err
#  define lllerr      llerr
#else
#  define lerr        (void)
#  define lllerr      (void)
#endif

#ifdef CONFIG_DEBUG_LIB_WARN
#  define lwarn       warn
#  define lllwarn     llwarn
#else
#  define lwarn       (void)
#  define lllwarn     (void)
#endif

#ifdef CONFIG_DEBUG_LIB_INFO
#  define linfo       info
#  define lllinfo     llinfo
#else
#  define linfo       (void)
#  define lllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_AUDIO_ERROR
#  define auderr      err
#  define audllerr    llerr
#else
#  define auderr      (void)
#  define audllerr    (void)
#endif

#ifdef CONFIG_DEBUG_AUDIO_WARN
#  define audwarn     warn
#  define audllwarn   llwarn
#else
#  define audwarn     (void)
#  define audllwarn   (void)
#endif

#ifdef CONFIG_DEBUG_AUDIO_INFO
#  define audinfo     info
#  define audllinfo   llinfo
#else
#  define audinfo     (void)
#  define audllinfo   (void)
#endif

#ifdef CONFIG_DEBUG_DMA_ERROR
#  define dmaerr      err
#  define dmallerr    llerr
#else
#  define dmaerr      (void)
#  define dmallerr    (void)
#endif

#ifdef CONFIG_DEBUG_DMA_WARN
#  define dmawarn     warn
#  define dmallwarn   llwarn
#else
#  define dmawarn     (void)
#  define dmallwarn   (void)
#endif

#ifdef CONFIG_DEBUG_DMA_INFO
#  define dmainfo     info
#  define dmallinfo   llinfo
#else
#  define dmainfo     (void)
#  define dmallinfo   (void)
#endif

#ifdef CONFIG_DEBUG_IRQ_ERROR
#  define irqerr      err
#  define irqllerr    llerr
#else
#  define irqerr      (void)
#  define irqllerr    (void)
#endif

#ifdef CONFIG_DEBUG_IRQ_WARN
#  define irqwarn     warn
#  define irqllwarn   llwarn
#else
#  define irqwarn     (void)
#  define irqllwarn   (void)
#endif

#ifdef CONFIG_DEBUG_IRQ_INFO
#  define irqinfo     info
#  define irqllinfo   llinfo
#else
#  define irqinfo     (void)
#  define irqllinfo   (void)
#endif

#ifdef CONFIG_DEBUG_LCD_ERROR
#  define lcderr      err
#  define lcdllerr    llerr
#else
#  define lcderr      (void)
#  define lcdllerr    (void)
#endif

#ifdef CONFIG_DEBUG_LCD_WARN
#  define lcdwarn     warn
#  define lcdllwarn   llwarn
#else
#  define lcdwarn     (void)
#  define lcdllwarn   (void)
#endif

#ifdef CONFIG_DEBUG_LCD_INFO
#  define lcdinfo     info
#  define lcdllinfo   llinfo
#else
#  define lcdinfo     (void)
#  define lcdllinfo   (void)
#endif

#ifdef CONFIG_DEBUG_LEDS_ERROR
#  define lederr      err
#  define ledllerr    llerr
#else
#  define lederr      (void)
#  define ledllerr    (void)
#endif

#ifdef CONFIG_DEBUG_LEDS_WARN
#  define ledwarn     warn
#  define ledllwarn   llwarn
#else
#  define ledwarn     (void)
#  define ledllwarn   (void)
#endif

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define ledinfo     info
#  define ledllinfo   llinfo
#else
#  define ledinfo     (void)
#  define ledllinfo   (void)
#endif

#ifdef CONFIG_DEBUG_GPIO_ERROR
#  define gpioerr     err
#  define gpiollerr   llerr
#else
#  define gpioerr     (void)
#  define gpiollerr   (void)
#endif

#ifdef CONFIG_DEBUG_GPIO_WARN
#  define gpiowarn    warn
#  define gpiollwarn  llwarn
#else
#  define gpiowarn    (void)
#  define gpiollwarn  (void)
#endif

#ifdef CONFIG_DEBUG_GPIO_INFO
#  define gpioinfo    info
#  define gpiollinfo  llinfo
#else
#  define gpioinfo    (void)
#  define gpiollinfo  (void)
#endif

#ifdef CONFIG_DEBUG_SENSORS_ERROR
#  define snerr       err
#  define snllerr     llerr
#else
#  define snerr       (void)
#  define snllerr     (void)
#endif

#ifdef CONFIG_DEBUG_SENSORS_WARN
#  define snwarn      warn
#  define snllwarn    llwarn
#else
#  define snwarn      (void)
#  define snllwarn    (void)
#endif

#ifdef CONFIG_DEBUG_SENSORS_INFO
#  define sninfo      info
#  define snllinfo    llinfo
#else
#  define sninfo      (void)
#  define snllinfo    (void)
#endif

#ifdef CONFIG_DEBUG_SPI_ERROR
#  define spierr      err
#  define spillerr    llerr
#else
#  define spierr      (void)
#  define spillerr    (void)
#endif

#ifdef CONFIG_DEBUG_SPI_WARN
#  define spiwarn     warn
#  define spillwarn   llwarn
#else
#  define spiwarn     (void)
#  define spillwarn   (void)
#endif

#ifdef CONFIG_DEBUG_SPI_INFO
#  define spiinfo     info
#  define spillinfo   llinfo
#else
#  define spiinfo     (void)
#  define spillinfo   (void)
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

#ifdef CONFIG_DEBUG_ANALOG
#  define aerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define ainfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define aerrdumpbuffer(m,b,n)
#  define ainfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_CAN
#  define canerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define caninfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define canerrdumpbuffer(m,b,n)
#  define caninfodumpbuffer(m,b,n)
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

#ifdef CONFIG_DEBUG_DMA
#  define dmaerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define dmainfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define dmaerrdumpbuffer(m,b,n)
#  define dmainfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_IRQ
#  define irqerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define irqinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define irqerrdumpbuffer(m,b,n)
#  define irqinfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_LCD
#  define lcderrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define lcdinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define lcderrdumpbuffer(m,b,n)
#  define lcdinfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_LEDS
#  define lederrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define ledinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define lederrdumpbuffer(m,b,n)
#  define ledinfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_GPIO
#  define gpioerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define gpioinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define gpioerrdumpbuffer(m,b,n)
#  define gpioinfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_SENSORS
#  define snerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define sninfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define snerrdumpbuffer(m,b,n)
#  define sninfodumpbuffer(m,b,n)
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
#ifndef CONFIG_ARCH_LOWPUTC
int alert(const char *format, ...);
#endif

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
