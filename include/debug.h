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
 *    missing (i.e., _info()), then it is common.  The common _info() macro
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
 * [a-z]llerr() -- Identical to [a-z]llinfo() except that it also requires that
 *    CONFIG_DEBUG_ERROR be defined. This is intended for important error-related
 *    information that you probably not want to suppress during normal debug
 *    general debugging.
 *
 * _alert() - is a special, high-priority, unconditional version that is really
 *    intended only for crash error reporting.
 */

#ifdef CONFIG_HAVE_FUNCTIONNAME
#  define EXTRA_FMT "%s: "
#  define EXTRA_ARG ,__FUNCTION__
#else
#  define EXTRA_FMT
#  define EXTRA_ARG
#endif

/* Debug macros will differ depending upon if the toolchain supports
 * macros with a variable number of arguments or not.
 */

#ifdef CONFIG_CPP_HAVE_VARARGS

/* C-99 style variadic macros are supported */

/* The actual logger function may be overridden in arch/debug.h if needed.
 * (Currently only if the pre-processor supports variadic macros)
 */

#ifndef __arch_syslog
#  define __arch_syslog syslog
#endif

#define _alert(format, ...) \
   __arch_syslog(LOG_EMERG, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)

#ifdef CONFIG_DEBUG_ERROR
#  define  _err(format, ...) \
   __arch_syslog(LOG_ERR, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)

# ifdef CONFIG_ARCH_LOWPUTC
#  define  _llerr(format, ...) \
   __arch_syslog(LOG_ERR, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
# else
#  define  _llerr(x...)
# endif
#else /* CONFIG_DEBUG_ERROR */
#  define  _err(x...)
#  define  _llerr(x...)
#endif

#ifdef CONFIG_DEBUG_WARN
#  define _warn(format, ...) \
   __arch_syslog(LOG_WARNING, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#else /* CONFIG_DEBUG_INFO */
#  define _warn(x...)
#endif /* CONFIG_DEBUG_INFO */

#ifdef CONFIG_DEBUG_INFO
#  define _info(format, ...) \
   __arch_syslog(LOG_INFO, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)

#  ifdef CONFIG_ARCH_LOWPUTC
#    define _llinfo(format, ...) \
     __arch_syslog(LOG_INFO, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#  else
#    define _llinfo(x...)
#  endif
#else /* CONFIG_DEBUG_INFO */
#  define _info(x...)
#  define _llinfo(x...)
#endif /* CONFIG_DEBUG_INFO */

/* Subsystem specific debug */

#ifdef CONFIG_DEBUG_MM_ERROR
#  define merr(format, ...)     _err(format, ##__VA_ARGS__)
#  define mllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define merr(x...)
#  define mllerr(x...)
#endif

#ifdef CONFIG_DEBUG_MM_WARN
#  define mwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define mwarn(x...)
#endif

#ifdef CONFIG_DEBUG_MM_INFO
#  define minfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define mllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define minfo(x...)
#  define mllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_SCHED_ERROR
#  define serr(format, ...)     _err(format, ##__VA_ARGS__)
#  define sllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define serr(x...)
#  define sllerr(x...)
#endif

#ifdef CONFIG_DEBUG_SCHED_WARN
#  define swarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define swarn(x...)
#endif

#ifdef CONFIG_DEBUG_SCHED_INFO
#  define sinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define sllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define sinfo(x...)
#  define sllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_SYSCALL_ERROR
#  define svcerr(format, ...)     _err(format, ##__VA_ARGS__)
#  define svcllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define svcerr(x...)
#  define svcllerr(x...)
#endif

#ifdef CONFIG_DEBUG_SYSCALL_WARN
#  define svcwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define svcwarn(x...)
#endif

#ifdef CONFIG_DEBUG_SYSCALL_INFO
#  define svcinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define svcllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define svcinfo(x...)
#  define svcllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_PAGING_ERROR
#  define pgerr(format, ...)     _err(format, ##__VA_ARGS__)
#  define pgllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define pgerr(x...)
#  define pgllerr(x...)
#endif

#ifdef CONFIG_DEBUG_PAGING_WARN
#  define pgwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define pgwarn(x...)
#endif

#ifdef CONFIG_DEBUG_PAGING_INFO
#  define pginfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define pgllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define pgerr(x...)
#  define pgllerr(x...)
#endif

#ifdef CONFIG_DEBUG_NET_ERROR
#  define nerr(format, ...)     _err(format, ##__VA_ARGS__)
#  define nllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define nerr(x...)
#  define nllerr(x...)
#endif

#ifdef CONFIG_DEBUG_NET_WARN
#  define nwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define nwarn(x...)
#endif

#ifdef CONFIG_DEBUG_NET_INFO
#  define ninfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define nllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define ninfo(x...)
#  define nllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_FS_ERROR
#  define ferr(format, ...)     _err(format, ##__VA_ARGS__)
#  define fllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define ferr(x...)
#  define fllerr(x...)
#endif

#ifdef CONFIG_DEBUG_FS_WARN
#  define fwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define fwarn(x...)
#endif

#ifdef CONFIG_DEBUG_FS_INFO
#  define finfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define fllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define finfo(x...)
#  define fllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_ERROR
#  define crypterr(format, ...)     _err(format, ##__VA_ARGS__)
#  define cryptllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define crypterr(x...)
#  define cryptllerr(x...)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_WARN
#  define cryptwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define cryptwarn(x...)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_INFO
#  define cryptinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define cryptllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define cryptinfo(x...)
#  define cryptllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_INPUT_ERROR
#  define ierr(format, ...)     _err(format, ##__VA_ARGS__)
#  define illerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define ierr(x...)
#  define illerr(x...)
#endif

#ifdef CONFIG_DEBUG_INPUT_WARN
#  define iwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define iwarn(x...)
#endif

#ifdef CONFIG_DEBUG_INPUT_INFO
#  define iinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define illinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define iinfo(x...)
#  define illinfo(x...)
#endif

#ifdef CONFIG_DEBUG_ANALOG_ERROR
#  define aerr(format, ...)     _err(format, ##__VA_ARGS__)
#  define allerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define aerr(x...)
#  define allerr(x...)
#endif

#ifdef CONFIG_DEBUG_ANALOG_WARN
#  define awarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define awarn(x...)
#endif

#ifdef CONFIG_DEBUG_ANALOG_INFO
#  define ainfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define allinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define ainfo(x...)
#  define allinfo(x...)
#endif

#ifdef CONFIG_DEBUG_CAN_ERROR
#  define canerr(format, ...)     _err(format, ##__VA_ARGS__)
#  define canllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define canerr(x...)
#  define canllerr(x...)
#endif

#ifdef CONFIG_DEBUG_CAN_WARN
#  define canwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define canwarn(x...)
#endif

#ifdef CONFIG_DEBUG_CAN_INFO
#  define caninfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define canllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define caninfo(x...)
#  define canllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_ERROR
#  define gerr(format, ...)     _err(format, ##__VA_ARGS__)
#  define gllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define gerr(x...)
#  define gllerr(x...)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_WARN
#  define gwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define gwarn(x...)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_INFO
#  define ginfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define gllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define ginfo(x...)
#  define gllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_BINFMT_ERROR
#  define berr(format, ...)     _err(format, ##__VA_ARGS__)
#  define bllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define berr(x...)
#  define bllerr(x...)
#endif

#ifdef CONFIG_DEBUG_BINFMT_WARN
#  define bwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define bwarn(x...)
#endif

#ifdef CONFIG_DEBUG_BINFMT_INFO
#  define binfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define bllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define binfo(x...)
#  define bllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_LIB_ERROR
#  define lerr(format, ...)     _err(format, ##__VA_ARGS__)
#  define lllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define lerr(x...)
#  define lllerr(x...)
#endif

#ifdef CONFIG_DEBUG_LIB_WARN
#  define lwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define lwarn(x...)
#endif

#ifdef CONFIG_DEBUG_LIB_INFO
#  define linfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define lllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define linfo(x...)
#  define lllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_AUDIO_ERROR
#  define auderr(format, ...)     _err(format, ##__VA_ARGS__)
#  define audllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define auderr(x...)
#  define audllerr(x...)
#endif

#ifdef CONFIG_DEBUG_AUDIO_WARN
#  define audwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define audwarn(x...)
#endif

#ifdef CONFIG_DEBUG_AUDIO_INFO
#  define audinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define audllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define audinfo(x...)
#  define audllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_DMA_ERROR
#  define dmaerr(format, ...)     _err(format, ##__VA_ARGS__)
#  define dmallerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define dmaerr(x...)
#  define dmallerr(x...)
#endif

#ifdef CONFIG_DEBUG_DMA_WARN
#  define dmawarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define dmawarn(x...)
#endif

#ifdef CONFIG_DEBUG_DMA_INFO
#  define dmainfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define dmallinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define dmainfo(x...)
#  define dmallinfo(x...)
#endif

#ifdef CONFIG_DEBUG_IRQ_ERROR
#  define irqerr(format, ...)     _err(format, ##__VA_ARGS__)
#  define irqllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define irqerr(x...)
#  define irqllerr(x...)
#endif

#ifdef CONFIG_DEBUG_IRQ_WARN
#  define irqwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define irqwarn(x...)
#endif

#ifdef CONFIG_DEBUG_IRQ_INFO
#  define irqinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define irqllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define irqinfo(x...)
#  define irqllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_LCD_ERROR
#  define lcderr(format, ...)     _err(format, ##__VA_ARGS__)
#  define lcdllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define lcderr(x...)
#  define lcdllerr(x...)
#endif

#ifdef CONFIG_DEBUG_LCD_WARN
#  define lcdwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define lcdwarn(x...)
#endif

#ifdef CONFIG_DEBUG_LCD_INFO
#  define lcdinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define lcdllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define lcdinfo(x...)
#  define lcdllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_LEDS_ERROR
#  define lederr(format, ...)     _err(format, ##__VA_ARGS__)
#  define ledllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define lederr(x...)
#  define ledllerr(x...)
#endif

#ifdef CONFIG_DEBUG_LEDS_WARN
#  define ledwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define ledwarn(x...)
#endif

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define ledinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define ledllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define ledinfo(x...)
#  define ledllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_GPIO_ERROR
#  define gpioerr(format, ...)    _err(format, ##__VA_ARGS__)
#  define gpiollerr(format, ...)  _llerr(format, ##__VA_ARGS__)
#else
#  define gpioerr(x...)
#  define gpiollerr(x...)
#endif

#ifdef CONFIG_DEBUG_GPIO_WARN
#  define gpiowarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define gpiowarn(x...)
#endif

#ifdef CONFIG_DEBUG_GPIO_INFO
#  define gpioinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define gpiollinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define gpioinfo(x...)
#  define gpiollinfo(x...)
#endif

#ifdef CONFIG_DEBUG_I2C_ERROR
#  define i2cerr(format, ...)    _err(format, ##__VA_ARGS__)
#  define i2cllerr(format, ...)  _llerr(format, ##__VA_ARGS__)
#else
#  define i2cerr(x...)
#  define i2cllerr(x...)
#endif

#ifdef CONFIG_DEBUG_I2C_WARN
#  define i2cwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define i2cwarn(x...)
#endif

#ifdef CONFIG_DEBUG_I2C_INFO
#  define i2cinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define i2cllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define i2cinfo(x...)
#  define i2cllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_I2S_ERROR
#  define i2serr(format, ...)    _err(format, ##__VA_ARGS__)
#  define i2sllerr(format, ...)  _llerr(format, ##__VA_ARGS__)
#else
#  define i2serr(x...)
#  define i2sllerr(x...)
#endif

#ifdef CONFIG_DEBUG_I2S_WARN
#  define i2swarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define i2swarn(x...)
#endif

#ifdef CONFIG_DEBUG_I2S_INFO
#  define i2sinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define i2sllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define i2sinfo(x...)
#  define i2sllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_PWM_ERROR
#  define pwmerr(format, ...)    _err(format, ##__VA_ARGS__)
#  define pwmllerr(format, ...)  _llerr(format, ##__VA_ARGS__)
#else
#  define pwmerr(x...)
#  define pwmllerr(x...)
#endif

#ifdef CONFIG_DEBUG_PWM_WARN
#  define pwmwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define pwmwarn(x...)
#endif

#ifdef CONFIG_DEBUG_PWM_INFO
#  define pwminfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define pwmllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define pwminfo(x...)
#  define pwmllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_RTC_ERROR
#  define rtcerr(format, ...)    _err(format, ##__VA_ARGS__)
#  define rtcllerr(format, ...)  _llerr(format, ##__VA_ARGS__)
#else
#  define rtcerr(x...)
#  define rtcllerr(x...)
#endif

#ifdef CONFIG_DEBUG_RTC_WARN
#  define rtcwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define rtcwarn(x...)
#endif

#ifdef CONFIG_DEBUG_RTC_INFO
#  define rtcinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define rtcllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define rtcinfo(x...)
#  define rtcllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_MEMCARD_ERROR
#  define mcerr(format, ...)    _err(format, ##__VA_ARGS__)
#  define mcllerr(format, ...)  _llerr(format, ##__VA_ARGS__)
#else
#  define mcerr(x...)
#  define mcllerr(x...)
#endif

#ifdef CONFIG_DEBUG_MEMCARD_WARN
#  define mcwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define mcwarn(x...)
#endif

#ifdef CONFIG_DEBUG_MEMCARD_INFO
#  define mcinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define mcllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define mcinfo(x...)
#  define mcllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_SENSORS_ERROR
#  define snerr(format, ...)     _err(format, ##__VA_ARGS__)
#  define snllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define snerr(x...)
#  define snllerr(x...)
#endif

#ifdef CONFIG_DEBUG_SENSORS_WARN
#  define snwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define snwarn(x...)
#endif

#ifdef CONFIG_DEBUG_SENSORS_INFO
#  define sninfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define snllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define sninfo(x...)
#  define snllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_SPI_ERROR
#  define spierr(format, ...)     _err(format, ##__VA_ARGS__)
#  define spillerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define spierr(x...)
#  define spillerr(x...)
#endif

#ifdef CONFIG_DEBUG_SPI_WARN
#  define spiwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define spiwarn(x...)
#endif

#ifdef CONFIG_DEBUG_SPI_INFO
#  define spiinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define spillinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define spiinfo(x...)
#  define spillinfo(x...)
#endif

#ifdef CONFIG_DEBUG_TIMER_ERROR
#  define tmrerr(format, ...)     _err(format, ##__VA_ARGS__)
#  define tmrllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define tmrerr(x...)
#  define tmrllerr(x...)
#endif

#ifdef CONFIG_DEBUG_TIMER_WARN
#  define tmrwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define tmrwarn(x...)
#endif

#ifdef CONFIG_DEBUG_TIMER_INFO
#  define tmrinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define tmrllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define tmrinfo(x...)
#  define tmrllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_USB_ERROR
#  define uerr(format, ...)     _err(format, ##__VA_ARGS__)
#  define ullerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define uerr(x...)
#  define ullerr(x...)
#endif

#ifdef CONFIG_DEBUG_USB_WARN
#  define uwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define uwarn(x...)
#endif

#ifdef CONFIG_DEBUG_USB_INFO
#  define uinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define ullinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define uinfo(x...)
#  define ullinfo(x...)
#endif

#ifdef CONFIG_DEBUG_WATCHDOG_ERROR
#  define wderr(format, ...)     _err(format, ##__VA_ARGS__)
#  define wdllerr(format, ...)   _llerr(format, ##__VA_ARGS__)
#else
#  define wderr(x...)
#  define wdllerr(x...)
#endif

#ifdef CONFIG_DEBUG_WATCHDOG_WARN
#  define wdwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define wdwarn(x...)
#endif

#ifdef CONFIG_DEBUG_WATCHDOG_INFO
#  define wdinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  define wdllinfo(format, ...) _llinfo(format, ##__VA_ARGS__)
#else
#  define wdinfo(x...)
#  define wdllinfo(x...)
#endif

#else /* CONFIG_CPP_HAVE_VARARGS */

/* Variadic macros NOT supported */

#ifndef CONFIG_ARCH_LOWPUTC
#  define _alert      (void)
# endif

#ifdef CONFIG_DEBUG_ERROR
#  ifndef CONFIG_ARCH_LOWPUTC
#    define _llerr    (void)
#  endif
#else
#  define _err        (void)
#  define _llerr      (void)
#endif

#ifndef CONFIG_DEBUG_WARN
#  define _warn       (void)
#endif

#ifdef CONFIG_DEBUG_INFO
#  ifndef CONFIG_ARCH_LOWPUTC
#    define _llinfo   (void)
#  endif
#else
#  define _info       (void)
#  define _llinfo     (void)
#endif

/* Subsystem specific debug */

#ifdef CONFIG_DEBUG_MM_ERROR
#  define merr         _err
#  define mllerr       _llerr
#else
#  define merr        (void)
#  define mllerr      (void)
#endif

#ifdef CONFIG_DEBUG_MM_WARN
#  define mwarn       _warn
#else
#  define mwarn       (void)
#endif

#ifdef CONFIG_DEBUG_MM_INFO
#  define minfo       _info
#  define mllinfo     _llinfo
#else
#  define minfo       (void)
#  define mllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_SCHED_ERROR
#  define serr         _err
#  define sllerr       _llerr
#else
#  define serr        (void)
#  define sllerr      (void)
#endif

#ifdef CONFIG_DEBUG_SCHED_WARN
#  define swarn       _warn
#else
#  define swarn       (void)
#endif

#ifdef CONFIG_DEBUG_SCHED_INFO
#  define sinfo       _info
#  define sllinfo     _llinfo
#else
#  define sinfo       (void)
#  define sllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_SYSCALL_ERROR
#  define svcerr       _err
#  define svcllerr     _llerr
#else
#  define svcerr      (void)
#  define svcllerr    (void)
#endif

#ifdef CONFIG_DEBUG_SYSCALL_WARN
#  define svcwarn     _warn
#else
#  define svcwarn     (void)
#endif

#ifdef CONFIG_DEBUG_SYSCALL_INFO
#  define svcinfo     _info
#  define svcllinfo   _llinfo
#else
#  define svcinfo     (void)
#  define svcllinfo   (void)
#endif

#ifdef CONFIG_DEBUG_PAGING_ERROR
#  define pgerr        _err
#  define pgllerr      _llerr
#else
#  define pgerr       (void)
#  define pgllerr     (void)
#endif

#ifdef CONFIG_DEBUG_PAGING_WARN
#  define pgwarn      _warn
#else
#  define pgwarn      (void)
#endif

#ifdef CONFIG_DEBUG_PAGING_INFO
#  define pginfo      _info
#  define pgllinfo    _llinfo
#else
#  define pginfo      (void)
#  define pgllinfo    (void)
#endif

#ifdef CONFIG_DEBUG_NET_ERROR
#  define nerr         _err
#  define nllerr       _llerr
#else
#  define nerr        (void)
#  define nllerr      (void)
#endif

#ifdef CONFIG_DEBUG_NET_WARN
#  define nwarn       _warn
#else
#  define nwarn       (void)
#endif

#ifdef CONFIG_DEBUG_NET_INFO
#  define ninfo       _info
#  define nllinfo     _llinfo
#else
#  define ninfo       (void)
#  define nllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_FS_ERROR
#  define ferr         _err
#  define fllerr       _llerr
#else
#  define ferr        (void)
#  define fllerr      (void)
#endif

#ifdef CONFIG_DEBUG_FS_WARN
#  define fwarn       _warn
#else
#  define fwarn       (void)
#endif

#ifdef CONFIG_DEBUG_FS_INFO
#  define finfo       _info
#  define fllinfo     _llinfo
#else
#  define finfo       (void)
#  define fllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_ERROR
#  define crypterr     _err
#  define cryptllerr   _llerr
#else
#  define crypterr    (void)
#  define cryptllerr  (void)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_WARN
#  define cryptwarn   _warn
#else
#  define cryptwarn   (void)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_INFO
#  define cryptinfo   _info
#  define cryptllinfo _llinfo
#else
#  define cryptinfo(x...)
#  define cryptllinfo(x...)
#endif

#ifdef CONFIG_DEBUG_INPUT_ERROR
#  define ierr         _err
#  define illerr       _llerr
#else
#  define ierr        (void)
#  define illerr      (void)
#endif

#ifdef CONFIG_DEBUG_INPUT_WARN
#  define iwarn       _warn
#else
#  define iwarn       (void)
#endif

#ifdef CONFIG_DEBUG_INPUT_INFO
#  define iinfo       _info
#  define illinfo     _llinfo
#else
#  define iinfo       (void)
#  define illinfo     (void)
#endif

#ifdef CONFIG_DEBUG_ANALOG_ERROR
#  define aerr         _err
#  define allerr       _llerr
#else
#  define aerr        (void)
#  define allerr      (void)
#endif

#ifdef CONFIG_DEBUG_ANALOG_WARN
#  define awarn       _warn
#else
#  define awarn       (void)
#endif

#ifdef CONFIG_DEBUG_ANALOG_INFO
#  define ainfo       _info
#  define allinfo     _llinfo
#else
#  define ainfo       (void)
#  define allinfo     (void)
#endif

#ifdef CONFIG_DEBUG_CAN_ERROR
#  define canerr       _err
#  define canllerr     _llerr
#else
#  define canerr      (void)
#  define canllerr    (void)
#endif

#ifdef CONFIG_DEBUG_CAN_WARN
#  define canwarn     _warn
#else
#  define canwarn     (void)
#endif

#ifdef CONFIG_DEBUG_CAN_INFO
#  define caninfo     _info
#  define canllinfo   _llinfo
#else
#  define caninfo     (void)
#  define canllinfo   (void)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_ERROR
#  define gerr         _err
#  define gllerr       _llerr
#else
#  define gerr        (void)
#  define gllerr      (void)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_WARN
#  define gwarn       _warn
#else
#  define gwarn       (void)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_INFO
#  define ginfo       _info
#  define gllinfo     _llinfo
#else
#  define ginfo       (void)
#  define gllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_BINFMT_ERROR
#  define berr         _err
#  define bllerr       _llerr
#else
#  define berr        (void)
#  define bllerr      (void)
#endif

#ifdef CONFIG_DEBUG_BINFMT_WARN
#  define bwarn       _warn
#else
#  define bwarn       (void)
#endif

#ifdef CONFIG_DEBUG_BINFMT_INFO
#  define binfo       _info
#  define bllinfo     _llinfo
#else
#  define binfo       (void)
#  define bllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_LIB_ERROR
#  define lerr         _err
#  define lllerr       _llerr
#else
#  define lerr        (void)
#  define lllerr      (void)
#endif

#ifdef CONFIG_DEBUG_LIB_WARN
#  define lwarn       _warn
#else
#  define lwarn       (void)
#endif

#ifdef CONFIG_DEBUG_LIB_INFO
#  define linfo       _info
#  define lllinfo     _llinfo
#else
#  define linfo       (void)
#  define lllinfo     (void)
#endif

#ifdef CONFIG_DEBUG_AUDIO_ERROR
#  define auderr       _err
#  define audllerr     _llerr
#else
#  define auderr      (void)
#  define audllerr    (void)
#endif

#ifdef CONFIG_DEBUG_AUDIO_WARN
#  define audwarn     _warn
#else
#  define audwarn     (void)
#endif

#ifdef CONFIG_DEBUG_AUDIO_INFO
#  define audinfo     _info
#  define audllinfo   _llinfo
#else
#  define audinfo     (void)
#  define audllinfo   (void)
#endif

#ifdef CONFIG_DEBUG_DMA_ERROR
#  define dmaerr       _err
#  define dmallerr     _llerr
#else
#  define dmaerr      (void)
#  define dmallerr    (void)
#endif

#ifdef CONFIG_DEBUG_DMA_WARN
#  define dmawarn     _warn
#else
#  define dmawarn     (void)
#endif

#ifdef CONFIG_DEBUG_DMA_INFO
#  define dmainfo     _info
#  define dmallinfo   _llinfo
#else
#  define dmainfo     (void)
#  define dmallinfo   (void)
#endif

#ifdef CONFIG_DEBUG_IRQ_ERROR
#  define irqerr       _err
#  define irqllerr     _llerr
#else
#  define irqerr      (void)
#  define irqllerr    (void)
#endif

#ifdef CONFIG_DEBUG_IRQ_WARN
#  define irqwarn     _warn
#else
#  define irqwarn     (void)
#endif

#ifdef CONFIG_DEBUG_IRQ_INFO
#  define irqinfo     _info
#  define irqllinfo   _llinfo
#else
#  define irqinfo     (void)
#  define irqllinfo   (void)
#endif

#ifdef CONFIG_DEBUG_LCD_ERROR
#  define lcderr       _err
#  define lcdllerr     _llerr
#else
#  define lcderr      (void)
#  define lcdllerr    (void)
#endif

#ifdef CONFIG_DEBUG_LCD_WARN
#  define lcdwarn     _warn
#else
#  define lcdwarn     (void)
#endif

#ifdef CONFIG_DEBUG_LCD_INFO
#  define lcdinfo     _info
#  define lcdllinfo   _llinfo
#else
#  define lcdinfo     (void)
#  define lcdllinfo   (void)
#endif

#ifdef CONFIG_DEBUG_LEDS_ERROR
#  define lederr       _err
#  define ledllerr     _llerr
#else
#  define lederr      (void)
#  define ledllerr    (void)
#endif

#ifdef CONFIG_DEBUG_LEDS_WARN
#  define ledwarn     _warn
#else
#  define ledwarn     (void)
#endif

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define ledinfo     _info
#  define ledllinfo   _llinfo
#else
#  define ledinfo     (void)
#  define ledllinfo   (void)
#endif

#ifdef CONFIG_DEBUG_GPIO_ERROR
#  define gpioerr      _err
#  define gpiollerr    _llerr
#else
#  define gpioerr     (void)
#  define gpiollerr   (void)
#endif

#ifdef CONFIG_DEBUG_GPIO_WARN
#  define gpiowarn    _warn
#else
#  define gpiowarn    (void)
#endif

#ifdef CONFIG_DEBUG_GPIO_INFO
#  define gpioinfo    _info
#  define gpiollinfo  _llinfo
#else
#  define gpioinfo    (void)
#  define gpiollinfo  (void)
#endif

#ifdef CONFIG_DEBUG_I2C_ERROR
#  define i2cerr       _err
#  define i2cllerr     _llerr
#else
#  define i2cerr      (void)
#  define i2cllerr    (void)
#endif

#ifdef CONFIG_DEBUG_I2C_WARN
#  define i2cwarn     _warn
#else
#  define i2cwarn     (void)
#endif

#ifdef CONFIG_DEBUG_I2C_INFO
#  define i2cinfo     _info
#  define i2cllinfo   _llinfo
#else
#  define i2cinfo     (void)
#  define i2cllinfo   (void)
#endif

#ifdef CONFIG_DEBUG_I2S_ERROR
#  define i2serr       _err
#  define i2sllerr     _llerr
#else
#  define i2serr      (void)
#  define i2sllerr    (void)
#endif

#ifdef CONFIG_DEBUG_I2S_WARN
#  define i2swarn     _warn
#else
#  define i2swarn     (void)
#endif

#ifdef CONFIG_DEBUG_I2S_INFO
#  define i2sinfo     _info
#  define i2sllinfo   _llinfo
#else
#  define i2sinfo     (void)
#  define i2sllinfo   (void)
#endif

#ifdef CONFIG_DEBUG_PWM_ERROR
#  define pwmerr       _err
#  define pwmllerr     _llerr
#else
#  define pwmerr      (void)
#  define pwmllerr    (void)
#endif

#ifdef CONFIG_DEBUG_PWM_WARN
#  define pwmwarn     _warn
#else
#  define pwmwarn     (void)
#endif

#ifdef CONFIG_DEBUG_PWM_INFO
#  define pwminfo     _info
#  define pwmllinfo   _llinfo
#else
#  define pwminfo     (void)
#  define pwmllinfo   (void)
#endif

#ifdef CONFIG_DEBUG_RTC_ERROR
#  define rtcerr       _err
#  define rtcllerr     _llerr
#else
#  define rtcerr      (void)
#  define rtcllerr    (void)
#endif

#ifdef CONFIG_DEBUG_RTC_WARN
#  define rtcwarn     _warn
#else
#  define rtcwarn     (void)
#endif

#ifdef CONFIG_DEBUG_RTC_INFO
#  define rtcinfo     _info
#  define rtcllinfo   _llinfo
#else
#  define rtcinfo     (void)
#  define rtcllinfo   (void)
#endif

#ifdef CONFIG_DEBUG_MEMCARD_ERROR
#  define mcerr        _err
#  define mcllerr      _llerr
#else
#  define mcerr       (void)
#  define mcllerr     (void)
#endif

#ifdef CONFIG_DEBUG_MEMCARD_WARN
#  define mcwarn      _warn
#else
#  define mcwarn      (void)
#endif

#ifdef CONFIG_DEBUG_MEMCARD_INFO
#  define mcinfo      _info
#  define mcllinfo    _llinfo
#else
#  define mcinfo      (void)
#  define mcllinfo    (void)
#endif

#ifdef CONFIG_DEBUG_SENSORS_ERROR
#  define snerr        _err
#  define snllerr      _llerr
#else
#  define snerr       (void)
#  define snllerr     (void)
#endif

#ifdef CONFIG_DEBUG_SENSORS_WARN
#  define snwarn      _warn
#else
#  define snwarn      (void)
#endif

#ifdef CONFIG_DEBUG_SENSORS_INFO
#  define sninfo      _info
#  define snllinfo    _llinfo
#else
#  define sninfo      (void)
#  define snllinfo    (void)
#endif

#ifdef CONFIG_DEBUG_SPI_ERROR
#  define spierr       _err
#  define spillerr     _llerr
#else
#  define spierr      (void)
#  define spillerr    (void)
#endif

#ifdef CONFIG_DEBUG_SPI_WARN
#  define spiwarn     _warn
#else
#  define spiwarn     (void)
#endif

#ifdef CONFIG_DEBUG_SPI_INFO
#  define spiinfo     _info
#  define spillinfo   _llinfo
#else
#  define spiinfo     (void)
#  define spillinfo   (void)
#endif

#ifdef CONFIG_DEBUG_TIMER_ERROR
#  define tmrerr       _err
#  define tmrllerr     _llerr
#else
#  define tmrerr      (void)
#  define tmrllerr    (void)
#endif

#ifdef CONFIG_DEBUG_TIMER_WARN
#  define tmrwarn     _warn
#else
#  define tmrwarn     (void)
#endif

#ifdef CONFIG_DEBUG_TIMER_INFO
#  define tmrinfo     _info
#  define tmrllinfo   _llinfo
#else
#  define tmrinfo     (void)
#  define tmrllinfo   (void)
#endif

#ifdef CONFIG_DEBUG_USB_ERROR
#  define uerr         _err
#  define ullerr       _llerr
#else
#  define uerr        (void)
#  define ullerr      (void)
#endif

#ifdef CONFIG_DEBUG_USB_WARN
#  define uwarn       _warn
#else
#  define uwarn       (void)
#endif

#ifdef CONFIG_DEBUG_USB_INFO
#  define uinfo       _info
#  define ullinfo     _llinfo
#else
#  define uinfo       (void)
#  define ullinfo     (void)
#endif

#ifdef CONFIG_DEBUG_WATCHDOG_ERROR
#  define wderr        _err
#  define wdllerr      _llerr
#else
#  define wderr       (void)
#  define wdllerr     (void)
#endif

#ifdef CONFIG_DEBUG_WATCHDOG_WARN
#  define wdwarn      _warn
#else
#  define wdwarn      (void)
#endif

#ifdef CONFIG_DEBUG_WATCHDOG_INFO
#  define wdinfo      _info
#  define wdllinfo    _llinfo
#else
#  define wdinfo      (void)
#  define wdllinfo    (void)
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

#ifdef CONFIG_DEBUG_SYSCALL
#  define svcerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define svcinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define svcerrdumpbuffer(m,b,n)
#  define svcinfodumpbuffer(m,b,n)
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

#ifdef CONFIG_DEBUG_I2C
#  define i2cerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define i2cinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define i2cerrdumpbuffer(m,b,n)
#  define i2cinfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_I2S
#  define i2serrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define i2sinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define i2serrdumpbuffer(m,b,n)
#  define i2sinfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_PWM
#  define pwmerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define pwminfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define pwmerrdumpbuffer(m,b,n)
#  define pwminfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_RTC
#  define rtcerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define rtcinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define rtcerrdumpbuffer(m,b,n)
#  define rtcinfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_MEMCARD
#  define mcerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define mcinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define mcerrdumpbuffer(m,b,n)
#  define mcinfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_SENSORS
#  define snerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define sninfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define snerrdumpbuffer(m,b,n)
#  define sninfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_SPI
#  define spierrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define spiinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define spierrdumpbuffer(m,b,n)
#  define spiinfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_TIMER
#  define tmrerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define tmrinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define tmrerrdumpbuffer(m,b,n)
#  define tmrinfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_USB
#  define uerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define uinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define uerrdumpbuffer(m,b,n)
#  define uinfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_WATCHDOG
#  define wderrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define wdinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define wderrdumpbuffer(m,b,n)
#  define wdinfodumpbuffer(m,b,n)
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
int _alert(const char *format, ...);
#endif

#ifdef CONFIG_DEBUG_ERROR
int  _err(const char *format, ...);

# ifdef CONFIG_ARCH_LOWPUTC
int  _llerr(const char *format, ...);
# endif
#endif /* CONFIG_DEBUG_ERROR */

#ifdef CONFIG_DEBUG_WARN
int _warn(const char *format, ...);
#endif

#ifdef CONFIG_DEBUG_INFO
int _info(const char *format, ...);

# ifdef CONFIG_ARCH_LOWPUTC
int _llinfo(const char *format, ...);
# endif
#endif /* CONFIG_DEBUG_INFO */
#endif /* CONFIG_CPP_HAVE_VARARGS */

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_DEBUG_H */
