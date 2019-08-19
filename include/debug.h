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

#ifdef CONFIG_DEBUG_ALERT
#  define _alert(format, ...) \
   __arch_syslog(LOG_EMERG, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#else /* CONFIG_DEBUG_ERROR */
#  define  _alert(x...)
#endif

#ifdef CONFIG_DEBUG_ERROR
#  define  _err(format, ...) \
   __arch_syslog(LOG_ERR, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#else
#  define  _err(x...)
#endif

#ifdef CONFIG_DEBUG_WARN
#  define _warn(format, ...) \
   __arch_syslog(LOG_WARNING, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#else
#  define _warn(x...)
#endif

#ifdef CONFIG_DEBUG_INFO
#  define _info(format, ...) \
   __arch_syslog(LOG_INFO, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#else
#  define _info(x...)
#endif

/* Subsystem specific debug */

#ifdef CONFIG_DEBUG_MM_ERROR
#  define merr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define merr(x...)
#endif

#ifdef CONFIG_DEBUG_MM_WARN
#  define mwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define mwarn(x...)
#endif

#ifdef CONFIG_DEBUG_MM_INFO
#  define minfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define minfo(x...)
#endif

#ifdef CONFIG_DEBUG_SCHED_ERROR
#  define serr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define serr(x...)
#endif

#ifdef CONFIG_DEBUG_SCHED_WARN
#  define swarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define swarn(x...)
#endif

#ifdef CONFIG_DEBUG_SCHED_INFO
#  define sinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define sinfo(x...)
#endif

#ifdef CONFIG_DEBUG_SYSCALL_ERROR
#  define svcerr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define svcerr(x...)
#endif

#ifdef CONFIG_DEBUG_SYSCALL_WARN
#  define svcwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define svcwarn(x...)
#endif

#ifdef CONFIG_DEBUG_SYSCALL_INFO
#  define svcinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define svcinfo(x...)
#endif

#ifdef CONFIG_DEBUG_PAGING_ERROR
#  define pgerr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define pgerr(x...)
#endif

#ifdef CONFIG_DEBUG_PAGING_WARN
#  define pgwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define pgwarn(x...)
#endif

#ifdef CONFIG_DEBUG_PAGING_INFO
#  define pginfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define pginfo(x...)
#endif

#ifdef CONFIG_DEBUG_NET_ERROR
#  define nerr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define nerr(x...)
#endif

#ifdef CONFIG_DEBUG_NET_WARN
#  define nwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define nwarn(x...)
#endif

#ifdef CONFIG_DEBUG_NET_INFO
#  define ninfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define ninfo(x...)
#endif

#ifdef CONFIG_DEBUG_POWER_ERROR
#  define pwrerr(format, ...)    _err(format, ##__VA_ARGS__)
#else
#  define pwrerr(x...)
#endif

#ifdef CONFIG_DEBUG_POWER_WARN
#  define pwrwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define pwrwarn(x...)
#endif

#ifdef CONFIG_DEBUG_POWER_INFO
#  define pwrinfo(format, ...)  _info(format, ##__VA_ARGS__)
#else
#  define pwrinfo(x...)
#endif

#ifdef CONFIG_DEBUG_WIRELESS_ERROR
#  define wlerr(format, ...)    _err(format, ##__VA_ARGS__)
#else
#  define wlerr(x...)
#endif

#ifdef CONFIG_DEBUG_WIRELESS_WARN
#  define wlwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define wlwarn(x...)
#endif

#ifdef CONFIG_DEBUG_WIRELESS_INFO
#  define wlinfo(format, ...)  _info(format, ##__VA_ARGS__)
#else
#  define wlinfo(x...)
#endif

#ifdef CONFIG_DEBUG_FS_ERROR
#  define ferr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define ferr(x...)
#endif

#ifdef CONFIG_DEBUG_FS_WARN
#  define fwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define fwarn(x...)
#endif

#ifdef CONFIG_DEBUG_FS_INFO
#  define finfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define finfo(x...)
#endif

#ifdef CONFIG_DEBUG_CONTACTLESS_ERROR
#  define ctlserr(format, ...)  _err(format, ##__VA_ARGS__)
#else
#  define ctlserr(x...)
#endif

#ifdef CONFIG_DEBUG_CONTACTLESS_WARN
#  define ctlswarn(format, ...) _warn(format, ##__VA_ARGS__)
#else
#  define ctlswarn(x...)
#endif

#ifdef CONFIG_DEBUG_CONTACTLESS_INFO
#  define ctlsinfo(format, ...) _info(format, ##__VA_ARGS__)
#else
#  define ctlsinfo(x...)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_ERROR
#  define crypterr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define crypterr(x...)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_WARN
#  define cryptwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define cryptwarn(x...)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_INFO
#  define cryptinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define cryptinfo(x...)
#endif

#ifdef CONFIG_DEBUG_INPUT_ERROR
#  define ierr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define ierr(x...)
#endif

#ifdef CONFIG_DEBUG_INPUT_WARN
#  define iwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define iwarn(x...)
#endif

#ifdef CONFIG_DEBUG_INPUT_INFO
#  define iinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define iinfo(x...)
#endif

#ifdef CONFIG_DEBUG_ANALOG_ERROR
#  define aerr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define aerr(x...)
#endif

#ifdef CONFIG_DEBUG_ANALOG_WARN
#  define awarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define awarn(x...)
#endif

#ifdef CONFIG_DEBUG_ANALOG_INFO
#  define ainfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define ainfo(x...)
#endif

#ifdef CONFIG_DEBUG_CAN_ERROR
#  define canerr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define canerr(x...)
#endif

#ifdef CONFIG_DEBUG_CAN_WARN
#  define canwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define canwarn(x...)
#endif

#ifdef CONFIG_DEBUG_CAN_INFO
#  define caninfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define caninfo(x...)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_ERROR
#  define gerr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define gerr(x...)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_WARN
#  define gwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define gwarn(x...)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_INFO
#  define ginfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define ginfo(x...)
#endif

#ifdef CONFIG_DEBUG_BINFMT_ERROR
#  define berr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define berr(x...)
#endif

#ifdef CONFIG_DEBUG_BINFMT_WARN
#  define bwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define bwarn(x...)
#endif

#ifdef CONFIG_DEBUG_BINFMT_INFO
#  define binfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define binfo(x...)
#endif

#ifdef CONFIG_DEBUG_LIB_ERROR
#  define lerr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define lerr(x...)
#endif

#ifdef CONFIG_DEBUG_LIB_WARN
#  define lwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define lwarn(x...)
#endif

#ifdef CONFIG_DEBUG_LIB_INFO
#  define linfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define linfo(x...)
#endif

#ifdef CONFIG_DEBUG_AUDIO_ERROR
#  define auderr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define auderr(x...)
#endif

#ifdef CONFIG_DEBUG_AUDIO_WARN
#  define audwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define audwarn(x...)
#endif

#ifdef CONFIG_DEBUG_AUDIO_INFO
#  define audinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define audinfo(x...)
#endif

#ifdef CONFIG_DEBUG_DMA_ERROR
#  define dmaerr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define dmaerr(x...)
#endif

#ifdef CONFIG_DEBUG_DMA_WARN
#  define dmawarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define dmawarn(x...)
#endif

#ifdef CONFIG_DEBUG_DMA_INFO
#  define dmainfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define dmainfo(x...)
#endif

#ifdef CONFIG_DEBUG_IRQ_ERROR
#  define irqerr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define irqerr(x...)
#endif

#ifdef CONFIG_DEBUG_IRQ_WARN
#  define irqwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define irqwarn(x...)
#endif

#ifdef CONFIG_DEBUG_IRQ_INFO
#  define irqinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define irqinfo(x...)
#endif

#ifdef CONFIG_DEBUG_LCD_ERROR
#  define lcderr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define lcderr(x...)
#endif

#ifdef CONFIG_DEBUG_LCD_WARN
#  define lcdwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define lcdwarn(x...)
#endif

#ifdef CONFIG_DEBUG_LCD_INFO
#  define lcdinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define lcdinfo(x...)
#endif

#ifdef CONFIG_DEBUG_LEDS_ERROR
#  define lederr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define lederr(x...)
#endif

#ifdef CONFIG_DEBUG_LEDS_WARN
#  define ledwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define ledwarn(x...)
#endif

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define ledinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define ledinfo(x...)
#endif

#ifdef CONFIG_DEBUG_GPIO_ERROR
#  define gpioerr(format, ...)    _err(format, ##__VA_ARGS__)
#else
#  define gpioerr(x...)
#endif

#ifdef CONFIG_DEBUG_GPIO_WARN
#  define gpiowarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define gpiowarn(x...)
#endif

#ifdef CONFIG_DEBUG_GPIO_INFO
#  define gpioinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define gpioinfo(x...)
#endif

#ifdef CONFIG_DEBUG_I2C_ERROR
#  define i2cerr(format, ...)    _err(format, ##__VA_ARGS__)
#else
#  define i2cerr(x...)
#endif

#ifdef CONFIG_DEBUG_I2C_WARN
#  define i2cwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define i2cwarn(x...)
#endif

#ifdef CONFIG_DEBUG_I2C_INFO
#  define i2cinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define i2cinfo(x...)
#endif

#ifdef CONFIG_DEBUG_I2S_ERROR
#  define i2serr(format, ...)    _err(format, ##__VA_ARGS__)
#else
#  define i2serr(x...)
#endif

#ifdef CONFIG_DEBUG_I2S_WARN
#  define i2swarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define i2swarn(x...)
#endif

#ifdef CONFIG_DEBUG_I2S_INFO
#  define i2sinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define i2sinfo(x...)
#endif

#ifdef CONFIG_DEBUG_PWM_ERROR
#  define pwmerr(format, ...)    _err(format, ##__VA_ARGS__)
#else
#  define pwmerr(x...)
#endif

#ifdef CONFIG_DEBUG_PWM_WARN
#  define pwmwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define pwmwarn(x...)
#endif

#ifdef CONFIG_DEBUG_PWM_INFO
#  define pwminfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define pwminfo(x...)
#endif

#ifdef CONFIG_DEBUG_RTC_ERROR
#  define rtcerr(format, ...)    _err(format, ##__VA_ARGS__)
#else
#  define rtcerr(x...)
#endif

#ifdef CONFIG_DEBUG_RTC_WARN
#  define rtcwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define rtcwarn(x...)
#endif

#ifdef CONFIG_DEBUG_RTC_INFO
#  define rtcinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define rtcinfo(x...)
#endif

#ifdef CONFIG_DEBUG_MEMCARD_ERROR
#  define mcerr(format, ...)    _err(format, ##__VA_ARGS__)
#else
#  define mcerr(x...)
#endif

#ifdef CONFIG_DEBUG_MEMCARD_WARN
#  define mcwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define mcwarn(x...)
#endif

#ifdef CONFIG_DEBUG_MEMCARD_INFO
#  define mcinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define mcinfo(x...)
#endif

#ifdef CONFIG_DEBUG_SENSORS_ERROR
#  define snerr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define snerr(x...)
#endif

#ifdef CONFIG_DEBUG_SENSORS_WARN
#  define snwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define snwarn(x...)
#endif

#ifdef CONFIG_DEBUG_SENSORS_INFO
#  define sninfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define sninfo(x...)
#endif

#ifdef CONFIG_DEBUG_SPI_ERROR
#  define spierr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define spierr(x...)
#endif

#ifdef CONFIG_DEBUG_SPI_WARN
#  define spiwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define spiwarn(x...)
#endif

#ifdef CONFIG_DEBUG_SPI_INFO
#  define spiinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define spiinfo(x...)
#endif

#ifdef CONFIG_DEBUG_TIMER_ERROR
#  define tmrerr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define tmrerr(x...)
#endif

#ifdef CONFIG_DEBUG_TIMER_WARN
#  define tmrwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define tmrwarn(x...)
#endif

#ifdef CONFIG_DEBUG_TIMER_INFO
#  define tmrinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define tmrinfo(x...)
#endif

#ifdef CONFIG_DEBUG_USB_ERROR
#  define uerr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define uerr(x...)
#endif

#ifdef CONFIG_DEBUG_USB_WARN
#  define uwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define uwarn(x...)
#endif

#ifdef CONFIG_DEBUG_USB_INFO
#  define uinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define uinfo(x...)
#endif

#ifdef CONFIG_DEBUG_WATCHDOG_ERROR
#  define wderr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define wderr(x...)
#endif

#ifdef CONFIG_DEBUG_WATCHDOG_WARN
#  define wdwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define wdwarn(x...)
#endif

#ifdef CONFIG_DEBUG_WATCHDOG_INFO
#  define wdinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define wdinfo(x...)
#endif

#else /* CONFIG_CPP_HAVE_VARARGS */

/* Variadic macros NOT supported */

#ifndef CONFIG_DEBUG_ALERT
#  define _alert      (void)
# endif

#ifndef CONFIG_DEBUG_ERROR
#  define _err        (void)
#endif

#ifndef CONFIG_DEBUG_WARN
#  define _warn       (void)
#endif

#ifndef CONFIG_DEBUG_INFO
#  define _info       (void)
#endif

/* Subsystem specific debug */

#ifdef CONFIG_DEBUG_MM_ERROR
#  define merr         _err
#else
#  define merr        (void)
#endif

#ifdef CONFIG_DEBUG_MM_WARN
#  define mwarn       _warn
#else
#  define mwarn       (void)
#endif

#ifdef CONFIG_DEBUG_MM_INFO
#  define minfo       _info
#else
#  define minfo       (void)
#endif

#ifdef CONFIG_DEBUG_SCHED_ERROR
#  define serr         _err
#else
#  define serr        (void)
#endif

#ifdef CONFIG_DEBUG_SCHED_WARN
#  define swarn       _warn
#else
#  define swarn       (void)
#endif

#ifdef CONFIG_DEBUG_SCHED_INFO
#  define sinfo       _info
#else
#  define sinfo       (void)
#endif

#ifdef CONFIG_DEBUG_SYSCALL_ERROR
#  define svcerr       _err
#else
#  define svcerr      (void)
#endif

#ifdef CONFIG_DEBUG_SYSCALL_WARN
#  define svcwarn     _warn
#else
#  define svcwarn     (void)
#endif

#ifdef CONFIG_DEBUG_SYSCALL_INFO
#  define svcinfo     _info
#else
#  define svcinfo     (void)
#endif

#ifdef CONFIG_DEBUG_PAGING_ERROR
#  define pgerr        _err
#else
#  define pgerr       (void)
#endif

#ifdef CONFIG_DEBUG_PAGING_WARN
#  define pgwarn      _warn
#else
#  define pgwarn      (void)
#endif

#ifdef CONFIG_DEBUG_PAGING_INFO
#  define pginfo      _info
#else
#  define pginfo      (void)
#endif

#ifdef CONFIG_DEBUG_NET_ERROR
#  define nerr         _err
#else
#  define nerr        (void)
#endif

#ifdef CONFIG_DEBUG_NET_WARN
#  define nwarn       _warn
#else
#  define nwarn       (void)
#endif

#ifdef CONFIG_DEBUG_NET_INFO
#  define ninfo       _info
#else
#  define ninfo       (void)
#endif

#ifdef CONFIG_DEBUG_POWER_ERROR
#  define pwrerr       _err
#else
#  define pwrerr       (void)
#endif

#ifdef CONFIG_DEBUG_POWER_WARN
#  define pwrwarn      _warn
#else
#  define pwrwarn      (void)
#endif

#ifdef CONFIG_DEBUG_POWER_INFO
#  define pwrinfo      _info
#else
#  define pwrinfo      (void)
#endif

#ifdef CONFIG_DEBUG_WIRELESS_ERROR
#  define wlerr       _err
#else
#  define wlerr       (void)
#endif

#ifdef CONFIG_DEBUG_WIRELESS_WARN
#  define wlwarn      _warn
#else
#  define wlwarn      (void)
#endif

#ifdef CONFIG_DEBUG_WIRELESS_INFO
#  define wlinfo      _info
#else
#  define wlinfo      (void)
#endif

#ifdef CONFIG_DEBUG_FS_ERROR
#  define ferr         _err
#else
#  define ferr        (void)
#endif

#ifdef CONFIG_DEBUG_FS_WARN
#  define fwarn       _warn
#else
#  define fwarn       (void)
#endif

#ifdef CONFIG_DEBUG_FS_INFO
#  define finfo       _info
#else
#  define finfo       (void)
#endif

#ifdef CONFIG_DEBUG_CONTACTLESS_ERROR
#  define ctlserr     _err
#else
#  define ctlserr     (void)
#endif

#ifdef CONFIG_DEBUG_CONTACTLESS_WARN
#  define ctlswarn    _warn
#else
#  define ctlswarn    (void)
#endif

#ifdef CONFIG_DEBUG_CONTACTLESS_INFO
#  define ctlsinfo    _info
#else
#  define ctlsinfo    (void)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_ERROR
#  define crypterr     _err
#else
#  define crypterr    (void)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_WARN
#  define cryptwarn   _warn
#else
#  define cryptwarn   (void)
#endif

#ifdef CONFIG_DEBUG_CRYPTO_INFO
#  define cryptinfo   _info
#else
#  define cryptinfo   (void)
#endif

#ifdef CONFIG_DEBUG_INPUT_ERROR
#  define ierr         _err
#else
#  define ierr        (void)
#endif

#ifdef CONFIG_DEBUG_INPUT_WARN
#  define iwarn       _warn
#else
#  define iwarn       (void)
#endif

#ifdef CONFIG_DEBUG_INPUT_INFO
#  define iinfo       _info
#else
#  define iinfo       (void)
#endif

#ifdef CONFIG_DEBUG_ANALOG_ERROR
#  define aerr         _err
#else
#  define aerr        (void)
#endif

#ifdef CONFIG_DEBUG_ANALOG_WARN
#  define awarn       _warn
#else
#  define awarn       (void)
#endif

#ifdef CONFIG_DEBUG_ANALOG_INFO
#  define ainfo       _info
#else
#  define ainfo       (void)
#endif

#ifdef CONFIG_DEBUG_CAN_ERROR
#  define canerr       _err
#else
#  define canerr      (void)
#endif

#ifdef CONFIG_DEBUG_CAN_WARN
#  define canwarn     _warn
#else
#  define canwarn     (void)
#endif

#ifdef CONFIG_DEBUG_CAN_INFO
#  define caninfo     _info
#else
#  define caninfo     (void)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_ERROR
#  define gerr         _err
#else
#  define gerr        (void)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_WARN
#  define gwarn       _warn
#else
#  define gwarn       (void)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_INFO
#  define ginfo       _info
#else
#  define ginfo       (void)
#endif

#ifdef CONFIG_DEBUG_BINFMT_ERROR
#  define berr         _err
#else
#  define berr        (void)
#endif

#ifdef CONFIG_DEBUG_BINFMT_WARN
#  define bwarn       _warn
#else
#  define bwarn       (void)
#endif

#ifdef CONFIG_DEBUG_BINFMT_INFO
#  define binfo       _info
#else
#  define binfo       (void)
#endif

#ifdef CONFIG_DEBUG_LIB_ERROR
#  define lerr         _err
#else
#  define lerr        (void)
#endif

#ifdef CONFIG_DEBUG_LIB_WARN
#  define lwarn       _warn
#else
#  define lwarn       (void)
#endif

#ifdef CONFIG_DEBUG_LIB_INFO
#  define linfo       _info
#else
#  define linfo       (void)
#endif

#ifdef CONFIG_DEBUG_AUDIO_ERROR
#  define auderr       _err
#else
#  define auderr      (void)
#endif

#ifdef CONFIG_DEBUG_AUDIO_WARN
#  define audwarn     _warn
#else
#  define audwarn     (void)
#endif

#ifdef CONFIG_DEBUG_AUDIO_INFO
#  define audinfo     _info
#else
#  define audinfo     (void)
#endif

#ifdef CONFIG_DEBUG_DMA_ERROR
#  define dmaerr       _err
#else
#  define dmaerr      (void)
#endif

#ifdef CONFIG_DEBUG_DMA_WARN
#  define dmawarn     _warn
#else
#  define dmawarn     (void)
#endif

#ifdef CONFIG_DEBUG_DMA_INFO
#  define dmainfo     _info
#else
#  define dmainfo     (void)
#endif

#ifdef CONFIG_DEBUG_IRQ_ERROR
#  define irqerr       _err
#else
#  define irqerr      (void)
#endif

#ifdef CONFIG_DEBUG_IRQ_WARN
#  define irqwarn     _warn
#else
#  define irqwarn     (void)
#endif

#ifdef CONFIG_DEBUG_IRQ_INFO
#  define irqinfo     _info
#else
#  define irqinfo     (void)
#endif

#ifdef CONFIG_DEBUG_LCD_ERROR
#  define lcderr       _err
#else
#  define lcderr      (void)
#endif

#ifdef CONFIG_DEBUG_LCD_WARN
#  define lcdwarn     _warn
#else
#  define lcdwarn     (void)
#endif

#ifdef CONFIG_DEBUG_LCD_INFO
#  define lcdinfo     _info
#else
#  define lcdinfo     (void)
#endif

#ifdef CONFIG_DEBUG_LEDS_ERROR
#  define lederr       _err
#else
#  define lederr      (void)
#endif

#ifdef CONFIG_DEBUG_LEDS_WARN
#  define ledwarn     _warn
#else
#  define ledwarn     (void)
#endif

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define ledinfo     _info
#else
#  define ledinfo     (void)
#endif

#ifdef CONFIG_DEBUG_GPIO_ERROR
#  define gpioerr      _err
#else
#  define gpioerr     (void)
#endif

#ifdef CONFIG_DEBUG_GPIO_WARN
#  define gpiowarn    _warn
#else
#  define gpiowarn    (void)
#endif

#ifdef CONFIG_DEBUG_GPIO_INFO
#  define gpioinfo    _info
#else
#  define gpioinfo    (void)
#endif

#ifdef CONFIG_DEBUG_I2C_ERROR
#  define i2cerr       _err
#else
#  define i2cerr      (void)
#endif

#ifdef CONFIG_DEBUG_I2C_WARN
#  define i2cwarn     _warn
#else
#  define i2cwarn     (void)
#endif

#ifdef CONFIG_DEBUG_I2C_INFO
#  define i2cinfo     _info
#else
#  define i2cinfo     (void)
#endif

#ifdef CONFIG_DEBUG_I2S_ERROR
#  define i2serr       _err
#else
#  define i2serr      (void)
#endif

#ifdef CONFIG_DEBUG_I2S_WARN
#  define i2swarn     _warn
#else
#  define i2swarn     (void)
#endif

#ifdef CONFIG_DEBUG_I2S_INFO
#  define i2sinfo     _info
#else
#  define i2sinfo     (void)
#endif

#ifdef CONFIG_DEBUG_PWM_ERROR
#  define pwmerr       _err
#else
#  define pwmerr      (void)
#endif

#ifdef CONFIG_DEBUG_PWM_WARN
#  define pwmwarn     _warn
#else
#  define pwmwarn     (void)
#endif

#ifdef CONFIG_DEBUG_PWM_INFO
#  define pwminfo     _info
#else
#  define pwminfo     (void)
#endif

#ifdef CONFIG_DEBUG_RTC_ERROR
#  define rtcerr       _err
#else
#  define rtcerr      (void)
#endif

#ifdef CONFIG_DEBUG_RTC_WARN
#  define rtcwarn     _warn
#else
#  define rtcwarn     (void)
#endif

#ifdef CONFIG_DEBUG_RTC_INFO
#  define rtcinfo     _info
#else
#  define rtcinfo     (void)
#endif

#ifdef CONFIG_DEBUG_MEMCARD_ERROR
#  define mcerr        _err
#else
#  define mcerr       (void)
#endif

#ifdef CONFIG_DEBUG_MEMCARD_WARN
#  define mcwarn      _warn
#else
#  define mcwarn      (void)
#endif

#ifdef CONFIG_DEBUG_MEMCARD_INFO
#  define mcinfo      _info
#else
#  define mcinfo      (void)
#endif

#ifdef CONFIG_DEBUG_SENSORS_ERROR
#  define snerr        _err
#else
#  define snerr       (void)
#endif

#ifdef CONFIG_DEBUG_SENSORS_WARN
#  define snwarn      _warn
#else
#  define snwarn      (void)
#endif

#ifdef CONFIG_DEBUG_SENSORS_INFO
#  define sninfo      _info
#else
#  define sninfo      (void)
#endif

#ifdef CONFIG_DEBUG_SPI_ERROR
#  define spierr       _err
#else
#  define spierr      (void)
#endif

#ifdef CONFIG_DEBUG_SPI_WARN
#  define spiwarn     _warn
#else
#  define spiwarn     (void)
#endif

#ifdef CONFIG_DEBUG_SPI_INFO
#  define spiinfo     _info
#else
#  define spiinfo     (void)
#endif

#ifdef CONFIG_DEBUG_TIMER_ERROR
#  define tmrerr       _err
#else
#  define tmrerr      (void)
#endif

#ifdef CONFIG_DEBUG_TIMER_WARN
#  define tmrwarn     _warn
#else
#  define tmrwarn     (void)
#endif

#ifdef CONFIG_DEBUG_TIMER_INFO
#  define tmrinfo     _info
#else
#  define tmrinfo     (void)
#endif

#ifdef CONFIG_DEBUG_USB_ERROR
#  define uerr         _err
#else
#  define uerr        (void)
#endif

#ifdef CONFIG_DEBUG_USB_WARN
#  define uwarn       _warn
#else
#  define uwarn       (void)
#endif

#ifdef CONFIG_DEBUG_USB_INFO
#  define uinfo       _info
#else
#  define uinfo       (void)
#endif

#ifdef CONFIG_DEBUG_WATCHDOG_ERROR
#  define wderr        _err
#else
#  define wderr       (void)
#endif

#ifdef CONFIG_DEBUG_WATCHDOG_WARN
#  define wdwarn      _warn
#else
#  define wdwarn      (void)
#endif

#ifdef CONFIG_DEBUG_WATCHDOG_INFO
#  define wdinfo      _info
#else
#  define wdinfo      (void)
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

#ifdef CONFIG_DEBUG_POWER
#  define pwrerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define pwrinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define pwrerrdumpbuffer(m,b,n)
#  define pwrinfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_WIRELESS
#  define wlerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define wlinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define wlerrdumpbuffer(m,b,n)
#  define wlinfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_FS
#  define ferrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define finfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define ferrdumpbuffer(m,b,n)
#  define finfodumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_CONTACTLESS
#  define ctlserrdumpbuffer(m,b,n) errdumpbuffer(m,b,n)
#  define ctlinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define ctlserrferrdumpbuffer(m,b,n)
#  define ctlinfodumpbuffer(m,b,n)
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
#ifdef CONFIG_DEBUG_ALERT
void _alert(const char *format, ...);
#endif

#ifdef CONFIG_DEBUG_ERROR
void _err(const char *format, ...);
#endif

#ifdef CONFIG_DEBUG_WARN
void _warn(const char *format, ...);
#endif

#ifdef CONFIG_DEBUG_INFO
void _info(const char *format, ...);
#endif
#endif /* CONFIG_CPP_HAVE_VARARGS */

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_DEBUG_H */
