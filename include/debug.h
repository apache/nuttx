/****************************************************************************
 * include/debug.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
#ifdef CONFIG_ARCH_CHIP_DEBUG_H
# include <arch/chip/debug.h>
#endif

#include <syslog.h>
#include <sys/uio.h>

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
 *    The first character of the macro name indicates the system
 *    (e.g., n=network, f=filesystm, etc.).  If the first character is
 *    missing (i.e., _info()), then it is common.  The common _info() macro
 *    is enabled simply with CONFIG_DEBUG_INFO.  Subsystem debug requires an
 *    additional configuration setting to enable it (e.g., CONFIG_DEBUG_NET
 *    for the network, CONFIG_DEBUG_FS for the file system, etc).
 *
 *    In general, error messages and output of importance use [a-z]err().
 *    [a-z]err() is implementation dependent but usually uses file
 *    descriptors. (that is a problem only because the interrupt task may
 *    have redirected stdout).  Therefore [a-z]err() should not be used in
 *    interrupt handlers.
 *
 * [a-z]warn() -- Identical to [a-z]info() except that it also requires that
 *    CONFIG_DEBUG_WARN be defined.  This is intended for important exception
 *    conditions that are potential errors (or perhaps real errors with non-
 *    fatal consequences).
 *
 * [a-z]err() -- Identical to [a-z]info() except that it also requires that
 *    CONFIG_DEBUG_ERROR be defined.  This is intended for important
 *    error-related information that you probably don't want to suppress
 *    during general debugging.
 *
 * _alert() - is a special, high-priority, unconditional version that is
 *    really intended only for crash error reporting.
 */

#if !defined(EXTRA_FMT) && !defined(EXTRA_ARG) && defined(CONFIG_HAVE_FUNCTIONNAME)
#  define EXTRA_FMT "%s: "
#  define EXTRA_ARG ,__FUNCTION__
#endif

#ifndef EXTRA_FMT
#  define EXTRA_FMT
#endif

#ifndef EXTRA_ARG
#  define EXTRA_ARG
#endif

/* Debug macros will differ depending upon if the toolchain supports
 * macros with a variable number of arguments or not.
 */

#ifdef CONFIG_CPP_HAVE_VARARGS
/* don't call syslog while performing the compiler's format check. */

#  define _none(format, ...) \
    do { if (0) syslog(LOG_ERR, format, ##__VA_ARGS__); } while (0)
#else
#  define _none       (void)
#endif

/* The actual logger function may be overridden in arch/debug.h if needed.
 * (Currently only if the pre-processor supports variadic macros)
 */

#ifndef __arch_syslog
#  define __arch_syslog syslog
#endif

#if !defined(CONFIG_DEBUG_ALERT)
#  define _alert      _none
#elif defined(CONFIG_CPP_HAVE_VARARGS)
#  define _alert(format, ...) \
   __arch_syslog(LOG_EMERG, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#endif

#if !defined(CONFIG_DEBUG_ERROR)
#  define _err        _none
#elif defined(CONFIG_CPP_HAVE_VARARGS)
#  define _err(format, ...) \
   __arch_syslog(LOG_ERR, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#endif

#if !defined(CONFIG_DEBUG_WARN)
#  define _warn       _none
#elif defined(CONFIG_CPP_HAVE_VARARGS)
#  define _warn(format, ...) \
   __arch_syslog(LOG_WARNING, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#endif

#if !defined(CONFIG_DEBUG_INFO)
#  define _info       _none
#elif defined(CONFIG_CPP_HAVE_VARARGS)
#  define _info(format, ...) \
   __arch_syslog(LOG_INFO, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#endif

/* Subsystem specific debug */

#ifdef CONFIG_DEBUG_MM_ERROR
#  define merr         _err
#else
#  define merr        _none
#endif

#ifdef CONFIG_DEBUG_MM_WARN
#  define mwarn       _warn
#else
#  define mwarn       _none
#endif

#ifdef CONFIG_DEBUG_MM_INFO
#  define minfo       _info
#else
#  define minfo       _none
#endif

#ifdef CONFIG_DEBUG_SCHED_ERROR
#  define serr         _err
#else
#  define serr        _none
#endif

#ifdef CONFIG_DEBUG_SCHED_WARN
#  define swarn       _warn
#else
#  define swarn       _none
#endif

#ifdef CONFIG_DEBUG_SCHED_INFO
#  define sinfo       _info
#else
#  define sinfo       _none
#endif

#ifdef CONFIG_DEBUG_SYSCALL_ERROR
#  define svcerr       _err
#else
#  define svcerr      _none
#endif

#ifdef CONFIG_DEBUG_SYSCALL_WARN
#  define svcwarn     _warn
#else
#  define svcwarn     _none
#endif

#ifdef CONFIG_DEBUG_SYSCALL_INFO
#  define svcinfo     _info
#else
#  define svcinfo     _none
#endif

#ifdef CONFIG_DEBUG_PAGING_ERROR
#  define pgerr        _err
#else
#  define pgerr       _none
#endif

#ifdef CONFIG_DEBUG_PAGING_WARN
#  define pgwarn      _warn
#else
#  define pgwarn      _none
#endif

#ifdef CONFIG_DEBUG_PAGING_INFO
#  define pginfo      _info
#else
#  define pginfo      _none
#endif

#ifdef CONFIG_DEBUG_NET_ERROR
#  define nerr         _err
#else
#  define nerr        _none
#endif

#ifdef CONFIG_DEBUG_NET_WARN
#  define nwarn       _warn
#else
#  define nwarn       _none
#endif

#ifdef CONFIG_DEBUG_NET_INFO
#  define ninfo       _info
#else
#  define ninfo       _none
#endif

#ifdef CONFIG_DEBUG_POWER_ERROR
#  define pwrerr       _err
#else
#  define pwrerr       _none
#endif

#ifdef CONFIG_DEBUG_POWER_WARN
#  define pwrwarn      _warn
#else
#  define pwrwarn      _none
#endif

#ifdef CONFIG_DEBUG_POWER_INFO
#  define pwrinfo      _info
#else
#  define pwrinfo      _none
#endif

#ifdef CONFIG_DEBUG_BATTERY_ERROR
#  define baterr       _err
#else
#  define baterr       _none
#endif

#ifdef CONFIG_DEBUG_BATTERY_WARN
#  define batwarn      _warn
#else
#  define batwarn      _none
#endif

#ifdef CONFIG_DEBUG_BATTERY_INFO
#  define batinfo      _info
#else
#  define batinfo      _none
#endif

#ifdef CONFIG_DEBUG_WIRELESS_ERROR
#  define wlerr       _err
#else
#  define wlerr       _none
#endif

#ifdef CONFIG_DEBUG_WIRELESS_WARN
#  define wlwarn      _warn
#else
#  define wlwarn      _none
#endif

#ifdef CONFIG_DEBUG_WIRELESS_INFO
#  define wlinfo      _info
#else
#  define wlinfo      _none
#endif

#ifdef CONFIG_DEBUG_FS_ERROR
#  define ferr         _err
#else
#  define ferr        _none
#endif

#ifdef CONFIG_DEBUG_FS_WARN
#  define fwarn       _warn
#else
#  define fwarn       _none
#endif

#ifdef CONFIG_DEBUG_FS_INFO
#  define finfo       _info
#else
#  define finfo       _none
#endif

#ifdef CONFIG_DEBUG_CONTACTLESS_ERROR
#  define ctlserr     _err
#else
#  define ctlserr     _none
#endif

#ifdef CONFIG_DEBUG_CONTACTLESS_WARN
#  define ctlswarn    _warn
#else
#  define ctlswarn    _none
#endif

#ifdef CONFIG_DEBUG_CONTACTLESS_INFO
#  define ctlsinfo    _info
#else
#  define ctlsinfo    _none
#endif

#ifdef CONFIG_DEBUG_CRYPTO_ERROR
#  define crypterr     _err
#else
#  define crypterr    _none
#endif

#ifdef CONFIG_DEBUG_CRYPTO_WARN
#  define cryptwarn   _warn
#else
#  define cryptwarn   _none
#endif

#ifdef CONFIG_DEBUG_CRYPTO_INFO
#  define cryptinfo   _info
#else
#  define cryptinfo   _none
#endif

#ifdef CONFIG_DEBUG_INPUT_ERROR
#  define ierr         _err
#else
#  define ierr        _none
#endif

#ifdef CONFIG_DEBUG_INPUT_WARN
#  define iwarn       _warn
#else
#  define iwarn       _none
#endif

#ifdef CONFIG_DEBUG_INPUT_INFO
#  define iinfo       _info
#else
#  define iinfo       _none
#endif

#ifdef CONFIG_DEBUG_ANALOG_ERROR
#  define aerr         _err
#else
#  define aerr        _none
#endif

#ifdef CONFIG_DEBUG_ANALOG_WARN
#  define awarn       _warn
#else
#  define awarn       _none
#endif

#ifdef CONFIG_DEBUG_ANALOG_INFO
#  define ainfo       _info
#else
#  define ainfo       _none
#endif

#ifdef CONFIG_DEBUG_CAN_ERROR
#  define canerr       _err
#else
#  define canerr      _none
#endif

#ifdef CONFIG_DEBUG_CAN_WARN
#  define canwarn     _warn
#else
#  define canwarn     _none
#endif

#ifdef CONFIG_DEBUG_CAN_INFO
#  define caninfo     _info
#else
#  define caninfo     _none
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_ERROR
#  define gerr         _err
#else
#  define gerr        _none
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_WARN
#  define gwarn       _warn
#else
#  define gwarn       _none
#endif

#ifdef CONFIG_DEBUG_GRAPHICS_INFO
#  define ginfo       _info
#else
#  define ginfo       _none
#endif

#ifdef CONFIG_DEBUG_BINFMT_ERROR
#  define berr         _err
#else
#  define berr        _none
#endif

#ifdef CONFIG_DEBUG_BINFMT_WARN
#  define bwarn       _warn
#else
#  define bwarn       _none
#endif

#ifdef CONFIG_DEBUG_BINFMT_INFO
#  define binfo       _info
#else
#  define binfo       _none
#endif

#ifdef CONFIG_DEBUG_LIB_ERROR
#  define lerr         _err
#else
#  define lerr        _none
#endif

#ifdef CONFIG_DEBUG_LIB_WARN
#  define lwarn       _warn
#else
#  define lwarn       _none
#endif

#ifdef CONFIG_DEBUG_LIB_INFO
#  define linfo       _info
#else
#  define linfo       _none
#endif

#ifdef CONFIG_DEBUG_AUDIO_ERROR
#  define auderr       _err
#else
#  define auderr      _none
#endif

#ifdef CONFIG_DEBUG_AUDIO_WARN
#  define audwarn     _warn
#else
#  define audwarn     _none
#endif

#ifdef CONFIG_DEBUG_AUDIO_INFO
#  define audinfo     _info
#else
#  define audinfo     _none
#endif

#ifdef CONFIG_DEBUG_DMA_ERROR
#  define dmaerr       _err
#else
#  define dmaerr      _none
#endif

#ifdef CONFIG_DEBUG_DMA_WARN
#  define dmawarn     _warn
#else
#  define dmawarn     _none
#endif

#ifdef CONFIG_DEBUG_DMA_INFO
#  define dmainfo     _info
#else
#  define dmainfo     _none
#endif

#ifdef CONFIG_DEBUG_IRQ_ERROR
#  define irqerr       _err
#else
#  define irqerr      _none
#endif

#ifdef CONFIG_DEBUG_IRQ_WARN
#  define irqwarn     _warn
#else
#  define irqwarn     _none
#endif

#ifdef CONFIG_DEBUG_IRQ_INFO
#  define irqinfo     _info
#else
#  define irqinfo     _none
#endif

#ifdef CONFIG_DEBUG_LCD_ERROR
#  define lcderr       _err
#else
#  define lcderr      _none
#endif

#ifdef CONFIG_DEBUG_LCD_WARN
#  define lcdwarn     _warn
#else
#  define lcdwarn     _none
#endif

#ifdef CONFIG_DEBUG_LCD_INFO
#  define lcdinfo     _info
#else
#  define lcdinfo     _none
#endif

#ifdef CONFIG_DEBUG_LEDS_ERROR
#  define lederr       _err
#else
#  define lederr      _none
#endif

#ifdef CONFIG_DEBUG_LEDS_WARN
#  define ledwarn     _warn
#else
#  define ledwarn     _none
#endif

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define ledinfo     _info
#else
#  define ledinfo     _none
#endif

#ifdef CONFIG_DEBUG_GPIO_ERROR
#  define gpioerr      _err
#else
#  define gpioerr     _none
#endif

#ifdef CONFIG_DEBUG_GPIO_WARN
#  define gpiowarn    _warn
#else
#  define gpiowarn    _none
#endif

#ifdef CONFIG_DEBUG_GPIO_INFO
#  define gpioinfo    _info
#else
#  define gpioinfo    _none
#endif

#ifdef CONFIG_DEBUG_I2C_ERROR
#  define i2cerr       _err
#else
#  define i2cerr      _none
#endif

#ifdef CONFIG_DEBUG_I2C_WARN
#  define i2cwarn     _warn
#else
#  define i2cwarn     _none
#endif

#ifdef CONFIG_DEBUG_I2C_INFO
#  define i2cinfo     _info
#else
#  define i2cinfo     _none
#endif

#ifdef CONFIG_DEBUG_I2S_ERROR
#  define i2serr       _err
#else
#  define i2serr      _none
#endif

#ifdef CONFIG_DEBUG_I2S_WARN
#  define i2swarn     _warn
#else
#  define i2swarn     _none
#endif

#ifdef CONFIG_DEBUG_I2S_INFO
#  define i2sinfo     _info
#else
#  define i2sinfo     _none
#endif

#ifdef CONFIG_DEBUG_PWM_ERROR
#  define pwmerr       _err
#else
#  define pwmerr      _none
#endif

#ifdef CONFIG_DEBUG_PWM_WARN
#  define pwmwarn     _warn
#else
#  define pwmwarn     _none
#endif

#ifdef CONFIG_DEBUG_PWM_INFO
#  define pwminfo     _info
#else
#  define pwminfo     _none
#endif

#ifdef CONFIG_DEBUG_RC_ERROR
#  define rcerr        _err
#else
#  define rcerr       _none
#endif

#ifdef CONFIG_DEBUG_RC_WARN
#  define rcwarn      _warn
#else
#  define rcwarn      _none
#endif

#ifdef CONFIG_DEBUG_RC_INFO
#  define rcinfo      _info
#else
#  define rcinfo      _none
#endif

#ifdef CONFIG_DEBUG_RTC_ERROR
#  define rtcerr       _err
#else
#  define rtcerr      _none
#endif

#ifdef CONFIG_DEBUG_RTC_WARN
#  define rtcwarn     _warn
#else
#  define rtcwarn     _none
#endif

#ifdef CONFIG_DEBUG_RTC_INFO
#  define rtcinfo     _info
#else
#  define rtcinfo     _none
#endif

#ifdef CONFIG_DEBUG_MEMCARD_ERROR
#  define mcerr        _err
#else
#  define mcerr       _none
#endif

#ifdef CONFIG_DEBUG_MEMCARD_WARN
#  define mcwarn      _warn
#else
#  define mcwarn      _none
#endif

#ifdef CONFIG_DEBUG_MEMCARD_INFO
#  define mcinfo      _info
#else
#  define mcinfo      _none
#endif

#ifdef CONFIG_DEBUG_SENSORS_ERROR
#  define snerr        _err
#else
#  define snerr       _none
#endif

#ifdef CONFIG_DEBUG_SENSORS_WARN
#  define snwarn      _warn
#else
#  define snwarn      _none
#endif

#ifdef CONFIG_DEBUG_SENSORS_INFO
#  define sninfo      _info
#else
#  define sninfo      _none
#endif

#ifdef CONFIG_DEBUG_SPI_ERROR
#  define spierr       _err
#else
#  define spierr      _none
#endif

#ifdef CONFIG_DEBUG_SPI_WARN
#  define spiwarn     _warn
#else
#  define spiwarn     _none
#endif

#ifdef CONFIG_DEBUG_SPI_INFO
#  define spiinfo     _info
#else
#  define spiinfo     _none
#endif

#ifdef CONFIG_DEBUG_TIMER_ERROR
#  define tmrerr       _err
#else
#  define tmrerr      _none
#endif

#ifdef CONFIG_DEBUG_TIMER_WARN
#  define tmrwarn     _warn
#else
#  define tmrwarn     _none
#endif

#ifdef CONFIG_DEBUG_TIMER_INFO
#  define tmrinfo     _info
#else
#  define tmrinfo     _none
#endif

#ifdef CONFIG_DEBUG_USB_ERROR
#  define uerr         _err
#else
#  define uerr        _none
#endif

#ifdef CONFIG_DEBUG_USB_WARN
#  define uwarn       _warn
#else
#  define uwarn       _none
#endif

#ifdef CONFIG_DEBUG_USB_INFO
#  define uinfo       _info
#else
#  define uinfo       _none
#endif

#ifdef CONFIG_DEBUG_WATCHDOG_ERROR
#  define wderr        _err
#else
#  define wderr       _none
#endif

#ifdef CONFIG_DEBUG_WATCHDOG_WARN
#  define wdwarn      _warn
#else
#  define wdwarn      _none
#endif

#ifdef CONFIG_DEBUG_WATCHDOG_INFO
#  define wdinfo      _info
#else
#  define wdinfo      _none
#endif

#ifdef CONFIG_DEBUG_MOTOR_ERROR
#  define mtrerr      _err
#else
#  define mtrerr      _none
#endif

#ifdef CONFIG_DEBUG_MOTOR_WARN
#  define mtrwarn     _warn
#else
#  define mtrwarn     _none
#endif

#ifdef CONFIG_DEBUG_MOTOR_INFO
#  define mtrinfo     _info
#else
#  define mtrinfo     _none
#endif

#ifdef CONFIG_DEBUG_VIDEO_ERROR
#  define verr        _err
#else
#  define verr        _none
#endif

#ifdef CONFIG_DEBUG_VIDEO_WARN
#  define vwarn       _warn
#else
#  define vwarn       _none
#endif

#ifdef CONFIG_DEBUG_VIDEO_INFO
#  define vinfo       _info
#else
#  define vinfo       _none
#endif

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

#ifdef CONFIG_DEBUG_MOTOR
#  define mtrerrdumpbuffer(m,b,n)  errdumpbuffer(m,b,n)
#  define mtrinfodumpbuffer(m,b,n) infodumpbuffer(m,b,n)
#else
#  define mtrerrdumpbuffer(m,b,n)
#  define mtrinfodumpbuffer(m,b,n)
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/* Type of the call out function pointer provided to
 * lib_dumphandler() or lib_dumpvhandler()
 */

typedef CODE void (*lib_dump_handler_t)(FAR void *arg,
                                        FAR const char *fmt, ...)
                  printflike(2, 3);

/* Dump a buffer of data with handler */

void lib_dumphandler(FAR const char *msg, FAR const uint8_t *buffer,
                     unsigned int buflen, lib_dump_handler_t handler,
                     FAR void *arg);

/* Do a pretty buffer dump from multiple buffers with handler. */

void lib_dumpvhandler(FAR const char *msg, FAR const struct iovec *iov,
                      int iovcnt, lib_dump_handler_t handler,
                      FAR void *arg);

/* Dump a buffer of data */

void lib_dumpbuffer(FAR const char *msg, FAR const uint8_t *buffer,
                    unsigned int buflen);

/* Do a pretty buffer dump from multiple buffers. */

void lib_dumpvbuffer(FAR const char *msg, FAR const struct iovec *iov,
                     int iovcnt);

/* Dump a buffer of data with fd */

void lib_dumpfile(int fd, FAR const char *msg, FAR const uint8_t *buffer,
                  unsigned int buflen);

/* Do a pretty buffer dump from multiple buffers with fd. */

void lib_dumpvfile(int fd, FAR const char *msg, FAR const struct iovec *iov,
                   int iovcnt);

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
