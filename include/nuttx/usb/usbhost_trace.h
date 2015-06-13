/****************************************************************************
 * include/nuttx/usb/usbhost_trace.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_USB_USBHOST_TRACE_H
#define __INCLUDE_NUTTX_USB_USBHOST_TRACE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Debug/Trace-related definitions */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_USB
#  undef CONFIG_DEBUG_VERBOSE
#endif

#ifndef CONFIG_USBHOST_TRACE
#  undef CONFIG_USBHOST_TRACE_VERBOSE
#endif

/* Trace support is needed if either USB host tracing or USB debug output is enabled */

#if defined(CONFIG_USBHOST_TRACE) || defined(CONFIG_DEBUG_USB)
#  define HAVE_USBHOST_TRACE 1
#  if defined(CONFIG_USBHOST_TRACE_VERBOSE) || defined(CONFIG_DEBUG_VERBOSE)
#    define HAVE_USBHOST_TRACE_VERBOSE 1
#  endif
#endif

/* Event encoding/decoding macros *******************************************/

#define TRACE_ENCODE1(id,u23)    (((uint32_t)(id) & 0x1ff) << 23 | \
                                  ((uint32_t)(u23) & 0x007fffff))
#define TRACE_ENCODE2(id,u7,u16) (((uint32_t)(id) & 0x1ff) << 23 | \
                                  ((uint32_t)(u7) & 0x7f) << 16 | \
                                  ((u16) & 0x0000ffff))

#define TRACE_DECODE_NDX(ev)     (((ev) >> 23) & 0x1ff)
#define TRACE_DECODE_U7(ev)      (((ev) >> 16) & 0x7f)
#define TRACE_DECODE_U16(ev)     ((ev) & 0x0000ffff)
#define TRACE_DECODE_U23(ev)     ((ev) & 0x007fffff)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Enumeration callback function signature */

typedef int (*usbhost_trcallback_t)(FAR uint32_t trace, FAR void *arg);

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
# define EXTERN extern "C"
extern "C"
{
#else
# define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_trace and usbhost_trace2
 *
 * Description:
 *  Record a USB event (tracing or USB debug must be enabled)
 *
 * Assumptions:
 *   May be called from an interrupt handler
 *
 ****************************************************************************/

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_USB
#endif

#ifdef HAVE_USBHOST_TRACE
void usbhost_trace1(uint16_t id, uint32_t u23);
void usbhost_trace2(uint16_t id, uint8_t u7, uint16_t u16);

#ifdef HAVE_USBHOST_TRACE_VERBOSE
#  define usbhost_vtrace1(id, u23)     usbhost_trace1(id, u23)
#  define usbhost_vtrace2(id, u7, u16) usbhost_trace2(id, u7, u16)
#else
#  define usbhost_vtrace1(id, u23)
#  define usbhost_vtrace2(id, u7, u16)
#endif

#else
#  define usbhost_trace1(id, u23)
#  define usbhost_trace2(id, u7, u16)
#  define usbhost_vtrace1(id, u23)
#  define usbhost_vtrace2(id, u7, u16)
#endif

/****************************************************************************
 * Name: usbhost_trenumerate
 *
 * Description:
 *   Enumerate all buffer trace data (will temporarily disable tracing)
 *
 * Assumptions:
 *   NEVER called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_TRACE
int usbhost_trenumerate(usbhost_trcallback_t callback, FAR void *arg);
#else
#  define usbhost_trenumerate(callback,arg)
#endif

/****************************************************************************
 * Name: usbhost_trdump
 *
 * Description:
 *   Used usbhost_trenumerate to dump all buffer trace data to syslog().
 *
 * Assumptions:
 *   NEVER called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_TRACE
int usbhost_trdump(void);
#else
#  define usbhost_trdump(void)
#endif

/****************************************************************************
 * Name: usbhost_trformat1 and usbhost_trformat2
 *
 * Description:
 *   This interface must be provided by platform specific logic that knows
 *   the HCDs encoding of USB trace data.
 *
 *   Given an 9-bit index, return a format string suitable for use with, say,
 *   printf.  The returned format is expected to handle two unsigned integer
 *   values.
 *
 ****************************************************************************/

#ifdef HAVE_USBHOST_TRACE
FAR const char *usbhost_trformat1(uint16_t id);
FAR const char *usbhost_trformat2(uint16_t id);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_USBHOST_TRACE_H */
