/****************************************************************************
 * include/nuttx/usbdev_trace.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_USBDEV_TRACE_H
#define __INCLUDE_NUTTX_USBDEV_TRACE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

#define TRACE_EVENT(id,num)      ((uint16)(id)|(num))
#define TRACE_CLASS(event)       ((event)&0xff00)
#define TRACE_INSTANCE(event)    ((event)0x00ff)

/* Initialization events */

#define TRACE_INIT               (0x0000)
#define TRACE_DEVINIT            TRACE_EVENT(TRACE_INIT, 0x0001)
#define TRACE_DEVUNINIT          TRACE_EVENT(TRACE_INIT, 0x0002)
#define TRACE_DEVREGISTER        TRACE_EVENT(TRACE_INIT, 0x0003)
#define TRACE_DEVUNREGISTER      TRACE_EVENT(TRACE_INIT, 0x0004)

/* API calls (see usbdev.h) */

#define TRACE_EP                 (0x0100)
#define TRACE_EPCONFIGURE        TRACE_EVENT(TRACE_EP, 0x0001)
#define TRACE_EPDISABLE          TRACE_EVENT(TRACE_EP, 0x0002)
#define TRACE_EPALLOCREQ         TRACE_EVENT(TRACE_EP, 0x0003)
#define TRACE_EPFREEREQ          TRACE_EVENT(TRACE_EP, 0x0004)
#define TRACE_EPALLOCBUFFER      TRACE_EVENT(TRACE_EP, 0x0005)
#define TRACE_EPFREEBUFFER       TRACE_EVENT(TRACE_EP, 0x0006)
#define TRACE_EPSUBMIT           TRACE_EVENT(TRACE_EP, 0x0007)
#define TRACE_EPCANCEL           TRACE_EVENT(TRACE_EP, 0x0008)
#define TRACE_EPSTALL            TRACE_EVENT(TRACE_EP, 0x0009)
#define TRACE_EPRESUME           TRACE_EVENT(TRACE_EP, 0x000a)

#define TRACE_DEV                (0x0200)
#define TRACE_DEVALLOCEP         TRACE_EVENT(TRACE_DEV, 0x0001)
#define TRACE_DEVFREEEP          TRACE_EVENT(TRACE_DEV, 0x0002)
#define TRACE_DEVGETFRAME        TRACE_EVENT(TRACE_DEV, 0x0003)
#define TRACE_DEVWAKEUP          TRACE_EVENT(TRACE_DEV, 0x0004)
#define TRACE_DEVSELFPOWERED     TRACE_EVENT(TRACE_DEV, 0x0005)
#define TRACE_DEVPULLUP          TRACE_EVENT(TRACE_DEV, 0x0006)

#define TRACE_CLASS              (0x0300)
#define TRACE_CLASSBIN           TRACE_EVENT(TRACE_CLASS, 0x0001)
#define TRACE_CLASSUNBIND        TRACE_EVENT(TRACE_CLASS, 0x0001)
#define TRACE_CLASSDISCONNECT    TRACE_EVENT(TRACE_CLASS, 0x0001)
#define TRACE_CLASSSETUP         TRACE_EVENT(TRACE_CLASS, 0x0001)
#define TRACE_CLASSSUSPEND       TRACE_EVENT(TRACE_CLASS, 0x0001)
#define TRACE_CLASSRESUME        TRACE_EVENT(TRACE_CLASS, 0x0001)

/* Interrupt events -- id is hardware specific */

#define TRACE_INTENTRY(id)       TRACE_EVENT(0x0400, id)
#define TRACE_INTDECODE(id)      TRACE_EVENT(0x0500, id)
#define TRACE_INTEXIT(id)        TRACE_EVENT(0x0600, id)

/* Data Transfer */

#define TRACE_READ(ep)           TRACE_EVENT(0x0700, ep)
#define TRACE_WRITE(ep)          TRACE_EVENT(0x0800, ep)
#define TRACE_COMPLETE(ep)       TRACE_EVENT(0x0900, ep)

/* Error events -- id is specific to the driver */

#define TRACE_ERROR(id)          TRACE_EVENT(0x0700, id)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The reported trace information */

struct usbtrace_s
{
  uint16 event;
  uint16 value;
};

/* Enumeration callback function signature */

typedef int (*trace_callback_t)(struct usbtrace_s *trace, void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
# define EXTERN extern "C"
extern "C" {
#else
# define EXTERN extern
#endif

/*******************************************************************************
 * Name: usbtrace_enable
 *
 * Description:
 *  Enable/disable tracing
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
EXTERN void usbtrace_enable(boolen enable);
#else
#  define usbtrace_enable(enable)
#endif

/*******************************************************************************
 * Name: usbtrace
 *
 * Description:
 *  Record a USB event (tracing must be enabled)
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
EXTERN void usbtrace(uint16 event, uint16 value);
#else
#  define usbtrace(event)
#endif

/*******************************************************************************
 * Name: usbtrace_enumerate
 *
 * Description:
 *   Enumerate all buffer trace data (tracing must be disabled)
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
EXTERN int usbtrace_enumerate(tracecallback_t *callback, void *arg);
#else
#  define usbtrace_enumerate(event)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USBDEV_TRACE_H */
