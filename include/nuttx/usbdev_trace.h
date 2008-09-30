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
#define TRACE_CLASS_ID(event)       ((event)&0xff00)
#define TRACE_INSTANCE(event)    ((event)0x00ff)

/* Event class IDs */

#define TRACE_INIT_ID            (0x0000) /* Initialization events */
#define TRACE_EP_ID              (0x0100) /* Endpoint API calls */
#define TRACE_DEV_ID             (0x0200) /* USB device API calls */
#define TRACE_CLASS_ID           (0x0300) /* USB class driver API calls */
#define TRACE_INTENTRY_ID        (0x0400) /* Interrupt handler entry */
#define TRACE_INTDECODE_ID       (0x0500) /* Decoded interrupt event */
#define TRACE_INTEXIT_ID         (0x0600) /* Interrupt handler exit */
#define TRACE_REQQUEUED_ID       (0x0700) /* Request queued */
#define TRACE_READ_ID            (0x0800) /* Read (OUT) action */
#define TRACE_WRITE_ID           (0x0900) /* Write (IN) action */
#define TRACE_COMPLETE_ID        (0x0a00) /* Request completed */
#define TRACE_DEVERROR_ID        (0x0b00) /* USB controller driver error event */
#define TRACE_CLSERROR_ID        (0x0c00) /* USB class driver error event */

/* Initialization events */

#define TRACE_DEVINIT            TRACE_EVENT(TRACE_INIT_ID, 0x0001)
#define TRACE_DEVUNINIT          TRACE_EVENT(TRACE_INIT_ID, 0x0002)
#define TRACE_DEVREGISTER        TRACE_EVENT(TRACE_INIT_ID, 0x0003)
#define TRACE_DEVUNREGISTER      TRACE_EVENT(TRACE_INIT_ID, 0x0004)

/* API calls (see usbdev.h) */

#define TRACE_EPCONFIGURE        TRACE_EVENT(TRACE_EP_ID, 0x0001)
#define TRACE_EPDISABLE          TRACE_EVENT(TRACE_EP_ID, 0x0002)
#define TRACE_EPALLOCREQ         TRACE_EVENT(TRACE_EP_ID, 0x0003)
#define TRACE_EPFREEREQ          TRACE_EVENT(TRACE_EP_ID, 0x0004)
#define TRACE_EPALLOCBUFFER      TRACE_EVENT(TRACE_EP_ID, 0x0005)
#define TRACE_EPFREEBUFFER       TRACE_EVENT(TRACE_EP_ID, 0x0006)
#define TRACE_EPSUBMIT           TRACE_EVENT(TRACE_EP_ID, 0x0007)
#define TRACE_EPCANCEL           TRACE_EVENT(TRACE_EP_ID, 0x0008)
#define TRACE_EPSTALL            TRACE_EVENT(TRACE_EP_ID, 0x0009)
#define TRACE_EPRESUME           TRACE_EVENT(TRACE_EP_ID, 0x000a)

#define TRACE_DEVALLOCEP         TRACE_EVENT(TRACE_DEV_ID, 0x0001)
#define TRACE_DEVFREEEP          TRACE_EVENT(TRACE_DEV_ID, 0x0002)
#define TRACE_DEVGETFRAME        TRACE_EVENT(TRACE_DEV_ID, 0x0003)
#define TRACE_DEVWAKEUP          TRACE_EVENT(TRACE_DEV_ID, 0x0004)
#define TRACE_DEVSELFPOWERED     TRACE_EVENT(TRACE_DEV_ID, 0x0005)
#define TRACE_DEVPULLUP          TRACE_EVENT(TRACE_DEV_ID, 0x0006)

#define TRACE_CLASSBIND          TRACE_EVENT(TRACE_CLASS_ID, 0x0001)
#define TRACE_CLASSUNBIND        TRACE_EVENT(TRACE_CLASS_ID, 0x0002)
#define TRACE_CLASSDISCONNECT    TRACE_EVENT(TRACE_CLASS_ID, 0x0003)
#define TRACE_CLASSSETUP         TRACE_EVENT(TRACE_CLASS_ID, 0x0004)
#define TRACE_CLASSSUSPEND       TRACE_EVENT(TRACE_CLASS_ID, 0x0005)
#define TRACE_CLASSRESUME        TRACE_EVENT(TRACE_CLASS_ID, 0x0006)

/* USB device controller interrupt events.  The 'id' is specific to the driver.
 * Particular values for 'id' are unique for a given implementation of a
 * controller driver
 */

#define TRACE_INTENTRY(id)       TRACE_EVENT(TRACE_INTENTRY_ID, id)
#define TRACE_INTDECODE(id)      TRACE_EVENT(TRACE_INTDECODE_ID, id)
#define TRACE_INTEXIT(id)        TRACE_EVENT(TRACE_INTEXIT_ID, id)

/* Data Transfer */

#define TRACE_REQQUEUED(ep)      TRACE_EVENT(TRACE_REQQUEUED_ID, ep)
#define TRACE_READ(ep)           TRACE_EVENT(TRACE_READ_ID, ep)
#define TRACE_WRITE(ep)          TRACE_EVENT(TRACE_WRITE_ID, ep)
#define TRACE_COMPLETE(ep)       TRACE_EVENT(TRACE_COMPLETE_ID, ep)

/* USB device controller error events.  The 'id' is specific to the driver.
 * Particular values for 'id' are unique for a given implementation of a
 * controller driver
 */

#define TRACE_DEVERROR(id)       TRACE_EVENT(TRACE_CLSERROR_ID, id)

/* USB class driver error events.  The 'id' is specific to the class driver,
 * but common to all driver controller instances.
 */

#define TRACE_CLSERROR(id)       TRACE_EVENT(TRACE_CLSERROR_ID, id)

/* Values of the class error ID used by the USB serial driver */

#define USBSER_TRACEERR_ALLOCCTRLREQ      0x0001
#define USBSER_TRACEERR_ALREADYCLOSED     0x0002
#define USBSER_TRACEERR_CONSOLEREGISTER   0x0003
#define USBSER_TRACEERR_DEVREGISTER       0x0004
#define USBSER_TRACEERR_EPRESPQ           0x0005
#define USBSER_TRACEERR_GETUNKNOWNDESC    0x0006
#define USBSER_TRACEERR_INALLOCEPFAIL     0x0007
#define USBSER_TRACEERR_INCONFIGEPFAIL    0x0008
#define USBSER_TRACEERR_INVALIDARG        0x0009
#define USBSER_TRACEERR_OUTALLOCEPFAIL    0x000a
#define USBSER_TRACEERR_OUTCONFIGEPFAIL   0x000b
#define USBSER_TRACEERR_RDALLOCREQ        0x000c
#define USBSER_TRACEERR_RDSHUTDOWN        0x000d
#define USBSER_TRACEERR_RDSUBMIT          0x000e
#define USBSER_TRACEERR_RDUNEXPECTED      0x000f
#define USBSER_TRACEERR_REQRESULT         0x0010
#define USBSER_TRACEERR_SETUPNOTCONNECTED 0x0011
#define USBSER_TRACEERR_SUBMITFAIL        0x0012
#define USBSER_TRACEERR_UARTREGISTER      0x0013
#define USBSER_TRACEERR_WRALLOCREQ        0x0014
#define USBSER_TRACEERR_WRSHUTDOWN        0x0015
#define USBSER_TRACEERR_WRUNEXPECTED      0x0016

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
 * Assumptions:
 * - Initial state is enabled
 * - May be called from an interrupt handler
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
EXTERN void usbtrace_enable(boolean enable);
#else
#  define usbtrace_enable(enable)
#endif

/*******************************************************************************
 * Name: usbtrace
 *
 * Description:
 *  Record a USB event (tracing must be enabled)
 *
 * Assumptions:
 *   May be called from an interrupt handler
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
EXTERN void usbtrace(uint16 event, uint16 value);
#else
#  define usbtrace(event, value)
#endif

/*******************************************************************************
 * Name: usbtrace_enumerate
 *
 * Description:
 *   Enumerate all buffer trace data (will temporarily disable tracing)
 *
 * Assumptions:
 *   NEVER called from an interrupt handler
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
EXTERN int usbtrace_enumerate(trace_callback_t callback, void *arg);
#else
#  define usbtrace_enumerate(event)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USBDEV_TRACE_H */
