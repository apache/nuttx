/****************************************************************************
 * drivers/usbdev/usbdev_trace.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/usbdev_trace.h>
#undef usbtrace

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_USBDEV_TRACE_NRECORDS
#  define CONFIG_USBDEV_TRACE_NRECORDS 128
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
static struct usbtrace_s g_trace[CONFIG_USBDEV_TRACE_NRECORDS];
static uint16 g_head = 0;
static uint16 g_tail = 0;
#endif

#if defined(CONFIG_USBDEV_TRACE) || (defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_USB))
static usbtrace_idset_t g_maskedidset = 0;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*******************************************************************************
 * Name: usbtrace_enable
 *
 * Description:
 *  Enable/disable tracing per trace ID.  The initial state is all IDs enabled.
 *
 * Input Parameters:
 *  idset - The bitset of IDs to be masked.  TRACE_ALLIDS enables all IDS; zero
 *  masks all IDs.
 *
 * Returned Value:
 *  The previous idset value.
 *
 * Assumptions:
 * - May be called from an interrupt handler
 *
 *******************************************************************************/

#if defined(CONFIG_USBDEV_TRACE) || (defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_USB))
usbtrace_idset_t usbtrace_enable(usbtrace_idset_t idset)
{
  irqstate_t flags;
  usbtrace_idset_t ret;

  /* The following read and write must be atomic */

  flags         = irqsave();
  ret           = g_maskedidset;
  g_maskedidset = idset;
  irqrestore(flags);
  return ret;
}
#endif /* CONFIG_USBDEV_TRACE || CONFIG_DEBUG && CONFIG_DEBUG_USB */

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

#if defined(CONFIG_USBDEV_TRACE) || (defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_USB))
void usbtrace(uint16 event, uint16 value)
{
  irqstate_t flags;

  /* Check if tracing is enabled for this ID */

  flags = irqsave();
  if ((g_maskedidset & TRACE_ID2BIT(event)) != 0)
    {
#ifdef CONFIG_USBDEV_TRACE
      /* Yes... save the new trace data at the nead */

      g_trace[g_head].event = event;
      g_trace[g_tail].event = value;

      /* Increment the head and (probably) the tail index */

      if (++g_head >= CONFIG_USBDEV_TRACE_NRECORDS)
        {
          g_head = 0;
        }

      if (g_head == g_tail)
        {
          if (++g_tail >= CONFIG_USBDEV_TRACE_NRECORDS)
            {
              g_tail = 0;
            }
        }
#else
      switch (event)
        {
        case TRACE_DEVINIT:
          lldbg("USB controller initialization: %04x\n", value);
          break;

        case TRACE_DEVUNINIT:
          lldbg("USB controller un-initialization: %04x\n", value);
          break;

        case TRACE_DEVREGISTER:
          lldbg("usbdev_register(): %04x\n", value);
          break;

        case TRACE_DEVUNREGISTER:
          lldbg("usbdev_unregister(): %04x\n", value);
          break;

        case TRACE_EPCONFIGURE:
          lldbg("Endpoint configure(): %04x\n", value);
          break;

        case TRACE_EPDISABLE:
          lldbg("Endpoint disable(): %04x\n", value);
          break;

        case TRACE_EPALLOCREQ:
          lldbg("Endpoint allocreq(): %04x\n", value);
          break;

        case TRACE_EPFREEREQ:
          lldbg("Endpoint freereq(): %04x\n", value);
          break;

        case TRACE_EPALLOCBUFFER:
          lldbg("Endpoint allocbuffer(): %04x\n", value);
          break;

        case TRACE_EPFREEBUFFER:
          lldbg("Endpoint freebuffer(): %04x\n", value);
          break;

        case TRACE_EPSUBMIT:
          lldbg("Endpoint submit(): %04x\n", value);
          break;

        case TRACE_EPCANCEL:
          lldbg("Endpoint cancel(): %04x\n", value);
          break;

        case TRACE_EPSTALL:
          lldbg("Endpoint stall(TRUE): %04x\n", value);
          break;

        case TRACE_EPRESUME:
          lldbg("Endpoint stall(FALSE): %04x\n", value);
          break;

        case TRACE_DEVALLOCEP:
          lldbg("Device allocep(): %04x\n", value);
          break;

        case TRACE_DEVFREEEP:
          lldbg("Device freeep(): %04x\n", value);
          break;

        case TRACE_DEVGETFRAME:
          lldbg("Device getframe(): %04x\n", value);
          break;

        case TRACE_DEVWAKEUP:
          lldbg("Device wakeup(): %04x\n", value);
          break;

        case TRACE_DEVSELFPOWERED:
          lldbg("Device selfpowered(): %04x\n", value);
          break;

        case TRACE_DEVPULLUP:
          lldbg("Device pullup(): %04x\n", value);
          break;

        case TRACE_CLASSBIND:
          lldbg("Class bind(): %04x\n", value);
          break;

        case TRACE_CLASSUNBIND:
          lldbg("Class unbind(): %04x\n", value);
          break;

        case TRACE_CLASSDISCONNECT:
          lldbg("Class disconnect(): %04x\n", value);
          break;

        case TRACE_CLASSSETUP:
          lldbg("Class setup(): %04x\n", value);
          break;

        case TRACE_CLASSSUSPEND:
          lldbg("Class suspend(): %04x\n", value);
          break;

        case TRACE_CLASSRESUME:
          lldbg("Class resume(): %04x\n", value);
          break;

        case TRACE_CLASSRDCOMPLETE:
          lldbg("Class RD request complete: %04x\n", value);
          break;

        case TRACE_CLASSWRCOMPLETE:
          lldbg("Class WR request complete: %04x\n", value);
          break;

        default:
          switch (TRACE_ID(event))
            {
            case TRACE_CLASSAPI_ID:        /* Other class driver system API calls */
              lldbg("Class API call %d: %04x\n", TRACE_DATA(event), value);
              break;

            case TRACE_CLASSSTATE_ID:      /* Track class driver state changes */
              lldbg("Class state %d: %04x\n", TRACE_DATA(event), value);
              break;

            case TRACE_INTENTRY_ID:        /* Interrupt handler entry */
              lldbg("Interrrupt %d entry: %04x\n", TRACE_DATA(event), value);
              break;

            case TRACE_INTDECODE_ID:       /* Decoded interrupt event */
              lldbg("Interrrupt decode %d: %04x\n", TRACE_DATA(event), value);
              break;

            case TRACE_INTEXIT_ID:         /* Interrupt handler exit */
              lldbg("Interrrupt %d exit: %04x\n", TRACE_DATA(event), value);
              break;

            case TRACE_OUTREQQUEUED_ID:    /* Request queued for OUT endpoint */
              lldbg("EP%d OUT request queued: %04x\n", TRACE_DATA(event), value);
              break;

            case TRACE_INREQQUEUED_ID:     /* Request queued for IN endpoint */
              lldbg("EP%d IN request queued: %04x\n", TRACE_DATA(event), value);
              break;

            case TRACE_READ_ID:            /* Read (OUT) action */
              lldbg("EP%d OUT read: %04x\n", TRACE_DATA(event), value);
              break;

            case TRACE_WRITE_ID:           /* Write (IN) action */
              lldbg("EP%d IN write: %04x\n", TRACE_DATA(event), value);
              break;

            case TRACE_COMPLETE_ID:        /* Request completed */
              lldbg("EP%d request complete: %04x\n", TRACE_DATA(event), value);
              break;

            case TRACE_DEVERROR_ID:        /* USB controller driver error event */
              lldbg("Controller error: %02x:%04x\n", TRACE_DATA(event), value);
              break;

            case TRACE_CLSERROR_ID:        /* USB class driver error event */
              lldbg("Class error: %02x:%04x\n", TRACE_DATA(event), value);
              break;

            default:
              lldbg("Unrecognized event: %02x:%02x:%04x\n",
                    TRACE_ID(event) >> 8, TRACE_DATA(event), value);
              break;
            }
        }
#endif /* CONFIG_USBDEV_TRACE */
    }
  irqrestore(flags);
}
#endif /* CONFIG_USBDEV_TRACE || CONFIG_DEBUG && CONFIG_DEBUG_USB */

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
int usbtrace_enumerate(trace_callback_t callback, void *arg)
{
  uint16 ndx = g_tail;
  int ret = OK;

  /* Temporarily disable tracing */

  usbtrace_enable(FALSE);

  /* Visit every entry, starting with the tail */

  while (g_tail != g_head)
    {
      /* Call the user provided callback */

      ret = callback(&g_trace[ndx], arg);
      if (ret != OK)
        {
          /* Abort the enumeration */

          break;
        }

      /* Increment the index */

      if (++ndx >= CONFIG_USBDEV_TRACE_NRECORDS)
        {
          ndx = 0;
        }
    }

  return ret;
}
#endif /* CONFIG_USBDEV_TRACE */
