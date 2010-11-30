/****************************************************************************
 * examples/usbstorage/usbstrg_main.c
 *
 *   Copyright (C) 2008-2010 Gregory Nutt. All rights reserved.
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
#include <stdio.h>
#include <unistd.h>
#include <debug.h>

#include <nuttx/usbdev.h>
#include <nuttx/usbdev_trace.h>

#include "usbstrg.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_USBSTRG_TRACEINIT
#  define TRACE_INIT_BITS       (TRACE_INIT_BIT)
#else
#  define TRACE_INIT_BITS       (0)
#endif

#define TRACE_ERROR_BITS        (TRACE_DEVERROR_BIT|TRACE_CLSERROR_BIT)

#ifdef CONFIG_EXAMPLES_USBSTRG_TRACECLASS
#  define TRACE_CLASS_BITS      (TRACE_CLASS_BIT|TRACE_CLASSAPI_BIT|TRACE_CLASSSTATE_BIT)
#else
#  define TRACE_CLASS_BITS      (0)
#endif

#ifdef CONFIG_EXAMPLES_USBSTRG_TRACETRANSFERS
#  define TRACE_TRANSFER_BITS   (TRACE_OUTREQQUEUED_BIT|TRACE_INREQQUEUED_BIT|TRACE_READ_BIT|\
                                 TRACE_WRITE_BIT|TRACE_COMPLETE_BIT)
#else
#  define TRACE_TRANSFER_BITS   (0)
#endif

#ifdef CONFIG_EXAMPLES_USBSTRG_TRACECONTROLLER
#  define TRACE_CONTROLLER_BITS (TRACE_EP_BIT|TRACE_DEV_BIT)
#else
#  define TRACE_CONTROLLER_BITS (0)
#endif

#ifdef CONFIG_EXAMPLES_USBSTRG_TRACEINTERRUPTS
#  define TRACE_INTERRUPT_BITS  (TRACE_INTENTRY_BIT|TRACE_INTDECODE_BIT|TRACE_INTEXIT_BIT)
#else
#  define TRACE_INTERRUPT_BITS  (0)
#endif

#define TRACE_BITSET            (TRACE_INIT_BITS|TRACE_ERROR_BITS|TRACE_CLASS_BITS|\
                                 TRACE_TRANSFER_BITS|TRACE_CONTROLLER_BITS|TRACE_INTERRUPT_BITS)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbstrg_enumerate
 ****************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
static int usbstrg_enumerate(struct usbtrace_s *trace, void *arg)
{
  switch (trace->event)
    {
    case TRACE_DEVINIT:
      message("USB controller initialization: %04x\n", trace->value);
      break;

    case TRACE_DEVUNINIT:
      message("USB controller un-initialization: %04x\n", trace->value);
      break;

    case TRACE_DEVREGISTER:
      message("usbdev_register(): %04x\n", trace->value);
      break;

    case TRACE_DEVUNREGISTER:
      message("usbdev_unregister(): %04x\n", trace->value);
      break;

    case TRACE_EPCONFIGURE:
      message("Endpoint configure(): %04x\n", trace->value);
      break;

    case TRACE_EPDISABLE:
      message("Endpoint disable(): %04x\n", trace->value);
      break;

    case TRACE_EPALLOCREQ:
      message("Endpoint allocreq(): %04x\n", trace->value);
      break;

    case TRACE_EPFREEREQ:
      message("Endpoint freereq(): %04x\n", trace->value);
      break;

    case TRACE_EPALLOCBUFFER:
      message("Endpoint allocbuffer(): %04x\n", trace->value);
      break;

    case TRACE_EPFREEBUFFER:
      message("Endpoint freebuffer(): %04x\n", trace->value);
      break;

    case TRACE_EPSUBMIT:
      message("Endpoint submit(): %04x\n", trace->value);
      break;

    case TRACE_EPCANCEL:
      message("Endpoint cancel(): %04x\n", trace->value);
      break;

    case TRACE_EPSTALL:
      message("Endpoint stall(true): %04x\n", trace->value);
      break;

    case TRACE_EPRESUME:
      message("Endpoint stall(false): %04x\n", trace->value);
      break;

    case TRACE_DEVALLOCEP:
      message("Device allocep(): %04x\n", trace->value);
      break;

    case TRACE_DEVFREEEP:
      message("Device freeep(): %04x\n", trace->value);
      break;

    case TRACE_DEVGETFRAME:
      message("Device getframe(): %04x\n", trace->value);
      break;

    case TRACE_DEVWAKEUP:
      message("Device wakeup(): %04x\n", trace->value);
      break;

    case TRACE_DEVSELFPOWERED:
      message("Device selfpowered(): %04x\n", trace->value);
      break;

    case TRACE_DEVPULLUP:
      message("Device pullup(): %04x\n", trace->value);
      break;

    case TRACE_CLASSBIND:
      message("Class bind(): %04x\n", trace->value);
      break;

    case TRACE_CLASSUNBIND:
      message("Class unbind(): %04x\n", trace->value);
      break;

    case TRACE_CLASSDISCONNECT:
      message("Class disconnect(): %04x\n", trace->value);
      break;

    case TRACE_CLASSSETUP:
      message("Class setup(): %04x\n", trace->value);
      break;

    case TRACE_CLASSSUSPEND:
      message("Class suspend(): %04x\n", trace->value);
      break;

    case TRACE_CLASSRESUME:
      message("Class resume(): %04x\n", trace->value);
      break;

    case TRACE_CLASSRDCOMPLETE:
      message("Class RD request complete: %04x\n", trace->value);
      break;

    case TRACE_CLASSWRCOMPLETE:
      message("Class WR request complete: %04x\n", trace->value);
      break;

    default:
      switch (TRACE_ID(trace->event))
        {
        case TRACE_CLASSAPI_ID:        /* Other class driver system API calls */
          message("Class API call %d: %04x\n", TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_CLASSSTATE_ID:      /* Track class driver state changes */
          message("Class state %d: %04x\n", TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_INTENTRY_ID:        /* Interrupt handler entry */
          message("Interrrupt %d entry: %04x\n", TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_INTDECODE_ID:       /* Decoded interrupt trace->event */
          message("Interrrupt decode %d: %04x\n", TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_INTEXIT_ID:         /* Interrupt handler exit */
          message("Interrrupt %d exit: %04x\n", TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_OUTREQQUEUED_ID:    /* Request queued for OUT endpoint */
          message("EP%d OUT request queued: %04x\n", TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_INREQQUEUED_ID:     /* Request queued for IN endpoint */
          message("EP%d IN request queued: %04x\n", TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_READ_ID:            /* Read (OUT) action */
          message("EP%d OUT read: %04x\n", TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_WRITE_ID:           /* Write (IN) action */
          message("EP%d IN write: %04x\n", TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_COMPLETE_ID:        /* Request completed */
          message("EP%d request complete: %04x\n", TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_DEVERROR_ID:        /* USB controller driver error event */
          message("Controller error: %02x:%04x\n", TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_CLSERROR_ID:        /* USB class driver error event */
          message("Class error: %02x:%04x\n", TRACE_DATA(trace->event), trace->value);
          break;

        default:
          message("Unrecognized event: %02x:%02x:%04x\n",
                  TRACE_ID(trace->event) >> 8, TRACE_DATA(trace->event), trace->value);
          break;
        }
    }
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * user_initialize
 ****************************************************************************/

#ifndef CONFIG_HAVE_WEAKFUNCTIONS
void user_initialize(void)
{
  /* Stub that must be provided only if the toolchain does not support weak
   * functions.
   */
}
#endif

/****************************************************************************
 * user_start
 ****************************************************************************/

int user_start(int argc, char *argv[])
{
  void *handle;
  int ret;

  /* Initialize USB trace output IDs */

  usbtrace_enable(TRACE_BITSET);

  /* Register block drivers (architecture-specific) */

  message("user_start: Creating block drivers\n");
  ret = usbstrg_archinitialize();
  if (ret < 0)
    {
      message("user_start: usbstrg_archinitialize failed: %d\n", -ret);
      return 1;
    }

  /* Then exports the LUN(s) */

  message("user_start: Configuring with NLUNS=%d\n", CONFIG_EXAMPLES_USBSTRG_NLUNS);
  ret = usbstrg_configure(CONFIG_EXAMPLES_USBSTRG_NLUNS, &handle);
  if (ret < 0)
    {
      message("user_start: usbstrg_configure failed: %d\n", -ret);
      usbstrg_uninitialize(handle);
      return 2;
    }
  message("user_start: handle=%p\n", handle);

  message("user_start: Bind LUN=0 to %s\n", CONFIG_EXAMPLES_USBSTRG_DEVPATH1);
  ret = usbstrg_bindlun(handle, CONFIG_EXAMPLES_USBSTRG_DEVPATH1, 0, 0, 0, false);
  if (ret < 0)
    {
      message("user_start: usbstrg_bindlun failed for LUN 1 using %s: %d\n",
               CONFIG_EXAMPLES_USBSTRG_DEVPATH1, -ret);
      usbstrg_uninitialize(handle);
      return 2;
    }

#if CONFIG_EXAMPLES_USBSTRG_NLUNS > 1

  message("user_start: Bind LUN=1 to %s\n", CONFIG_EXAMPLES_USBSTRG_DEVPATH2);
  ret = usbstrg_bindlun(handle, CONFIG_EXAMPLES_USBSTRG_DEVPATH2, 1, 0, 0, false);
  if (ret < 0)
    {
      message("user_start: usbstrg_bindlun failed for LUN 2 using %s: %d\n",
               CONFIG_EXAMPLES_USBSTRG_DEVPATH2, -ret);
      usbstrg_uninitialize(handle);
      return 3;
    }

#if CONFIG_EXAMPLES_USBSTRG_NLUNS > 2

  message("user_start: Bind LUN=2 to %s\n", CONFIG_EXAMPLES_USBSTRG_DEVPATH3);
  ret = usbstrg_bindlun(handle, CONFIG_EXAMPLES_USBSTRG_DEVPATH3, 2, 0, 0, false);
  if (ret < 0)
    {
      message("user_start: usbstrg_bindlun failed for LUN 3 using %s: %d\n",
               CONFIG_EXAMPLES_USBSTRG_DEVPATH3, -ret);
      usbstrg_uninitialize(handle);
      return 4;
    }

#endif
#endif

  ret = usbstrg_exportluns(handle);
  if (ret < 0)
    {
      message("user_start: usbstrg_exportluns failed: %d\n", -ret);
      usbstrg_uninitialize(handle);
      return 5;
    }

  /* Now just hang around and monitor the USB storage activity */

#ifndef CONFIG_DISABLE_SIGNALS
  for (;;)
    {
      msgflush();
      sleep(5);

#ifdef CONFIG_USBDEV_TRACE
      message("\nuser_start: USB TRACE DATA:\n");
      ret =  usbtrace_enumerate(usbstrg_enumerate, NULL);
      if (ret < 0)
        {
          message("user_start: usbtrace_enumerate failed: %d\n", -ret);
          usbstrg_uninitialize(handle);
          return 6;
        }
#else
      message("user_start: Still alive\n");
#endif
    }
#else
     message("user_start: Exiting\n");
 #endif
}

