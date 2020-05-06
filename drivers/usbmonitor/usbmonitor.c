/****************************************************************************
 * drivers/usbmonitor/usbmonitor.c
 *
 *   Copyright (C) 2013, 2016-2018 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/progmem.h>

#include <sys/types.h>
#include <stdarg.h>
#include <stdbool.h>
#include <unistd.h>
#include <sched.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/signal.h>
#include <nuttx/kthread.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/usb/usbhost_trace.h>

#ifdef CONFIG_USBMONITOR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_USBMONITOR_STACKSIZE
#  define CONFIG_USBMONITOR_STACKSIZE 2048
#endif

#ifndef CONFIG_USBMONITOR_PRIORITY
#  define CONFIG_USBMONITOR_PRIORITY 50
#endif

#ifndef CONFIG_USBMONITOR_INTERVAL
#  define CONFIG_USBMONITOR_INTERVAL 2
#endif

/* USB device trace selection */

#ifdef CONFIG_USBDEV_TRACE
#  ifdef CONFIG_USBMONITOR_TRACEINIT
#    define TRACE_INIT_BITS       (TRACE_INIT_BIT)
#  else
#    define TRACE_INIT_BITS       (0)
#  endif

#  define TRACE_ERROR_BITS        (TRACE_DEVERROR_BIT|TRACE_CLSERROR_BIT)

#  ifdef CONFIG_USBMONITOR_TRACECLASS
#    define TRACE_CLASS_BITS      (TRACE_CLASS_BIT|TRACE_CLASSAPI_BIT|\
                                   TRACE_CLASSSTATE_BIT)
#  else
#    define TRACE_CLASS_BITS      (0)
#  endif

#  ifdef CONFIG_USBMONITOR_TRACETRANSFERS
#    define TRACE_TRANSFER_BITS   (TRACE_OUTREQQUEUED_BIT|TRACE_INREQQUEUED_BIT|\
                                   TRACE_READ_BIT|TRACE_WRITE_BIT|\
                                   TRACE_COMPLETE_BIT)
#  else
#    define TRACE_TRANSFER_BITS   (0)
#  endif

#  ifdef CONFIG_USBMONITOR_TRACECONTROLLER
#    define TRACE_CONTROLLER_BITS (TRACE_EP_BIT|TRACE_DEV_BIT)
#  else
#    define TRACE_CONTROLLER_BITS (0)
#  endif

#  ifdef CONFIG_USBMONITOR_TRACEINTERRUPTS
#    define TRACE_INTERRUPT_BITS  (TRACE_INTENTRY_BIT|TRACE_INTDECODE_BIT|\
                                   TRACE_INTEXIT_BIT)
#  else
#    define TRACE_INTERRUPT_BITS  (0)
#  endif

#  define TRACE_BITSET            (TRACE_INIT_BITS|TRACE_ERROR_BITS|\
                                   TRACE_CLASS_BITS|TRACE_TRANSFER_BITS|\
                                   TRACE_CONTROLLER_BITS|TRACE_INTERRUPT_BITS)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct usbmon_state_s
{
  volatile bool started;
  volatile bool stop;
  pid_t pid;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct usbmon_state_s g_usbmonitor;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
static int usbtrace_syslog(FAR const char *fmt, ...)
{
  va_list ap;

  /* Let vsyslog do the real work */

  va_start(ap, fmt);
  vsyslog(LOG_INFO, fmt, ap);
  va_end(ap);
  return OK;
}

static int usbmonitor_tracecallback(struct usbtrace_s *trace, void *arg)
{
  usbtrace_trprintf(usbtrace_syslog, trace->event, trace->value);
  return 0;
}
#endif

static int usbmonitor_daemon(int argc, char **argv)
{
  uinfo("Running: %d\n", g_usbmonitor.pid);

  /* Loop until we detect that there is a request to stop. */

  while (!g_usbmonitor.stop)
    {
      nxsig_sleep(CONFIG_USBMONITOR_INTERVAL);
#ifdef CONFIG_USBDEV_TRACE
      usbtrace_enumerate(usbmonitor_tracecallback, NULL);
#endif
#ifdef CONFIG_USBHOST_TRACE
      usbhost_trdump();
#endif
    }

  /* Stopped */

  g_usbmonitor.stop    = false;
  g_usbmonitor.started = false;
  uinfo("Stopped: %d\n", g_usbmonitor.pid);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbmonitor_start
 *
 *   Start the USB monitor kernal daemon.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

int usbmonitor_start(void)
{
  /* Has the monitor already started? */

  sched_lock();
  if (!g_usbmonitor.started)
    {
      int ret;

      /* No.. start it now */

#ifdef CONFIG_USBDEV_TRACE
      /* First, initialize any USB tracing options that were requested */

      usbtrace_enable(TRACE_BITSET);
#endif

      /* Then start the USB monitoring daemon */

      g_usbmonitor.started = true;
      g_usbmonitor.stop    = false;

      ret = kthread_create("USB Monitor", CONFIG_USBMONITOR_PRIORITY,
                           CONFIG_USBMONITOR_STACKSIZE,
                           (main_t)usbmonitor_daemon,
                           (FAR char * const *)NULL);
      if (ret < 0)
        {
          uerr("ERROR: Failed to start the USB monitor: %d\n",
               ret);
        }
      else
        {
          g_usbmonitor.pid = ret;
          uinfo("Started: %d\n", g_usbmonitor.pid);
          ret = OK;
        }

      sched_unlock();
      return ret;
    }

  sched_unlock();
  uinfo("%s: %d\n",
        g_usbmonitor.stop ? "Stopping" : "Running", g_usbmonitor.pid);
  return OK;
}

/****************************************************************************
 * Name: usbmonitor_stop
 *
 *   Stop the USB monitor kernel daemon.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

int usbmonitor_stop(void)
{
  /* Has the monitor already started? */

  if (g_usbmonitor.started)
    {
      /* Stop the USB monitor.  The next time the monitor wakes up,
       * it will see the stop indication and will exist.
       */

      uinfo("Stopping: %d\n", g_usbmonitor.pid);
      g_usbmonitor.stop = true;

#ifdef CONFIG_USBDEV_TRACE
      /* We may as well disable tracing since there is no listener */

      usbtrace_enable(0);
#endif
    }

  uinfo("Stopped: %d\n", g_usbmonitor.pid);
  return 0;
}

#endif /* CONFIG_USBMONITOR */
