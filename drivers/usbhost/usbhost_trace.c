/****************************************************************************
 * drivers/usbhost/usbhost_trace.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/usb/usbhost_trace.h>
#undef usbtrace

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_USBHOST_TRACE_NRECORDS
#  define CONFIG_USBHOST_TRACE_NRECORDS 128
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

#ifdef CONFIG_USBHOST_TRACE
static uint32_t g_trace[CONFIG_USBHOST_TRACE_NRECORDS];
static volatile uint16_t g_head = 0;
static volatile uint16_t g_tail = 0;
static volatile bool g_disabled = false;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_trsyslog
 *
 * Description:
 *  Dump trace data to the syslog()
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_TRACE
static int usbhost_trsyslog(uint32_t event, FAR void *arg)
{
  FAR const char *fmt;
  uint16_t id;

  /* Decode the trace data */

  id = TRACE_DECODE_NDX(event);

  /* Get the format associated with the trace.  Try the one argument format
   * first.
   */

  fmt = usbhost_trformat1(id);
  if (fmt)
    {
      /* Just print the data using syslog() */

      syslog(LOG_INFO, fmt, (unsigned int)TRACE_DECODE_U23(event));
    }

  /* No, then it must the two argument format first. */

  else
    {
      fmt = usbhost_trformat2(id);
      DEBUGASSERT(fmt);
      if (fmt)
        {
          /* Just print the data using syslog() */

          syslog(LOG_INFO, fmt, (unsigned int)TRACE_DECODE_U7(event),
                 (unsigned int)TRACE_DECODE_U16(event));
        }
    }

  return OK;
}
#endif /* CONFIG_USBHOST_TRACE */

/****************************************************************************
 * Name: usbhost_trace_common
 *
 * Description:
 *  Record a USB event (tracing or USB debug must be enabled)
 *
 * Assumptions:
 *   May be called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_TRACE
void usbhost_trace_common(uint32_t event)
{
  irqstate_t flags;

  /* Check if tracing is enabled for this ID */

  flags = enter_critical_section();
  if (!g_disabled)
    {
      /* Yes... save the new trace data at the head */

      g_trace[g_head] = event;

      /* Increment the head and (probably) the tail index */

      if (++g_head >= CONFIG_USBHOST_TRACE_NRECORDS)
        {
          g_head = 0;
        }

      if (g_head == g_tail)
        {
          if (++g_tail >= CONFIG_USBHOST_TRACE_NRECORDS)
            {
              g_tail = 0;
            }
        }
    }
  leave_critical_section(flags);
}
#endif /* CONFIG_USBHOST_TRACE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_trace1 and usbhost_trace2
 *
 * Description:
 *  Record a USB event (tracing or USB debug must be enabled)
 *
 * Assumptions:
 *   May be called from an interrupt handler
 *
 ****************************************************************************/

#if defined(CONFIG_USBHOST_TRACE) || \
   (defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_USB))

void usbhost_trace1(uint16_t id, uint32_t u23)
{
#ifdef CONFIG_USBHOST_TRACE
  usbhost_trace_common(TRACE_ENCODE1(id, u23));
#else
  FAR const char *fmt;

  /* Get the format associated with the trace */

  fmt = usbhost_trformat1(id);
  DEBUGASSERT(fmt);

  /* Just print the data using syslog() */

  syslog(LOG_INFO, fmt, (unsigned int)u23);
#endif
}

void usbhost_trace2(uint16_t id, uint8_t u7, uint16_t u16)
{
#ifdef CONFIG_USBHOST_TRACE
  usbhost_trace_common(TRACE_ENCODE2(id, u7, u16));
#else
  FAR const char *fmt;

  /* Get the format associated with the trace */

  fmt = usbhost_trformat2(id);
  DEBUGASSERT(fmt);

  /* Just print the data using syslog() */

  syslog(LOG_INFO, fmt, u7, u16);
#endif
}

#endif /* CONFIG_USBHOST_TRACE || CONFIG_DEBUG_FEATURES && CONFIG_DEBUG_USB */

/****************************************************************************
 * Name: usbtrace_enumerate
 *
 * Description:
 *   Enumerate all buffer trace data (will temporarily disable tracing)
 *
 * Assumptions:
 *   NEVER called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_TRACE
int usbhost_trenumerate(usbhost_trcallback_t callback, FAR void *arg)
{
  uint16_t ndx;
  int ret = OK;

  /* Temporarily disable tracing to avoid conflicts */

  g_disabled = true;

  /* Visit every entry, starting with the tail */

  for (ndx = g_tail; ndx != g_head; )
    {
      /* Call the user provided callback */

      ret = callback(g_trace[ndx], arg);
      if (ret != OK)
        {
          /* Abort the enumeration */

          break;
        }

      /* Increment the index */

      if (++ndx >= CONFIG_USBHOST_TRACE_NRECORDS)
        {
          ndx = 0;
        }
    }

  /* Discard the trace data after it has been reported */

  g_tail = g_head;

  /* Restore tracing state */

  g_disabled = false;
  return ret;
}
#endif /* CONFIG_USBHOST_TRACE */

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
int usbhost_trdump(void)
{
  return usbhost_trenumerate(usbhost_trsyslog, NULL);
}
#endif /* CONFIG_USBHOST_TRACE */
