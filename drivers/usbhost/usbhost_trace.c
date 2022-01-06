/****************************************************************************
 * drivers/usbhost/usbhost_trace.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>
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
  if (fmt == NULL)
    {
      return;
    }

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
  if (fmt == NULL)
    {
      return;
    }

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
