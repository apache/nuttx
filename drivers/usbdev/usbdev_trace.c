/****************************************************************************
 * drivers/usbdev/usbdev_trace.c
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
#include <stdarg.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/usb/usbdev_trace.h>

#undef usbtrace

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_USBDEV_TRACE_NRECORDS
#  define CONFIG_USBDEV_TRACE_NRECORDS 128
#endif

#ifndef CONFIG_USBDEV_TRACE_INITIALIDSET
#  define CONFIG_USBDEV_TRACE_INITIALIDSET 0
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
static uint16_t g_head = 0;
static uint16_t g_tail = 0;
#endif

#if defined(CONFIG_USBDEV_TRACE) || (defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_USB))
static usbtrace_idset_t g_maskedidset = CONFIG_USBDEV_TRACE_INITIALIDSET;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbtrace_syslog
 ****************************************************************************/

#if !defined(CONFIG_USBDEV_TRACE) && \
    (defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_USB))
static int usbtrace_syslog(const char *fmt, ...)
{
  va_list ap;

  /* Let vsyslog do the real work */

  va_start(ap, fmt);
  vsyslog(LOG_INFO, fmt, ap);
  va_end(ap);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbtrace_enable
 *
 * Description:
 *  Enable/disable tracing per trace ID.  The initial state is all IDs
 *  enabled.
 *
 * Input Parameters:
 *  idset - The bitset of IDs to be masked.  TRACE_ALLIDS enables all IDS;
 *  zero masks all IDs.
 *
 * Returned Value:
 *  The previous idset value.
 *
 * Assumptions:
 * - May be called from an interrupt handler
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_TRACE) || \
   (defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_USB))
usbtrace_idset_t usbtrace_enable(usbtrace_idset_t idset)
{
  irqstate_t flags;
  usbtrace_idset_t ret;

  /* The following read and write must be atomic */

  flags         = enter_critical_section();
  ret           = g_maskedidset;
  g_maskedidset = idset;
  leave_critical_section(flags);
  return ret;
}
#endif /* CONFIG_USBDEV_TRACE || CONFIG_DEBUG_FEATURES && CONFIG_DEBUG_USB */

/****************************************************************************
 * Name: usbtrace
 *
 * Description:
 *  Record a USB event (tracing must be enabled)
 *
 * Assumptions:
 *   May be called from an interrupt handler
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_TRACE) || (defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_USB))
void usbtrace(uint16_t event, uint16_t value)
{
  irqstate_t flags;

  /* Check if tracing is enabled for this ID */

  flags = enter_critical_section();
  if ((g_maskedidset & TRACE_ID2BIT(event)) != 0)
    {
#ifdef CONFIG_USBDEV_TRACE
      /* Yes... save the new trace data at the head */

      g_trace[g_head].event = event;
      g_trace[g_head].value = value;

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
      /* Just print the data using syslog */

      usbtrace_trprintf(usbtrace_syslog, event, value);
#endif
    }

  leave_critical_section(flags);
}
#endif /* CONFIG_USBDEV_TRACE || CONFIG_DEBUG_FEATURES && CONFIG_DEBUG_USB */

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

#ifdef CONFIG_USBDEV_TRACE
int usbtrace_enumerate(trace_callback_t callback, void *arg)
{
  uint16_t ndx;
  uint32_t idset;
  int ret = OK;

  /* Temporarily disable tracing */

  idset = usbtrace_enable(0);

  /* Visit every entry, starting with the tail */

  for (ndx = g_tail; ndx != g_head; )
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

  /* Discard the trace data after it has been reported */

  g_tail = g_head;

  /* Restore tracing state */

  usbtrace_enable(idset);
  return ret;
}
#endif /* CONFIG_USBDEV_TRACE */
