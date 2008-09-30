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

#include <arch/irq.h>
#include <nuttx/usbdev_trace.h>

#ifdef CONFIG_USBDEV_TRACE

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

static uint16 g_disablecount = 0; /* != 0 means tracing is disabled */
static struct usbtrace_s g_trace[CONFIG_USBDEV_TRACE_NRECORDS];
static uint16 g_head = 0;
static uint16 g_tail = 0;

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
 *   Enable/disable tracing
 *
 * Assumptions:
 * - Initial state is enabled
 * - May be called from an interrupt handler
 *
 *******************************************************************************/

void usbtrace_enable(boolean enable)
{
  irqstate_t flags;

  flags = irqsave();
  if (enable)
    {
      if (g_disablecount < 0xffff)
        {
          g_disablecount++;
        }
    }
  else if (g_disablecount > 0)
    {
      g_disablecount--;
    }
  irqrestore(flags);
}

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

void usbtrace(uint16 event, uint16 value)
{
  irqstate_t flags;

  /* Check if tracing is enabled */

  flags = irqsave();
  if (!g_disablecount)
    {
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
    }
  irqrestore(flags);
}

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
