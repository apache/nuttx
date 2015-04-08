/****************************************************************************
 * arch/sh/src/common/up_assert.c
 *
 *   Copyright (C) 2008-2009, 2012-2015 Gregory Nutt. All rights reserved.
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

/* Output debug info if stack dump is selected -- even if debug is not
 * selected.
 */

#ifdef CONFIG_ARCH_STACKDUMP
# undef  CONFIG_DEBUG
# undef  CONFIG_DEBUG_VERBOSE
# define CONFIG_DEBUG 1
# define CONFIG_DEBUG_VERBOSE 1
#endif

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/usb/usbdev_trace.h>

#include <nuttx/usb/usbdev_trace.h>

#include "up_arch.h"
#include "sched/sched.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USB trace dumping */

#ifndef CONFIG_USBDEV_TRACE
#  undef CONFIG_ARCH_USBDUMP
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _up_assert
 ****************************************************************************/

static void _up_assert(int errorcode) noreturn_function;
static void _up_assert(int errorcode)
{
  /* Are we in an interrupt handler or the idle task? */

  if (current_regs || ((struct tcb_s*)g_readytorun.head)->pid == 0)
    {
       (void)irqsave();
        for (;;)
          {
#ifdef CONFIG_ARCH_LEDS
            board_led_on(LED_PANIC);
            up_mdelay(250);
            board_led_off(LED_PANIC);
            up_mdelay(250);
#endif
          }
    }
  else
    {
      exit(errorcode);
    }
}

/****************************************************************************
 * Name: assert_tracecallback
 ****************************************************************************/

#ifdef CONFIG_ARCH_USBDUMP
static int usbtrace_syslog(FAR const char *fmt, ...)
{
  va_list ap;
  int ret;

  /* Let vsyslog do the real work */

  va_start(ap, fmt);
  ret = lowvsyslog(LOG_INFO, fmt, ap);
  va_end(ap);
  return ret;
}

static int assert_tracecallback(FAR struct usbtrace_s *trace, FAR void *arg)
{
  usbtrace_trprintf(usbtrace_syslog, trace->event, trace->value);
  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_assert
 ****************************************************************************/

void up_assert(const uint8_t *filename, int lineno)
{
#if CONFIG_TASK_NAME_SIZE > 0 && defined(CONFIG_DEBUG)
  struct tcb_s *rtcb = (struct tcb_s*)g_readytorun.head;
#endif

  board_led_on(LED_ASSERTION);

#if CONFIG_TASK_NAME_SIZE > 0
  lldbg("Assertion failed at file:%s line: %d task: %s\n",
        filename, lineno, rtcb->name);
#else
  lldbg("Assertion failed at file:%s line: %d\n",
        filename, lineno);
#endif

  up_dumpstate();

#ifdef CONFIG_ARCH_USBDUMP
  /* Dump USB trace data */

  (void)usbtrace_enumerate(assert_tracecallback, NULL);
#endif

#ifdef CONFIG_BOARD_CRASHDUMP
  board_crashdump(up_getsp(), g_readytorun.head, filename, lineno);
#endif

  _up_assert(EXIT_FAILURE);
}
