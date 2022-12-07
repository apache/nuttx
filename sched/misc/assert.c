/****************************************************************************
 * sched/misc/assert.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <nuttx/panic_notifier.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/syslog/syslog.h>

#include <assert.h>
#include <debug.h>
#include <stdlib.h>

#include "irq/irq.h"
#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_BOARD_RESET_ON_ASSERT
#  define CONFIG_BOARD_RESET_ON_ASSERT 0
#endif

/* USB trace dumping */

#ifndef CONFIG_USBDEV_TRACE
#  undef CONFIG_ARCH_USBDUMP
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: assert_tracecallback
 ****************************************************************************/

#ifdef CONFIG_ARCH_USBDUMP
static int usbtrace_syslog(FAR const char *fmt, ...)
{
  va_list ap;

  /* Let vsyslog do the real work */

  va_start(ap, fmt);
  vsyslog(LOG_EMERG, fmt, ap);
  va_end(ap);
  return OK;
}

static int assert_tracecallback(FAR struct usbtrace_s *trace, FAR void *arg)
{
  usbtrace_trprintf(usbtrace_syslog, trace->event, trace->value);
  return 0;
}
#endif

/****************************************************************************
 * Name: assert_end
 ****************************************************************************/

static void assert_end(void)
{
  /* Flush any buffered SYSLOG data */

  syslog_flush();

  /* Are we in an interrupt handler or the idle task? */

  if (up_interrupt_context() || running_task()->flink == NULL)
    {
#if CONFIG_BOARD_RESET_ON_ASSERT >= 1
      board_reset(CONFIG_BOARD_ASSERT_RESET_VALUE);
#endif

      /* Disable interrupts on this CPU */

      up_irq_save();

#ifdef CONFIG_SMP
      /* Try (again) to stop activity on other CPUs */

      spin_trylock(&g_cpu_irqlock);
#endif

      for (; ; )
        {
          up_mdelay(250);
        }
    }
  else
    {
#if CONFIG_BOARD_RESET_ON_ASSERT >= 2
      board_reset(CONFIG_BOARD_ASSERT_RESET_VALUE);
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void _assert(FAR const char *filename, int linenum)
{
  /* Flush any buffered SYSLOG data (from prior to the assertion) */

  syslog_flush();

#if CONFIG_BOARD_RESET_ON_ASSERT < 2
  if (!up_interrupt_context() && running_task()->flink != NULL)
    {
      panic_notifier_call_chain(PANIC_TASK, NULL);
    }
  else
#endif
    {
      panic_notifier_call_chain(PANIC_KERNEL, NULL);
    }

#ifdef CONFIG_SMP
#  if CONFIG_TASK_NAME_SIZE > 0
  _alert("Assertion failed CPU%d at file:%s line: %d task: %s\n",
         up_cpu_index(), filename, linenum, running_task()->name);
#  else
  _alert("Assertion failed CPU%d at file:%s line: %d\n",
         up_cpu_index(), filename, linenum);
#  endif
#else
#  if CONFIG_TASK_NAME_SIZE > 0
  _alert("Assertion failed at file:%s line: %d task: %s\n",
         filename, linenum, running_task()->name);
#  else
  _alert("Assertion failed at file:%s line: %d\n",
         filename, linenum);
#  endif
#endif

#ifdef CONFIG_ARCH_USBDUMP
  /* Dump USB trace data */

  usbtrace_enumerate(assert_tracecallback, NULL);
#endif

#ifdef CONFIG_BOARD_CRASHDUMP
  board_crashdump(up_getsp(), running_task(), filename, linenum);
#endif

  /* Flush any buffered SYSLOG data (from the above) */

  syslog_flush();

  up_assert(filename, linenum);
  assert_end();
  exit(EXIT_FAILURE);
}
