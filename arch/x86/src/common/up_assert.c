/****************************************************************************
 * arch/x86/src/common/up_assert.c
 *
 *   Copyright (C) 2011-2016, 2018 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/arch.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/usb/usbdev_trace.h>

#include <arch/board/board.h>

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

#ifndef CONFIG_BOARD_RESET_ON_ASSERT
#  define CONFIG_BOARD_RESET_ON_ASSERT 0
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_stackdump
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void up_stackdump(uint32_t sp, uint32_t stack_base)
{
  uint32_t stack ;

  for (stack = sp & ~0x1f; stack < stack_base; stack += 32)
    {
      uint32_t *ptr = (uint32_t*)stack;
      _alert("%08x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
            stack, ptr[0], ptr[1], ptr[2], ptr[3],
            ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}
#else
# define up_stackdump()
#endif

/****************************************************************************
 * Name: assert_tracecallback
 ****************************************************************************/

#ifdef CONFIG_ARCH_USBDUMP
static int usbtrace_syslog(FAR const char *fmt, ...)
{
  va_list ap;
  int ret;

  /* Let nx_vsyslog do the real work */

  va_start(ap, fmt);
  ret = nx_vsyslog(LOG_EMERG, fmt, &ap);
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
 * Name: up_dumpstate
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void up_dumpstate(void)
{
  struct tcb_s *rtcb = this_task();
  uint32_t sp = up_getsp();
  uint32_t ustackbase;
  uint32_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  uint32_t istackbase;
  uint32_t istacksize;
#endif

  /* Get the limits on the user stack memory */

  if (rtcb->pid == 0)
    {
      ustackbase = g_idle_topstack - 4;
      ustacksize = CONFIG_IDLETHREAD_STACKSIZE;
    }
  else
    {
      ustackbase = (uint32_t)rtcb->adj_stack_ptr;
      ustacksize = (uint32_t)rtcb->adj_stack_size;
    }

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  istackbase = (uint32_t)&g_intstackbase;
  istacksize = (CONFIG_ARCH_INTERRUPTSTACK & ~3) - 4;

  /* Show interrupt stack info */

  _alert("sp:     %08x\n", sp);
  _alert("IRQ stack:\n");
  _alert("  base: %08x\n", istackbase);
  _alert("  size: %08x\n", istacksize);

  /* Does the current stack pointer lie within the interrupt
   * stack?
   */

  if (sp <= istackbase && sp > istackbase - istacksize)
    {
      /* Yes.. dump the interrupt stack */

      up_stackdump(sp, istackbase);

      /* Extract the user stack pointer which should lie
       * at the base of the interrupt stack.
       */

      sp = g_intstackbase;
      _alert("sp:     %08x\n", sp);
    }

  /* Show user stack info */

  _alert("User stack:\n");
  _alert("  base: %08x\n", ustackbase);
  _alert("  size: %08x\n", ustacksize);
#else
  _alert("sp:         %08x\n", sp);
  _alert("stack base: %08x\n", ustackbase);
  _alert("stack size: %08x\n", ustacksize);
#endif

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp > ustackbase || sp <= ustackbase - ustacksize)
    {
#if !defined(CONFIG_ARCH_INTERRUPTSTACK) || CONFIG_ARCH_INTERRUPTSTACK < 4
      _alert("ERROR: Stack pointer is not within allocated stack\n");
#endif
    }
  else
    {
      up_stackdump(sp, ustackbase);
    }

  /* Then dump the registers (if available) */

  if (g_current_regs != NULL)
    {
      up_registerdump((uint32_t*)g_current_regs);
    }

#ifdef CONFIG_ARCH_USBDUMP
  /* Dump USB trace data */

  (void)usbtrace_enumerate(assert_tracecallback, NULL);
#endif
}
#else
# define up_dumpstate()
#endif

/****************************************************************************
 * Name: _up_assert
 ****************************************************************************/

static void _up_assert(int errorcode) noreturn_function;
static void _up_assert(int errorcode)
{
  /* Flush any buffered SYSLOG data */

  (void)syslog_flush();

  /* Are we in an interrupt handler or the idle task? */

  if (g_current_regs || (this_task())->pid == 0)
    {
       (void)up_irq_save();
        for (;;)
          {
#if CONFIG_BOARD_RESET_ON_ASSERT >= 1
            board_reset(0);
#endif
#ifdef CONFIG_ARCH_LEDS
            board_autoled_on(LED_PANIC);
            up_mdelay(250);
            board_autoled_off(LED_PANIC);
            up_mdelay(250);
#endif
          }
    }
  else
    {
#if CONFIG_BOARD_RESET_ON_ASSERT >= 2
      board_reset(0);
#endif
      exit(errorcode);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_assert
 ****************************************************************************/

void up_assert(const uint8_t *filename, int lineno)
{
#if CONFIG_TASK_NAME_SIZE > 0 && defined(CONFIG_DEBUG_ALERT)
  struct tcb_s *rtcb = this_task();
#endif

  board_autoled_on(LED_ASSERTION);

  /* Flush any buffered SYSLOG data (from prior to the assertion) */

  (void)syslog_flush();

#if CONFIG_TASK_NAME_SIZE > 0
  _alert("Assertion failed at file:%s line: %d task: %s\n",
        filename, lineno, rtcb->name);
#else
  _alert("Assertion failed at file:%s line: %d\n",
        filename, lineno);
#endif

  up_dumpstate();

  /* Flush any buffered SYSLOG data (from the above) */

  (void)syslog_flush();

#ifdef CONFIG_BOARD_CRASHDUMP
  board_crashdump(up_getsp(), this_task(), filename, lineno);
#endif

  _up_assert(EXIT_FAILURE);
}
