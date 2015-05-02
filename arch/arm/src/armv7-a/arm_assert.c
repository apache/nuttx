/****************************************************************************
 * arch/arm/src/armv7-a/arm_assert.c
 *
 *   Copyright (C) 2013-2015 Gregory Nutt. All rights reserved.
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

/* The following is just intended to keep some ugliness out of the mainline
 * code.  We are going to print the task name if:
 *
 *  CONFIG_TASK_NAME_SIZE > 0 &&    <-- The task has a name
 *  (defined(CONFIG_DEBUG)    ||    <-- And the debug is enabled (lldbg used)
 *   defined(CONFIG_ARCH_STACKDUMP) <-- Or lowsyslog() is used
 */

#undef CONFIG_PRINT_TASKNAME
#if CONFIG_TASK_NAME_SIZE > 0 && (defined(CONFIG_DEBUG) || defined(CONFIG_ARCH_STACKDUMP))
#  define CONFIG_PRINT_TASKNAME 1
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getsp
 ****************************************************************************/

/* I don't know if the builtin to get SP is enabled */

static inline uint32_t up_getsp(void)
{
  uint32_t sp;
  __asm__
  (
    "\tmov %0, sp\n\t"
    : "=r"(sp)
  );
  return sp;
}

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
      lldbg("%08x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}
#else
#  define up_stackdump(sp,stack_base)
#endif

/****************************************************************************
 * Name: up_taskdump
 ****************************************************************************/

#ifdef CONFIG_STACK_COLORATION
static void up_taskdump(FAR struct tcb_s *tcb, FAR void *arg)
{
  /* Dump interesting properties of this task */

#ifdef CONFIG_PRINT_TASKNAME
  lldbg("%s: PID=%d Stack Used=%lu of %lu\n",
        tcb->name, tcb->pid, (unsigned long)up_check_tcbstack(tcb),
        (unsigned long)tcb->adj_stack_size);
#else
  lldbg("PID: %d Stack Used=%lu of %lu\n",
        tcb->pid, (unsigned long)up_check_tcbstack(tcb),
        (unsigned long)tcb->adj_stack_size);
#endif
}
#endif

/****************************************************************************
 * Name: up_showtasks
 ****************************************************************************/

#ifdef CONFIG_STACK_COLORATION
static inline void up_showtasks(void)
{
  /* Dump interesting properties of each task in the crash environment */

  sched_foreach(up_taskdump, NULL);
}
#else
#  define up_showtasks()
#endif

/****************************************************************************
 * Name: up_registerdump
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static inline void up_registerdump(void)
{
  /* Are user registers available from interrupt processing? */

  if (current_regs)
    {
      int regs;

      /* Yes.. dump the interrupt registers */

      for (regs = REG_R0; regs <= REG_R15; regs += 8)
        {
          uint32_t *ptr = (uint32_t*)&current_regs[regs];
          lldbg("R%d: %08x %08x %08x %08x %08x %08x %08x %08x\n",
                 regs, ptr[0], ptr[1], ptr[2], ptr[3],
                 ptr[4], ptr[5], ptr[6], ptr[7]);
        }

      lldbg("CPSR: %08x\n", current_regs[REG_CPSR]);
    }
}
#else
# define up_registerdump()
#endif

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
 * Name: up_dumpstate
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void up_dumpstate(void)
{
  struct tcb_s *rtcb = (struct tcb_s*)g_readytorun.head;
  uint32_t sp   = up_getsp();
  uint32_t ustackbase;
  uint32_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  uint32_t istackbase;
  uint32_t istacksize;
#endif
#ifdef CONFIG_ARCH_KERNEL_STACK
  uint32_t kstackbase = 0;
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

  lldbg("Current sp: %08x\n", sp);

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  /* Get the limits on the interrupt stack memory */

  istackbase = (uint32_t)&g_intstackbase;
  istacksize = (CONFIG_ARCH_INTERRUPTSTACK & ~3);

  /* Show interrupt stack info */

  lldbg("Interrupt stack:\n");
  lldbg("  base: %08x\n", istackbase);
  lldbg("  size: %08x\n", istacksize);
#ifdef CONFIG_STACK_COLORATION
  lldbg("  used: %08x\n", up_check_intstack());
#endif
#endif

  /* Show user stack info */

  lldbg("User stack:\n");
  lldbg("  base: %08x\n", ustackbase);
  lldbg("  size: %08x\n", ustacksize);
#ifdef CONFIG_STACK_COLORATION
  lldbg("  used: %08x\n", up_check_tcbstack(rtcb));
#endif

#ifdef CONFIG_ARCH_KERNEL_STACK
  /* This this thread have a kernel stack allocated? */

  if (rtcb->xcp.kstack)
    {
      kstackbase = (uint32_t)rtcb->xcp.kstack + CONFIG_ARCH_KERNEL_STACKSIZE - 4;

      lldbg("Kernel stack:\n");
      lldbg("  base: %08x\n", kstackbase);
      lldbg("  size: %08x\n", CONFIG_ARCH_KERNEL_STACKSIZE);
    }
#endif

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  /* Does the current stack pointer lie within the interrupt stack? */

  if (sp > istackbase - istacksize && sp < istackbase)
    {
      /* Yes.. dump the interrupt stack */

      lldbg("Interrupt Stack\n", sp);
      up_stackdump(sp, istackbase);

      /* Extract the user stack pointer which should lie
       * at the base of the interrupt stack.
       */

      sp = g_intstackbase;
      lldbg("User sp: %08x\n", sp);
    }
#endif

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp > ustackbase - ustacksize && sp < ustackbase)
    {
      lldbg("User Stack\n", sp);
      up_stackdump(sp, ustackbase);
    }

#ifdef CONFIG_ARCH_KERNEL_STACK
  /* Dump the user stack if the stack pointer lies within the allocated
   * kernel stack memory.
   */

  if (sp >= (uint32_t)rtcb->xcp.kstack && sp < kstackbase)
    {
      lldbg("Kernel Stack\n", sp);
      up_stackdump(sp, kstackbase);
    }
#endif

  /* Then dump the registers (if available) */

  up_registerdump();

  /* Dump the state of all tasks (if available) */

  up_showtasks();

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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_assert
 ****************************************************************************/

void up_assert(const uint8_t *filename, int lineno)
{
#ifdef CONFIG_PRINT_TASKNAME
  struct tcb_s *rtcb = (struct tcb_s*)g_readytorun.head;
#endif
  board_led_on(LED_ASSERTION);

#ifdef CONFIG_PRINT_TASKNAME
  lldbg("Assertion failed at file:%s line: %d task: %s\n",
        filename, lineno, rtcb->name);
#else
  lldbg("Assertion failed at file:%s line: %d\n",
        filename, lineno);
#endif
  up_dumpstate();

#ifdef CONFIG_BOARD_CRASHDUMP
  board_crashdump(up_getsp(), g_readytorun.head, filename, lineno);
#endif

  _up_assert(EXIT_FAILURE);
}
