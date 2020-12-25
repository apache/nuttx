/****************************************************************************
 * arch/arm/src/armv7-a/arm_assert.c
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

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/usb/usbdev_trace.h>

#include <arch/board/board.h>

#include "sched/sched.h"
#include "irq/irq.h"

#include "arm_arch.h"
#include "arm_internal.h"
#include "chip.h"

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
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static uint32_t s_last_regs[XCPTCONTEXT_REGS];
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
      uint32_t *ptr = (uint32_t *)stack;
      _alert("%08x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
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

#if CONFIG_TASK_NAME_SIZE > 0
  _alert("%s: PID=%d Stack Used=%lu of %lu\n",
        tcb->name, tcb->pid, (unsigned long)up_check_tcbstack(tcb),
        (unsigned long)tcb->adj_stack_size);
#else
  _alert("PID: %d Stack Used=%lu of %lu\n",
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

  nxsched_foreach(up_taskdump, NULL);
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
  volatile uint32_t *regs = CURRENT_REGS;
  int reg;

  /* Are user registers available from interrupt processing? */

  if (regs == NULL)
    {
      /* No.. capture user registers by hand */

      arm_saveusercontext(s_last_regs);
      regs = s_last_regs;
    }

  /* Dump the interrupt registers */

  for (reg = REG_R0; reg <= REG_R15; reg += 8)
    {
      uint32_t *ptr = (uint32_t *)&regs[reg];
      _alert("R%d: %08x %08x %08x %08x %08x %08x %08x %08x\n",
             reg, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }

  _alert("CPSR: %08x\n", regs[REG_CPSR]);
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
 * Name: up_dumpstate
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void up_dumpstate(void)
{
  struct tcb_s *rtcb = running_task();
  uint32_t sp   = arm_getsp();
  uint32_t ustackbase;
  uint32_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 7
  uint32_t istackbase;
  uint32_t istacksize;
#endif
#ifdef CONFIG_ARCH_KERNEL_STACK
  uint32_t kstackbase = 0;
#endif

  /* Dump the CPU registers (if available) */

  up_registerdump();

  /* Get the limits on the user stack memory */

  ustackbase = (uint32_t)rtcb->adj_stack_ptr;
  ustacksize = (uint32_t)rtcb->adj_stack_size;

  _alert("Current sp: %08x\n", sp);

#if CONFIG_ARCH_INTERRUPTSTACK > 7
  /* Get the limits on the interrupt stack memory */

#ifdef CONFIG_SMP
  istackbase = (uint32_t)arm_intstack_base();
#else
  istackbase = (uint32_t)&g_intstackbase;
#endif
  istacksize = (CONFIG_ARCH_INTERRUPTSTACK & ~7);

  /* Show interrupt stack info */

  _alert("Interrupt stack:\n");
  _alert("  base: %08x\n", istackbase);
  _alert("  size: %08x\n", istacksize);
#ifdef CONFIG_STACK_COLORATION
  _alert("  used: %08x\n", up_check_intstack());
#endif
#endif

  /* Show user stack info */

  _alert("User stack:\n");
  _alert("  base: %08x\n", ustackbase);
  _alert("  size: %08x\n", ustacksize);
#ifdef CONFIG_STACK_COLORATION
  _alert("  used: %08x\n", up_check_tcbstack(rtcb));
#endif

#ifdef CONFIG_ARCH_KERNEL_STACK
  /* Does this thread have a kernel stack allocated? */

  if (rtcb->xcp.kstack)
    {
      kstackbase = (uint32_t)rtcb->xcp.kstack +
                   CONFIG_ARCH_KERNEL_STACKSIZE;

      _alert("Kernel stack:\n");
      _alert("  base: %08x\n", kstackbase);
      _alert("  size: %08x\n", CONFIG_ARCH_KERNEL_STACKSIZE);
    }
#endif

#if CONFIG_ARCH_INTERRUPTSTACK > 7
  /* Does the current stack pointer lie within the interrupt stack? */

  if (sp > istackbase - istacksize && sp < istackbase)
    {
      /* Yes.. dump the interrupt stack */

      _alert("Interrupt Stack\n", sp);
      up_stackdump(sp, istackbase);
    }
  else if (CURRENT_REGS)
    {
      _alert("ERROR: Stack pointer is not within the interrupt stack\n");
      up_stackdump(istackbase - istacksize, istackbase);
    }
#endif

  /* Extract the user stack pointer if we are in an interrupt handler.
   * If we are not in an interrupt handler.  Then sp is the user stack
   * pointer (and the above range check should have failed).
   */

  if (CURRENT_REGS)
    {
      sp = CURRENT_REGS[REG_R13];
      _alert("User sp: %08x\n", sp);
    }

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp > ustackbase - ustacksize && sp < ustackbase)
    {
      _alert("User Stack\n", sp);
      up_stackdump(sp, ustackbase);
    }

#ifdef CONFIG_ARCH_KERNEL_STACK
  /* Dump the user stack if the stack pointer lies within the allocated
   * kernel stack memory.
   */

  else if (sp >= (uint32_t)rtcb->xcp.kstack && sp < kstackbase)
    {
      _alert("Kernel Stack\n", sp);
      up_stackdump(sp, kstackbase);
    }
#endif
  else
    {
      _alert("ERROR: Stack pointer is not within the allocated stack\n");
      up_stackdump(ustackbase - ustacksize, ustackbase);
#ifdef CONFIG_ARCH_KERNEL_STACK
      up_stackdump((uint32_t)rtcb->xcp.kstack, kstackbase);
#endif
    }

#ifdef CONFIG_SMP
  /* Show the CPU number */

  _alert("CPU%d:\n", up_cpu_index());
#endif

  /* Dump the state of all tasks (if available) */

  up_showtasks();

#ifdef CONFIG_ARCH_USBDUMP
  /* Dump USB trace data */

  usbtrace_enumerate(assert_tracecallback, NULL);
#endif
}
#else
# define up_dumpstate()
#endif

/****************************************************************************
 * Name: _up_assert
 ****************************************************************************/

static void _up_assert(void)
{
  /* Flush any buffered SYSLOG data */

  syslog_flush();

  /* Are we in an interrupt handler or the idle task? */

  if (CURRENT_REGS || running_task()->flink == NULL)
    {
      /* Disable interrupts on this CPU */

      up_irq_save();

      for (; ; )
        {
#ifdef CONFIG_SMP
          /* Try (again) to stop activity on other CPUs */

          spin_trylock(&g_cpu_irqlock);
#endif

#if CONFIG_BOARD_RESET_ON_ASSERT >= 1
          board_reset(CONFIG_BOARD_ASSERT_RESET_VALUE);
#endif
#ifdef CONFIG_ARCH_LEDS
          /* FLASH LEDs a 2Hz */

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
      board_reset(CONFIG_BOARD_ASSERT_RESET_VALUE);
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_assert
 ****************************************************************************/

void up_assert(const char *filename, int lineno)
{
#if CONFIG_TASK_NAME_SIZE > 0 && defined(CONFIG_DEBUG_ALERT)
  struct tcb_s *rtcb = running_task();
#endif

  board_autoled_on(LED_ASSERTION);

  /* Flush any buffered SYSLOG data (prior to the assertion) */

  syslog_flush();

#ifdef CONFIG_SMP
#if CONFIG_TASK_NAME_SIZE > 0
  _alert("Assertion failed CPU%d at file:%s line: %d task: %s\n",
        up_cpu_index(), filename, lineno, rtcb->name);
#else
  _alert("Assertion failed CPU%d at file:%s line: %d\n",
        up_cpu_index(), filename, lineno);
#endif
#else
#if CONFIG_TASK_NAME_SIZE > 0
  _alert("Assertion failed at file:%s line: %d task: %s\n",
        filename, lineno, rtcb->name);
#else
  _alert("Assertion failed at file:%s line: %d\n",
        filename, lineno);
#endif
#endif

  up_dumpstate();

  /* Flush any buffered SYSLOG data (from the above) */

  syslog_flush();

#ifdef CONFIG_BOARD_CRASHDUMP
  board_crashdump(arm_getsp(), running_task(), filename, lineno);
#endif

  _up_assert();
}
