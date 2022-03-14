/****************************************************************************
 * arch/arm/src/common/arm_assert.c
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
#include "arm_internal.h"

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

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_stackdump
 ****************************************************************************/

static void arm_stackdump(uint32_t sp, uint32_t stack_top)
{
  uint32_t stack;

  /* Flush any buffered SYSLOG data to avoid overwrite */

  syslog_flush();

  for (stack = sp & ~0x1f; stack < (stack_top & ~0x1f); stack += 32)
    {
      uint32_t *ptr = (uint32_t *)stack;
      _alert("%08x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}

/****************************************************************************
 * Name: arm_registerdump
 ****************************************************************************/

static void arm_registerdump(FAR volatile uint32_t *regs)
{
  /* Dump the interrupt registers */

  _alert("R0: %08x R1: %08x R2: %08x  R3: %08x\n",
         regs[REG_R0], regs[REG_R1], regs[REG_R2], regs[REG_R3]);
#ifdef CONFIG_ARM_THUMB
  _alert("R4: %08x R5: %08x R6: %08x  FP: %08x\n",
         regs[REG_R4], regs[REG_R5], regs[REG_R6], regs[REG_R7]);
  _alert("R8: %08x SB: %08x SL: %08x R11: %08x\n",
         regs[REG_R8], regs[REG_R9], regs[REG_R10], regs[REG_R11]);
#else
  _alert("R4: %08x R5: %08x R6: %08x  R7: %08x\n",
         regs[REG_R4], regs[REG_R5], regs[REG_R6], regs[REG_R7]);
  _alert("R8: %08x SB: %08x SL: %08x  FP: %08x\n",
         regs[REG_R8], regs[REG_R9], regs[REG_R10], regs[REG_R11]);
#endif
  _alert("IP: %08x SP: %08x LR: %08x  PC: %08x\n",
         regs[REG_R12], regs[REG_R13], regs[REG_R14], regs[REG_R15]);

#if defined(REG_BASEPRI)
  _alert("xPSR: %08x BASEPRI: %08x CONTROL: %08x\n",
         regs[REG_XPSR], regs[REG_BASEPRI], getcontrol());
#elif defined(REG_PRIMASK)
  _alert("xPSR: %08x PRIMASK: %08x CONTROL: %08x\n",
         regs[REG_XPSR], regs[REG_PRIMASK], getcontrol());
#elif defined(REG_CPSR)
  _alert("CPSR: %08x\n", regs[REG_CPSR]);
#endif

#ifdef REG_EXC_RETURN
  _alert("EXC_RETURN: %08x\n", regs[REG_EXC_RETURN]);
#endif
}

/****************************************************************************
 * Name: arm_dump_task
 ****************************************************************************/

static void arm_dump_task(FAR struct tcb_s *tcb, FAR void *arg)
{
#ifdef CONFIG_STACK_COLORATION
  uint32_t stack_filled = 0;
  uint32_t stack_used;
#endif
#ifdef CONFIG_SCHED_CPULOAD
  struct cpuload_s cpuload;
  uint32_t fracpart;
  uint32_t intpart;
  uint32_t tmp;

  clock_cpuload(tcb->pid, &cpuload);

  if (cpuload.total > 0)
    {
      tmp      = (1000 * cpuload.active) / cpuload.total;
      intpart  = tmp / 10;
      fracpart = tmp - 10 * intpart;
    }
  else
    {
      intpart  = 0;
      fracpart = 0;
    }
#endif

#ifdef CONFIG_STACK_COLORATION
  stack_used = up_check_tcbstack(tcb);
  if (tcb->adj_stack_size > 0 && stack_used > 0)
    {
      /* Use fixed-point math with one decimal place */

      stack_filled = 10 * 100 * stack_used / tcb->adj_stack_size;
    }
#endif

  /* Dump interesting properties of this task */

  _alert("  %4d   %4d"
#ifdef CONFIG_STACK_COLORATION
         "   %7lu"
#endif
         "   %7lu"
#ifdef CONFIG_STACK_COLORATION
         "   %3" PRId32 ".%1" PRId32 "%%%c"
#endif
#ifdef CONFIG_SCHED_CPULOAD
         "   %3" PRId32 ".%01" PRId32 "%%"
#endif
#if CONFIG_TASK_NAME_SIZE > 0
         "   %s"
#endif
         "\n",
         tcb->pid, tcb->sched_priority,
#ifdef CONFIG_STACK_COLORATION
         (unsigned long)up_check_tcbstack(tcb),
#endif
         (unsigned long)tcb->adj_stack_size
#ifdef CONFIG_STACK_COLORATION
         , stack_filled / 10, stack_filled % 10,
         (stack_filled >= 10 * 80 ? '!' : ' ')
#endif
#ifdef CONFIG_SCHED_CPULOAD
         , intpart, fracpart
#endif
#if CONFIG_TASK_NAME_SIZE > 0
         , tcb->name
#endif
        );
}

/****************************************************************************
 * Name: arm_dump_backtrace
 ****************************************************************************/

#ifdef CONFIG_SCHED_BACKTRACE
static void arm_dump_backtrace(FAR struct tcb_s *tcb, FAR void *arg)
{
  /* Show back trace */

  sched_dumpstack(tcb->pid);
}
#endif

/****************************************************************************
 * Name: arm_showtasks
 ****************************************************************************/

static void arm_showtasks(void)
{
#if CONFIG_ARCH_INTERRUPTSTACK > 7
#  ifdef CONFIG_STACK_COLORATION
  uint32_t stack_used = up_check_intstack();
  uint32_t stack_filled = 0;

  if ((CONFIG_ARCH_INTERRUPTSTACK & ~7) > 0 && stack_used > 0)
    {
      /* Use fixed-point math with one decimal place */

      stack_filled = 10 * 100 *
                     stack_used / (CONFIG_ARCH_INTERRUPTSTACK & ~7);
    }
#  endif
#endif

  /* Dump interesting properties of each task in the crash environment */

  _alert("   PID    PRI"
#ifdef CONFIG_STACK_COLORATION
         "      USED"
#endif
         "     STACK"
#ifdef CONFIG_STACK_COLORATION
         "   FILLED "
#endif
#ifdef CONFIG_SCHED_CPULOAD
         "      CPU"
#endif
#if CONFIG_TASK_NAME_SIZE > 0
         "   COMMAND"
#endif
         "\n");

#if CONFIG_ARCH_INTERRUPTSTACK > 7
  _alert("  ----   ----"
#  ifdef CONFIG_STACK_COLORATION
         "   %7lu"
#  endif
         "   %7lu"
#  ifdef CONFIG_STACK_COLORATION
         "   %3" PRId32 ".%1" PRId32 "%%%c"
#  endif
#  ifdef CONFIG_SCHED_CPULOAD
         "     ----"
#  endif
#  if CONFIG_TASK_NAME_SIZE > 0
         "   irq"
#  endif
         "\n"
#  ifdef CONFIG_STACK_COLORATION
         , (unsigned long)stack_used
#  endif
         , (unsigned long)(CONFIG_ARCH_INTERRUPTSTACK & ~7)
#  ifdef CONFIG_STACK_COLORATION
         , stack_filled / 10, stack_filled % 10,
         (stack_filled >= 10 * 80 ? '!' : ' ')
#  endif
        );
#endif

  nxsched_foreach(arm_dump_task, NULL);
#ifdef CONFIG_SCHED_BACKTRACE
  nxsched_foreach(arm_dump_backtrace, NULL);
#endif
}

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
 * Name: arm_dump_stack
 ****************************************************************************/

static void arm_dump_stack(const char *tag, uint32_t sp,
                           uint32_t base, uint32_t size, bool force)
{
  uint32_t top = base + size;

  _alert("%s Stack:\n", tag);
  _alert("sp:     %08x\n", sp);
  _alert("  base: %08x\n", base);
  _alert("  size: %08x\n", size);

  if (sp >= base && sp < top)
    {
      arm_stackdump(sp, top);
    }
  else
    {
      _alert("ERROR: %s Stack pointer is not within the stack\n", tag);

      if (force)
        {
          arm_stackdump(base, top);
        }
    }
}

/****************************************************************************
 * Name: arm_dumpstate
 ****************************************************************************/

static void arm_dumpstate(void)
{
  FAR struct tcb_s *rtcb = running_task();
  uint32_t sp = up_getsp();

  /* Show back trace */

#ifdef CONFIG_SCHED_BACKTRACE
  sched_dumpstack(rtcb->pid);
#endif

  /* Update the xcp context */

  if (CURRENT_REGS)
    {
      memcpy(rtcb->xcp.regs,
             (FAR uintptr_t *)CURRENT_REGS, XCPTCONTEXT_SIZE);
    }
  else
    {
      arm_saveusercontext(rtcb->xcp.regs);
    }

  /* Dump the registers */

  arm_registerdump(rtcb->xcp.regs);

  /* Dump the irq stack */

#if CONFIG_ARCH_INTERRUPTSTACK > 7
  arm_dump_stack("IRQ", sp,
#  ifdef CONFIG_SMP
                 (uint32_t)arm_intstack_alloc(),
#  else
                 (uint32_t)&g_intstackalloc,
#  endif
                 (CONFIG_ARCH_INTERRUPTSTACK & ~7),
                 !!CURRENT_REGS);

  if (CURRENT_REGS)
    {
      sp = CURRENT_REGS[REG_R13];
    }
#endif

  /* Dump the user stack */

  arm_dump_stack("User", sp,
                 (uint32_t)rtcb->stack_base_ptr,
                 (uint32_t)rtcb->adj_stack_size,
#ifdef CONFIG_ARCH_KERNEL_STACK
                 false
#else
                 true
#endif
                );

#ifdef CONFIG_ARCH_KERNEL_STACK
  arm_dump_stack("Kernel", sp,
                 (uint32_t)rtcb->xcp.kstack,
                 CONFIG_ARCH_KERNEL_STACKSIZE,
                 false);
#endif

  /* Dump the state of all tasks (if available) */

  arm_showtasks();

#ifdef CONFIG_ARCH_USBDUMP
  /* Dump USB trace data */

  usbtrace_enumerate(assert_tracecallback, NULL);
#endif
}
#endif /* CONFIG_ARCH_STACKDUMP */

/****************************************************************************
 * Name: arm_assert
 ****************************************************************************/

static void arm_assert(void)
{
  /* Flush any buffered SYSLOG data */

  syslog_flush();

  /* Are we in an interrupt handler or the idle task? */

  if (CURRENT_REGS || (running_task())->flink == NULL)
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
  board_autoled_on(LED_ASSERTION);

  /* Flush any buffered SYSLOG data (prior to the assertion) */

  syslog_flush();

  _alert("Assertion failed "
#ifdef CONFIG_SMP
         "CPU%d "
#endif
         "at file:%s line: %d"
#if CONFIG_TASK_NAME_SIZE > 0
         " task: %s"
#endif
         "\n",
#ifdef CONFIG_SMP
         up_cpu_index(),
#endif
         filename, lineno
#if CONFIG_TASK_NAME_SIZE > 0
         , running_task()->name
#endif
        );

#ifdef CONFIG_ARCH_STACKDUMP
  arm_dumpstate();
#endif

  /* Flush any buffered SYSLOG data (from the above) */

  syslog_flush();

#ifdef CONFIG_BOARD_CRASHDUMP
  board_crashdump(up_getsp(), running_task(), filename, lineno);
#endif

  arm_assert();
}
