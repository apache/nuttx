/****************************************************************************
 * arch/risc-v/src/common/riscv_assert.c
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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <syscall.h>

#include <nuttx/irq.h>
#include <nuttx/tls.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/usb/usbdev_trace.h>

#include <arch/board/board.h>

#include "sched/sched.h"
#include "irq/irq.h"
#include "riscv_internal.h"

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

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_stackdump
 ****************************************************************************/

static void riscv_stackdump(uintptr_t sp, uintptr_t stack_top)
{
  uintptr_t stack;

  /* Flush any buffered SYSLOG data to avoid overwrite */

  syslog_flush();

  for (stack = sp & ~0x1f; stack < (stack_top & ~0x1f); stack += 32)
    {
      uint32_t *ptr = (uint32_t *)stack;
      _alert("%" PRIxREG ": %08" PRIx32 " %08" PRIx32 " %08" PRIx32
             " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32
             " %08" PRIx32 "\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}

/****************************************************************************
 * Name: riscv_registerdump
 ****************************************************************************/

static inline void riscv_registerdump(volatile uintptr_t *regs)
{
  /* Are user registers available from interrupt processing? */

  _alert("EPC: %" PRIxREG "\n", regs[REG_EPC]);
  _alert("A0: %" PRIxREG " A1: %" PRIxREG " A2: %" PRIxREG
         " A3: %" PRIxREG "\n",
         regs[REG_A0], regs[REG_A1], regs[REG_A2], regs[REG_A3]);
  _alert("A4: %" PRIxREG " A5: %" PRIxREG " A6: %" PRIxREG
         " A7: %" PRIxREG "\n",
         regs[REG_A4], regs[REG_A5], regs[REG_A6], regs[REG_A7]);
  _alert("T0: %" PRIxREG " T1: %" PRIxREG " T2: %" PRIxREG
         " T3: %" PRIxREG "\n",
         regs[REG_T0], regs[REG_T1], regs[REG_T2], regs[REG_T3]);
  _alert("T4: %" PRIxREG " T5: %" PRIxREG " T6: %" PRIxREG "\n",
         regs[REG_T4], regs[REG_T5], regs[REG_T6]);
  _alert("S0: %" PRIxREG " S1: %" PRIxREG " S2: %" PRIxREG
         " S3: %" PRIxREG "\n",
         regs[REG_S0], regs[REG_S1], regs[REG_S2], regs[REG_S3]);
  _alert("S4: %" PRIxREG " S5: %" PRIxREG " S6: %" PRIxREG
         " S7: %" PRIxREG "\n",
         regs[REG_S4], regs[REG_S5], regs[REG_S6], regs[REG_S7]);
  _alert("S8: %" PRIxREG " S9: %" PRIxREG " S10: %" PRIxREG
         " S11: %" PRIxREG "\n",
         regs[REG_S8], regs[REG_S9], regs[REG_S10], regs[REG_S11]);
#ifdef RISCV_SAVE_GP
  _alert("GP: %" PRIxREG " SP: %" PRIxREG " FP: %" PRIxREG
         " TP: %" PRIxREG " RA: %" PRIxREG "\n",
         regs[REG_GP], regs[REG_SP], regs[REG_FP], regs[REG_TP],
         regs[REG_RA]);
#else
  _alert("SP: %" PRIxREG " FP: %" PRIxREG " TP: %" PRIxREG
         " RA: %" PRIxREG "\n",
         regs[REG_SP], regs[REG_FP], regs[REG_TP], regs[REG_RA]);
#endif
}

/****************************************************************************
 * Name: riscv_dump_task
 ****************************************************************************/

static void riscv_dump_task(struct tcb_s *tcb, void *arg)
{
  char args[64] = "";
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

#ifndef CONFIG_DISABLE_PTHREAD
  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_PTHREAD)
    {
      FAR struct pthread_tcb_s *ptcb = (FAR struct pthread_tcb_s *)tcb;

      snprintf(args, sizeof(args), " %p", ptcb->arg);
    }
  else
#endif
    {
      FAR char **argv = tcb->group->tg_info->argv + 1;
      size_t npos = 0;

      while (*argv != NULL && npos < sizeof(args))
        {
          npos += snprintf(args + npos, sizeof(args) - npos, " %s", *argv++);
        }
    }

  /* Dump interesting properties of this task */

  _alert("  %4d   %4d"
#ifdef CONFIG_SMP
         "  %4d"
#endif
         "   %7lu"
#ifdef CONFIG_STACK_COLORATION
         "   %7lu   %3" PRId32 ".%1" PRId32 "%%%c"
#endif
#ifdef CONFIG_SCHED_CPULOAD
         "   %3" PRId32 ".%01" PRId32 "%%"
#endif
         "   %s%s\n"
         , tcb->pid, tcb->sched_priority
#ifdef CONFIG_SMP
         , tcb->cpu
#endif
         , (unsigned long)tcb->adj_stack_size
#ifdef CONFIG_STACK_COLORATION
         , (unsigned long)up_check_tcbstack(tcb)
         , stack_filled / 10, stack_filled % 10
         , (stack_filled >= 10 * 80 ? '!' : ' ')
#endif
#ifdef CONFIG_SCHED_CPULOAD
         , intpart, fracpart
#endif
#if CONFIG_TASK_NAME_SIZE > 0
         , tcb->name
#else
         , "<noname>"
#endif
         , args
        );
}

/****************************************************************************
 * Name: riscv_dump_backtrace
 ****************************************************************************/

#ifdef CONFIG_SCHED_BACKTRACE
static void riscv_dump_backtrace(struct tcb_s *tcb, void *arg)
{
  /* Show back trace */

  sched_dumpstack(tcb->pid);
}
#endif

/****************************************************************************
 * Name: riscv_showtasks
 ****************************************************************************/

static inline void riscv_showtasks(void)
{
#if CONFIG_ARCH_INTERRUPTSTACK > 15
#  ifdef CONFIG_STACK_COLORATION
  uint32_t stack_used = up_check_intstack();
  uint32_t stack_filled = 0;

  if ((CONFIG_ARCH_INTERRUPTSTACK & ~15) > 0 && stack_used > 0)
    {
      /* Use fixed-point math with one decimal place */

      stack_filled = 10 * 100 *
                     stack_used / (CONFIG_ARCH_INTERRUPTSTACK & ~15);
    }
#  endif
#endif

  /* Dump interesting properties of each task in the crash environment */

  _alert("   PID    PRI"
#ifdef CONFIG_SMP
         "   CPU"
#endif
         "     STACK"
#ifdef CONFIG_STACK_COLORATION
         "      USED   FILLED "
#endif
#ifdef CONFIG_SCHED_CPULOAD
         "      CPU"
#endif
         "   COMMAND\n");

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  _alert("  ----   ----"
#  ifdef CONFIG_SMP
         "  ----"
#  endif
         "   %7u"
#  ifdef CONFIG_STACK_COLORATION
         "   %7" PRId32 "   %3" PRId32 ".%1" PRId32 "%%%c"
#  endif
#  ifdef CONFIG_SCHED_CPULOAD
         "     ----"
#  endif
         "   irq\n"
         , (CONFIG_ARCH_INTERRUPTSTACK & ~15)
#  ifdef CONFIG_STACK_COLORATION
         , stack_used
         , stack_filled / 10, stack_filled % 10,
         (stack_filled >= 10 * 80 ? '!' : ' ')
#  endif
        );
#endif

  nxsched_foreach(riscv_dump_task, NULL);
#ifdef CONFIG_SCHED_BACKTRACE
  nxsched_foreach(riscv_dump_backtrace, NULL);
#endif
}

/****************************************************************************
 * Name: riscv_dump_stack
 ****************************************************************************/

static void riscv_dump_stack(const char *tag, uintptr_t sp,
                             uintptr_t base, uint32_t size, bool force)
{
  uintptr_t top = base + size;

  _alert("%s Stack:\n", tag);
  _alert("sp:     %08" PRIxPTR "\n", sp);
  _alert("  base: %08" PRIxPTR "\n", base);
  _alert("  size: %08" PRIx32 "\n", size);

  if (sp >= base && sp < top)
    {
      riscv_stackdump(sp, top);
    }
  else
    {
      _alert("ERROR: %s Stack pointer is not within the stack\n", tag);

      if (force)
        {
          riscv_stackdump(base, top);
        }
    }
}

/****************************************************************************
 * Name: riscv_dumpstate
 ****************************************************************************/

static void riscv_dumpstate(void)
{
  struct tcb_s *rtcb = running_task();
  uintptr_t sp = up_getsp();

  /* Show back trace */

#ifdef CONFIG_SCHED_BACKTRACE
  sched_dumpstack(rtcb->pid);
#endif

  /* Update the xcp context */

  if (CURRENT_REGS)
    {
      rtcb->xcp.regs = (uintptr_t *)CURRENT_REGS;
    }
  else
    {
      up_saveusercontext(rtcb->xcp.regs);
    }

  /* Dump the registers (if available) */

  riscv_registerdump(rtcb->xcp.regs);

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  riscv_dump_stack("IRQ", sp,
                   (uintptr_t)&g_intstackalloc,
                   (CONFIG_ARCH_INTERRUPTSTACK & ~15),
                   !!CURRENT_REGS);
  if (CURRENT_REGS)
    {
      sp = CURRENT_REGS[REG_SP];
    }
#endif

  riscv_dump_stack("User", sp,
                   (uintptr_t)rtcb->stack_base_ptr,
                   (uint32_t)rtcb->adj_stack_size,
#ifdef CONFIG_ARCH_KERNEL_STACK
                 false
#else
                 true
#endif
                );

#ifdef CONFIG_ARCH_KERNEL_STACK
  riscv_dump_stack("Kernel", sp,
                   (uintptr_t)rtcb->xcp.kstack,
                   CONFIG_ARCH_KERNEL_STACKSIZE,
                   false);
#endif
}
#else
#  define riscv_dumpstate()
#endif /* CONFIG_ARCH_STACKDUMP */

/****************************************************************************
 * Name: riscv_assert
 ****************************************************************************/

static void riscv_assert(void)
{
  /* Flush any buffered SYSLOG data */

  syslog_flush();

  /* Are we in an interrupt handler or the idle task? */

  if (CURRENT_REGS || running_task()->flink == NULL)
    {
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
 * Name: assert_tracecallback
 ****************************************************************************/

#ifdef CONFIG_ARCH_USBDUMP
static int usbtrace_syslog(const char *fmt, ...)
{
  va_list ap;

  /* Let vsyslog do the real work */

  va_start(ap, fmt);
  vsyslog(LOG_EMERG, fmt, ap);
  va_end(ap);
  return OK;
}

static int assert_tracecallback(struct usbtrace_s *trace, void *arg)
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

void up_assert(const char *filename, int lineno)
{
  board_autoled_on(LED_ASSERTION);

  /* Flush any buffered SYSLOG data (from prior to the assertion) */

  syslog_flush();

#ifdef CONFIG_SMP
#if CONFIG_TASK_NAME_SIZE > 0
  _alert("Assertion failed CPU%d at file:%s line: %d task: %s\n",
         up_cpu_index(), filename, lineno, running_task()->name);
#else
  _alert("Assertion failed CPU%d at file:%s line: %d\n",
         up_cpu_index(), filename, lineno);
#endif
#else
#if CONFIG_TASK_NAME_SIZE > 0
  _alert("Assertion failed at file:%s line: %d task: %s\n",
         filename, lineno, running_task()->name);
#else
  _alert("Assertion failed at file:%s line: %d\n",
         filename, lineno);
#endif
#endif

  riscv_dumpstate();

#ifdef CONFIG_SMP
  /* Show the CPU number */

  _alert("CPU%d:\n", up_cpu_index());
#endif

  /* Dump the state of all tasks (if available) */

  riscv_showtasks();

#ifdef CONFIG_ARCH_USBDUMP
  /* Dump USB trace data */

  usbtrace_enumerate(assert_tracecallback, NULL);
#endif

  /* Flush any buffered SYSLOG data (from the above) */

  syslog_flush();

#ifdef CONFIG_BOARD_CRASHDUMP
  board_crashdump(up_getsp(), running_task(), filename, lineno);
#endif

  riscv_assert();
}
