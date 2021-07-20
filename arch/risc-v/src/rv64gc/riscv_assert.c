/****************************************************************************
 * arch/risc-v/src/rv64gc/riscv_assert.c
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

#include "riscv_arch.h"
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
 * Name: up_stackdump
 ****************************************************************************/

static void up_stackdump(uint64_t sp, uintptr_t stack_top)
{
  uintptr_t stack;

  for (stack = sp & ~0x1f; stack < stack_top; stack += 32)
    {
      uint32_t *ptr = (uint32_t *)stack;
      _alert("%08" PRIxPTR ": %08" PRIx32 " %08" PRIx32 " %08" PRIx32
             " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32
             " %08" PRIx32 "\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}

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

static inline void up_registerdump(void)
{
  /* Are user registers available from interrupt processing? */

  if (CURRENT_REGS)
    {
      _alert("EPC:%016" PRIx64 " \n",
             CURRENT_REGS[REG_EPC]);

      _alert("A0:%016" PRIx64 " A1:%01" PRIx64 "6 A2:%016" PRIx64
             " A3:%016" PRIx64 " \n",
             CURRENT_REGS[REG_A0], CURRENT_REGS[REG_A1],
             CURRENT_REGS[REG_A2], CURRENT_REGS[REG_A3]);

      _alert("A4:%016" PRIx64 " A5:%016" PRIx64 "A6:%016" PRIx64
             " A7:%016" PRIx64 " \n",
             CURRENT_REGS[REG_A4], CURRENT_REGS[REG_A5],
             CURRENT_REGS[REG_A6], CURRENT_REGS[REG_A7]);

      _alert("T0:%016" PRIx64 " T1:%016" PRIx64 " T2:%016" PRIx64
             " T3:%016" PRIx64 " \n",
             CURRENT_REGS[REG_T0], CURRENT_REGS[REG_T1],
             CURRENT_REGS[REG_T2], CURRENT_REGS[REG_T3]);

      _alert("T4:%016" PRIx64 " T5:%016" PRIx64 " T6:%016" PRIx64 " \n",
             CURRENT_REGS[REG_T4], CURRENT_REGS[REG_T5],
             CURRENT_REGS[REG_T6]);

      _alert("S0:%016" PRIx64 " S1:%016" PRIx64 " S2:%016" PRIx64
             " S3:%016" PRIx64 " \n",
             CURRENT_REGS[REG_S0], CURRENT_REGS[REG_S1],
             CURRENT_REGS[REG_S2], CURRENT_REGS[REG_S3]);

      _alert("S4:%016" PRIx64 " S5:%016" PRIx64 " S6:%016" PRIx64
             " S7:%016" PRIx64 " \n",
             CURRENT_REGS[REG_S4], CURRENT_REGS[REG_S5],
             CURRENT_REGS[REG_S6], CURRENT_REGS[REG_S7]);

      _alert("S8:%016" PRIx64 " S9:%016" PRIx64 " S10:%016" PRIx64
             " S11:%016" PRIx64 " \n",
             CURRENT_REGS[REG_S8], CURRENT_REGS[REG_S9],
             CURRENT_REGS[REG_S10], CURRENT_REGS[REG_S11]);

#ifdef RISCV_SAVE_GP
      _alert("GP:%016" PRIx64 " SP:%016" PRIx64 " FP:%016" PRIx64
             " TP:%016" PRIx64 " RA:%016" PRIx64 " \n",
             CURRENT_REGS[REG_GP], CURRENT_REGS[REG_SP],
             CURRENT_REGS[REG_FP], CURRENT_REGS[REG_TP],
             CURRENT_REGS[REG_RA]);
#else
      _alert("SP:%016" PRIx64 " FP:%016" PRIx64 " TP:%016" PRIx64
             " RA:%016" PRIx64 " \n",
             CURRENT_REGS[REG_SP], CURRENT_REGS[REG_FP],
             CURRENT_REGS[REG_TP], CURRENT_REGS[REG_RA]);
#endif
    }
}

/****************************************************************************
 * Name: up_dumpstate
 ****************************************************************************/

static void up_dumpstate(void)
{
  struct tcb_s *rtcb = running_task();
  uint64_t sp = up_getsp();
  uintptr_t ustackbase;
  uintptr_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 15
  uintptr_t istackbase;
  uintptr_t istacksize;
#endif

  /* Dump the registers (if available) */

  up_registerdump();

  /* Get the limits on the user stack memory */

  ustackbase = (uintptr_t)rtcb->stack_base_ptr;
  ustacksize = (uintptr_t)rtcb->adj_stack_size;

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  istackbase = (uintptr_t)&g_intstackalloc;
  istacksize = (CONFIG_ARCH_INTERRUPTSTACK & ~15);

  /* Show interrupt stack info */

  _alert("sp:     %016" PRIx64 "\n", sp);
  _alert("IRQ stack:\n");
  _alert("  base: %016" PRIxPTR "\n", istackbase);
  _alert("  size: %016" PRIxPTR "\n", istacksize);

  /* Does the current stack pointer lie within the interrupt
   * stack?
   */

  if (sp >= istackbase && sp < istackbase + istacksize)
    {
      /* Yes.. dump the interrupt stack */

      up_stackdump(sp, istackbase + istacksize);

      /* Extract the user stack pointer */

      sp = CURRENT_REGS[REG_SP];
      _alert("sp:     %016" PRIx64 "\n", sp);
    }
  else if (CURRENT_REGS)
    {
      _alert("ERROR: Stack pointer is not within the interrupt stack\n");
      up_stackdump(istackbase, istackbase + istacksize);
    }

  /* Show user stack info */

  _alert("User stack:\n");
  _alert("  base: %016" PRIxPTR "\n", ustackbase);
  _alert("  size: %016" PRIxPTR "\n", ustacksize);
#else
  _alert("sp:         %016" PRIx64 "\n", sp);
  _alert("stack base: %016" PRIxPTR "\n", ustackbase);
  _alert("stack size: %016" PRIxPTR "\n", ustacksize);
#endif

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp >= ustackbase && sp < ustackbase + ustacksize)
    {
      _alert("ERROR: Stack pointer is not within allocated stack\n");
      up_stackdump(ustackbase, ustackbase + ustacksize);
    }
  else
    {
      up_stackdump(sp, ustackbase + ustacksize);
    }
}

#endif /* CONFIG_ARCH_STACKDUMP */

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

  up_dumpstate();

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

  /* Flush any buffered SYSLOG data (from the above) */

  syslog_flush();

#ifdef CONFIG_BOARD_CRASHDUMP
  board_crashdump(up_getsp(), running_task(), filename, lineno);
#endif

  _up_assert();
}
