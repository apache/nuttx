/****************************************************************************
 * arch/risc-v/src/rv32im/riscv_assert.c
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

#include "riscv_arch.h"
#include "sched/sched.h"
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_stackdump
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void riscv_stackdump(uint32_t sp, uint32_t stack_top)
{
  uint32_t stack;

  for (stack = sp & ~0x1f; stack < stack_top; stack += 32)
    {
      uint32_t *ptr = (uint32_t *)stack;
      _alert("%08x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}
#else
#  define riscv_stackdump(sp, stack_top)
#endif

/****************************************************************************
 * Name: riscv_taskdump
 ****************************************************************************/

#ifdef CONFIG_STACK_COLORATION
static void riscv_taskdump(FAR struct tcb_s *tcb, FAR void *arg)
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
 * Name: riscv_showtasks
 ****************************************************************************/

#ifdef CONFIG_STACK_COLORATION
static inline void riscv_showtasks(void)
{
  /* Dump interesting properties of each task in the crash environment */

  nxsched_foreach(riscv_taskdump, NULL);
}
#else
#  define riscv_showtasks()
#endif

/****************************************************************************
 * Name: riscv_registerdump
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static inline void riscv_registerdump(void)
{
  /* Are user registers available from interrupt processing? */

  if (g_current_regs)
    {
      _alert("EPC:%08x \n",
            g_current_regs[REG_EPC]);
      _alert("A0:%08x A1:%08x A2:%08x A3:%08x A4:%08x A5:%08x "
             "A6:%08x A7:%08x\n",
            g_current_regs[REG_A0], g_current_regs[REG_A1],
            g_current_regs[REG_A2], g_current_regs[REG_A3],
            g_current_regs[REG_A4], g_current_regs[REG_A5],
            g_current_regs[REG_A6], g_current_regs[REG_A7]);
      _alert("T0:%08x T1:%08x T2:%08x T3:%08x T4:%08x T5:%08x T6:%08x\n",
            g_current_regs[REG_T0], g_current_regs[REG_T1],
            g_current_regs[REG_T2], g_current_regs[REG_T3],
            g_current_regs[REG_T4], g_current_regs[REG_T5],
            g_current_regs[REG_T6]);
      _alert("S0:%08x S1:%08x S2:%08x S3:%08x S4:%08x S5:%08x "
             "S6:%08x S7:%08x\n",
            g_current_regs[REG_S0], g_current_regs[REG_S1],
            g_current_regs[REG_S2], g_current_regs[REG_S3],
            g_current_regs[REG_S4], g_current_regs[REG_S5],
            g_current_regs[REG_S6], g_current_regs[REG_S7]);
      _alert("S8:%08x S9:%08x S10:%08x S11:%08x\n",
            g_current_regs[REG_S8], g_current_regs[REG_S9],
            g_current_regs[REG_S10], g_current_regs[REG_S11]);
#ifdef RISCV_SAVE_GP
      _alert("GP:%08x SP:%08x FP:%08x TP:%08x RA:%08x\n",
            g_current_regs[REG_GP], g_current_regs[REG_SP],
            g_current_regs[REG_FP], g_current_regs[REG_TP],
            g_current_regs[REG_RA]);
#else
      _alert("SP:%08x FP:%08x TP:%08x RA:%08x\n",
            g_current_regs[REG_SP], g_current_regs[REG_FP],
            g_current_regs[REG_TP], g_current_regs[REG_RA]);
#endif
    }
}
#else
#  define riscv_registerdump()
#endif

/****************************************************************************
 * Name: riscv_dumpstate
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void riscv_dumpstate(void)
{
  struct tcb_s *rtcb = running_task();
  uint32_t sp = up_getsp();
  uint32_t ustackbase;
  uint32_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 15
  uint32_t istackbase;
  uint32_t istacksize;
#endif

  /* Dump the registers (if available) */

  riscv_registerdump();

  /* Get the limits on the user stack memory */

  ustackbase = (uint32_t)rtcb->stack_base_ptr;
  ustacksize = (uint32_t)rtcb->adj_stack_size;

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  istackbase = (uint32_t)&g_intstackalloc;
  istacksize = (CONFIG_ARCH_INTERRUPTSTACK & ~15);

  /* Show interrupt stack info */

  _alert("sp:     %08x\n", sp);
  _alert("IRQ stack:\n");
  _alert("  base: %08x\n", istackbase);
  _alert("  size: %08x\n", istacksize);

  /* Does the current stack pointer lie within the interrupt
   * stack?
   */

  if (sp >= istackbase && sp < istackbase + istacksize)
    {
      /* Yes.. dump the interrupt stack */

      riscv_stackdump(sp, istackbase + istacksize);

      /* Extract the user stack pointer which should lie
       * at the base of the interrupt stack.
       */

      sp = (uint32_t)&g_intstacktop;
      _alert("sp:     %08x\n", sp);
    }
  else if (g_current_regs)
    {
      _alert("ERROR: Stack pointer is not within the interrupt stack\n");
      riscv_stackdump(istackbase, istackbase + istacksize);
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

  if (sp >= ustackbase && sp < ustackbase + ustacksize)
    {
      riscv_stackdump(sp, ustackbase + ustacksize);
    }
  else
    {
      _alert("ERROR: Stack pointer is not within allocated stack\n");
      riscv_stackdump(ustackbase, ustackbase + ustacksize);
    }
}
#else
#  define riscv_dumpstate()
#endif

/****************************************************************************
 * Name: riscv_assert
 ****************************************************************************/

static void riscv_assert(void)
{
  /* Flush any buffered SYSLOG data */

  syslog_flush();

  /* Are we in an interrupt handler or the idle task? */

  if (g_current_regs || running_task()->flink == NULL)
    {
      up_irq_save();
      for (; ; )
        {
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

#if CONFIG_TASK_NAME_SIZE > 0
  _alert("Assertion failed at file:%s line: %d task: %s\n",
         filename, lineno, running_task()->name);
#else
  _alert("Assertion failed at file:%s line: %d\n",
        filename, lineno);
#endif

  riscv_dumpstate();

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
