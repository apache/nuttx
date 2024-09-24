/****************************************************************************
 * arch/x86_64/src/intel64/intel64_handlers.c
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
#include <nuttx/compiler.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/signal.h>
#include <arch/io.h>
#include <assert.h>
#include <debug.h>
#include <inttypes.h>
#include <syscall.h>
#include <arch/board/board.h>

#include "x86_64_internal.h"
#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define X2APIC_EOI    0x80b

#define APIC_EOI_ACK  0

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: common_handler
 *
 * Description:
 *   Common logic for the ISR/IRQ handlers
 *
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_INTERRUPTS
static uint64_t *common_handler(int irq, uint64_t *regs)
{
  struct tcb_s **running_task = &g_running_tasks[this_cpu()];
  struct tcb_s *tcb;
  int cpu;

  if (*running_task != NULL)
    {
      (*running_task)->xcp.regs = regs;
    }

  /* Current regs non-zero indicates that we are processing an interrupt;
   * g_current_regs is also used to manage interrupt level context switches.
   *
   * Nested interrupts are not supported.
   */

  DEBUGASSERT(up_current_regs() == NULL);
  up_set_current_regs(regs);

  /* Deliver the IRQ */

  irq_dispatch(irq, regs);

  /* Check for a context switch.  If a context switch occurred, then
   * g_current_regs will have a different value than it did on entry.  If an
   * interrupt level context switch has occurred, then the establish the
   * correct address environment before returning from the interrupt.
   */

  if (regs != up_current_regs())
    {
#ifdef CONFIG_ARCH_ADDRENV
      /* Make sure that the address environment for the previously
       * running task is closed down gracefully (data caches dump,
       * MMU flushed) and set up the address environment for the new
       * thread at the head of the ready-to-run list.
       */

      addrenv_switch(NULL);
#endif

      /* Update scheduler parameters */

      cpu = this_cpu();
      nxsched_suspend_scheduler(*running_task);
      nxsched_resume_scheduler(this_task());

      /* Record the new "running" task when context switch occurred.
       * g_running_tasks[] is only used by assertion logic for reporting
       * crashes.
       */

      tcb = current_task(cpu);
      g_running_tasks[cpu] = tcb;

      /* Restore the cpu lock */

      restore_critical_section(tcb, cpu);
    }

  /* If a context switch occurred while processing the interrupt then
   * g_current_regs may have change value.  If we return any value different
   * from the input regs, then the lower level will know that a context
   * switch occurred during interrupt processing.
   */

  regs = (uint64_t *)up_current_regs();

  /* Set g_current_regs to NULL to indicate that we are no longer in an
   * interrupt handler.
   */

  up_set_current_regs(NULL);
  return regs;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: isr_handler
 *
 * Description:
 *   This gets called from ISR vector handling logic in broadwell_vectors.S
 *
 ****************************************************************************/

uint64_t *isr_handler(uint64_t *regs, uint64_t irq)
{
  struct tcb_s **running_task = &g_running_tasks[this_cpu()];

  if (*running_task != NULL)
    {
      (*running_task)->xcp.regs = regs;
    }

  board_autoled_on(LED_INIRQ);

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  /* Doesn't return */

  PANIC();

  /* To keep the compiler happy */

  return regs;
#else

  DEBUGASSERT(up_current_regs() == NULL);
  up_set_current_regs(regs);

  switch (irq)
    {
      case 0:
      case 16:
        __asm__ volatile("fnclex":::"memory");
        nxsig_kill(this_task()->pid, SIGFPE);
        break;

      default:
        /* Let's say, all ISR are asserted when REALLY BAD things happened.
         * Don't even brother to recover, just dump the regs and PANIC.
         */

        _alert("PANIC:\n");
        _alert("Exception %" PRId64 " occurred "
               "with error code %" PRId64 ":\n",
               irq, regs[REG_ERRCODE]);

        PANIC_WITH_REGS("panic", regs);

        up_trash_cpu();
        break;
  }

  /* Maybe we need a context switch */

  regs = (uint64_t *)up_current_regs();

  /* Set g_current_regs to NULL to indicate that we are no longer in an
   * interrupt handler.
   */

  up_set_current_regs(NULL);
  return regs;
#endif
}

/****************************************************************************
 * Name: irq_handler
 *
 * Description:
 *   This gets called from IRQ vector handling logic in intel64_vectors.S
 *
 ****************************************************************************/

uint64_t *irq_handler(uint64_t *regs, uint64_t irq_no)
{
  board_autoled_on(LED_INIRQ);

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  /* Doesn't return */

  PANIC();

  /* To keep the compiler happy */

  return regs;
#else
  uint64_t *ret;
  int irq;

  /* Get the IRQ number */

  irq = (int)irq_no;

  /* Dispatch the interrupt */

  ret = common_handler(irq, regs);

  /* Send an EOI (end of interrupt) signal to the APIC */

  write_msr(X2APIC_EOI, APIC_EOI_ACK);
  board_autoled_off(LED_INIRQ);
  return ret;
#endif
}

/****************************************************************************
 * Name: irq_xcp_regs
 *
 * Description:
 *   Return current task XCP registers area.
 *
 * ASSUMPTION:
 *   Interrupts are disabled.
 *
 *   This function should be called only form intel64_vector.S file !
 *   Any other use must be carefully considered.
 *
 ****************************************************************************/

uint64_t *irq_xcp_regs(void)
{
  /* This must be the simplest as possible, so we not use too much registers.
   * Now this function corrupts only RAX and RDX registers regardless of
   * the compiler optimization.
   */

  return (current_task(this_cpu()))->xcp.regs;
}
