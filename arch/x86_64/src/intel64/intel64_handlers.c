/****************************************************************************
 *  arch/x86_64/src/intel64/intel64_handlers.c
 *
 *   Copyright (C) 2011-2012, 2014-2015 Gregory Nutt,
 *                 2020 Chung-Fan Yang.
 *   All rights reserved.
 *
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Chung-Fan Yang <sonic.tw.tp@gmail.com>
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
#include <nuttx/compiler.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/signal.h>
#include <arch/io.h>
#include <syscall.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define X2APIC_EOI		0x80b

#define APIC_EOI_ACK		0

/****************************************************************************
 * Global Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
  board_autoled_on(LED_INIRQ);

  /* Current regs non-zero indicates that we are processing an interrupt;
   * g_current_regs is also used to manage interrupt level context switches.
   *
   * Nested interrupts are not supported.
   */

  DEBUGASSERT(g_current_regs == NULL);
  g_current_regs = regs;

  /* Deliver the IRQ */

  irq_dispatch(irq, regs);

#if defined(CONFIG_ARCH_FPU) || defined(CONFIG_ARCH_ADDRENV)
  /* Check for a context switch.  If a context switch occurred, then
   * g_current_regs will have a different value than it did on entry.  If an
   * interrupt level context switch has occurred, then restore the floating
   * point state and the establish the correct address environment before
   * returning from the interrupt.
   */

  if (regs != g_current_regs)
    {
#ifdef CONFIG_ARCH_FPU
      /* Restore floating point registers */

      up_restorefpu((uint64_t*)g_current_regs);
#endif

#ifdef CONFIG_ARCH_ADDRENV
      /* Make sure that the address environment for the previously
       * running task is closed down gracefully (data caches dump,
       * MMU flushed) and set up the address environment for the new
       * thread at the head of the ready-to-run list.
       */

      (void)group_addrenv(NULL);
#endif
    }
#endif

  /* If a context switch occurred while processing the interrupt then
   * g_current_regs may have change value.  If we return any value different
   * from the input regs, then the lower level will know that a context
   * switch occurred during interrupt processing.
   */

  regs = (uint64_t*)g_current_regs;

  /* Set g_current_regs to NULL to indicate that we are no longer in an
   * interrupt handler.
   */

  g_current_regs = NULL;
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

#define SIGFPE 8

uint64_t *isr_handler(uint64_t *regs, uint64_t irq)
{
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  board_autoled_on(LED_INIRQ);
  PANIC(); /* Doesn't return */
  return regs;               /* To keep the compiler happy */
#else
  uint64_t *ret;

  DEBUGASSERT(g_current_regs == NULL);
  g_current_regs = regs;

  switch(irq)
    {
      case 0:
      case 16:
        asm volatile("fnclex":::"memory");
        nxsig_kill(this_task()->pid, SIGFPE);
        break;
      default:
        /* Let's say, all ISR are asserted when REALLY BAD things happened */
        /* Don't even brother to recover, just dump the regs and PANIC*/
        _alert("PANIC:\n");
        _alert("Exception %lld occurred with error code %lld:\n", irq, regs[REG_ERRCODE]);

        up_registerdump(regs);

        up_trash_cpu();
        break;

  }

  // Maybe we need a context switch
  regs = (uint64_t*)g_current_regs;

  /* Set g_current_regs to NULL to indicate that we are no longer in an
   * interrupt handler.
   */

  g_current_regs = NULL;
  return regs;
#endif
}

/****************************************************************************
 * Name: isr_handler
 *
 * Description:
 *   This gets called from IRQ vector handling logic in intel64_vectors.S
 *
 ****************************************************************************/

uint64_t *irq_handler(uint64_t *regs, uint64_t irq_no)
{
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  board_autoled_on(LED_INIRQ);
  PANIC(); /* Doesn't return */
  return regs;               /* To keep the compiler happy */
#else
  uint64_t *ret;
  int irq;

  board_autoled_on(LED_INIRQ);

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
