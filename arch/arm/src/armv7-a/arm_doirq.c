/****************************************************************************
 * arch/arm/src/armv7-a/arm_doirq.c
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

#include <stdint.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "arm_internal.h"

#include "group/group.h"
#include "gic.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A bit set of pending, non-maskable SGI interrupts, on bit set for each
 * supported CPU.
 */

#ifdef CONFIG_ARMV7A_HAVE_GICv2
#ifdef CONFIG_SMP
static uint16_t g_sgi_pending[CONFIG_SMP_NCPUS];
#else
static uint16_t g_sgi_pending[1];
#endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _arm_doirq
 *
 * Description:
 *   Receives the one decoded interrupt and dispatches control to the
 *   attached interrupt handler.
 *
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_INTERRUPTS
static inline uint32_t *_arm_doirq(int irq, uint32_t *regs)
{
  /* Current regs non-zero indicates that we are processing an interrupt;
   * CURRENT_REGS is also used to manage interrupt level context switches.
   */

  CURRENT_REGS = regs;

  /* Deliver the IRQ */

  irq_dispatch(irq, regs);

#if defined(CONFIG_ARCH_FPU) || defined(CONFIG_ARCH_ADDRENV)
  /* Check for a context switch.  If a context switch occurred, then
   * CURRENT_REGS will have a different value than it did on entry.  If an
   * interrupt level context switch has occurred, then restore the floating
   * point state and the establish the correct address environment before
   * returning from the interrupt.
   */

  if (regs != CURRENT_REGS)
    {
#ifdef CONFIG_ARCH_FPU
      /* Restore floating point registers */

      arm_restorefpu((uint32_t *)CURRENT_REGS);
#endif

#ifdef CONFIG_ARCH_ADDRENV
      /* Make sure that the address environment for the previously
       * running task is closed down gracefully (data caches dump,
       * MMU flushed) and set up the address environment for the new
       * thread at the head of the ready-to-run list.
       */

      group_addrenv(NULL);
#endif
    }
#endif

  /* Set CURRENT_REGS to NULL to indicate that we are no longer in an
   * interrupt handler.
   */

  regs         = (uint32_t *)CURRENT_REGS;
  CURRENT_REGS = NULL;

  return regs;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_doirq
 *
 * Description:
 *   Receives the decoded GIC interrupt information and dispatches control
 *   to the attached interrupt handler.  There are two versions:
 *
 *   1) For the simple case where all interrupts are maskable.  In that
 *      simple case, arm_doirq() is simply a wrapper for the inlined
 *      _arm_do_irq() that does the real work.
 *
 *   2) With the GICv2, there are 16 non-maskable software generated
 *      interrupts (SGIs) that also come through arm_doirq().  In that case,
 *      we must avoid nesting interrupt handling and serial the processing.
 *
 ****************************************************************************/

#ifndef CONFIG_ARMV7A_HAVE_GICv2
uint32_t *arm_doirq(int irq, uint32_t *regs)
{
  board_autoled_on(LED_INIRQ);
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else
  /* Nested interrupts are not supported */

  DEBUGASSERT(CURRENT_REGS == NULL);

  /* Dispatch the interrupt to its attached handler */

  regs = _arm_doirq(irq, regs);
#endif

  board_autoled_off(LED_INIRQ);
  return regs;
}
#endif

#ifdef CONFIG_ARMV7A_HAVE_GICv2
uint32_t *arm_doirq(int irq, uint32_t *regs)
{
#ifndef CONFIG_SUPPRESS_INTERRUPTS
  uint32_t bit;
  int cpu;
  int i;
#endif

  board_autoled_on(LED_INIRQ);
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();

#else
  /* Get the CPU processing the interrupt */

#ifdef CONFIG_SMP
  cpu = up_cpu_index();
#else
  cpu = 0;
#endif

  /* Non-zero CURRENT_REGS indicates that we are already processing an
   * interrupt.  This could be a normal event for the case of the GICv2;
   * Software generated interrupts are non-maskable.
   *
   * REVISIT: There is no support for nested SGIs!  That will cause an
   * assertion below.  There is also no protection for concurrent access
   * to g_sgi_pending for that case.
   */

  if (CURRENT_REGS != NULL)
    {
      int ndx = irq - GIC_IRQ_SGI0;
      bit = (1 << (ndx));

      /* Only an SGI should cause this event.  We also cannot support
       * multiple pending SGI interrupts.
       */

      DEBUGASSERT((unsigned int)irq <= GIC_IRQ_SGI15 &&
                  (g_sgi_pending[cpu] & bit) == 0);

      /* Mare the SGI as pending and return immediately */

      sinfo("SGI%d pending\n", ndx);
      g_sgi_pending[cpu] |= bit;
      return regs;
    }

  /* Dispatch the interrupt to its attached handler */

   regs = _arm_doirq(irq, regs);

  /* Then loop dispatching any pending SGI interrupts that occcurred during
   * processing of the interrupts.
   */

  for (i = 0; i < 16 && g_sgi_pending[cpu] != 0; i++)
    {
      /* Check if this SGI is pending */

      bit = (1 << i);
      if ((g_sgi_pending[cpu] & bit) != 0)
        {
          /* Clear the pending bit */

          g_sgi_pending[cpu] &= ~bit;

          /* And dispatch the SGI */

          sinfo("Dispatching pending SGI%d\n", i + GIC_IRQ_SGI0);
          regs = _arm_doirq(i + GIC_IRQ_SGI0, regs);
        }
    }
#endif

  board_autoled_off(LED_INIRQ);
  return regs;
}
#endif
