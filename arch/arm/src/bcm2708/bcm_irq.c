/****************************************************************************
 * arch/arm/src/bcm/bcm_irq.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "arm.h"
#include "up_arch.h"
#include "up_internal.h"
#include "group/group.h"

#include "bcm_aux.h"
#include "bcm_gpio.h"
#include "chip/bcm2708_irq.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

volatile uint32_t *g_current_regs[1];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   This function is called by up_initialize() during the bring-up of the
 *   system.  It is the responsibility of this function to but the interrupt
 *   subsystem into the working and ready state.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Disable all interrupts */

  putreg32(IRQ_DBR_ALLINTS, BCM_IRQ_DBR);
  putreg32(IRQ_DIR1_ALLINTS, BCM_IRQ_DIR1);
  putreg32(IRQ_DIR2_ALLINTS, BCM_IRQ_DIR2);

  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

  /* Intitialize AUX interrupts */

  bcm_aux_irqinitialize();

#ifdef CONFIG_BCM2708_GPIO_IRQ
  /* Initialize GPIO interrrupts */

  bcm_gpio_irqinitialize();
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts */

  up_irq_restore(SVC_MODE | PSR_F_BIT);
#endif
}

/****************************************************************************
 * Name: up_decodeirq
 *
 * Description:
 *   This function is called from the IRQ vector handler in arm_vectors.S.
 *   At this point, the interrupt has been taken and the registers have
 *   been saved on the stack.  This function simply needs to determine the
 *   the irq number of the interrupt and then to call irq_dispatch to
 *   dispatch the interrupt.
 *
 *  Input parameters:
 *   regs - A pointer to the register save area on the stack.
 *
 ****************************************************************************/

void up_decodeirq(uint32_t *regs)
{
  uint32_t bpr;
  uint32_t ipr;
  uint32_t mask;
  int bitno;
  int irq;

  /* Current regs non-zero indicates that we are processing an interrupt;
   * CURRENT_REGS is also used to manage interrupt level context switches.
   *
   * Nested interrupts are not supported.
   */

  DEBUGASSERT(CURRENT_REGS == NULL);
  CURRENT_REGS = regs;

  /* Read the basic pending register first */

  bpr = getreg32(BCM_IRQ_BPR);

  /* Handle any pending interrupts in the BPR register first */

  for (bitno = BPR_BIT_FIRST;
       (bpr & BPR_BIT_IRQMASK) != 0 && bitno <= BPR_BIT_LAST;
       bitno++)
    {
      mask = 1 << bitno;
      if ((bpr & mask) != 0)
        {
          /* Clear the bit in the BPR in hope that we may be
           * able to terminate the loop early.
           */

          bpr &= ~mask;

          /* Dispatch the interrupt */

          irq = bitno + IPR1_IRQ_FIRST - IPR1_BIT_FIRST;
          irq_dispatch(irq, regs);
        }
    }

  /* Check for pending interrupts in IPR1 */

  if ((bpr & BCM_IRQ_PENDING1) != 0)
    {
      /* Read the pending 1 register */

      ipr = getreg32(BCM_IRQ_IPR1);

      /* Handle any pending interrupts in the IPR1 register first */

      for (bitno = IPR1_BIT_FIRST;
           (ipr & IPR1_BIT_IRQMASK) != 0 && bitno <= IPR1_BIT_LAST;
           bitno++)
        {
          mask = 1 << bitno;
          if ((ipr & mask) != 0)
            {
              /* Clear the bit in the IPR1 in hope that we may be
               * able to terminate the loop early.
               */

              ipr &= ~mask;

              /* Dispatch the interrupt */

              irq = bitno + IPR1_IRQ_FIRST - IPR1_BIT_FIRST;
              irq_dispatch(irq, regs);
            }
        }
    }

  /* Check for pending interrupts in IPR2 */

  if ((bpr & BCM_IRQ_PENDING2) != 0)
    {
      /* Read the pending 2 register */

      ipr = getreg32(BCM_IRQ_IPR2);

      /* Handle any pending interrupts in the IPR2 register first */

      for (bitno = IPR2_BIT_FIRST;
           (ipr & IPR2_BIT_IRQMASK) != 0 && bitno <= IPR2_BIT_LAST;
           bitno++)
        {
          mask = 1 << bitno;
          if ((ipr & mask) != 0)
            {
              /* Clear the bit in the IPR2 in hope that we may be
               * able to terminate the loop early.
               */

              ipr &= ~mask;

              /* Dispatch the interrupt */

              irq = bitno + IPR2_IRQ_FIRST - IPR2_BIT_FIRST;
              irq_dispatch(irq, regs);
            }
        }
    }

#if defined(CONFIG_ARCH_FPU) || defined(CONFIG_ARCH_ADDRENV)
  /* Check for a context switch.  If a context switch occurred, then
   * CURRENT_REGS will have a different value than it did on entry.  If an
   * interrupt level context switch has occurred, then restore the
   * floating point state and the establish the correct address environment
   * before returning from the interrupt.
   */

  if (regs != CURRENT_REGS)
    {
#ifdef CONFIG_ARCH_FPU
      /* Restore floating point registers */

      up_restorefpu((uint32_t *)CURRENT_REGS);
#endif

#ifdef CONFIG_ARCH_ADDRENV
      /* Make sure that the address environment for the previously running
       * task is closed down gracefully (data caches dump, MMU flushed) and
       * set up the address environment for the new thread at the head of
       * the ready-to-run list.
       */

      (void)group_addrenv(NULL);
#endif
    }
#endif

  /* Set CURRENT_REGS to NULL to indicate that we are no longer in an
   * interrupt handler.
   */

  CURRENT_REGS = NULL;
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  unsigned int bitno;

  if (irq <= BPR_IRQ_LAST)
    {
      bitno = irq - BPR_IRQ_FIRST + BPR_BIT_FIRST;
      putreg32(BCM_IRQ_DBR, (uint32_t)1 << bitno);
    }
  else if (irq <= IPR1_IRQ_LAST)
    {
      bitno = irq - IPR1_IRQ_FIRST + IPR1_BIT_FIRST;
      putreg32(BCM_IRQ_DIR1, (uint32_t)1 << bitno);
    }
  else if (irq <= IPR2_IRQ_LAST)
    {
      bitno = irq - IPR2_IRQ_FIRST + IPR2_BIT_FIRST;
      putreg32(BCM_IRQ_DIR2, (uint32_t)1 << bitno);
    }
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  unsigned int bitno;

  if (irq <= BPR_IRQ_LAST)
    {
      bitno = irq - BPR_IRQ_FIRST + BPR_BIT_FIRST;
      putreg32(BCM_IRQ_EBR, (uint32_t)1 << bitno);
    }
  else if (irq <= IPR1_IRQ_LAST)
    {
      bitno = irq - IPR1_IRQ_FIRST + IPR1_BIT_FIRST;
      putreg32(BCM_IRQ_EIR1, (uint32_t)1 << bitno);
    }
  else if (irq <= IPR2_IRQ_LAST)
    {
      bitno = irq - IPR2_IRQ_FIRST + IPR2_BIT_FIRST;
      putreg32(BCM_IRQ_EIR2, (uint32_t)1 << bitno);
    }
}
