/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_irq.c
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "arm.h"
#include "arm_internal.h"
#include "lpc31_intc.h"
#include "lpc31_cgudrvr.h"
#include "lpc31.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  int irq;

  /* Enable clock to interrupt controller */

  lpc31_enableclock(CLKID_AHB2INTCCLK);  /*  AHB_TO_INTC_CLK */
  lpc31_enableclock(CLKID_INTCCLK);      /*  INTC_CLK */

  /* Set the vector base. We don't use direct vectoring,
   * so these are set to 0.
   */

  putreg32(0, LPC31_INTC_VECTOR0);
  putreg32(0, LPC31_INTC_VECTOR1);

  /* Set the priority threshold to 0, i.e. don't mask any interrupt on the
   * basis of priority level, for both targets (IRQ/FIQ)
   */

  putreg32(0, LPC31_INTC_PRIORITYMASK0); /* Proc interrupt request 0: IRQ */
  putreg32(0, LPC31_INTC_PRIORITYMASK1); /* Proc interrupt request 1: FIQ */

  /* Disable all interrupts. Start from index 1 since 0 is unused. */

  for (irq = 0; irq < NR_IRQS; irq++)
    {
      /* Initialize as high-active, disable the interrupt, set target to IRQ,
       * Set priority level to 1 (= lowest) for all the interrupt lines
       */

      uint32_t address = LPC31_INTC_REQUEST(irq + 1);
      putreg32(INTC_REQUEST_WEACTLOW | INTC_REQUEST_WEENABLE |
               INTC_REQUEST_TARGET_IRQ | INTC_REQUEST_PRIOLEVEL(1) |
               INTC_REQUEST_WEPRIO, address);
    }

  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts */

  up_irq_restore(PSR_MODE_SYS | PSR_F_BIT);
#endif
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
  /* Get the address of the request register corresponding to this
   * interrupt source
   */

  uint32_t address = LPC31_INTC_REQUEST(irq + 1);

  /* Clear the ENABLE bit with WE_ENABLE=1.  Configuration settings will be
   * preserved because WE_TARGET is zero.
   */

  putreg32(INTC_REQUEST_WEENABLE, address);
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
  /* Get the address of the request register corresponding to this
   * interrupt source
   */

  uint32_t address = LPC31_INTC_REQUEST(irq + 1);

  /* Set the ENABLE bit with WE_ENABLE=1.  Configuration settings will be
   * preserved because WE_TARGET is zero.
   */

  putreg32(INTC_REQUEST_ENABLE | INTC_REQUEST_WEENABLE, address);
}

/****************************************************************************
 * Name: arm_ack_irq
 *
 * Description:
 *   Acknowledge the interrupt
 *
 ****************************************************************************/

void arm_ack_irq(int irq)
{
  /* Get the address of the request register corresponding to this
   * interrupt source
   */

  uint32_t address = LPC31_INTC_REQUEST(irq + 1);

  /* Clear the pending interrupt (INTC_REQUEST_CLRSWINT=1) while keeping
   * interrupts enabled (ENABLE=1 && WE_ENABLE=1). Configuration settings
   * will be preserved because WE_TARGET is zero.
   */

  putreg32(INTC_REQUEST_CLRSWINT | INTC_REQUEST_ENABLE |
           INTC_REQUEST_WEENABLE, address);
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
#warning "Not implemented"
  return OK;
}
#endif
