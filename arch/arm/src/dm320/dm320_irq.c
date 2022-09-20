/****************************************************************************
 * arch/arm/src/dm320/dm320_irq.c
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
#include <nuttx/arch.h>

#include "arm.h"
#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The value of _svectors is defined in ld.script.  It could be hard-
 * coded because we know that correct IRAM area is 0xffc00000.
 */

extern int _svectors; /* Type does not matter */

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
  /* Clear, disable and configure all interrupts. */

  putreg16(0, DM320_INTC_EINT0);      /* Mask all IRQs/FIQs */
  putreg16(0, DM320_INTC_EINT1);
  putreg16(0, DM320_INTC_EINT2);

  putreg16(0, DM320_INTC_INTRAW);     /* No masked interrupts in status */

  putreg16(0, DM320_INTC_FISEL0);     /* No FIQs */
  putreg16(0, DM320_INTC_FISEL1);
  putreg16(0, DM320_INTC_FISEL2);

  putreg16(0xffff, DM320_INTC_FIQ0);  /* Clear all pending FIQs */
  putreg16(0xffff, DM320_INTC_FIQ1);
  putreg16(0xffff, DM320_INTC_FIQ2);

  putreg16(0xffff, DM320_INTC_IRQ0);  /* Clear all pending IRQs */
  putreg16(0xffff, DM320_INTC_IRQ1);
  putreg16(0xffff, DM320_INTC_IRQ2);

  /* Make sure that the base addresses are zero and that
   * the table increment is 4 bytes.
   */

  putreg16(0, DM320_INTC_EABASE0);
  putreg16(0, DM320_INTC_EABASE1);

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
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
  /* Disable the interrupt by clearing the corresponding bit in
   * the IRQ enable register.
   */

  if (irq < 16)
    {
      /* IRQs0-15 are controlled by the IRQ0 enable register
       * Clear the associated bit to disable the interrupt
       */

      putreg16((getreg16(DM320_INTC_EINT0) &
                ~(1 << irq)), DM320_INTC_EINT0);
    }
  else if (irq < 32)
    {
      /* IRQs16-31 are controlled by the IRQ1 enable register
       * Clear the associated bit to disable the interrupt
       */

      putreg16((getreg16(DM320_INTC_EINT1) &
                ~(1 << (irq - 16))), DM320_INTC_EINT1);
    }
  else
    {
      /* IRQs32- are controlled by the IRQ2 enable register
       * Clear the associated bit to disable the interrupt
       */

      putreg16((getreg16(DM320_INTC_EINT2) &
                ~(1 << (irq - 32))), DM320_INTC_EINT2);
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
  /* Enable the interrupt by setting the corresponding bit in
   * the IRQ enable register.
   */

  if (irq < 16)
    {
      /* IRQs0-15 are controlled by the IRQ0 enable register
       * Set the associated bit to enable the interrupt
       */

      putreg16((getreg16(DM320_INTC_EINT0) |
               (1 << irq)), DM320_INTC_EINT0);
    }
  else if (irq < 32)
    {
      /* IRQs16-31 are controlled by the IRQ1 enable register
       * Set the associated bit to enable the interrupt
       */

      putreg16((getreg16(DM320_INTC_EINT1) |
               (1 << (irq - 16))), DM320_INTC_EINT1);
    }
  else
    {
      /* IRQs32- are controlled by the IRQ2 enable register
       * Set the associated bit to enable the interrupt
       */

      putreg16((getreg16(DM320_INTC_EINT2) |
               (1 << (irq - 32))), DM320_INTC_EINT2);
    }
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
  /* Acknowledge the interrupt by setting the corresponding bit in the
   * IRQ status register.
   */

  if (irq < 16)
    {
      /* Set the associated status bit to clear the interrupt */

      putreg16((1 << irq), DM320_INTC_IRQ0);
    }
  else if (irq < 32)
    {
      /* Set the associated status bit to clear the interrupt  */

      putreg16((1 << (irq - 16)), DM320_INTC_IRQ1);
    }
  else
    {
      /* Set the associated status bit to clear the interrupt */

      putreg16((1 << (irq - 32)), DM320_INTC_IRQ2);
    }
}
