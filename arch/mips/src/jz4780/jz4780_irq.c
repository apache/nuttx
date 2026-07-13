/****************************************************************************
 * arch/mips/src/jz4780/jz4780_irq.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/irq.h>
#include <arch/jz4780/cp0.h>

#include "mips_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt controller definitions *****************************************/

/* Number of interrupt enable/interrupt status registers */

#define INT_NREGS ((NR_IRQS + 31) >> 5)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: jz4780_prioritize_irq
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint32_t regval;

  /* Disable all interrupts */

  putreg32(0xffffffff, ICMSR0);
  putreg32(0xffffffff, ICMSR1);

  cp0_putebase(0x80000000);

  /* Set the INTCTL vector spacing to non-zero */

  regval = cp0_getintctl() & ~(0b1111 << 5);
  regval |= 1 << 5;
  cp0_putintctl(regval);

  /* Set the IV bit in the CAUSE register */

  cp0_putcause(cp0_getcause() | CP0_CAUSE_IV);

  /* Clear the EXL and BEV bits in the STATUS register */

  regval = cp0_getstatus();
  regval &= ~(CP0_STATUS_ERL | CP0_STATUS_EXL | CP0_STATUS_BEV);
  cp0_putstatus(regval);

  /* Initialize GPIO change notification handling */

#ifdef CONFIG_JZ4780_GPIOIRQ
  jz4780_gpioirqinitialize();
#endif

  /* Attach and enable software interrupts */

  irq_attach(IRQ_SWINT0, mips_swint0, NULL);
  up_enable_irq(IRQ_SWINT0);

  /* And finally, enable interrupts */

  /* Enable Interrupts */

  asm volatile("ei    %0" : "=r"(regval));

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* Then enable all interrupt levels */

  up_irq_restore(CP0_STATUS_IM_ALL);
#else
  /* Enable only software interrupts */

  up_irq_restore(CP0_STATUS_IM_SWINTS);
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
  DEBUGASSERT((unsigned)irq < NR_IRQS);

  if (irq >= IRQ_TMR0 && irq <= IRQ_TMR7)
    {
      uint32_t n = irq - IRQ_TMR0;
      putreg32(TMCR_FMCL(n), TMSR);
    }
  else if (irq < 32)
    {
      if (irq == JZ4780_IRQ_TCU1)
        {
          putreg32(TMCR_FMCL5, TMSR);
        }

      putreg32(1 << irq, ICMSR0);
    }
  else if (irq < 64)
    {
      putreg32(1 << (irq % 32), ICMSR1);
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
  DEBUGASSERT((unsigned)irq < NR_IRQS);

  if (irq >= IRQ_TMR0 && irq <= IRQ_TMR7)
    {
      uint32_t n = irq - IRQ_TMR0;
      putreg32(TMCR_FMCL(n), TMCR);
      putreg32(1 << JZ4780_IRQ_TCU2, ICMCR0);
    }
  else if (irq < 32)
    {
      if (irq == JZ4780_IRQ_TCU1)
        {
          putreg32(TMCR_FMCL5, TMCR);
        }

      putreg32(1 << irq, ICMCR0);
    }
  else if (irq < 64)
    {
      putreg32(1 << (irq % 32), ICMCR1);
    }
}

/****************************************************************************
 * Name: up_pending_irq
 *
 * Description:
 *   Return true if the interrupt is pending and unmasked.
 *
 ****************************************************************************/

bool up_pending_irq(int irq)
{
  DEBUGASSERT((unsigned)irq < NR_IRQS);

  if (irq < 32)
    {
      return getreg32(ICPR0) & 1 << irq;
    }
  else if (irq < 64)
    {
      return getreg32(ICPR1) & 1 << (irq % 32);
    }

  return false;
}

/****************************************************************************
 * Name: up_clrpend_irq
 *
 * Description:
 *   Clear any pending interrupt
 *
 ****************************************************************************/

void mips_clrpend_irq(int irq)
{
  /* Acknowledge the interrupt by clearing the associated bit in the IFS
   * register.  It is necessary to do this BEFORE lowering the interrupt
   * priority level otherwise recursive interrupts would occur.
   */
}

void mips_clrpend_sw0(void)
{
  mips_clrpend_irq(IRQ_SWINT0);
}
