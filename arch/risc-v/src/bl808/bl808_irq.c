/****************************************************************************
 * arch/risc-v/src/bl808/bl808_irq.c
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
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "riscv_internal.h"
#include "riscv_ipi.h"
#include "chip.h"

#include "hardware/bl808_m0ic.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: m0ic_mask_irq
 *
 * Description:
 *   Masks or unmasks an interrupt in the M0 Interrupt Controller.
 *
 * Input parameters:
 *   irq  - IRQ number to mask or unmask
 *   mask - 0 to unmask (enable), 1 to mask (disable)
 *
 ****************************************************************************/

void m0ic_mask_irq(int irq, bool mask)
{
  int m0_extirq = irq - RISCV_IRQ_EXT
    - BL808_M0_IRQ_OFFSET - BL808_IRQ_NUM_BASE;

  if (mask)
    {
      modifyreg32(BL808_M0IC_MASK(m0_extirq / 32),
                  0, 1 << (m0_extirq % 32));
    }
  else
    {
      modifyreg32(BL808_M0IC_MASK(m0_extirq / 32),
                  1 << (m0_extirq % 32), 0);
    }
}

/****************************************************************************
 * Name: m0ic_interrupt
 *
 * Description:
 *   Interrupt handler for M0 interrupt controller. Reads status registers
 *   to find source, and dispatches the appropriate handler.
 *
 ****************************************************************************/

static int __m0ic_interrupt(int irq, void *context, void *arg)
{
  uint32_t status_0 = getreg32(BL808_M0IC_STATUS(0));
  uint32_t status_1 = getreg32(BL808_M0IC_STATUS(1));

  /* Check status_0 for interrupt source */

  int m0_extirq = ffs(status_0) - 1;
  if (m0_extirq < 0)
    {
      /* Source not in status_0. Check status_1 */

      m0_extirq = ffs(status_1) + 32 - 1;
      if (m0_extirq < 32)
        {
          /* Interrupt goes off on startup without any
           * status bits set. When this happens, just return.
           */

          return OK;
        }
    }

  int irqn = m0_extirq + BL808_IRQ_NUM_BASE
    + BL808_M0_IRQ_OFFSET + RISCV_IRQ_SEXT;

  irq_dispatch(irqn, NULL);

  putreg32(status_0, BL808_M0IC_CLEAR(0));
  putreg32(status_1, BL808_M0IC_CLEAR(1));

  /* M0IC interrupts respond to the rising edge of the
   * source interrupts. If the source is held high but
   * the M0IC interrupt is cleared, the interrupt
   * never happens. So, use masks to refresh the interrupt.
   */

  m0ic_mask_irq(irqn, 1);
  m0ic_mask_irq(irqn, 0);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  uintptr_t claim;

  /* Disable S-Mode interrupts */

  up_irq_save();

  /* Attach the common interrupt handler */

  riscv_exception_attach();

  /* Disable all global interrupts */

  putreg32(0x0, BL808_PLIC_ENABLE1);
  putreg32(0x0, BL808_PLIC_ENABLE2);

  /* Clear pendings in PLIC */

  claim = getreg32(BL808_PLIC_CLAIM);
  putreg32(claim, BL808_PLIC_CLAIM);

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
  riscv_stack_color(g_intstackalloc, intstack_size);
#endif

  /* Set priority for all global interrupts to 1 (lowest) */

  int id;

  for (id = 1; id <= NR_IRQS; id++)
    {
      putreg32(1, (uintptr_t)(BL808_PLIC_PRIORITY + 4 * id));
    }

  /* Set irq threshold to 0 (permits all global interrupts) */

  putreg32(0, BL808_PLIC_THRESHOLD);

#ifdef CONFIG_SMP
  /* Clear IPI for CPU0 */

  riscv_ipi_clear(0);

  up_enable_irq(RISCV_IRQ_SOFT);
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS

  /* And finally, enable interrupts */

  up_irq_enable();
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
  int extirq;

  if (irq == RISCV_IRQ_SOFT)
    {
      /* Read sstatus & clear software interrupt enable in sie */

      CLEAR_CSR(CSR_IE, IE_SIE);
    }
  else if (irq == RISCV_IRQ_TIMER)
    {
      /* Read sstatus & clear timer interrupt enable in sie */

      CLEAR_CSR(CSR_IE, IE_TIE);
    }
  else if (irq > RISCV_IRQ_EXT)
    {
      extirq = irq - RISCV_IRQ_EXT;

      if (0 <= extirq && extirq <= BL808_D0_MAX_EXTIRQ)
        {
          /* Clear enable bit for the irq */

          modifyreg32(BL808_PLIC_ENABLE1 + (4 * (extirq / 32)),
                      1 << (extirq % 32), 0);
        }
      else if ((BL808_D0_MAX_EXTIRQ + 1) <= extirq
               && extirq <= (BL808_M0_MAX_EXTIRQ
                             + BL808_M0_IRQ_OFFSET))
        {
          m0ic_mask_irq(irq, 1);
        }
      else
        {
          PANIC();
        }
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
  int extirq;

  if (irq == RISCV_IRQ_SOFT)
    {
      /* Read sstatus & set software interrupt enable in sie */

      SET_CSR(CSR_IE, IE_SIE);
    }
  else if (irq == RISCV_IRQ_TIMER)
    {
      /* Read sstatus & set timer interrupt enable in sie */

      SET_CSR(CSR_IE, IE_TIE);
    }
  else if (irq > RISCV_IRQ_EXT)
    {
      extirq = irq - RISCV_IRQ_EXT;

      if (0 <= extirq && extirq <= BL808_D0_MAX_EXTIRQ)
        {
          /* Set enable bit for the irq */

          modifyreg32(BL808_PLIC_ENABLE1 + (4 * (extirq / 32)),
                      0, 1 << (extirq % 32));
        }
      else if ((BL808_D0_MAX_EXTIRQ + 1) <= extirq
               && extirq <= (BL808_M0_MAX_EXTIRQ
                             + BL808_M0_IRQ_OFFSET))
        {
          m0ic_mask_irq(irq, 0);
        }
      else
        {
          PANIC();
        }
    }
}

irqstate_t up_irq_enable(void)
{
  irqstate_t oldstat;

  /* Enable external interrupts (sie) */

  SET_CSR(CSR_IE, IE_EIE);

  /* Read and enable global interrupts (sie) in sstatus */

  oldstat = READ_AND_SET_CSR(CSR_STATUS, STATUS_IE);

  /* Enable IRQs from M0IC */

  /* First, clear interrupts */

  putreg32(0xffffffff, BL808_M0IC_CLEAR(0));
  putreg32(0xffffffff, BL808_M0IC_CLEAR(1));

  /* Mask all sources */

  putreg32(0xffffffff, BL808_M0IC_MASK(0));
  putreg32(0xffffffff, BL808_M0IC_MASK(1));

  int ret = irq_attach(BL808_IRQ_M0IC, __m0ic_interrupt, NULL);
  if (ret == OK)
    {
      up_enable_irq(BL808_IRQ_M0IC);
    }
  else
    {
      PANIC();
    }

  return oldstat;
}
