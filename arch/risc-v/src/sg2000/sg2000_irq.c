/****************************************************************************
 * arch/risc-v/src/sg2000/sg2000_irq.c
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

  putreg32(0x0, SG2000_PLIC_ENABLE1);
  putreg32(0x0, SG2000_PLIC_ENABLE2);

  /* Clear pendings in PLIC */

  claim = getreg32(SG2000_PLIC_CLAIM);
  putreg32(claim, SG2000_PLIC_CLAIM);

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
  riscv_stack_color(g_intstackalloc, intstack_size);
#endif

  /* Set priority for all global interrupts to 1 (lowest) */

  int id;

  for (id = 1; id <= NR_IRQS; id++)
    {
      putreg32(1, (uintptr_t)(SG2000_PLIC_PRIORITY + 4 * id));
    }

  /* Set irq threshold to 0 (permits all global interrupts) */

  putreg32(0, SG2000_PLIC_THRESHOLD);

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

      /* Clear enable bit for the irq */

      if (0 <= extirq && extirq <= 63)
        {
          modifyreg32(SG2000_PLIC_ENABLE1 + (4 * (extirq / 32)),
                      1 << (extirq % 32), 0);
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

      /* Set enable bit for the irq */

      if (0 <= extirq && extirq <= 63)
        {
          modifyreg32(SG2000_PLIC_ENABLE1 + (4 * (extirq / 32)),
                      0, 1 << (extirq % 32));
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

  return oldstat;
}
