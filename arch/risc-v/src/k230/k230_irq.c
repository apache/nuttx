/****************************************************************************
 * arch/risc-v/src/k230/k230_irq.c
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

#define STATUS_LOW  (READ_CSR(CSR_STATUS) & 0xffffffff) /* STATUS low part */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Disable Machine interrupts */

  up_irq_save();

  /* Disable all global interrupts */

  putreg32(0x0, K230_PLIC_ENABLE1);
  putreg32(0x0, K230_PLIC_ENABLE2);

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
  riscv_stack_color(g_intstackalloc, intstack_size);
#endif

  /* Set priority for all global interrupts to 1 (lowest) */

  int id;

  for (id = 1; id <= NR_IRQS; id++)
    {
      putreg32(1, (uintptr_t)(K230_PLIC_PRIORITY + 4 * id));
    }

  sinfo("prioritized %d irqs\n", NR_IRQS);

  /* Set irq threshold to 0 (permits all global interrupts) */

  putreg32(0, K230_PLIC_THRESHOLD);

  /* Attach the common interrupt handler */

  riscv_exception_attach();

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
      /* Read m/sstatus & clear machine software interrupt enable in m/sie */

      CLEAR_CSR(CSR_IE, IE_SIE);
    }
  else if (irq == RISCV_IRQ_TIMER)
    {
      /* Disable timer interrupt in m/sie */

      CLEAR_CSR(CSR_IE, IE_TIE);
    }
  else if (irq > RISCV_IRQ_EXT)
    {
      extirq = irq - RISCV_IRQ_EXT;

      /* Clear enable bit for the irq */

      if (0 <= extirq && extirq <= K230_PLIC_IRQS)
        {
          modifyreg32(K230_PLIC_ENABLE1 + (4 * (extirq / 32)),
                      1 << (extirq % 32), 0);
        }
      else
        {
          PANIC();
        }
    }

  sinfo("ie=%lx sts=%lx irq=%d\n", READ_CSR(CSR_IE), STATUS_LOW, irq);
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
      /* Read m/sstatus & set machine software interrupt enable in m/sie */

      SET_CSR(CSR_IE, IE_SIE);
    }
  else if (irq == RISCV_IRQ_TIMER)
    {
      /* Enable timer interrupt in m/sie */

      SET_CSR(CSR_IE, IE_TIE);
    }
  else if (irq > RISCV_IRQ_EXT)
    {
      extirq = irq - RISCV_IRQ_EXT;

      /* Enable the irq in PLIC */

      if (0 <= extirq && extirq < K230_PLIC_IRQS)
        {
          modifyreg32(K230_PLIC_ENABLE1 + (4 * (extirq / 32)),
                      0, 1 << (extirq % 32));
        }
      else
        {
          PANIC();
        }
    }

  sinfo("ie=%lx sts=%lx irq=%d\n", READ_CSR(CSR_IE), STATUS_LOW, irq);
}

irqstate_t up_irq_enable(void)
{
  irqstate_t oldstat;

  /* Enable external interrupts (mie/sie) */

  SET_CSR(CSR_IE, IE_EIE);

  /* Read and enable global interrupts (M/SIE) in m/sstatus */

  oldstat = READ_AND_SET_CSR(CSR_STATUS, STATUS_IE);
  sinfo("ie=%lx sts=%lx ctx=%d\n", READ_CSR(CSR_IE), STATUS_LOW,
        XCPTCONTEXT_SIZE);

  return oldstat;
}
