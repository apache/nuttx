/****************************************************************************
 * arch/risc-v/src/k210/k210_irq.c
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
#include "k210.h"

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

  putreg32(0x0, K210_PLIC_ENABLE1);
  putreg32(0x0, K210_PLIC_ENABLE2);

  /* Clear pendings in PLIC */

  uint32_t val = getreg32(K210_PLIC_CLAIM);
  putreg32(val, K210_PLIC_CLAIM);

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  size_t intstack_size = 0;
  intstack_size = ((CONFIG_ARCH_INTERRUPTSTACK * CONFIG_SMP_NCPUS) & ~15);
  riscv_stack_color(g_intstackalloc, intstack_size);
#endif

  /* Set priority for all global interrupts to 1 (lowest) */

  int id;

  for (id = 1; id <= 52; id++)
    {
      putreg32(1, (uintptr_t)K210_PLIC_PRIORITY + (4 * id));
    }

  /* Set irq threshold to 0 (permits all global interrupts) */

  putreg32(0, K210_PLIC_THRESHOLD);

  /* Attach the common interrupt handler */

  riscv_exception_attach();

#ifdef CONFIG_SMP
  /* Clear RISCV_IPI for CPU0 */

  putreg32(0, RISCV_IPI);

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

  if (irq == RISCV_IRQ_MSOFT)
    {
      /* Read mstatus & clear machine software interrupt enable in mie */

      CLEAR_CSR(mie, MIE_MSIE);
    }
  else if (irq == RISCV_IRQ_MTIMER)
    {
      /* Read mstatus & clear machine timer interrupt enable in mie */

      CLEAR_CSR(mie, MIE_MTIE);
    }
  else if (irq > RISCV_IRQ_MEXT)
    {
      extirq = irq - RISCV_IRQ_MEXT;

      /* Clear enable bit for the irq */

      if (0 <= extirq && extirq <= 63)
        {
          modifyreg32(K210_PLIC_ENABLE1 + (4 * (extirq / 32)),
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

  if (irq == RISCV_IRQ_MSOFT)
    {
      /* Read mstatus & set machine software interrupt enable in mie */

      SET_CSR(mie, MIE_MSIE);
    }
  else if (irq == RISCV_IRQ_MTIMER)
    {
      /* Read mstatus & set machine timer interrupt enable in mie */

      SET_CSR(mie, MIE_MTIE);
    }
  else if (irq > RISCV_IRQ_MEXT)
    {
      extirq = irq - RISCV_IRQ_MEXT;

      /* Set enable bit for the irq */

      if (0 <= extirq && extirq <= 63)
        {
          modifyreg32(K210_PLIC_ENABLE1 + (4 * (extirq / 32)),
                      0, 1 << (extirq % 32));
        }
      else
        {
          PANIC();
        }
    }
}

/****************************************************************************
 * Name: riscv_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void riscv_ack_irq(int irq)
{
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Return the current interrupt state and enable interrupts
 *
 ****************************************************************************/

irqstate_t up_irq_enable(void)
{
  irqstate_t oldstat;

#if 1
  /* Enable MEIE (machine external interrupt enable) */

  /* TODO: should move to up_enable_irq() */

  SET_CSR(mie, MIE_MEIE);
#endif

  /* Read mstatus & set machine interrupt enable (MIE) in mstatus */

  oldstat = READ_AND_SET_CSR(mstatus, MSTATUS_MIE);
  return oldstat;
}
