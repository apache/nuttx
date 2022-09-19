/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_irq.c
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
#include "mpfs.h"
#include "mpfs_plic.h"

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

  /* Disable all global interrupts for current hart */

  uintptr_t iebase = mpfs_plic_get_iebase();

  putreg32(0x0, iebase + 0);
  putreg32(0x0, iebase + 4);
  putreg32(0x0, iebase + 8);
  putreg32(0x0, iebase + 12);
  putreg32(0x0, iebase + 16);
  putreg32(0x0, iebase + 20);

  /* Clear pendings in PLIC (for current hart) */

  uintptr_t claim_address = mpfs_plic_get_claimbase();
  uint32_t val = getreg32(claim_address);
  putreg32(val, claim_address);

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
  riscv_stack_color(g_intstackalloc, intstack_size);
#endif

  /* Set priority for all global interrupts to 1 (lowest) */

  int id;

  for (id = 1; id <= NR_IRQS; id++)
    {
      putreg32(1, (uintptr_t)(MPFS_PLIC_PRIORITY + (4 * id)));
    }

  /* Set irq threshold to 0 (permits all global interrupts) */

  uintptr_t threshold_address = mpfs_plic_get_thresholdbase();
  putreg32(0, threshold_address);

  /* Attach the common interrupt handler */

  riscv_exception_attach();

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
  int extirq = 0;

  if (irq == RISCV_IRQ_SOFT)
    {
      /* Read m/sstatus & clear machine software interrupt enable in m/sie */

      CLEAR_CSR(CSR_IE, IE_SIE);
    }
  else if (irq == RISCV_IRQ_TIMER)
    {
      /* Read m/sstatus & clear timer interrupt enable in m/sie */

      CLEAR_CSR(CSR_IE, IE_TIE);
    }
  else if (irq >= MPFS_IRQ_EXT_START)
    {
      extirq = irq - MPFS_IRQ_EXT_START;

      /* Clear enable bit for the irq */

      uintptr_t iebase = mpfs_plic_get_iebase();

      if (0 <= extirq && extirq <= NR_IRQS - MPFS_IRQ_EXT_START)
        {
          modifyreg32(iebase + (4 * (extirq / 32)), 1 << (extirq % 32), 0);
        }
      else
        {
          ASSERT(false);
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
      /* Read m/sstatus & set machine software interrupt enable in m/sie */

      SET_CSR(CSR_IE, IE_SIE);
    }
  else if (irq == RISCV_IRQ_TIMER)
    {
      /* Read m/sstatus & set timer interrupt enable in m/sie */

      SET_CSR(CSR_IE, IE_TIE);
    }
  else if (irq >= MPFS_IRQ_EXT_START)
    {
      extirq = irq - MPFS_IRQ_EXT_START;

      /* Set enable bit for the irq */

      uintptr_t iebase = mpfs_plic_get_iebase();

      if (0 <= extirq && extirq <= NR_IRQS - MPFS_IRQ_EXT_START)
        {
          modifyreg32(iebase + (4 * (extirq / 32)), 0, 1 << (extirq % 32));
        }
      else
        {
          ASSERT(false);
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

  /* Enable external interrupts (mie/sie) */

  SET_CSR(CSR_IE, IE_EIE);

  /* Read and enable global interrupts (M/SIE) in m/sstatus */

  oldstat = READ_AND_SET_CSR(CSR_STATUS, STATUS_IE);

  return oldstat;
}
