/****************************************************************************
 * arch/risc-v/src/litex/litex_irq.c
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
#include "litex.h"

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

  asm volatile ("csrw %0, %1" :: "i"(LITEX_MMASK_CSR), "r"(0));

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
  riscv_stack_color(g_intstackalloc, intstack_size);
#endif

  /* litex vexriscv dont have priority and threshold control */

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
  int extirq;
  int mask;

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
      extirq--;

      /* Clear enable bit for the irq */

      if (0 <= extirq && extirq <= 31)
        {
          asm volatile ("csrr %0, %1" : "=r"(mask) : "i"(LITEX_MMASK_CSR));
          mask &= ~(1 << extirq);
          asm volatile ("csrw %0, %1" :: "i"(LITEX_MMASK_CSR), "r"(mask));
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
  int mask;

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
      extirq--;

      /* Set enable bit for the irq */

      if (0 <= extirq && extirq <= 31)
        {
          asm volatile ("csrr %0, %1" : "=r"(mask) : "i"(LITEX_MMASK_CSR));
          mask |= (1 << extirq);
          asm volatile ("csrw %0, %1" :: "i"(LITEX_MMASK_CSR), "r"(mask));
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

#if 1
  /* Enable MEIE (machine external interrupt enable) */

  /* TODO: should move to up_enable_irq() */

  SET_CSR(mie, MIE_MEIE);
#endif

  /* Read mstatus & set machine interrupt enable (MIE) in mstatus */

  oldstat = READ_AND_SET_CSR(mstatus, MSTATUS_MIE);
  return oldstat;
}
