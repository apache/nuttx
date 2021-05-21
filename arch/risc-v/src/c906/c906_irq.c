/****************************************************************************
 * arch/risc-v/src/c906/c906_irq.c
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
#include <arch/irq.h>

#include "riscv_internal.h"
#include "riscv_arch.h"

#include "c906.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint64_t *g_current_regs[1];

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

  /* enable access from supervisor mode */

  putreg32(0x1, C906_PLIC_CTRL);

  /* Disable all global interrupts */

  putreg32(0x0, C906_PLIC_MIE0);
  putreg32(0x0, C906_PLIC_MIE1);

  /* Clear pendings in PLIC */

  uint32_t val = getreg32(C906_PLIC_MCLAIM);
  putreg32(val, C906_PLIC_MCLAIM);

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
  riscv_stack_color((FAR void *)&g_intstackalloc, intstack_size);
#endif

  /* Set priority for all global interrupts to 1 (lowest) */

  int id;

  for (id = 1; id <= NR_IRQS; id++)
    {
      putreg32(1, (uintptr_t)(C906_PLIC_PRIORITY + (4 * id)));
    }

  /* Set irq threshold to 0 (permits all global interrupts) */

  putreg32(0, C906_PLIC_MTHRESHOLD);

  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

  /* Attach the ecall interrupt handler */

  irq_attach(C906_IRQ_ECALLM, riscv_swint, NULL);

#ifdef CONFIG_BUILD_PROTECTED
  irq_attach(C906_IRQ_ECALLU, riscv_swint, NULL);
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
  int extirq = 0;
  uint64_t oldstat = 0;

  if (irq == C906_IRQ_MSOFT)
    {
      /* Read mstatus & clear machine software interrupt enable in mie */

      asm volatile ("csrrc %0, mie, %1": "=r" (oldstat) : "r"(MIE_MSIE));
    }
  else if (irq == C906_IRQ_MTIMER)
    {
      /* Read mstatus & clear machine timer interrupt enable in mie */

      asm volatile ("csrrc %0, mie, %1": "=r" (oldstat) : "r"(MIE_MTIE));
    }
  else if (irq >= C906_IRQ_PERI_START)
    {
      extirq = irq - C906_IRQ_PERI_START;

      /* Clear enable bit for the irq */

      if (0 <= extirq && extirq <= 63)
        {
          modifyreg32(C906_PLIC_MIE0 + (4 * (extirq / 32)),
                      1 << (extirq % 32), 0);
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
  uint64_t oldstat;

  if (irq == C906_IRQ_MSOFT)
    {
      /* Read mstatus & set machine software interrupt enable in mie */

      asm volatile ("csrrs %0, mie, %1": "=r" (oldstat) : "r"(MIE_MSIE));
    }
  else if (irq == C906_IRQ_MTIMER)
    {
      /* Read mstatus & set machine timer interrupt enable in mie */

      asm volatile ("csrrs %0, mie, %1": "=r" (oldstat) : "r"(MIE_MTIE));
    }
  else if (irq >= C906_IRQ_PERI_START)
    {
      extirq = irq - C906_IRQ_PERI_START;

      /* Set enable bit for the irq */

      if (0 <= extirq && extirq <= 63)
        {
          modifyreg32(C906_PLIC_MIE0 + (4 * (extirq / 32)),
                      0, 1 << (extirq % 32));
        }
      else
        {
          ASSERT(false);
        }
    }
}

/****************************************************************************
 * Name: riscv_get_newintctx
 *
 * Description:
 *   Return initial mstatus when a task is created.
 *
 ****************************************************************************/

uint32_t riscv_get_newintctx(void)
{
  /* Set machine previous privilege mode to machine mode. Reegardless of
   * how NuttX is configured and of what kind of thread is being started.
   * That is because all threads, even user-mode threads will start in
   * kernel trampoline at nxtask_start() or pthread_start().
   * The thread's privileges will be dropped before transitioning to
   * user code. Also set machine previous interrupt enable.
   */

#ifdef CONFIG_ARCH_FPU
  return (MSTATUS_FS_INIT | MSTATUS_MPPM | MSTATUS_MPIE);
#else
  return (MSTATUS_MPPM | MSTATUS_MPIE);
#endif
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
  uint64_t oldstat;

  /* Enable MEIE (machine external interrupt enable) */

  /* TODO: should move to up_enable_irq() */

  asm volatile ("csrrs %0, mie, %1": "=r" (oldstat) : "r"(MIE_MEIE));

  /* Read mstatus & set machine interrupt enable (MIE) in mstatus */

  asm volatile ("csrrs %0, mstatus, %1": "=r" (oldstat) : "r"(MSTATUS_MIE));
  return oldstat;
}
