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
#include <arch/irq.h>

#include "riscv_internal.h"
#include "riscv_arch.h"

#include "mpfs.h"

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

  /* Disable timer interrupt (in case of hotloading with debugger) */

  up_disable_irq(MPFS_IRQ_MTIMER);

  /* enable access from supervisor mode */

  putreg32(0x1, MPFS_PLIC_CTRL);

  /* Disable all global interrupts for current hart */

  uint64_t hart_id = READ_CSR(mhartid);

  /* hart0 is E51 we can't run on that (need different irq handling) */

  DEBUGASSERT(hart_id != 0);

  uint32_t *miebase = (uint32_t *)(MPFS_PLIC_H1_MIE0 +
                                  (hart_id - 1) * MPFS_HART_MIE_OFFSET);

  putreg32(0x0, miebase + 0);
  putreg32(0x0, miebase + 1);
  putreg32(0x0, miebase + 2);
  putreg32(0x0, miebase + 3);
  putreg32(0x0, miebase + 4);
  putreg32(0x0, miebase + 5);

  /* Clear pendings in PLIC (for current hart) */

  uintptr_t claim_address = MPFS_PLIC_H1_MCLAIM +
                            ((hart_id - 1) * MPFS_PLIC_NEXTHART_OFFSET);
  uint32_t val = getreg32(claim_address);
  putreg32(val, claim_address);

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
  riscv_stack_color((FAR void *)&g_intstackalloc, intstack_size);
#endif

  /* Set priority for all global interrupts to 1 (lowest) */

  int id;

  for (id = 1; id <= NR_IRQS; id++)
    {
      putreg32(1, (uintptr_t)(MPFS_PLIC_PRIORITY + (4 * id)));
    }

  /* Set irq threshold to 0 (permits all global interrupts) */

  uint32_t *threshold_address = (uint32_t *)(MPFS_PLIC_H1_MTHRESHOLD +
                                ((hart_id - 1) * MPFS_PLIC_NEXTHART_OFFSET));
  putreg32(0, threshold_address);

  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

  /* Attach the ecall interrupt handler */

  irq_attach(MPFS_IRQ_ECALLM, riscv_swint, NULL);

#ifdef CONFIG_BUILD_PROTECTED
  irq_attach(MPFS_IRQ_ECALLU, riscv_swint, NULL);
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

  if (irq == MPFS_IRQ_MSOFT)
    {
      /* Read mstatus & clear machine software interrupt enable in mie */

      asm volatile ("csrrc %0, mie, %1": "=r" (oldstat) : "r"(MIE_MSIE));
    }
  else if (irq == MPFS_IRQ_MTIMER)
    {
      /* Read mstatus & clear machine timer interrupt enable in mie */

      asm volatile ("csrrc %0, mie, %1": "=r" (oldstat) : "r"(MIE_MTIE));
    }
  else if (irq >= MPFS_IRQ_EXT_START)
    {
      extirq = irq - MPFS_IRQ_EXT_START;

      /* Clear enable bit for the irq */

      uint64_t hart_id = READ_CSR(mhartid);
      uintptr_t miebase = MPFS_PLIC_H1_MIE0 +
                          ((hart_id - 1) * MPFS_HART_MIE_OFFSET);

      if (0 <= extirq && extirq <= NR_IRQS - MPFS_IRQ_EXT_START)
        {
          modifyreg32(miebase + (4 * (extirq / 32)), 1 << (extirq % 32), 0);
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

  if (irq == MPFS_IRQ_MSOFT)
    {
      /* Read mstatus & set machine software interrupt enable in mie */

      asm volatile ("csrrs %0, mie, %1": "=r" (oldstat) : "r"(MIE_MSIE));
    }
  else if (irq == MPFS_IRQ_MTIMER)
    {
      /* Read mstatus & set machine timer interrupt enable in mie */

      asm volatile ("csrrs %0, mie, %1": "=r" (oldstat) : "r"(MIE_MTIE));
    }
  else if (irq >= MPFS_IRQ_EXT_START)
    {
      extirq = irq - MPFS_IRQ_EXT_START;

      /* Set enable bit for the irq */

      uint64_t hart_id = READ_CSR(mhartid);
      uintptr_t miebase = MPFS_PLIC_H1_MIE0 +
                          ((hart_id - 1) * MPFS_HART_MIE_OFFSET);

      if (0 <= extirq && extirq <= NR_IRQS - MPFS_IRQ_EXT_START)
        {
          modifyreg32(miebase + (4 * (extirq / 32)), 0, 1 << (extirq % 32));
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

  asm volatile ("csrrs %0, mie, %1": "=r" (oldstat) : "r"(MIE_MEIE));

  /* Read mstatus & set machine interrupt enable (MIE) in mstatus */

  asm volatile ("csrrs %0, mstatus, %1": "=r" (oldstat) : "r"(MSTATUS_MIE));
  return oldstat;
}
