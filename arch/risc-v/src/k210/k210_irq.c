/****************************************************************************
 * arch/risc-v/src/k210/k210_irq.c
 *
 *   Copyright (C) 2019 Masayuki Ishikawa. All rights reserved.
 *   Author: Masayuki Ishikawa <masayuki.ishikawa@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/irq.h>

#include "riscv_internal.h"
#include "riscv_arch.h"

#include "k210.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_SMP
/* For the case of configurations with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

volatile uint64_t *g_current_regs[CONFIG_SMP_NCPUS];
#else
volatile uint64_t *g_current_regs[1];
#endif

#ifdef CONFIG_SMP
extern int riscv_pause_handler(int irq, void *c, FAR void *arg);
#endif

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

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 7
  size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~7);
  up_stack_color((FAR void *)((uintptr_t)&g_intstackbase - intstack_size),
                 intstack_size);
#endif

  /* Set priority for all global interrupts to 1 (lowest) */

  int id;

  for (id = 1; id <= 52; id++)
    {
      putreg32(1, (uintptr_t)K210_PLIC_PRIORITY + (4 * id));
    }

  /* Set irq threshold to 0 (permits all global interrupts) */

  putreg32(0, K210_PLIC_THRESHOLD);

  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

  /* Attach the ecall interrupt handler */

  irq_attach(K210_IRQ_ECALLM, up_swint, NULL);

#ifdef CONFIG_BUILD_PROTECTED
  irq_attach(K210_IRQ_ECALLU, up_swint, NULL);
#endif

#ifdef CONFIG_SMP
  /* Clear MSOFT for CPU0 */

  putreg32(0, K210_CLINT_MSIP);

  /* Setup MSOFT for CPU0 with pause handler */

  irq_attach(K210_IRQ_MSOFT, riscv_pause_handler, NULL);
  up_enable_irq(K210_IRQ_MSOFT);
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
  uint64_t oldstat;

  if (irq == K210_IRQ_MSOFT)
    {
      /* Read mstatus & clear machine software interrupt enable in mie */

      asm volatile ("csrrc %0, mie, %1": "=r" (oldstat) : "r"(MIE_MSIE));
    }
  else if (irq == K210_IRQ_MTIMER)
    {
      /* Read mstatus & clear machine timer interrupt enable in mie */

      asm volatile ("csrrc %0, mie, %1": "=r" (oldstat) : "r"(MIE_MTIE));
    }
  else if (irq > K210_IRQ_MEXT)
    {
      extirq = irq - K210_IRQ_MEXT;

      /* Clear enable bit for the irq */

      if (0 <= extirq && extirq <= 63)
        {
          modifyreg32(K210_PLIC_ENABLE1 + (4 * (extirq / 32)),
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

  if (irq == K210_IRQ_MSOFT)
    {
      /* Read mstatus & set machine software interrupt enable in mie */

      asm volatile ("csrrs %0, mie, %1": "=r" (oldstat) : "r"(MIE_MSIE));
    }
  else if (irq == K210_IRQ_MTIMER)
    {
      /* Read mstatus & set machine timer interrupt enable in mie */

      asm volatile ("csrrs %0, mie, %1": "=r" (oldstat) : "r"(MIE_MTIE));
    }
  else if (irq > K210_IRQ_MEXT)
    {
      extirq = irq - K210_IRQ_MEXT;

      /* Set enable bit for the irq */

      if (0 <= extirq && extirq <= 63)
        {
          modifyreg32(K210_PLIC_ENABLE1 + (4 * (extirq / 32)),
                      0, 1 << (extirq % 32));
        }
      else
        {
          ASSERT(false);
        }
    }
}

/****************************************************************************
 * Name: up_get_newintctx
 *
 * Description:
 *   Return initial mstatus when a task is created.
 *
 ****************************************************************************/

uint32_t up_get_newintctx(void)
{
  /* Set machine previous privilege mode to machine mode. Reegardless of
   * how NuttX is configured and of what kind of thread is being started.
   * That is because all threads, even user-mode threads will start in
   * kernel trampoline at nxtask_start() or pthread_start().
   * The thread's privileges will be dropped before transitioning to
   * user code. Also set machine previous interrupt enable.
   */

  return (MSTATUS_MPPM | MSTATUS_MPIE);
}

/****************************************************************************
 * Name: up_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void up_ack_irq(int irq)
{
}

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Return the current interrupt state and disable interrupts
 *
 ****************************************************************************/

irqstate_t up_irq_save(void)
{
  uint64_t oldstat;

  /* Read mstatus & clear machine interrupt enable (MIE) in mstatus */

  asm volatile ("csrrc %0, mstatus, %1": "=r" (oldstat) : "r"(MSTATUS_MIE));
  return oldstat;
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore previous IRQ mask state
 *
 ****************************************************************************/

void up_irq_restore(irqstate_t flags)
{
  /* Write flags to mstatus */

  asm volatile("csrw mstatus, %0" : /* no output */ : "r" (flags));
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

#if 1
  /* Enable MEIE (machine external interrupt enable) */

  /* TODO: should move to up_enable_irq() */

  asm volatile ("csrrs %0, mie, %1": "=r" (oldstat) : "r"(MIE_MEIE));
#endif

  /* Read mstatus & set machine interrupt enable (MIE) in mstatus */

  asm volatile ("csrrs %0, mstatus, %1": "=r" (oldstat) : "r"(MSTATUS_MIE));
  return oldstat;
}
