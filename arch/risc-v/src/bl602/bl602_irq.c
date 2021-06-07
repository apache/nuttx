/****************************************************************************
 * arch/risc-v/src/bl602/bl602_irq.c
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
#include <nuttx/board.h>
#include <arch/irq.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "riscv_arch.h"

#include "hardware/bl602_clic.h"

#include "chip.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void bl_irq_enable(unsigned int source)
{
  putreg8(1, BL602_CLIC_INTIE + source);
}

static inline void bl_irq_disable(unsigned int source)
{
  putreg8(0, BL602_CLIC_INTIE + source);
}

static inline void bl_irq_pending_set(unsigned int source)
{
  putreg8(1, BL602_CLIC_INTIP + source);
}

static inline void bl_irq_pending_clear(unsigned int source)
{
  putreg8(0, BL602_CLIC_INTIP + source);
}

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

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  /* Colorize the interrupt stack for debug purposes */

  size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
  riscv_stack_color((FAR void *)&g_intstackalloc, intstack_size);
#endif

  /* currents_regs is non-NULL only while processing an interrupt */

  g_current_regs = NULL;

  /* Attach the ecall interrupt handler */

  irq_attach(BL602_IRQ_ECALLM, riscv_swint, NULL);

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
  uint32_t oldstat;

  if (irq == BL602_IRQ_MSOFT)
    {
      /* Read mstatus & clear machine software interrupt enable in mie */

      asm volatile("csrrc %0, mie, %1" : "=r"(oldstat) : "r"(MIE_MSIE));
    }
  else if (irq == BL602_IRQ_MTIMER)
    {
      putreg8(0, CLIC_TIMER_ENABLE_ADDRESS);

      /* Read mstatus & clear machine timer interrupt enable in mie */

      asm volatile("csrrc %0, mie, %1" : "=r"(oldstat) : "r"(MIE_MTIE));
    }
  else
    {
      ASSERT(irq < 64 + 16 + BL602_IRQ_ASYNC);
      bl_irq_disable(irq - BL602_IRQ_ASYNC);
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
  uint32_t oldstat;

  if (irq == BL602_IRQ_MSOFT)
    {
      /* Read mstatus & set machine software interrupt enable in mie */

      asm volatile("csrrs %0, mie, %1" : "=r"(oldstat) : "r"(MIE_MSIE));
    }
  else if (irq == BL602_IRQ_MTIMER)
    {
      putreg8(1, CLIC_TIMER_ENABLE_ADDRESS);

      /* Read mstatus & set machine timer interrupt enable in mie */

      asm volatile("csrrs %0, mie, %1"
                   : "=r"(oldstat)
                   : "r"(MIE_MTIE | 0x1 << 11));
    }
  else
    {
      ASSERT(irq < 64 + 16 + BL602_IRQ_ASYNC);
      bl_irq_enable(irq - BL602_IRQ_ASYNC);
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
  /* Set machine previous privilege mode to machine mode.
   * Also set machine previous interrupt enable
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
 * Name: up_irq_save
 *
 * Description:
 *   Return the current interrupt state and disable interrupts
 *
 ****************************************************************************/

irqstate_t up_irq_save(void)
{
  uint32_t oldstat;

  /* Read mstatus & clear machine interrupt enable (MIE) in mstatus */

  asm volatile("csrrc %0, mstatus, %1" : "=r"(oldstat) : "r"(MSTATUS_MIE));
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

  asm volatile("csrw mstatus, %0"
               : /* no output */
               : "r"(flags));
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
  uint32_t oldstat;

  /* Enable MEIE (machine external interrupt enable) */

  asm volatile("csrrs %0, mie, %1" : "=r"(oldstat) : "r"(MIE_MEIE));

  /* Read mstatus & set machine interrupt enable (MIE) in mstatus */

  asm volatile("csrrs %0, mstatus, %1" : "=r"(oldstat) : "r"(MSTATUS_MIE));
  return oldstat;
}
