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
#include <nuttx/irq.h>

#include "riscv_internal.h"
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
  riscv_stack_color((void *)&g_intstackalloc, intstack_size);
#endif

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
  if (irq == RISCV_IRQ_MSOFT)
    {
      /* Read mstatus & clear machine software interrupt enable in mie */

      CLEAR_CSR(mie, MIE_MSIE);
    }
  else if (irq == RISCV_IRQ_MTIMER)
    {
      putreg8(0, CLIC_TIMER_ENABLE_ADDRESS);

      /* Read mstatus & clear machine timer interrupt enable in mie */

      CLEAR_CSR(mie, MIE_MTIE);
    }
  else
    {
      ASSERT(irq < 64 + 16 + RISCV_IRQ_ASYNC);
      bl_irq_disable(irq - RISCV_IRQ_ASYNC);
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
  if (irq == RISCV_IRQ_MSOFT)
    {
      /* Read mstatus & set machine software interrupt enable in mie */

      SET_CSR(mie, MIE_MSIE);
    }
  else if (irq == RISCV_IRQ_MTIMER)
    {
      putreg8(1, CLIC_TIMER_ENABLE_ADDRESS);

      /* Read mstatus & set machine timer interrupt enable in mie */

      SET_CSR(mie, MIE_MTIE | 0x1 << 11);
    }
  else
    {
      ASSERT(irq < 64 + 16 + RISCV_IRQ_ASYNC);
      bl_irq_enable(irq - RISCV_IRQ_ASYNC);
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
  uint32_t oldstat;

  /* Enable MEIE (machine external interrupt enable) */

  SET_CSR(mie, MIE_MEIE);

  /* Read mstatus & set machine interrupt enable (MIE) in mstatus */

  oldstat = READ_AND_SET_CSR(mstatus, MSTATUS_MIE);
  return oldstat;
}
