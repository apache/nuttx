/****************************************************************************
 * arch/risc-v/src/erbium/erbium_irq.c
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
#include <nuttx/irq.h>

#include "chip.h"
#include "riscv_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   Initialize hart 0 interrupt state and PLIC priorities, attach the
 *   common exception handlers, and enable interrupts unless suppressed.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  int id;

  up_irq_save();
  WRITE_CSR(CSR_IE, 0);
  putreg32(0, ERBIUM_PLIC_ENABLE1);

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  riscv_stack_color(g_intstackalloc, CONFIG_ARCH_INTERRUPTSTACK & ~15);
#endif

  for (id = 1; id <= ERBIUM_PLIC_NDEV; id++)
    {
      putreg32(1, ERBIUM_PLIC_PRIORITY + 4 * id);
    }

  putreg32(0, ERBIUM_PLIC_THRESHOLD);
  riscv_exception_attach();

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Mask a supported hart-local interrupt or PLIC source. Ignore numbers
 *   that do not identify a supported interrupt.
 *
 * Input Parameters:
 *   irq - NuttX interrupt number to disable.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  if (irq == RISCV_IRQ_SOFT)
    {
      CLEAR_CSR(CSR_IE, IE_SIE);
    }
  else if (irq == RISCV_IRQ_TIMER)
    {
      CLEAR_CSR(CSR_IE, IE_TIE);
    }
  else if (irq == RISCV_IRQ_EXT)
    {
      CLEAR_CSR(CSR_IE, IE_EIE);
    }
  else if (irq > RISCV_IRQ_EXT && irq < NR_IRQS)
    {
      modifyreg32(ERBIUM_PLIC_ENABLE1, 1u << (irq - RISCV_IRQ_EXT), 0);
    }
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Unmask a supported hart-local interrupt or PLIC source. Ignore numbers
 *   that do not identify a supported interrupt.
 *
 * Input Parameters:
 *   irq - NuttX interrupt number to enable.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  if (irq == RISCV_IRQ_SOFT)
    {
      SET_CSR(CSR_IE, IE_SIE);
    }
  else if (irq == RISCV_IRQ_TIMER)
    {
      SET_CSR(CSR_IE, IE_TIE);
    }
  else if (irq == RISCV_IRQ_EXT)
    {
      SET_CSR(CSR_IE, IE_EIE);
    }
  else if (irq > RISCV_IRQ_EXT && irq < NR_IRQS)
    {
      modifyreg32(ERBIUM_PLIC_ENABLE1, 0, 1u << (irq - RISCV_IRQ_EXT));
    }
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Enable machine external interrupt delivery and global interrupts.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The previous status register value, for restoring interrupt state.
 *
 ****************************************************************************/

irqstate_t up_irq_enable(void)
{
  up_enable_irq(RISCV_IRQ_EXT);
  return READ_AND_SET_CSR(CSR_STATUS, STATUS_IE);
}
