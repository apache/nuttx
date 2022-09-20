/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_irq.c
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
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "rv32m1.h"
#include "hardware/rv32m1_eu.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rv32m1_intmuxisr
 ****************************************************************************/

LOCATE_ITCM
static int rv32m1_intmuxisr(int irq, void *context, void *arg)
{
  UNUSED(irq);
  UNUSED(context);
  UNUSED(arg);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  int i;
  uint32_t base;
  uint32_t regval;
  uint32_t regaddr;

  /* Disable Machine interrupts */

  up_irq_save();

  rv32m1_pcc_clock_enable(RV32M1_PCC_INTMUX0);

  /* reset all the intmux channels */

  for (i = 0; i < 8; ++i)
    {
      base = RV32M1_INTMUX_CH_BASE(i);
      regaddr = base + INTMUX_CH_CSR_OFFSET;
      regval = getreg32(regaddr);
      regval |= INTMUX_CSR_RST;
      putreg32(regval, regaddr);
    }

  /* Disable all global interrupts */

  putreg32(0, RV32M1_EU_INTPTEN);

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
  riscv_stack_color((void *)((uintptr_t)&g_intstacktop - intstack_size),
                 intstack_size);
#endif

  /* Clear all pending flags */

  putreg32(0xffffffff, RV32M1_EU_INTPTPENDCLR);
  putreg32(0xffffffff, RV32M1_EU_EVTPENDCLR);
  putreg32(0xffffffff, RV32M1_EU_INTPTSECURE);

  /* Attach INTMUX ISR */

  irq_attach(RV32M1_IRQ_INTMUX0, rv32m1_intmuxisr, NULL);

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

LOCATE_ITCM
void up_disable_irq(int irq)
{
  int extirq;
  uint32_t regval;

  if (irq >= RV32M1_IRQ_MEXT)
    {
      if (irq >= RV32M1_IRQ_INTMUX)
        {
          int const subirq = irq - RV32M1_IRQ_INTMUX;
          int const chn = 0;
          uint32_t regaddr = RV32M1_INTMUX_CH_BASE(chn) +
                             INTMUX_CH_IER_OFFSET;

          regval = getreg32(regaddr);
          regval &= ~(1 << subirq);
          putreg32(regval, regaddr);
        }
      else
        {
          extirq = irq - RV32M1_IRQ_MEXT;

          /* Clear enable bit for the irq */

          regval = getreg32(RV32M1_EU_INTPTEN);
          regval &= ~(1 << extirq);
          putreg32(regval, RV32M1_EU_INTPTEN);
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

LOCATE_ITCM
void up_enable_irq(int irq)
{
  int extirq;
  uint32_t regval;

  if (irq >= RV32M1_IRQ_MEXT)
    {
      if (irq >= RV32M1_IRQ_INTMUX)
        {
          int const subirq = irq - RV32M1_IRQ_INTMUX;
          int const chn = 0;
          uint32_t regaddr = RV32M1_INTMUX_CH_BASE(chn) +
                             INTMUX_CH_IER_OFFSET;

          regval = getreg32(regaddr);
          regval |= 1 << subirq;
          putreg32(regval, regaddr);

          extirq = RV32M1_IRQ_INTMUX0 - RV32M1_IRQ_MEXT;
        }
      else
        {
          extirq = irq - RV32M1_IRQ_MEXT;
        }

      regval = getreg32(RV32M1_EU_INTPTEN);

      /* Set enable bit for the irq */

      regval |= 1 << extirq;
      putreg32(regval, RV32M1_EU_INTPTEN);

      /* Read INTPTEN back to make it sure */

      getreg32(RV32M1_EU_INTPTEN);
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
  board_autoled_on(LED_CPU);
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
