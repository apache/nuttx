/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_irq.c
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
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "riscv_internal.h"
#include "hardware/gd32vw55x_eclic.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* All interrupts are configured with the same ECLIC level so that NuttX
 * handles them without hardware preemption (the kernel is not built for
 * nested interrupts).
 */

#define ECLIC_IRQ_LEVEL   1

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  int i;

  /* Disable Machine interrupts */

  up_irq_save();

  /* ECLIC global setup: all 4 control bits are level bits, threshold 0 */

  putreg8(4 << ECLIC_CFG_NLBITS_SHIFT, GD32VW55X_ECLIC_CFG);
  putreg8(0, GD32VW55X_ECLIC_MTH);

  /* Every source: disabled, not pending, level-triggered, non-vectored,
   * same level
   */

  for (i = 0; i < GD32VW55X_NIRQS; i++)
    {
      putreg8(0, GD32VW55X_ECLIC_INTIE(i));
      putreg8(0, GD32VW55X_ECLIC_INTIP(i));
      putreg8(ECLIC_INTATTR_TRIG_LEVEL, GD32VW55X_ECLIC_INTATTR(i));
      putreg8(ECLIC_INTCTL_LEVEL(ECLIC_IRQ_LEVEL),
              GD32VW55X_ECLIC_INTCTL(i));
    }

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  /* Colorize the interrupt stack for debug purposes */

  size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
  riscv_stack_color(g_intstackalloc, intstack_size);
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
 *   Disable the IRQ specified by 'irq'.  With the ECLIC every interrupt
 *   source - including the core software (3) and timer (7) inputs - is
 *   gated by its INTIE bit; the classic mie CSR is not used.
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  int extirq = irq - RISCV_IRQ_ASYNC;

  DEBUGASSERT(extirq >= 0 && extirq < GD32VW55X_NIRQS);

  putreg8(0, GD32VW55X_ECLIC_INTIE(extirq));
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
  int extirq = irq - RISCV_IRQ_ASYNC;

  DEBUGASSERT(extirq >= 0 && extirq < GD32VW55X_NIRQS);

  putreg8(ECLIC_INTIE_IE, GD32VW55X_ECLIC_INTIE(extirq));
}

/****************************************************************************
 * Name: gd32vw55x_irq_set_trigger
 *
 * Description:
 *   Select the trigger mode (level / rising edge / falling edge) of an
 *   interrupt source.  The Wi-Fi and BLE glue uses positive-edge triggers
 *   for some radio interrupts.
 *
 ****************************************************************************/

void gd32vw55x_irq_set_trigger(int irq, uint8_t trig)
{
  int extirq = irq - RISCV_IRQ_ASYNC;
  uint8_t regval;

  DEBUGASSERT(extirq >= 0 && extirq < GD32VW55X_NIRQS);

  regval = getreg8(GD32VW55X_ECLIC_INTATTR(extirq));
  regval = (regval & ~ECLIC_INTATTR_TRIG_MASK) | trig;
  putreg8(regval, GD32VW55X_ECLIC_INTATTR(extirq));
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Return the current interrupt state and enable interrupts.  In ECLIC
 *   mode the per-source gating is in the ECLIC INTIE registers; the only
 *   global gate is mstatus.MIE (the mie CSR is not used).
 *
 ****************************************************************************/

irqstate_t up_irq_enable(void)
{
  return READ_AND_SET_CSR(CSR_MSTATUS, MSTATUS_MIE);
}

/****************************************************************************
 * Name: riscv_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ.  The ECLIC only auto-clears the pending bit
 *   (INTIP) in hardware-vectored mode; NuttX dispatches every interrupt
 *   through the common trap handler, so an edge-triggered source keeps
 *   INTIP set and re-fires forever unless it is cleared here.  Level-
 *   triggered sources are cleared at the peripheral and INTIP just mirrors
 *   the line, so they are left alone (writing INTIP while the line is high
 *   would only re-assert it).  The BLE radio interrupts are edge-triggered
 *   (see the SDK ble_irq_enable()); the Wi-Fi ones are level.
 *
 ****************************************************************************/

void riscv_ack_irq(int irq)
{
  int extirq = irq - RISCV_IRQ_ASYNC;
  uint8_t attr;

  if (extirq < 0 || extirq >= GD32VW55X_NIRQS)
    {
      return;
    }

  attr = getreg8(GD32VW55X_ECLIC_INTATTR(extirq));
  if ((attr & ECLIC_INTATTR_TRIG_MASK) != ECLIC_INTATTR_TRIG_LEVEL)
    {
      putreg8(0, GD32VW55X_ECLIC_INTIP(extirq));
    }
}
