/****************************************************************************
 * arch/arm/src/pp502x/pp502x_irq.c
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
#include <nuttx/arch.h>

#include "arm.h"
#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO interrupt enable registers: three modules of four ports */

#define PP502X_GPIO_NMODULES        3
#define PP502X_GPIO_NPORTS          4
#define PP502X_GPIO_MODULE_STRIDE   0x80
#define PP502X_GPIO_INT_EN_OFFSET   0x50

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  uintptr_t base;
  int module;
  int port;

  /* Mask every source on both cores and drop forced sources, so that
   * nothing the boot loader left enabled is delivered.
   */

  putreg32(0xffffffff, PP502X_INTC_CPU_DIS);
  putreg32(0xffffffff, PP502X_INTC_HI_CPU_DIS);
  putreg32(0xffffffff, PP502X_INTC_COP_DIS);
  putreg32(0xffffffff, PP502X_INTC_HI_COP_DIS);
  putreg32(0xffffffff, PP502X_INTC_FORCEDCLR);
  putreg32(0xffffffff, PP502X_INTC_HI_FORCEDCLR);
  putreg32(PP502X_INTC_CPU_PRIO_FIQ, PP502X_INTC_CPU_PRIO);
  putreg32(0, PP502X_INTC_HI_CPU_PRIO);

  for (module = 0; module < PP502X_GPIO_NMODULES; module++)
    {
      base = PP502X_GPIO_BASE + module * PP502X_GPIO_MODULE_STRIDE +
             PP502X_GPIO_INT_EN_OFFSET;

      for (port = 0; port < PP502X_GPIO_NPORTS; port++)
        {
          putreg32(0, base + 4 * port);
        }
    }

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  arm_color_intstack();
  up_irq_restore(PSR_MODE_SYS | PSR_F_BIT);
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq' for the CPU.  The high bank cascade
 *   stays enabled for the other high bank sources.
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  if ((unsigned)irq < PP502X_IRQ_HIBASE)
    {
      putreg32(1u << irq, PP502X_INTC_CPU_DIS);
    }
  else if ((unsigned)irq < NR_IRQS)
    {
      putreg32(1u << (irq - PP502X_IRQ_HIBASE), PP502X_INTC_HI_CPU_DIS);
    }
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq' for the CPU.  The enable registers
 *   are write-1-to-set, so no read-modify-write is needed.  A high bank
 *   source also needs the cascade bit in the normal bank.
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  if ((unsigned)irq < PP502X_IRQ_HIBASE)
    {
      putreg32(1u << irq, PP502X_INTC_CPU_EN);
    }
  else if ((unsigned)irq < NR_IRQS)
    {
      putreg32(1u << (irq - PP502X_IRQ_HIBASE), PP502X_INTC_HI_CPU_EN);
      putreg32(PP502X_INTC_HI_CASCADE, PP502X_INTC_CPU_EN);
    }
}

/****************************************************************************
 * Name: arm_ack_irq
 *
 * Description:
 *   Acknowledge the interrupt.  Sources are acknowledged at the peripheral
 *   (the timers by a read of their value register), so there is nothing to
 *   do at the controller.
 *
 ****************************************************************************/

void arm_ack_irq(int irq)
{
}
