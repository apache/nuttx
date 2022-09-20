/****************************************************************************
 * arch/or1k/src/mor1kx/up_irq.c
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>
#include <arch/spr.h>

#include "up_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Disable all interrupts */

  /* Set all interrupts (and exceptions) to the default priority */

  /* And finally, enable interrupts */

  up_irq_enable();
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
  uint32_t mr;

  DEBUGASSERT((unsigned)irq < NR_IRQS);

  irqinfo("irq: %d\n", irq);

  if (irq <= 31)
    {
      mfspr(SPR_PIC_MR, mr);
      mr &= ~(1 << irq);
      mtspr(SPR_PIC_MR, mr);
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
  uint32_t mr;

  DEBUGASSERT((unsigned)irq < NR_IRQS);

  irqinfo("irq: %d\n", irq);

  if (irq <= 31)
    {
      mfspr(SPR_PIC_MR, mr);
      mr |= (1 << irq);
      mtspr(SPR_PIC_MR, mr);
    }
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
  if (irq <= 31)
    {
      /* uint32_t sr = (1 << irq); */

      uint32_t sr = 0;
      mtspr(SPR_PIC_SR, sr);
    }
}

/****************************************************************************
 * Name: or1k_dump_pic
 *
 * Description:
 *   Dump programmable interrupt controller registers
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_IRQ_INFO
void or1k_dump_pic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = enter_critical_section();

  leave_critical_section(flags);
}

#else
#  define or1k_dump_pic(msg, irq)
#endif
