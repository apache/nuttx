/****************************************************************************
 * arch/rh850/src/u2c/u2c_irq.c
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

#include <errno.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "rh850_internal.h"
#include "hardware/u2c_intc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* The low and high EIC bytes are written separately so that EIRF is never
 * written by mask or priority changes.
 */

static uintptr_t u2c_eic(int irq)
{
  return (irq < U2C_INTC1_CHANNELS ? U2C_INTC1_BASE : U2C_INTC2_BASE) +
         2 * irq;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   Mask all channels, direct vector method (EITB = 0) at the lowest
 *   priority, no register bank, then enable EI level interrupts.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  int irq;

  up_irq_save();

  for (irq = 0; irq < NR_IRQS; irq++)
    {
      putreg8(U2C_EIC_EIMK | U2C_EIC_EIP_MASK, u2c_eic(irq));
    }

  __asm__ __volatile__("ldsr r0, 15, 2" : : : "memory");   /* RBCR0 = 0 */

#ifdef CONFIG_STACK_COLORATION
  rh850_stack_color(g_intstackalloc, CONFIG_ARCH_INTERRUPTSTACK & ~3);
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_enable_irq
 ****************************************************************************/

void up_enable_irq(int irq)
{
  uintptr_t eic;

  if (irq >= 0 && irq < NR_IRQS)
    {
      eic = u2c_eic(irq);
      putreg8(getreg8(eic) & ~U2C_EIC_EIMK, eic);
    }
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Mask a channel; a pending request flag stays set.
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  uintptr_t eic;

  if (irq >= 0 && irq < NR_IRQS)
    {
      eic = u2c_eic(irq);
      putreg8(getreg8(eic) | U2C_EIC_EIMK, eic);
    }
}

#ifdef CONFIG_ARCH_IRQPRIO
/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority (EIP 0-15, 0 is the highest) of a channel.
 *
 ****************************************************************************/

int up_prioritize_irq(int irq, int priority)
{
  return u2c_irq_setpriority(irq, priority);
}
#endif

/****************************************************************************
 * Name: u2c_irq_setpriority
 *
 * Description:
 *   Set EIP (0-15, 0 is the highest) with the direct vector method.
 *
 ****************************************************************************/

int u2c_irq_setpriority(int irq, int priority)
{
  uintptr_t eic;
  uint8_t regval;

  if (irq < 0 || irq >= NR_IRQS || priority < 0 ||
      priority > U2C_EIC_EIP_MASK)
    {
      return -EINVAL;
    }

  eic    = u2c_eic(irq);
  regval = getreg8(eic) & ~(U2C_EIC_EITB | U2C_EIC_EIP_MASK);
  putreg8(regval | priority, eic);
  return OK;
}

/****************************************************************************
 * Name: u2c_irq_raise
 *
 * Description:
 *   Set the request flag of a channel by software.
 *
 ****************************************************************************/

void u2c_irq_raise(int irq)
{
  putreg8(U2C_EIC_HI_EIRF, u2c_eic(irq) + 1);
}

/****************************************************************************
 * Name: u2c_irq_pending
 *
 * Description:
 *   Return and clear the request flag of a channel.
 *
 ****************************************************************************/

bool u2c_irq_pending(int irq)
{
  uintptr_t eic = u2c_eic(irq) + 1;

  if ((getreg8(eic) & U2C_EIC_HI_EIRF) != 0)
    {
      putreg8(0, eic);
      return true;
    }

  return false;
}
