/****************************************************************************
 * arch/arm/src/armv8-m/arm_secure_irq.c
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
#include <arch/irq.h>

#include "arm_arch.h"
#include "nvic.h"

#ifdef CONFIG_ARCH_HAVE_TRUSTZONE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_set_secure_irq
 *
 * Description:
 *   Secure an IRQ
 *
 ****************************************************************************/

void up_secure_irq(int irq, bool secure)
{
  uint32_t regaddr;
  uint32_t regval;
  int shift;

  DEBUGASSERT(irq >= NVIC_IRQ_FIRST && irq < NR_IRQS);

  irq    -= NVIC_IRQ_FIRST;
  regaddr = NVIC_IRQ_TARGET(irq);

  regval      = getreg32(regaddr);
  shift       = irq & 0x1f;
  regval     &= ~(1 << shift);
  regval     |= !secure << shift;
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: up_secure_irq_all
 *
 * Description:
 *   Secure all IRQ
 *
 ****************************************************************************/

void up_secure_irq_all(bool secure)
{
  int i;

  for (i = 0; i <= NR_IRQS - NVIC_IRQ_FIRST; i += 32)
    {
      putreg32(secure ? 0x0 : 0xffffffff, NVIC_IRQ_TARGET(i));
    }
}

#endif /* CONFIG_ARCH_HAVE_TRUSTZONE */
