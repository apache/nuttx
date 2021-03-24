/****************************************************************************
 * arch/arm/src/kl/kl_irqprio.c
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

#include <sys/types.h>
#include <assert.h>
#include <arch/irq.h>

#include "nvic.h"
#include "arm_arch.h"

#include "kl_irq.h"

#ifdef CONFIG_ARCH_IRQPRIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int up_prioritize_irq(int irq, int priority)
{
  uint32_t regaddr;
  uint32_t regval;
  int shift;

  DEBUGASSERT(irq == KL_IRQ_SVCALL ||
              irq == KL_IRQ_PENDSV ||
              irq == KL_IRQ_SYSTICK ||
             (irq >= KL_IRQ_EXTINT && irq < NR_IRQS));
  DEBUGASSERT(priority >= NVIC_SYSH_PRIORITY_MAX &&
              priority <= NVIC_SYSH_PRIORITY_MIN);

  /* Check for external interrupt */

  if (irq >= KL_IRQ_EXTINT && irq < (KL_IRQ_EXTINT + 32))
    {
      /* ARMV6M_NVIC_IPR() maps register IPR0-IPR7 with four settings per
       * register.
       */

      regaddr = ARMV6M_NVIC_IPR(irq >> 2);
      shift   = (irq & 3) << 3;
    }

  /* Handle processor exceptions.  Only SVCall, PendSV, and SysTick can be
   * reprioritized.  And we will not permit modification of SVCall through
   * this function.
   */

  else if (irq == KL_IRQ_PENDSV)
    {
      regaddr = ARMV6M_SYSCON_SHPR2;
      shift   = SYSCON_SHPR3_PRI_14_SHIFT;
    }
  else if (irq == KL_IRQ_SYSTICK)
    {
      regaddr = ARMV6M_SYSCON_SHPR2;
      shift   = SYSCON_SHPR3_PRI_15_SHIFT;
    }
  else
    {
      return ERROR;
    }

  /* Set the priority */

  regval  = getreg32(regaddr);
  regval &= ~((uint32_t)0xff << shift);
  regval |= ((uint32_t)priority << shift);
  putreg32(regval, regaddr);
  return OK;
}
#endif
