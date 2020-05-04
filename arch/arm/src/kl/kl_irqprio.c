/****************************************************************************
 * arch/arm/src/kl/kl_irqprio.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
