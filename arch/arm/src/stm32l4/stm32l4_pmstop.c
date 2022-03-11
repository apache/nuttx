/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_pmstop.c
 *
 *   Copyright (C) 2012, 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
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

#include <stdbool.h>

#include "arm_internal.h"
#include "nvic.h"
#include "stm32l4_pwr.h"
#include "stm32l4_pm.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int do_stop(void)
{
  uint32_t regval;

  /* Set SLEEPDEEP bit of Cortex System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval |= NVIC_SYSCON_SLEEPDEEP;
  putreg32(regval, NVIC_SYSCON);

  /* Sleep until the wakeup interrupt or event occurs */

#ifdef CONFIG_PM_WFE
  /* Mode: SLEEP + Entry with WFE */

  asm("wfe");
#else
  /* Mode: SLEEP + Entry with WFI */

  asm("wfi");
#endif

  /* Clear SLEEPDEEP bit of Cortex System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval &= ~NVIC_SYSCON_SLEEPDEEP;
  putreg32(regval, NVIC_SYSCON);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_pmstop
 *
 * Description:
 *   Enter STOP mode.
 *
 * Input Parameters:
 *   lpds - true: To further reduce power consumption in Stop mode, put the
 *          internal voltage regulator in low-power mode using the LPDS bit
 *          of the Power control register (PWR_CR).
 *
 * Returned Value:
 *   Zero means that the STOP was successfully entered and the system has
 *   been re-awakened.  The internal voltage regulator is back to its
 *   original state.  Otherwise, STOP mode did not occur and a negated
 *   errno value is returned to indicate the cause of the failure.
 *
 ****************************************************************************/

int stm32l4_pmstop(bool lpds)
{
  uint32_t regval;

  /* Clear Low-Power Mode Selection (LPMS) bits in power control
   * register 1.
   */

  regval  = getreg32(STM32L4_PWR_CR1);
  regval &= ~PWR_CR1_LPMS_MASK;

  /* Select Stop 1 mode with low-power regulator if so requested */

  if (lpds)
    {
      regval |= PWR_CR1_LPMS_STOP1LPR;
    }

  putreg32(regval, STM32L4_PWR_CR1);

  return do_stop();
}

/****************************************************************************
 * Name: stm32l4_pmstop2
 *
 * Description:
 *   Enter STOP2 mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero means that the STOP2 was successfully entered and the system has
 *   been re-awakened.  Otherwise, STOP2 mode did not occur and a negated
 *   errno value is returned to indicate the cause of the failure.
 *
 ****************************************************************************/

int stm32l4_pmstop2(void)
{
  uint32_t regval;

  regval  = getreg32(STM32L4_PWR_CR1);
#ifdef CONFIG_STM32L4_SRAM3_HEAP
  /* SRAM3 is used as heap, so it must not be powered off in Stop 2 mode. */

  regval |= PWR_CR1_RRSTP;
#endif

  /* Select Stop 2 mode in power control register 1. */

  regval &= ~PWR_CR1_LPMS_MASK;
  regval |= PWR_CR1_LPMS_STOP2;
  putreg32(regval, STM32L4_PWR_CR1);

  return do_stop();
}
