/****************************************************************************
 * arch/arm/src/stm32h7/stm32_pmstop.c
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

#include <stdbool.h>

#include "arm_arch.h"
#include "nvic.h"
#include "stm32_pwr.h"
#include "stm32_pm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pmstop
 *
 * Description:
 *   Enter STOP mode.
 *
 * Input Parameters:
 *   lpds - true: To further reduce power consumption in Stop mode, put the
 *          internal voltage regulator in low-power under-drive mode using
 *          the LPDS and LPUDS bits of the Power control register (PWR_CR1).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_pmstop(bool lpds)
{
  uint32_t regval;

  /* Clear the Low Power Deep Sleep (LPDS) bit in the CPU power control
   * register.
   */

  regval  = getreg32(STM32_PWR_CR1);
  regval &= ~(PWR_CR1_LPDS | PWR_CR1_SVOS_MASK);

  /* Set low-power regulator mode and voltage scaling.  */

  if (lpds)
    {
      regval |= PWR_CR1_LPDS | PWR_CR1_SVOS_S5;
    }
  else
    {
      /* Set regulator to normal (S3) mode */

      regval |= PWR_CR1_SVOS_S3;
    }

  putreg32(regval, STM32_PWR_CR1);

  /* Clear the domain standby bits so D1, D2 and D3 remain in DStop mode */

  regval  = getreg32(STM32_PWR_CPUCR);
  regval &= ~(STM32_PWR_CPUCR_PDDS_D1 | STM32_PWR_CPUCR_PDDS_D2 |
              STM32_PWR_CPUCR_PDDS_D3);
  putreg32(regval, STM32_PWR_CPUCR);

  /* Set SLEEPDEEP bit of Cortex System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval |= NVIC_SYSCON_SLEEPDEEP;
  putreg32(regval, NVIC_SYSCON);

  /* Sleep until the wakeup interrupt or event occurs */

#ifdef CONFIG_PM_WFE
  /* Mode: SLEEP + Entry with WFE */

  asm volatile ("wfe");
#else
  /* Mode: SLEEP + Entry with WFI */

  asm volatile ("wfi");
#endif

  /* Clear deep sleep bits, so that MCU does not go into deep sleep in
   * idle.
   */

  /* Clear SLEEPDEEP bit of Cortex System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval &= ~NVIC_SYSCON_SLEEPDEEP;
  putreg32(regval, NVIC_SYSCON);
}
