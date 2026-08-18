/****************************************************************************
 * arch/arm/src/stm32h5/stm32_pmstop.c
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

#include <stdbool.h>

#include "arm_internal.h"
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
 *   svos5 - true: To further reduce power consumption in Stop mode, put the
 *           internal voltage regulator in "stop mode voltage scaling"
 *           scale 5 which is the lowest-power stop mode.
 *           false: Use SVOS3 which is the default voltage scaling value.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_pmstop(bool svos5)
{
  uint32_t regval;

  /* Clear the Low Power Mode Selection (LPMS) bit in the power mode control
   * register to specify Stop mode when entering DeepSleep.
   */

  regval  = getreg32(STM32_PWR_PMCR);
  regval &= ~(PWR_PMCR_LPMS | PWR_PMCR_SVOS_MASK);

  /* Set low-power voltage scaling.  */

  if (svos5)
    {
      regval |= PWR_PMCR_SVOS_SVOS5;
    }
  else
    {
      /* Set regulator to normal (S3) mode */

      regval |= PWR_PMCR_SVOS_SVOS3;
    }

  putreg32(regval, STM32_PWR_PMCR);

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
