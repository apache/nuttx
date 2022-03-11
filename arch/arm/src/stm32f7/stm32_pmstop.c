/****************************************************************************
 * arch/arm/src/stm32f7/stm32_pmstop.c
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

  /* Clear the Power Down Deep Sleep (PDDS), the Low Power Deep Sleep
   * (LPDS), Under-Drive Enable in Stop Mode (UDEN), Flash Power Down in
   * Stop Mode (FPDS), Main Regulator in Deepsleep Under-Drive Mode (MRUDS),
   * and Low-power Regulator in Deepsleep Under-Drive Mode (LPUDS) bits in
   * the power control register.
   */

  regval  = getreg32(STM32_PWR_CR1);
  regval &= ~(PWR_CR1_LPDS | PWR_CR1_PDDS | PWR_CR1_FPDS);
  regval &= ~(PWR_CR1_UDEN_ENABLE | PWR_CR1_MRUDS | PWR_CR1_LPUDS);

  /* Set under-drive enabled with low-power regulator.  */

  if (lpds)
    {
      regval |= PWR_CR1_UDEN_ENABLE | PWR_CR1_LPUDS | PWR_CR1_LPDS;
    }

  putreg32(regval, STM32_PWR_CR1);

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

  /* Clear the Power Down Deep Sleep (PDDS), the Low Power Deep Sleep
   * (LPDS) bits, Under-Drive Enable in Stop Mode (UDEN), Main Regulator
   * in Deepsleep Under-Drive Mode (MRUDS), and Low-power Regulator in
   * Deepsleep Under-Drive Mode (LPUDS) in the power control register.
   */

  regval  = getreg32(STM32_PWR_CR1);
  regval &= ~(PWR_CR1_LPDS | PWR_CR1_PDDS);
  regval &= ~(PWR_CR1_UDEN_ENABLE | PWR_CR1_MRUDS | PWR_CR1_LPUDS);
  putreg32(regval, STM32_PWR_CR1);

  /* Clear SLEEPDEEP bit of Cortex System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval &= ~NVIC_SYSCON_SLEEPDEEP;
  putreg32(regval, NVIC_SYSCON);
}
