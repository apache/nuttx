/****************************************************************************
 * arch/arm/src/stm32h5/stm32_pmstandby.c
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
#include "stm32_rcc.h"
#include "stm32_pwr.h"
#include "stm32_pm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pmstandby
 *
 * Description:
 *   Enter STANDBY mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_pmstandby(void)
{
  uint32_t regval;

  /* Clear the wake-up flags before resetting. */

  modifyreg32(STM32_PWR_PMCR, 0, PWR_PMCR_CSSF);
  modifyreg32(STM32_PWR_WUSCR, 0, PWR_WUSCR_CWUF1 | PWR_WUSCR_CWUF2 |
                                  PWR_WUSCR_CWUF3 | PWR_WUSCR_CWUF4 |
                                  PWR_WUSCR_CWUF5 | PWR_WUSCR_CWUF6 |
                                  PWR_WUSCR_CWUF7 | PWR_WUSCR_CWUF8);

  /* Clear reset flags. */

  modifyreg32(STM32_RCC_RSR, 0, RCC_RSR_RMVF);

  /* Set the Low Power Mode to Standby in the Power Mode Control Register.
   * Unlike H7 which has multiple power domains (D1, D2, D3), H5 uses a
   * single bit in PMCR to select between Stop and Standby modes.
   * Setting LPMS bit to 1 selects Standby mode.
   */

  modifyreg32(STM32_PWR_PMCR, 0, PWR_PMCR_LPMS);

  /* Set SLEEPDEEP bit of Cortex System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval |= NVIC_SYSCON_SLEEPDEEP;
  putreg32(regval, NVIC_SYSCON);

  /* Sleep until the wakeup reset occurs */

  asm("wfi");
}
