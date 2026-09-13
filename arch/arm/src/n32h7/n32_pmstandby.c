/****************************************************************************
 * arch/arm/src/n32h7/n32_pmstandby.c
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
#include "n32_rcc.h"
#include "n32_pwr.h"
#include "n32_pm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_pmstandby
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

void n32_pmstandby(void)
{
  uint32_t regval;

  /* Clear the wake-up flags before resetting. */

  modifyreg32(N32_PWR_C1_PWR_CR, 0, PWR_M7CTRL1_CWKUPF);
  modifyreg32(N32_PWR_C1_PWR_CR2, PWR_M7CTRL2_STOP2EN, 0);

  /* Set the domain Power Down Deep Sleep (PDDS) bits in the power control
   * register so that D1, D2, and D3 will go into the DStop state.
   */

  modifyreg32(N32_PWR_C1_PWR_CR, 0, PWR_M7CTRL1_PDSEN);

  /* Set SLEEPDEEP bit of Cortex System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval |= NVIC_SYSCON_SLEEPDEEP;
  putreg32(regval, NVIC_SYSCON);

  /* Sleep until the wakeup reset occurs */

  asm("wfi");
}
