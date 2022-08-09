/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_pmstandby.c
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

#include "nvic.h"
#include "stm32wb_pwr.h"
#include "stm32wb_pm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_pmstandby
 *
 * Description:
 *   Enter STANDBY mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, this function will not return (STANDBY mode can only be
 *   terminated with a reset event).  Otherwise, STANDBY mode did not occur
 *   and a negated errno value is returned to indicate the cause of the
 *   failure.
 *
 ****************************************************************************/

int stm32wb_pmstandby(void)
{
  uint32_t regval;

  /* Clear the Wake-Up Flags by setting the CWUFx bits in the power status
   * clear register
   */

  regval = PWR_SCR_CWUF1 | PWR_SCR_CWUF2 | PWR_SCR_CWUF3 |
           PWR_SCR_CWUF4 | PWR_SCR_CWUF5;
  putreg32(regval, STM32WB_PWR_SCR);

  /* Select Standby mode */

  regval  = getreg32(STM32WB_PWR_CR1);
  regval &= ~PWR_CR1_LPMS_MASK;
  regval |= PWR_CR1_LPMS_STANDBY;

  putreg32(regval, STM32WB_PWR_CR1);

  /* Set SLEEPDEEP bit of Cortex System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval |= NVIC_SYSCON_SLEEPDEEP;
  putreg32(regval, NVIC_SYSCON);

  /* Sleep until the wakeup reset occurs */

  __asm__ volatile ("wfi");

  return OK;  /* Won't get here */
}
