/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_pmstop.c
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

  __asm__ volatile ("wfe");
#else
  /* Mode: SLEEP + Entry with WFI */

  __asm__ volatile ("wfi");
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
 * Name: stm32wb_pmstop
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

int stm32wb_pmstop(bool lpds)
{
  uint32_t regval;

  /* Clear Low-Power Mode Selection (LPMS) bits in power control
   * register CR1.
   */

  regval  = getreg32(STM32WB_PWR_CR1);
  regval &= ~PWR_CR1_LPMS_MASK;

  /* Select Stop 1 mode with low-power regulator if so requested */

  if (lpds)
    {
      regval |= PWR_CR1_LPMS_STOP1;
    }

  putreg32(regval, STM32WB_PWR_CR1);

  return do_stop();
}

/****************************************************************************
 * Name: stm32wb_pmstop2
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

int stm32wb_pmstop2(void)
{
  uint32_t regval;

  /* Select Stop 2 mode in power control register 1. */

  regval  = getreg32(STM32WB_PWR_CR1);
  regval &= ~PWR_CR1_LPMS_MASK;
  regval |= PWR_CR1_LPMS_STOP2;
  putreg32(regval, STM32WB_PWR_CR1);

  return do_stop();
}
