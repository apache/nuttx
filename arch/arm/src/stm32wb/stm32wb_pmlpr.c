/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_pmlpr.c
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
#include "stm32wb_pwr.h"
#include "stm32wb_pm.h"
#include "stm32wb_rcc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_pmlpr
 *
 * Description:
 *   Enter Low-Power Run (LPR) mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero means that LPR was successfully entered. Otherwise, LPR mode was
 *   not entered and a negated errno value is returned to indicate the cause
 *   of the failure.
 *
 ****************************************************************************/

int stm32wb_pmlpr(void)
{
  uint32_t regval;

  /* Enable MSI clock */

  regval  = getreg32(STM32WB_RCC_CR);
  regval |= RCC_CR_MSION;

  /* Set MSI clock to 2 MHz */

  regval &= ~RCC_CR_MSIRANGE_MASK;
  regval |= RCC_CR_MSIRANGE_2M; /* 2 MHz */
  putreg32(regval, STM32WB_RCC_CR);

  /* Select MSI clock as system clock source */

  regval  = getreg32(STM32WB_RCC_CFGR);
  regval &= ~RCC_CFGR_SW_MASK;
  regval |= RCC_CFGR_SW_MSI;
  putreg32(regval, STM32WB_RCC_CFGR);

  /* Wait until the MSI source is used as the system clock source */

  while ((getreg32(STM32WB_RCC_CFGR) & RCC_CFGR_SWS_MASK) !=
          RCC_CFGR_SWS_MSI)
    {
    }

  /* Enable Low-Power Run */

  regval  = getreg32(STM32WB_PWR_CR1);
  regval |= PWR_CR1_LPR;
  putreg32(regval, STM32WB_PWR_CR1);

  return OK;
}
