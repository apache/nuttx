/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_ipcc.c
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

#include <stdint.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32wb_ipcc.h"
#include "hardware/stm32wb_rcc.h"
#include "hardware/stm32wb_exti.h"
#include "hardware/stm32wb_pwr.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_ipccreset
 *
 * Description:
 *   Reset the IPCC registers to default state
 *
 ****************************************************************************/

void stm32wb_ipccreset(void)
{
  uint32_t regval;

  /* Disable CPU1 IPCC interrupts */

  putreg32(0x00000000, STM32WB_IPCC_C1CR);

  /* Clear CPU1 IPCC receive channel status */

  putreg32(IPCC_C1SCR_CLR_MASK, STM32WB_IPCC_C1SCR);

  /* Clear CPU2 IPCC receive channel status */

  putreg32(IPCC_C2SCR_CLR_MASK, STM32WB_IPCC_C2SCR);

  /* Disable CPU1 transmit/receive channels */

  regval = getreg32(STM32WB_IPCC_C1MR);
  regval |= IPCC_C1MR_OM_MASK | IPCC_C1MR_FM_MASK;
  putreg32(regval, STM32WB_IPCC_C1MR);

  /* Disable CPU2 transmit/receive channels */

  regval = getreg32(STM32WB_IPCC_C2MR);
  regval |= IPCC_C2MR_OM_MASK | IPCC_C2MR_FM_MASK;
  putreg32(regval, STM32WB_IPCC_C2MR);
}

/****************************************************************************
 * Name: stm32wb_ipccenable
 *
 * Description:
 *   Enable the IPCC and start CPU2
 *
 ****************************************************************************/

void stm32wb_ipccenable(void)
{
  uint32_t regval;

  /* CPU2 IPCC clock enable */

  regval = getreg32(STM32WB_RCC_C2AHB3ENR);
  regval |= RCC_C2AHB3ENR_IPCCEN;
  putreg32(regval, STM32WB_RCC_C2AHB3ENR);

  /* Enable EXTI event request for C1SEV interrupt to CPU2 */

  regval = getreg32(STM32WB_EXTI_C2EMR2);
  regval |= EXTI_C2EMR2_EM(EXTI_EVT_C1SEV);
  putreg32(regval, STM32WB_EXTI_C2EMR2);

  /* Enable EXTI rising edge trigger for C1SEV interrupt to CPU2 */

  regval = getreg32(STM32WB_EXTI_RTSR2);
  regval |= EXTI_RTSR2_RT(EXTI_EVT_C1SEV);
  putreg32(regval, STM32WB_EXTI_RTSR2);

  /* Set the internal event flag and send an event to CPU2 */

  __asm__ volatile ("sev");

  /* Clear the internal event flag */

  __asm__ volatile ("wfe");

  /* Boot CPU2 after reset or wakeup from stop or standby modes */

  regval = getreg32(STM32WB_PWR_CR4);
  regval |= PWR_CR4_C2BOOT;
  putreg32(regval, STM32WB_PWR_CR4);
}
