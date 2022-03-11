/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_lse.c
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

#include "arm_internal.h"
#include "stm32_pwr.h"
#include "stm32_rcc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rcc_enablelse
 *
 * Description:
 *   Enable the External Low-Speed (LSE) oscillator.
 *
 * Todo:
 *   Check for LSE good timeout and return with -1,
 *
 ****************************************************************************/

void stm32_rcc_enablelse(void)
{
  /* The LSE is in the RTC domain and write access is denied to this domain
   * after reset, you have to enable write access using DBP bit in the PWR CR
   * register before to configuring the LSE.
   */

  stm32_pwr_enablebkp(true);

#if defined(CONFIG_ARCH_CHIP_STM32L0)
  /* Enable the External Low-Speed (LSE) oscillator by setting the LSEON bit
   * the RCC CSR register.
   */

  modifyreg32(STM32_RCC_CSR, 0, RCC_CSR_LSEON);

  /* Wait for the LSE clock to be ready */

  while ((getreg32(STM32_RCC_CSR) & RCC_CSR_LSERDY) == 0)
    {
    }

#elif defined(CONFIG_ARCH_CHIP_STM32F0)
  /* Enable the External Low-Speed (LSE) oscillator by setting the LSEON bit
   * the RCC BDCR register.
   */

  modifyreg16(STM32_RCC_BDCR, 0, RCC_BDCR_LSEON);

  /* Wait for the LSE clock to be ready */

  while ((getreg16(STM32_RCC_BDCR) & RCC_BDCR_LSERDY) == 0)
    {
    }

#endif

  /* Disable backup domain access if it was disabled on entry */

  stm32_pwr_enablebkp(false);
}
