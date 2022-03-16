/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_lse.c
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
#include "stm32l4_pwr.h"
#include "stm32l4_rcc.h"
#include "stm32l4_waste.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_STM32L4_RTC_LSECLOCK_START_DRV_CAPABILITY
# if CONFIG_STM32L4_RTC_LSECLOCK_START_DRV_CAPABILITY < 0 || \
     CONFIG_STM32L4_RTC_LSECLOCK_START_DRV_CAPABILITY > 3
#  error "Invalid LSE drive capability setting"
#endif
#endif

#ifdef CONFIG_STM32L4_RTC_LSECLOCK_RUN_DRV_CAPABILITY
# if CONFIG_STM32L4_RTC_LSECLOCK_RUN_DRV_CAPABILITY < 0 || \
     CONFIG_STM32L4_RTC_LSECLOCK_RUN_DRV_CAPABILITY > 3
#  error "Invalid LSE drive capability setting"
#endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_rcc_enablelse
 *
 * Description:
 *   Enable the External Low-Speed (LSE) oscillator.
 *
 ****************************************************************************/

void stm32l4_rcc_enablelse(void)
{
  bool writable;
  uint32_t regval;

  /* Check if the External Low-Speed (LSE) oscillator is already running. */

  regval = getreg32(STM32L4_RCC_BDCR);

  if ((regval & (RCC_BDCR_LSEON | RCC_BDCR_LSERDY)) !=
                (RCC_BDCR_LSEON | RCC_BDCR_LSERDY))
    {
      /* The LSE is in the RTC domain and write access is denied to this
       * domain after reset, you have to enable write access using DBP bit
       * in the PWR CR register before to configuring the LSE.
       */

      writable = stm32l4_pwr_enablebkp(true);

      /* Enable the External Low-Speed (LSE) oscillator by setting the
       * LSEON bit the RCC BDCR register.
       */

      regval |= RCC_BDCR_LSEON;

#ifdef CONFIG_STM32L4_RTC_LSECLOCK_START_DRV_CAPABILITY
      /* Set start-up drive capability for LSE oscillator. */

      regval &= ~RCC_BDCR_LSEDRV_MASK;
      regval |= CONFIG_STM32L4_RTC_LSECLOCK_START_DRV_CAPABILITY <<
                RCC_BDCR_LSEDRV_SHIFT;
#endif

      putreg32(regval, STM32L4_RCC_BDCR);

      /* Wait for the LSE clock to be ready */

      while (((regval = getreg32(STM32L4_RCC_BDCR)) & RCC_BDCR_LSERDY) == 0)
        {
          stm32l4_waste();
        }

#if defined(CONFIG_STM32L4_RTC_LSECLOCK_RUN_DRV_CAPABILITY) && \
    CONFIG_STM32L4_RTC_LSECLOCK_START_DRV_CAPABILITY != \
    CONFIG_STM32L4_RTC_LSECLOCK_RUN_DRV_CAPABILITY

#  if CONFIG_STM32L4_RTC_LSECLOCK_RUN_DRV_CAPABILITY != 0
#    error "STM32L4 only allows lowering LSE drive capability to zero"
#  endif

      /* Set running drive capability for LSE oscillator. */

      regval &= ~RCC_BDCR_LSEDRV_MASK;
      regval |= CONFIG_STM32L4_RTC_LSECLOCK_RUN_DRV_CAPABILITY <<
                RCC_BDCR_LSEDRV_SHIFT;
      putreg32(regval, STM32L4_RCC_BDCR);
#endif

      /* Disable backup domain access if it was disabled on entry */

      stm32l4_pwr_enablebkp(writable);
    }
}
