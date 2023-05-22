/****************************************************************************
 * arch/arm/src/stm32h7/stm32_lse.c
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
#include "stm32_rcc.h"
#include "stm32_pwr.h"
#include "stm32_dbgmcu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LSERDY_TIMEOUT (500 * CONFIG_BOARD_LOOPSPERMSEC)

#ifdef CONFIG_STM32H7_RTC_LSECLOCK_START_DRV_CAPABILITY
#  if CONFIG_STM32H7_RTC_LSECLOCK_START_DRV_CAPABILITY < 0 || \
      CONFIG_STM32H7_RTC_LSECLOCK_START_DRV_CAPABILITY > 3
#    error "Invalid LSE drive capability setting"
#  endif
#endif

#ifdef CONFIG_STM32H7_RTC_LSECLOCK_RUN_DRV_CAPABILITY
#  if CONFIG_STM32H7_RTC_LSECLOCK_RUN_DRV_CAPABILITY < 0 || \
      CONFIG_STM32H7_RTC_LSECLOCK_RUN_DRV_CAPABILITY > 3
#    error "Invalid LSE drive capability setting"
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* See errata ES0392 Rev 7. 2.2.14  LSE oscillator driving capability
 * selection bits are swapped.
 */

static const uint32_t drives_rev_y[4] =
{
    RCC_BDCR_LSEDRV_LOW,
    RCC_BDCR_LSEDRV_MEDLO_Y,
    RCC_BDCR_LSEDRV_MEDHI_Y,
    RCC_BDCR_LSEDRV_HIGH
};

const uint32_t drives_rev[4] =
{
    RCC_BDCR_LSEDRV_LOW,
    RCC_BDCR_LSEDRV_MEDLO,
    RCC_BDCR_LSEDRV_MEDHI,
    RCC_BDCR_LSEDRV_HIGH
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rcc_enablelse
 *
 * Description:
 *   Enable the External Low-Speed (LSE) oscillator.
 *
 ****************************************************************************/

void stm32_rcc_enablelse(void)
{
  uint32_t         regval;
  volatile int32_t timeout;
#ifdef CONFIG_STM32H7_RTC_AUTO_LSECLOCK_START_DRV_CAPABILITY
  volatile int32_t drive = 0;
#endif
  const uint32_t   *drives = drives_rev_y;

  /* Check if the External Low-Speed (LSE) oscillator is already running. */

  regval = getreg32(STM32_RCC_BDCR);

  if ((regval & (RCC_BDCR_LSEON | RCC_BDCR_LSERDY)) !=
                (RCC_BDCR_LSEON | RCC_BDCR_LSERDY))
    {
      /* Check Silicon for LSE oscillator driving capability selection bits
       * are swapped errata.
       */

      if ((getreg32(STM32_DEBUGMCU_BASE) & DBGMCU_IDCODE_REVID_MASK) !=
                   STM32_IDCODE_REVID_Y)
        {
          drives = drives_rev;
        }

      /* The LSE is in the RTC domain and write access is denied to this
       * domain after reset, you have to enable write access using DBP bit
       * in the PWR CR register before to configuring the LSE.
       */

      stm32_pwr_enablebkp(true);

      /* Enable the External Low-Speed (LSE) oscillator by setting the
       * LSEON bit the RCC BDCR register.
       */

      regval |= RCC_BDCR_LSEON;

#ifdef CONFIG_STM32H7_RTC_LSECLOCK_START_DRV_CAPABILITY
      /* Set start-up drive capability for LSE oscillator. With the
       * enable off
       */

      regval &= ~(RCC_BDCR_LSEDRV_MASK | RCC_BDCR_LSEON);
      regval |= drives[CONFIG_STM32H7_RTC_LSECLOCK_START_DRV_CAPABILITY];
      putreg32(regval, STM32_RCC_BDCR);
      regval |= RCC_BDCR_LSEON;
#endif

#ifdef CONFIG_STM32H7_RTC_AUTO_LSECLOCK_START_DRV_CAPABILITY
      do
        {
          regval &= ~(RCC_BDCR_LSEDRV_MASK | RCC_BDCR_LSEON);
          regval |= drives[drive++];
          putreg32(regval, STM32_RCC_BDCR);
          regval |= RCC_BDCR_LSEON;
#endif

          putreg32(regval, STM32_RCC_BDCR);

          /* Wait for the LSE clock to be ready (or until a timeout elapsed)
           */

          for (timeout = LSERDY_TIMEOUT; timeout > 0; timeout--)
            {
              /* Check if the LSERDY flag is the set in the BDCR */

              regval = getreg32(STM32_RCC_BDCR);

              if (regval & RCC_BDCR_LSERDY)
                {
                  /* If so, then break-out with timeout > 0 */

                  break;
                }
            }

#ifdef CONFIG_STM32H7_RTC_AUTO_LSECLOCK_START_DRV_CAPABILITY
          if (timeout != 0)
            {
              break;
            }
        }
      while (drive < sizeof(drives_rev_y) / sizeof(drives_rev_y[0]));

#endif
#if defined(CONFIG_STM32H7_RTC_LSECLOCK_RUN_DRV_CAPABILITY) && \
    CONFIG_STM32H7_RTC_LSECLOCK_START_DRV_CAPABILITY != \
    CONFIG_STM32H7_RTC_LSECLOCK_RUN_DRV_CAPABILITY
      /* Set running drive capability for LSE oscillator. */

      regval &= ~RCC_BDCR_LSEDRV_MASK;
      regval |= drives[CONFIG_STM32H7_RTC_LSECLOCK_RUN_DRV_CAPABILITY];
      putreg32(regval, STM32_RCC_BDCR);
#endif

      /* Disable backup domain access if it was disabled on entry */

      stm32_pwr_enablebkp(false);
    }
}
