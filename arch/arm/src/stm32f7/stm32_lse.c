/****************************************************************************
 * arch/arm/src/stm32f7/stm32_lse.c
 *
 *   Copyright (C) 2017, 2021 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david.sidrane@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_arch.h"

#include "stm32_rcc.h"
#include "stm32_pwr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LSERDY_TIMEOUT (500 * CONFIG_BOARD_LOOPSPERMSEC)

#ifdef CONFIG_STM32F7_RTC_LSECLOCK_START_DRV_CAPABILITY
# if CONFIG_STM32F7_RTC_LSECLOCK_START_DRV_CAPABILITY < 0 || \
     CONFIG_STM32F7_RTC_LSECLOCK_START_DRV_CAPABILITY > 3
#  error "Invalid LSE drive capability setting"
#endif
#endif

#ifdef CONFIG_STM32F7_RTC_LSECLOCK_RUN_DRV_CAPABILITY
# if CONFIG_STM32F7_RTC_LSECLOCK_RUN_DRV_CAPABILITY < 0 || \
     CONFIG_STM32F7_RTC_LSECLOCK_RUN_DRV_CAPABILITY > 3
#  error "Invalid LSE drive capability setting"
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t drives[4] =
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
#ifdef CONFIG_STM32F7_RTC_AUTO_LSECLOCK_START_DRV_CAPABILITY
  volatile int32_t drive = 0;
#endif

  /* Check if the External Low-Speed (LSE) oscillator is already running. */

  regval = getreg32(STM32_RCC_BDCR);

  if ((regval & (RCC_BDCR_LSEON | RCC_BDCR_LSERDY)) !=
                (RCC_BDCR_LSEON | RCC_BDCR_LSERDY))
    {
      /* The LSE is in the RTC domain and write access is denied to this
       * domain after reset, you have to enable write access using DBP bit
       * in the PWR CR register before to configuring the LSE.
       */

      stm32_pwr_enablebkp(true);

      /* Enable the External Low-Speed (LSE) oscillator by setting the
       * LSEON bit the RCC BDCR register.
       */

      regval |= RCC_BDCR_LSEON;

#ifdef CONFIG_STM32F7_RTC_LSECLOCK_START_DRV_CAPABILITY
      /* Set start-up drive capability for LSE oscillator. With the
       * enable on.
       */

      regval &= ~(RCC_BDCR_LSEDRV_MASK);
      regval |= drives[CONFIG_STM32F7_RTC_LSECLOCK_START_DRV_CAPABILITY];
#endif

#ifdef CONFIG_STM32F7_RTC_AUTO_LSECLOCK_START_DRV_CAPABILITY
      do
        {
          regval &= ~(RCC_BDCR_LSEDRV_MASK);
          regval |= drives[drive++];
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

#ifdef CONFIG_STM32F7_RTC_AUTO_LSECLOCK_START_DRV_CAPABILITY
          if (timeout != 0)
            {
              break;
            }
        }
      while (drive < sizeof(drives) / sizeof(drives[0]));
#endif
#if defined(CONFIG_STM32F7_RTC_LSECLOCK_RUN_DRV_CAPABILITY) && \
    CONFIG_STM32F7_RTC_LSECLOCK_START_DRV_CAPABILITY != \
    CONFIG_STM32F7_RTC_LSECLOCK_RUN_DRV_CAPABILITY
      /* Set running drive capability for LSE oscillator. */

      regval &= ~RCC_BDCR_LSEDRV_MASK;
      regval |= drives[CONFIG_STM32F7_RTC_LSECLOCK_RUN_DRV_CAPABILITY];
      putreg32(regval, STM32_RCC_BDCR);
#endif

      /* Disable backup domain access if it was disabled on entry */

      stm32_pwr_enablebkp(false);
    }
}
