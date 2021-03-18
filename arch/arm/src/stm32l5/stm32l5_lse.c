/****************************************************************************
 * arch/arm/src/stm32l5/stm32l5_lse.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Michael Jung <mijung@gmx.net>
 *
 * Based on arch/arm/src/stm32l4/stm32l4_lse.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: dev@ziggurat29.com
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

#include "stm32l5_pwr.h"
#include "stm32l5_rcc.h"
#include "stm32l5_waste.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LSERDY_TIMEOUT (500 * CONFIG_BOARD_LOOPSPERMSEC)

#ifdef CONFIG_STM32L5_RTC_LSECLOCK_START_DRV_CAPABILITY
# if CONFIG_STM32L5_RTC_LSECLOCK_START_DRV_CAPABILITY < 0 || \
     CONFIG_STM32L5_RTC_LSECLOCK_START_DRV_CAPABILITY > 3
#  error "Invalid LSE drive capability setting"
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STM32L5_RTC_AUTO_LSECLOCK_START_DRV_CAPABILITY
static const uint32_t drives[4] =
{
    RCC_BDCR_LSEDRV_LOW,
    RCC_BDCR_LSEDRV_MEDLO,
    RCC_BDCR_LSEDRV_MEDHI,
    RCC_BDCR_LSEDRV_HIGH
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l5_rcc_enablelse
 *
 * Description:
 *   Enable the External Low-Speed (LSE) oscillator and the LSE system clock.
 *
 ****************************************************************************/

void stm32l5_rcc_enablelse(void)
{
  bool writable;
  uint32_t regval;
  volatile int32_t timeout;
#ifdef CONFIG_STM32L5_RTC_AUTO_LSECLOCK_START_DRV_CAPABILITY
  volatile int32_t drive = 0;
#endif

  /* Check if both the External Low-Speed (LSE) oscillator and the LSE system
   * clock are already running.
   */

  regval = getreg32(STM32L5_RCC_BDCR);

  if ((regval & (RCC_BDCR_LSEON | RCC_BDCR_LSERDY |
                 RCC_BDCR_LSESYSEN | RCC_BDCR_LSESYSEN)) !=
                (RCC_BDCR_LSEON | RCC_BDCR_LSERDY |
                 RCC_BDCR_LSESYSEN | RCC_BDCR_LSESYSEN))
    {
      /* The LSE is in the RTC domain and write access is denied to this
       * domain after reset, you have to enable write access using DBP bit in
       * the PWR CR register before to configuring the LSE.
       */

      writable = stm32l5_pwr_enablebkp(true);

      /* Enable the External Low-Speed (LSE) oscillator by setting the LSEON
       * bit the RCC BDCR register.
       */

      regval |= RCC_BDCR_LSEON;

#ifdef CONFIG_STM32L5_RTC_LSECLOCK_START_DRV_CAPABILITY
      /* Set start-up drive capability for LSE oscillator.  LSE must be OFF
       * to change drive strength.
       */

      regval &= ~(RCC_BDCR_LSEDRV_MASK | RCC_BDCR_LSEON);
      regval |= CONFIG_STM32L5_RTC_LSECLOCK_START_DRV_CAPABILITY <<
                RCC_BDCR_LSEDRV_SHIFT;
      putreg32(regval, STM32L5_RCC_BDCR);
      regval |= RCC_BDCR_LSEON;
#endif

#ifdef CONFIG_STM32L5_RTC_AUTO_LSECLOCK_START_DRV_CAPABILITY
      do
        {
          regval &= ~(RCC_BDCR_LSEDRV_MASK | RCC_BDCR_LSEON);
          regval |= drives[drive++];
          putreg32(regval, STM32L5_RCC_BDCR);
          regval |= RCC_BDCR_LSEON;
#endif

          putreg32(regval, STM32L5_RCC_BDCR);

          /* Wait for the LSE clock to be ready (or until a timeout elapsed)
           */

          for (timeout = LSERDY_TIMEOUT; timeout > 0; timeout--)
            {
              /* Check if the LSERDY flag is the set in the BDCR */

              regval = getreg32(STM32L5_RCC_BDCR);

              if (regval & RCC_BDCR_LSERDY)
                {
                  /* If so, then break-out with timeout > 0 */

                  break;
                }
            }

#ifdef CONFIG_STM32L5_RTC_AUTO_LSECLOCK_START_DRV_CAPABILITY
          if (timeout != 0)
            {
              break;
            }
        }
      while (drive < sizeof(drives) / sizeof(drives[0]));
#endif

      if (timeout != 0)
        {
          /* Enable LSE system clock.  The LSE system clock seems to provide
           * a means to gate the LSE clock distribution to peripherals.  It
           * must be enabled for MSI PLL mode (syncing the MSI to the LSE).
           */

          regval |= RCC_BDCR_LSESYSEN;

          putreg32(regval, STM32L5_RCC_BDCR);

          /* Wait for the LSE system clock to be ready */

          while (!((regval = getreg32(STM32L5_RCC_BDCR)) &
                   RCC_BDCR_LSESYSRDY))
            {
              stm32l5_waste();
            }
        }

#ifdef CONFIG_STM32L5_RTC_LSECLOCK_LOWER_RUN_DRV_CAPABILITY

      /* Set running drive capability for LSE oscillator. */

      regval &= ~RCC_BDCR_LSEDRV_MASK;
      regval |= RCC_BDCR_LSEDRV_LOW << RCC_BDCR_LSEDRV_SHIFT;
      putreg32(regval, STM32L5_RCC_BDCR);
#endif

      /* Disable backup domain access if it was disabled on entry */

      (void)stm32l5_pwr_enablebkp(writable);
    }
}
