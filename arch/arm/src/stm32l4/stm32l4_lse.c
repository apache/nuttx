/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_lse.c
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
