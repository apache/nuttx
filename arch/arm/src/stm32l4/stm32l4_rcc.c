/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_rcc.c
 *
 *   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2016 Sebastien Lorquet. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
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

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "stm32l4_rcc.h"
#include "stm32l4_flash.h"
#include "stm32l4.h"
#include "stm32l4_waste.h"
#include "stm32l4_rtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * that is a very long delay, but if the clock does not become ready we are
 * hosed anyway.
 */

#define HSERDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Include chip-specific clocking initialization logic */

#if defined(CONFIG_STM32L4_STM32L4X3)
#  include "stm32l4x3xx_rcc.c"
#elif defined(CONFIG_STM32L4_STM32L4X5)
#  include "stm32l4x5xx_rcc.c"
#elif defined(CONFIG_STM32L4_STM32L4X6)
#  include "stm32l4x6xx_rcc.c"
#elif defined(CONFIG_STM32L4_STM32L4XR)
#  include "stm32l4xrxx_rcc.c"
#else
#  error "Unsupported STM32L4 chip"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rcc_resetbkp
 *
 * Description:
 *   The RTC needs to reset the Backup Domain to change RTCSEL and resetting
 *   the Backup Domain renders to disabling the LSE as consequence.   In order
 *   to avoid resetting the Backup Domain when we already configured LSE we
 *   will reset the Backup Domain early (here).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_STM32L4_PWR) && defined(CONFIG_STM32L4_RTC)
static inline void rcc_resetbkp(void)
{
  bool init_stat;

  /* Check if the RTC is already configured */

  init_stat = stm32l4_rtc_is_initialized();
  if (!init_stat)
    {
      uint32_t bkregs[STM32L4_RTC_BKCOUNT];
      int i;

      /* Backup backup-registers before RTC reset. */

      for (i = 0; i < STM32L4_RTC_BKCOUNT; i++)
        {
          bkregs[i] = getreg32(STM32L4_RTC_BKR(i));
        }

       /* Enable write access to the backup domain (RTC registers, RTC
        * backup data registers and backup SRAM).
        */

      stm32l4_pwr_enablebkp(true);

      /* We might be changing RTCSEL - to ensure such changes work, we must
       * reset the backup domain (having backed up the RTC_MAGIC token)
       */

       modifyreg32(STM32L4_RCC_BDCR, 0, RCC_BDCR_BDRST);
       modifyreg32(STM32L4_RCC_BDCR, RCC_BDCR_BDRST, 0);

       /* Restore backup-registers, except RTC related. */

       for (i = 0; i < STM32L4_RTC_BKCOUNT; i++)
         {
           if (RTC_MAGIC_REG == STM32L4_RTC_BKR(i))
             {
               continue;
             }

           putreg32(bkregs[i], STM32L4_RTC_BKR(i));
         }

       stm32l4_pwr_enablebkp(false);
    }
}
#else
#  define rcc_resetbkp()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_clockconfig
 *
 * Description:
 *   Called to establish the clock settings based on the values in board.h.
 *   This function (by default) will reset most everything, enable the PLL,
 *   and enable peripheral clocking for all peripherals enabled in the NuttX
 *   configuration file.
 *
 *   If CONFIG_ARCH_BOARD_STM32L4_CUSTOM_CLOCKCONFIG is defined, then clocking
 *   will be enabled by an externally provided, board-specific function called
 *   stm32l4_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32l4_clockconfig(void)
{
  /* Make sure that we are starting in the reset state */

  rcc_reset();

  /* Reset backup domain if appropriate */

  rcc_resetbkp();

#if defined(CONFIG_ARCH_BOARD_STM32L4_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  stm32l4_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions in board.h */

  stm32l4_stdclockconfig();

#endif

  /* Enable peripheral clocking */

  rcc_enableperipherals();
}

/************************************************************************************
 * Name: stm32l4_clockenable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings in board.h.
 *   This function is only available to support low-power modes of operation:  When
 *   re-awakening from deep-sleep modes, it is necessary to re-enable/re-start the
 *   PLL
 *
 *   This functional performs a subset of the operations performed by
 *   stm32l4_clockconfig():  It does not reset any devices, and it does not reset the
 *   currently enabled peripheral clocks.
 *
 *   If CONFIG_ARCH_BOARD_STM32L4_CUSTOM_CLOCKCONFIG is defined, then clocking will
 *   be enabled by an externally provided, board-specific function called
 *   stm32l4_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_PM
void stm32l4_clockenable(void)
{
#if defined(CONFIG_ARCH_BOARD_STM32L4_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  stm32l4_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions in board.h */

  stm32l4_stdclockconfig();

#endif
}
#endif
