/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_rcc.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

#include "hardware/stm32_flash.h"
#include "stm32_rcc.h"
#include "stm32_hsi48.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_STM32F0L0G0_RNG
#  ifndef STM32_USE_CLK48
#    error RNG requires CLK48 enabled
#  endif
#endif

#ifdef CONFIG_STM32F0L0G0_USB
#  ifndef STM32_USE_CLK48
#    error USB requires CLK48 enabled
#  endif
#endif

/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * that is a very long delay, but if the clock does not become ready we are
 * hosed anyway.
 */

#define HSERDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Include chip-specific clocking initialization logic */

#if defined(CONFIG_ARCH_CHIP_STM32F0)
#  include "stm32f0_rcc.c"
#elif defined(CONFIG_ARCH_CHIP_STM32L0)
#  include "stm32l0_rcc.c"
#elif defined(CONFIG_ARCH_CHIP_STM32G0)
#  include "stm32g0_rcc.c"
#else
#  error "Unsupported STM32F0/L0 RCC"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rcc_resetbkp
 *
 * Description:
 *   The RTC needs to reset the Backup Domain to change RTCSEL and resetting
 *   the Backup Domain renders to disabling the LSE as consequence.
 *   In order to avoid resetting the Backup Domain when we already configured
 *   LSE we will reset the Backup Domain early (here).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_STM32F0L0G0_RTC) && defined(CONFIG_STM32F0L0G0_PWR)
static inline void rcc_resetbkp(void)
{
  uint32_t regval;

  /* Check if the RTC is already configured */

  stm32_pwr_initbkp(false);

  regval = getreg32(RTC_MAGIC_REG);
  if (regval != RTC_MAGIC && regval != RTC_MAGIC_TIME_SET)
    {
      stm32_pwr_enablebkp(true);

      /* We might be changing RTCSEL - to ensure such changes work, we must
       * reset the backup domain (having backed up the RTC_MAGIC token)
       */

      modifyreg32(STM32_RCC_BDCR, 0, RCC_BDCR_BDRST);
      modifyreg32(STM32_RCC_BDCR, RCC_BDCR_BDRST, 0);

      stm32_pwr_enablebkp(false);
    }
}
#else
#  define rcc_resetbkp()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_clockconfig
 *
 * Description:
 *   Called to establish the clock settings based on the values in board.h.
 *   This function (by default) will reset most everything, enable the PLL,
 *   and enable peripheral clocking for all peripherals enabled in the NuttX
 *   configuration file.
 *
 *   If CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG is defined, then clocking
 *   will be enabled by an externally provided, board-specific function
 *   called stm32_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_clockconfig(void)
{
  /* Make sure that we are starting in the reset state */

  rcc_reset();

  /* Reset backup domain if appropriate */

  rcc_resetbkp();

#if defined(CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG)
  /* Invoke Board Custom Clock Configuration */

  stm32_board_clockconfig();

#else
  /* Invoke standard, fixed clock configuration based on definitions
   * in board.h
   */

  stm32_stdclockconfig();

#endif

  /* Enable peripheral clocking */

  rcc_enableperipherals();
}

/****************************************************************************
 * Name: stm32_clockenable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings in
 *   board.h.  This function is only available to support low-power modes of
 *   operation:  When re-awakening from deep-sleep modes, it is necessary to
 *   re-enable/re-start the PLL
 *
 *   This functional performs a subset of the operations performed by
 *   stm32_clockconfig():  It does not reset any devices, and it does not
 *   reset the currently enabled peripheral clocks.
 *
 *   If CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG is defined, then clocking
 *   will be enabled by an externally provided, board-specific function
 *   called stm32_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void stm32_clockenable(void)
{
#if defined(CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG)
  /* Invoke Board Custom Clock Configuration */

  stm32_board_clockconfig();

#else
  /* Invoke standard, fixed clock configuration based on definitions
   * in board.h
   */

  stm32_stdclockconfig();

#endif
}
#endif
