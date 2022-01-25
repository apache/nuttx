/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_rcc.c
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
 *   If CONFIG_ARCH_BOARD_STM32F0G0L0_CUSTOM_CLOCKCONFIG is defined, then
 *   clocking will be enabled by an externally provided, board-specific
 *   function called stm32_board_clockconfig().
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

#if defined(CONFIG_ARCH_BOARD_STM32F0G0L0_CUSTOM_CLOCKCONFIG)
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
 *   If CONFIG_ARCH_BOARD_STM32F0G0L0_CUSTOM_CLOCKCONFIG is defined, then
 *   clocking will be enabled by an externally provided, board-specific
 *   function called stm32_board_clockconfig().
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
#if defined(CONFIG_ARCH_BOARD_STM32F0G0L0_CUSTOM_CLOCKCONFIG)
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
