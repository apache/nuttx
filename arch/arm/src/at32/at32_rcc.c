/****************************************************************************
 * arch/arm/src/at32/at32_rcc.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include "chip.h"
#include "at32_gpio.h"
#include "at32_rcc.h"
#include "at32_rtc.h"
#include "at32_flash.h"
#include "at32.h"
#include "at32_waste.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * that is a very long delay, but if the clock does not become ready we are
 * hosed anyway.
 */

#define HSERDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Include chip-specific clocking initialization logic */

#if defined(CONFIG_AT32_AT32F43XX)
#  include "at32f43xxx_rcc.c"
#else
#  error "Unsupported AT32 chip"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

# define AT32_RCC_XXX        AT32_CRM_BPDC
# define RCC_XXX_YYYRST      CRM_BPDC_BPDRST

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rcc_resetbkp
 *
 * Description:
 *   The RTC needs to reset the Backup Domain to change RTCSEL and resetting
 *   the Backup Domain renders to disabling the LSE as consequence.
 *   In order to avoid resetting the Backup Domain when we already
 *   configured LSE we will reset the Backup Domain early (here).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_AT32_RTC) && defined(CONFIG_AT32_PWR)
static inline void rcc_resetbkp(void)
{
  uint32_t regval;

  /* Check if the RTC is already configured */

  at32_pwr_initbkp(false);

  regval = getreg32(RTC_MAGIC_REG);
  if (regval != RTC_MAGIC && regval != RTC_MAGIC_TIME_SET)
    {
      at32_pwr_enablebkp(true);

      /* We might be changing RTCSEL - to ensure such changes work, we must
       * reset the backup domain (having backed up the RTC_MAGIC token)
       */

      modifyreg32(AT32_RCC_XXX, 0, RCC_XXX_YYYRST);
      modifyreg32(AT32_RCC_XXX, RCC_XXX_YYYRST, 0);

      at32_pwr_enablebkp(false);
    }
}
#else
#  define rcc_resetbkp()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_clockconfig
 *
 * Description:
 *   Called to establish the clock settings based on the values in board.h.
 *   This function (by default) will reset most everything, enable the PLL,
 *   and enable peripheral clocking for all peripherals enabled in the NuttX
 *   configuration file.
 *
 *   If CONFIG_ARCH_BOARD_AT32_CUSTOM_CLOCKCONFIG is defined, then clocking
 *   will be enabled by an externally provided, board-specific function
 *   called at32_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void at32_clockconfig(void)
{
  /* Make sure that we are starting in the reset state */

  rcc_reset();

  /* Reset backup domain if appropriate */

  rcc_resetbkp();

#if defined(CONFIG_ARCH_BOARD_AT32_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  at32_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions
   * in board.h
   */

  at32_stdclockconfig();

#endif

  /* Enable peripheral clocking */

  rcc_enableperipherals();

#ifdef CONFIG_AT32_SYSCFG_IOCOMPENSATION
  /* Enable I/O Compensation */

  at32_iocompensation();
#endif
}

/****************************************************************************
 * Name: at32_clockenable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings
 *   in board.h. This function is only available to support low-power
 *   modes of operation:  When re-awakening from deep-sleep modes, it is
 *   necessary to re-enable/re-start the PLL
 *
 *   This functional performs a subset of the operations performed by
 *   at32_clockconfig():  It does not reset any devices, and it does not
 *   reset the currently enabled peripheral clocks.
 *
 *   If CONFIG_ARCH_BOARD_AT32_CUSTOM_CLOCKCONFIG is defined, then clocking
 *   will be enabled by an externally provided, board-specific function
 *   called at32_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void at32_clockenable(void)
{
#if defined(CONFIG_ARCH_BOARD_AT32_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  at32_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions
   * in board.h
   */

  at32_stdclockconfig();

#endif
}
#endif
