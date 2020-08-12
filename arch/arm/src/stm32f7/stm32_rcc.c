/****************************************************************************
 * arch/arm/src/stm32f7/stm32_rcc.c
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "stm32_pwr.h"

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
 * Included Files
 ****************************************************************************/

/* Include chip-specific clocking initialization logic */

#if defined(CONFIG_STM32F7_STM32F72XX) || defined(CONFIG_STM32F7_STM32F73XX)
#  include "stm32f72xx73xx_rcc.c"
#elif defined(CONFIG_STM32F7_STM32F74XX) || defined(CONFIG_STM32F7_STM32F75XX)
#  include "stm32f74xx75xx_rcc.c"
#elif defined(CONFIG_STM32F7_STM32F76XX) || defined(CONFIG_STM32F7_STM32F77XX)
#  include "stm32f76xx77xx_rcc.c"
#else
#  error "Unsupported STM32 F7 chip"
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
 *   configurationfile.
 *
 *   If CONFIG_STM32F7_CUSTOM_CLOCKCONFIG is defined, then clocking
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

#if defined(CONFIG_STM32F7_PWR)

  /* Insure the bkp is initialized */

  stm32_pwr_initbkp(false);
#endif

#if defined(CONFIG_STM32F7_CUSTOM_CLOCKCONFIG)

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

#ifdef CONFIG_STM32F7_SYSCFG_IOCOMPENSATION
  /* Enable I/O Compensation */

  stm32_iocompensation();
#endif
}

/****************************************************************************
 * Name: stm32_clockenable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings
 *   in board.h. This function is only available to support low-power
 *   modes of operation:  When re-awakening from deep-sleep modes, it is
 *   necessary to re-enable/re-start the PLL
 *
 *   This functional performs a subset of the operations performed by
 *   stm32_clockconfig():  It does not reset any devices, and it does not
 *   reset the currently enabled peripheral clocks.
 *
 *   If CONFIG_STM32F7_CUSTOM_CLOCKCONFIG is defined, then clocking
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
#if defined(CONFIG_STM32F7_CUSTOM_CLOCKCONFIG)

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
