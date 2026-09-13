/****************************************************************************
 * arch/arm/src/n32h7/n32_rcc.c
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
#include "n32_gpio.h"
#include "n32_rcc.h"
#include "n32_pwr.h"

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

#if defined(CONFIG_N32H7_N32H76X)
#  include "n32h76x_rcc.c"
#else
#  error "Unsupported N32H7 chip"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_clockconfig
 *
 * Description:
 *   Called to establish the clock settings based on the values in board.h.
 *   This function (by default) will reset most everything, enable the PLL,
 *   and enable peripheral clocking for all peripherals enabled in the NuttX
 *   configuration file.
 *
 *   If CONFIG_N32H7_CUSTOM_CLOCKCONFIG is defined, then clocking will be
 *   enabled by an externally provided, board-specific function called
 *   n32_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void n32_clockconfig(void)
{
#ifndef CONFIG_N32H7_BYPASS_CLOCKCONFIG
  /* Make sure that we are starting in the reset state */

  rcc_reset();

#  if defined(CONFIG_N32H7_PWR)

  /* Insure the bkp is initialized */

  n32_pwr_initbkp(false);
#  endif

#  if defined(CONFIG_N32H7_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  n32_board_clockconfig();

#  else

  /* Invoke standard, fixed clock configuration based on definitions in
   * board.h
   */

  n32_stdclockconfig();

#  endif
#endif /* !CONFIG_N32H7_BYPASS_CLOCKCONFIG */

  /* Enable peripheral clocking */

  rcc_enableperipherals();

#ifdef CONFIG_N32H7_SYSCFG_IOCOMPENSATION
  /* Enable I/O Compensation */

  n32_iocompensation();
#endif
}

/****************************************************************************
 * Name: n32_clockenable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings in
 *   board.h. This function is only available to support low-power modes of
 *   operation:  When re-awakening from deep-sleep modes, it is necessary to
 *   re-enable/re-start the PLL
 *
 *   This functional performs a subset of the operations performed by
 *   n32_clockconfig():  It does not reset any devices, and it does not
 *   reset the currently enabled peripheral clocks.
 *
 *   If CONFIG_N32H7_CUSTOM_CLOCKCONFIG is defined, then clocking will be
 *   enabled by an externally provided, board-specific function called
 *   n32_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void n32_clockenable(void)
{
#if defined(CONFIG_N32H7_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  n32_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions in
   * board.h
   */

  n32_stdclockconfig();

#endif
}
#endif
