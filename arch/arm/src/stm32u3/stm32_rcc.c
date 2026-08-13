/****************************************************************************
 * arch/arm/src/stm32u3/stm32_rcc.c
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

#include <arch/board/board.h>

#include "stm32_rcc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name
 *
 * Description
 *   Called to establish the clock settings based on the values in board.h.
 *   This function (by default) will reset most everything, enable the PLL,
 *   and enable peripheral clocking for all peripherals enabled in the NuttX
 *   configuration file.
 *
 *   If CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG is defined, then
 *   clocking will be enabled by an externally provided, board-specific
 *   function called stm32_board_clockconfig().
 *
 * Input Parameters
 *   None
 *
 * Returned Value
 *   None
 *
 ****************************************************************************/

void stm32_clockconfig(void)
{
#if defined(CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  stm32_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions in
   * board.h
   */

  stm32_stdclockconfig();

#endif

  /* Enable peripheral clocking */

  stm32_rcc_enableperipherals();
}

/****************************************************************************
 * Name
 *
 * Description
 *   Re-enable the clock and restore the clock settings based on settings in
 *   board.h.  This function is only available to support low-power modes of
 *   operation
 *   re-enable/re-start the PLL
 *
 *   This function performs a subset of the operations performed by
 *   stm32_clockconfig()
 *   reset the currently enabled peripheral clocks.
 *
 *   If CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG is defined, then
 *   clocking will be enabled by an externally provided, board-specific
 *   function called stm32_board_clockconfig().
 *
 * Input Parameters
 *   None
 *
 * Returned Value
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

  /* Invoke standard, fixed clock configuration based on definitions in
   * board.h
   */

  stm32_stdclockconfig();

#endif
}
#endif
