/****************************************************************************
 * arch/arm/src/stm32wl5/stm32wl5_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32WL5_STM32WL5_RCC_H
#define __ARCH_ARM_SRC_STM32WL5_STM32WL5_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"
#include "chip.h"

#include "hardware/stm32wl5_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wl5_mcoconfig
 *
 * Description:
 *   Selects the clock source to output on MC pin (PA8) for stm32wl562xx
 *   PA8 should be configured in alternate function mode.
 *
 * Input Parameters:
 *   source - One of the definitions for the RCC_CFGR_MCO definitions from
 *     chip/stm32wl562xx_rcc.h {RCC_CFGR_SYSCLK, RCC_CFGR_INTCLK,
 *     RCC_CFGR_EXTCLK, RCC_CFGR_PLLCLKd2, RCC_CFGR_PLL2CLK,
 *     RCC_CFGR_PLL3CLKd2, RCC_CFGR_XT1, RCC_CFGR_PLL3CLK}
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32wl5_mcoconfig(uint32_t source)
{
  uint32_t regval;

  /* Set MCO source */

  regval = getreg32(STM32WL5_RCC_CFGR);
  regval &= ~(RCC_CFGR_MCOSEL_MASK);
  regval |= (source & RCC_CFGR_MCOSEL_MASK);
  putreg32(regval, STM32WL5_RCC_CFGR);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wl5_clockconfig
 *
 * Description:
 *   Called to establish the clock settings based on the values in board.h.
 *   This function (by default) will reset most everything, enable the PLL,
 *   and enable peripheral clocking for all periperipherals enabled in the
 *   NuttX configuration file.
 *
 *   If CONFIG_ARCH_BOARD_STM32WL5_CUSTOM_CLOCKCONFIG is defined, then
 *   clocking will be enabled by an externally provided, board-specific
 *   function called stm32wl5_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32wl5_clockconfig(void);

/****************************************************************************
 * Name: stm32wl5_board_clockconfig
 *
 * Description:
 *   Any STM32WL5 board may replace the "standard" board clock configuration
 *   logic with its own, custom clock configuration logic.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BOARD_STM32WL5_CUSTOM_CLOCKCONFIG
void stm32wl5_board_clockconfig(void);
#endif

/****************************************************************************
 * Name: stm32wl5_stdclockconfig
 *
 * Description:
 *   The standard logic to configure the clocks based on settings in board.h.
 *   Applicable if no custom clock config is provided.  This function is
 *   chip type specific and implemented in corresponding modules such as e.g.
 *   stm32wl562xx_rcc.c
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_STM32WL5_CUSTOM_CLOCKCONFIG
void stm32wl5_stdclockconfig(void);
#endif

/****************************************************************************
 * Name: stm32wl5_clockenable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings in
 *   board.h.  This function is only available to support low-power modes of
 *   operation:  When re-awakening from deep-sleep modes, it is necessary to
 *   re-enable/re-start the PLL
 *
 *   This function performs a subset of the operations performed by
 *   stm32wl5_clockconfig():  It does not reset any devices, and it does not
 *   reset the currently enabled peripheral clocks.
 *
 *   If CONFIG_ARCH_BOARD_STM32WL5_CUSTOM_CLOCKCONFIG is defined, then
 *   clocking will be enabled by an externally provided, board-specific
 *   function called stm32wl5_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void stm32wl5_clockenable(void);
#endif

/****************************************************************************
 * Name: stm32wl5_rcc_enablelse
 *
 * Description:
 *   Enable the External Low-Speed (LSE) Oscillator.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32wl5_rcc_enablelse(void);

/****************************************************************************
 * Name: stm32wl5_rcc_enablelsi
 *
 * Description:
 *   Enable the Internal Low-Speed (LSI) RC Oscillator.
 *
 ****************************************************************************/

void stm32wl5_rcc_enablelsi(void);

/****************************************************************************
 * Name: stm32wl5_rcc_disablelsi
 *
 * Description:
 *   Disable the Internal Low-Speed (LSI) RC Oscillator.
 *
 ****************************************************************************/

void stm32wl5_rcc_disablelsi(void);

/****************************************************************************
 * Name: stm32wl5_rcc_enableperipherals
 *
 * Description:
 *   Enable all the chip peripherals according to configuration.  This is
 *   chip type specific and thus implemented in corresponding modules such as
 *   e.g. stm32wl562xx_rcc.c
 *
 ****************************************************************************/

void stm32wl5_rcc_enableperipherals(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32WL5_STM32WL5_RCC_H */
