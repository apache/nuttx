/****************************************************************************
 * arch/arm/src/stm32u5/stm32_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32U5_STM32_RCC_H
#define __ARCH_ARM_SRC_STM32U5_STM32_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"
#include "chip.h"

#if defined(CONFIG_STM32U5_STM32U585XX)
#  include "hardware/stm32u585xx_rcc.h"
#else
#  error "Unsupported STM32U5 chip"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#elseO
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mcoconfig
 *
 * Description:
 *   Selects the clock source to output on MCO pin (PA8) for stm32u585xx.
 *   PA8 should be configured in alternate function mode.
 *
 * Input Parameters:
 *   source - One of the RCC_CFGR_MCO definitions from chip/stm32u585xx_rcc.h
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_mcoconfig(uint32_t source)
{
  uint32_t regval;

  /* Set MCO source */

  regval = getreg32(STM32_RCC_CFGR1);
  regval &= ~(RCC_CFGR1_MCOSEL_MASK);
  regval |= (source & RCC_CFGR1_MCOSEL_MASK);
  putreg32(regval, STM32_RCC_CFGR1);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_clockconfig
 *
 * Description:
 *   Called to establish the clock settings based on the values in board.h.
 *   This function (by default) will reset most everything, enable the PLL,
 *   and enable peripheral clocking for all periperipherals enabled in the
 *   NuttX configuration file.
 *
 *   If CONFIG_ARCH_BOARD_STM32U5_CUSTOM_CLOCKCONFIG is defined, then
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

void stm32_clockconfig(void);

/****************************************************************************
 * Name: stm32_board_clockconfig
 *
 * Description:
 *   Any STM32U5 board may replace the "standard" board clock configuration
 *   logic with its own, custom clock configuration logic.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BOARD_STM32U5_CUSTOM_CLOCKCONFIG
void stm32_board_clockconfig(void);
#endif

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   The standard logic to configure the clocks based on settings in board.h.
 *   Applicable if no custom clock config is provided.  This function is
 *   chip type specific and implemented in corresponding modules such as e.g.
 *   stm3262xx_rcc.c
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_STM32U5_CUSTOM_CLOCKCONFIG
void stm32_stdclockconfig(void);
#endif

/****************************************************************************
 * Name: stm32_clockenable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings in
 *   board.h.  This function is only available to support low-power modes of
 *   operation:  When re-awakening from deep-sleep modes, it is necessary to
 *   re-enable/re-start the PLL
 *
 *   This function performs a subset of the operations performed by
 *   stm32_clockconfig():  It does not reset any devices, and it does not
 *   reset the currently enabled peripheral clocks.
 *
 *   If CONFIG_ARCH_BOARD_STM32U5_CUSTOM_CLOCKCONFIG is defined, then
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
void stm32_clockenable(void);
#endif

/****************************************************************************
 * Name: stm32_rcc_enablelse
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

void stm32_rcc_enablelse(void);

/****************************************************************************
 * Name: stm32_rcc_enablelsi
 *
 * Description:
 *   Enable the Internal Low-Speed (LSI) RC Oscillator.
 *
 ****************************************************************************/

void stm32_rcc_enablelsi(void);

/****************************************************************************
 * Name: stm32_rcc_disablelsi
 *
 * Description:
 *   Disable the Internal Low-Speed (LSI) RC Oscillator.
 *
 ****************************************************************************/

void stm32_rcc_disablelsi(void);

/****************************************************************************
 * Name: stm32_rcc_enableperipherals
 *
 * Description:
 *   Enable all the chip peripherals according to configuration.  This is
 *   chip type specific and thus implemented in corresponding modules such as
 *   e.g. stm3262xx_rcc.c
 *
 ****************************************************************************/

void stm32_rcc_enableperipherals(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32U5_STM32_RCC_H */
