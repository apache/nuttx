/****************************************************************************
 * arch/arm/src/stm32h7/stm32_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_RCC_H
#define __ARCH_ARM_SRC_STM32H7_STM32_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"
#include "hardware/stm32_rcc.h"

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
 * Public Data
 ****************************************************************************/

/* This symbol references the Cortex-M7 vector table (as positioned by the
 * linker script).  The standard location for the vector table is at the
 * beginning of FLASH at address 0x0800:0000.  If we are using the STMicro
 * DFU bootloader, then the vector table will be offset to a different
 * location in FLASH and we will need to set the NVIC vector location to
 * this alternative location.
 */

extern uint32_t _vectors[];  /* See armv7-m/arm_vectors.c */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mco1config
 *
 * Description:
 *   Selects the clock source to output on MCO1 pin (PA8). PA8 should be
 *   configured in alternate function mode.
 *
 * Input Parameters:
 *   source - One of the definitions for the RCC_CFGR_MCO1 definitions from
 *     chip/stm32h7xxxx_rcc.h {RCC_CFGR_MCO1_HSI, RCC_CFGR_MCO1_LSE,
 *     RCC_CFGR_MCO1_HSE, RCC_CFGR_MCO1_PLL}
 *   div - One of the definitions for the RCC_CFGR_MCO1PRE definitions from
 *     chip/stm32h7xxxx_rcc.h {RCC_CFGR_MCO1PRE_NONE, RCC_CFGR_MCO1PRE_DIV2,
 *     RCC_CFGR_MCO1PRE_DIV3, RCC_CFGR_MCO1PRE_DIV4, RCC_CFGR_MCO1PRE_DIV5}
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_mco1config(uint32_t source, uint32_t div)
{
  uint32_t regval;

  regval = getreg32(STM32_RCC_CFGR);
  regval &= ~(RCC_CFGR_MCO1_MASK | RCC_CFGR_MCO1PRE_MASK);
  regval |= (source | div);
  putreg32(regval, STM32_RCC_CFGR);
}

/****************************************************************************
 * Name: stm32_mco2config
 *
 * Description:
 *   Selects the clock source to output on MCO2 pin (PC9). PC9 should be
 *   configured in alternate function mode.
 *
 * Input Parameters:
 *   source - One of the definitions for the RCC_CFGR_MCO2 definitions from
 *     chip/stn32h7xxxx_rcc.h {RCC_CFGR_MCO2_SYSCLK, RCC_CFGR_MCO2_PLLI2S,
 *     RCC_CFGR_MCO2_HSE, RCC_CFGR_MCO2_PLL}
 *   div - One of the definitions for the RCC_CFGR_MCO2PRE definitions from
 *     chip/stn32h7xxxx_rcc.h {RCC_CFGR_MCO2PRE_NONE, RCC_CFGR_MCO2PRE_DIV2,
 *     RCC_CFGR_MCO2PRE_DIV3, RCC_CFGR_MCO2PRE_DIV4, RCC_CFGR_MCO2PRE_DIV5}
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_mco2config(uint32_t source, uint32_t div)
{
  uint32_t regval;

  regval = getreg32(STM32_RCC_CFGR);
  regval &= ~(RCC_CFGR_MCO2_MASK | RCC_CFGR_MCO2PRE_MASK);
  regval |= (source | div);
  putreg32(regval, STM32_RCC_CFGR);
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
 *   and enable peripheral clocking for all peripherals enabled in the NuttX
 *   configuration file.
 *
 *   If CONFIG_STM32H7_CUSTOM_CLOCKCONFIG is defined, then clocking will be
 *   enabled by an externally provided, board-specific function called
 *   stm32_board_clockconfig().
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
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Setup the system using the "standard" clock configuration
 *
 ****************************************************************************/

void stm32_stdclockconfig(void);

/****************************************************************************
 * Name: stm32_board_clockconfig
 *
 * Description:
 *   Any STM32 board may replace the "standard" board clock configuration
 *   logic with its own, custom clock configuration logic.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_CUSTOM_CLOCKCONFIG
void stm32_board_clockconfig(void);
#endif

/****************************************************************************
 * Name: stm32_clockenable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings in
 *   board.h. This function is only available to support low-power modes of
 *   operation:  When re-awakening from deep-sleep modes, it is necessary to
 *   re-enable/re-start the PLL
 *
 *   This functional performs a subset of the operations performed by
 *   stm32_clockconfig():  It does not reset any devices, and it does not
 *   reset the currently enabled peripheral clocks.
 *
 *   If CONFIG_STM32H7_CUSTOM_CLOCKCONFIG is defined, then clocking will be
 *   enabled by an externally provided, board-specific function called
 *   stm32_board_clockconfig().
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

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_RCC_H */
