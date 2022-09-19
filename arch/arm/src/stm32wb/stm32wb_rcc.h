/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_STM32WB_RCC_H
#define __ARCH_ARM_SRC_STM32WB_STM32WB_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32wb_rcc.h"

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
 * Public Types
 ****************************************************************************/

/* CRS synchronization source */

enum crs_syncsrc_e
{
  SYNCSRC_NONE = 0, /* No SYNC signal */
  SYNCSRC_GPIO,     /* GPIO selected as SYNC signal source */
  SYNCSRC_LSE,      /* LSE selected as SYNC signal source */
  SYNCSRC_USB,      /* USB SOF selected as SYNC signal source */
};

typedef enum crs_syncsrc_e crs_syncsrc_t;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_mcoconfig
 *
 * Description:
 *   Selects the clock source to output and clock divider on MC pin
 *   (PA8/PA15/PB6). The pin should be configured in a proper alternate
 *   function mode.
 *
 * Input Parameters:
 *   source - One of the definitions for the RCC_CFGR_MCOSEL
 *   divider - One of the definitions for the RCC_CFGR_MCOPRE
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32wb_mcoconfig(uint32_t source, uint32_t divider)
{
  uint32_t regval;

  regval = getreg32(STM32WB_RCC_CFGR);

    /* Set MCO source */

  regval &= ~(RCC_CFGR_MCOSEL_MASK);
  regval |= (source & RCC_CFGR_MCOSEL_MASK);

    /* Set MCO divider */

  regval &= ~(RCC_CFGR_MCOPRE_MASK);
  regval |= (divider & RCC_CFGR_MCOPRE_MASK);
  putreg32(regval, STM32WB_RCC_CFGR);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_clockconfig
 *
 * Description:
 *   Called to establish the clock settings based on the values in board.h.
 *   This function (by default) will reset most everything, enable the PLL,
 *   and enable peripheral clocking for all periperipherals enabled in the
 *   NuttX configuration file.
 *
 *   If CONFIG_ARCH_BOARD_STM32WB_CUSTOM_CLOCKCONFIG is defined, then
 *   clocking will be enabled by an externally provided, board-specific
 *   function called stm32wb_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32wb_clockconfig(void);

/****************************************************************************
 * Name: stm32wb_board_clockconfig
 *
 * Description:
 *   Any STM32WB board may replace the "standard" board clock configuration
 *   logic with its own, custom clock configuration logic.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BOARD_STM32WB_CUSTOM_CLOCKCONFIG
void stm32wb_board_clockconfig(void);
#endif

/****************************************************************************
 * Name: stm32wb_clockenable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings in
 *   board.h.
 *   This function is only available to support low-power modes of operation:
 *   When re-awakening from deep-sleep modes, it is necessary to re-enable/
 *   re-start the PLL
 *
 *   This functional performs a subset of the operations performed by
 *   stm32wb_clockconfig():  It does not reset any devices, and it does not
 *   reset the currently enabled peripheral clocks.
 *
 *   If CONFIG_ARCH_BOARD_STM32WB_CUSTOM_CLOCKCONFIG is defined, then
 *   clocking will be enabled by an externally provided, board-specific
 *   function called stm32wb_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void stm32wb_clockenable(void);
#endif

/****************************************************************************
 * Name: stm32wb_rcc_enable_lse
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

void stm32wb_rcc_enable_lse(void);

/****************************************************************************
 * Name: stm32wb_rcc_enable_lsi
 *
 * Description:
 *   Enable the Internal Low-Speed (LSI) RC Oscillator.
 *
 ****************************************************************************/

void stm32wb_rcc_enable_lsi(void);

/****************************************************************************
 * Name: stm32wb_rcc_disable_lsi
 *
 * Description:
 *   Disable the Internal Low-Speed (LSI) RC Oscillator.
 *
 ****************************************************************************/

void stm32wb_rcc_disable_lsi(void);

/****************************************************************************
 * Name: stm32wb_rcc_enable_hsi48
 *
 * Description:
 *   HSI48 clock signal is generated from an internal 48 MHz RC oscillator
 *   and can be used directly as a system clock or divided and be used as
 *   PLL input.
 *
 *   The internal 48MHz RC oscillator is mainly dedicated to provide a high
 *   precision clock to the USB peripheral by means of a special Clock
 *   Recovery System (CRS) circuitry, which could use the USB SOF signal or
 *   the LSE or an external signal to automatically adjust the oscillator
 *   frequency on-fly, in a very small steps. This oscillator can also be
 *   used as a system clock source when the system is in run mode; it will
 *   be disabled as soon as the system enters in Stop or Standby mode. When
 *   the CRS is not used, the HSI48 RC oscillator runs on its default
 *   frequency which is subject to manufacturing process variations.
 *
 * Input Parameters:
 *   Identifies the syncrhonization source for the HSI48.  When used as the
 *   USB source clock, this must be set to SYNCSRC_USB.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32wb_rcc_enable_hsi48(crs_syncsrc_t syncsrc);

/****************************************************************************
 * Name: stm32wb_rcc_disable_hsi48
 *
 * Description:
 *   Disable the HSI48 clock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32wb_rcc_disable_hsi48(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32WB_STM32WB_RCC_H */
