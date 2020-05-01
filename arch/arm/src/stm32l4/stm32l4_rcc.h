/************************************************************************************
 * arch/arm/src/stm32l4/stm32l4_rcc.h
 *
 *   Copyright (C) 2009, 2011-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.orgr>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_RCC_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_RCC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "arm_arch.h"
#include "chip.h"

#if defined(CONFIG_STM32L4_STM32L4X3)
#  include "hardware/stm32l4x3xx_rcc.h"
#elif defined(CONFIG_STM32L4_STM32L4X5)
#  include "hardware/stm32l4x5xx_rcc.h"
#elif defined(CONFIG_STM32L4_STM32L4X6)
#  include "hardware/stm32l4x6xx_rcc.h"
#elif defined(CONFIG_STM32L4_STM32L4XR)
#  include "hardware/stm32l4xrxx_rcc.h"
#else
#  error "Unsupported STM32L4 chip"
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#elseO
#define EXTERN extern
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* This symbol references the Cortex-M4 vector table (as positioned by the linker
 * script, ld.script or ld.script.dfu.  The standard location for the vector table is
 * at the beginning of FLASH at address 0x0800:0000.  If we are using the STMicro DFU
 * bootloader, then the vector table will be offset to a different location in FLASH
 * and we will need to set the NVIC vector location to this alternative location.
 */

extern uint32_t _vectors[];  /* See stm32l4_vectors.S */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32l4_mcoconfig
 *
 * Description:
 *   Selects the clock source to output on MC pin (PA8) for stm32f10xxx.
 *   PA8 should be configured in alternate function mode.
 *
 * Input Parameters:
 *   source - One of the definitions for the RCC_CFGR_MCO definitions from
 *     chip/stm32f10xxx_rcc.h {RCC_CFGR_SYSCLK, RCC_CFGR_INTCLK, RCC_CFGR_EXTCLK,
 *     RCC_CFGR_PLLCLKd2, RCC_CFGR_PLL2CLK, RCC_CFGR_PLL3CLKd2, RCC_CFGR_XT1,
 *     RCC_CFGR_PLL3CLK}
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static inline void stm32l4_mcoconfig(uint32_t source)
{
  uint32_t regval;

  /* Set MCO source */

  regval = getreg32(STM32L4_RCC_CFGR);
  regval &= ~(RCC_CFGR_MCO_MASK);
  regval |= (source & RCC_CFGR_MCO_MASK);
  putreg32(regval, STM32L4_RCC_CFGR);
}

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: stm32l4_clockconfig
 *
 * Description:
 *   Called to establish the clock settings based on the values in board.h.  This
 *   function (by default) will reset most everything, enable the PLL, and enable
 *   peripheral clocking for all periperipherals enabled in the NuttX configuration
 *   file.
 *
 *   If CONFIG_ARCH_BOARD_STM32L4_CUSTOM_CLOCKCONFIG is defined, then clocking will
 *   be enabled by an externally provided, board-specific function called
 *   stm32l4_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

void stm32l4_clockconfig(void);

/************************************************************************************
 * Name: stm32l4_board_clockconfig
 *
 * Description:
 *   Any STM32L4 board may replace the "standard" board clock configuration logic with
 *   its own, custom clock configuration logic.
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_BOARD_STM32L4_CUSTOM_CLOCKCONFIG
void stm32l4_board_clockconfig(void);
#endif

/************************************************************************************
 * Name: stm32l4_clockenable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings in board.h.
 *   This function is only available to support low-power modes of operation:  When
 *   re-awakening from deep-sleep modes, it is necessary to re-enable/re-start the
 *   PLL
 *
 *   This functional performs a subset of the operations performed by
 *   stm32l4_clockconfig():  It does not reset any devices, and it does not reset the
 *   currently enabled peripheral clocks.
 *
 *   If CONFIG_ARCH_BOARD_STM32L4_CUSTOM_CLOCKCONFIG is defined, then clocking will
 *   be enabled by an externally provided, board-specific function called
 *   stm32l4_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_PM
void stm32l4_clockenable(void);
#endif

/************************************************************************************
 * Name: stm32l4_rcc_enablelse
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
 ************************************************************************************/

void stm32l4_rcc_enablelse(void);

/****************************************************************************
 * Name: stm32l4_rcc_enablelsi
 *
 * Description:
 *   Enable the Internal Low-Speed (LSI) RC Oscillator.
 *
 ****************************************************************************/

void stm32l4_rcc_enablelsi(void);

/****************************************************************************
 * Name: stm32l4_rcc_disablelsi
 *
 * Description:
 *   Disable the Internal Low-Speed (LSI) RC Oscillator.
 *
 ****************************************************************************/

void stm32l4_rcc_disablelsi(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_RCC_H */
