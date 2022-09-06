/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_rcu.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_GD32F4XX_RCU_H
#define __ARCH_ARM_SRC_GD32F4_GD32F4XX_RCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if defined(CONFIG_GD32F4_GD32F4XX)
#  include "hardware/gd32f4xx_rcu.h"
#else
#  error "Unknown GD32 chip"
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
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#define RCU_PERI_REG_SHIFT      (6)
#define RCU_GPIOA_EN            ((GD32_RCU_AHB1EN_OFFSET << RCU_PERI_REG_SHIFT) | 0)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_clockconfig
 *
 * Description:
 *   Called to initialize the GD32F4.  This does whatever setup is needed to
 *   put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void gd32_clockconfig(void);

/****************************************************************************
 * Name: gd32_rcu_ckout0_config
 *
 * Description:
 *   Configure the CK_OUT0 clock source and divider. CK_OUT0 is connected
 *   to PA8. PA8 should be configured in alternate function mode.
 ****************************************************************************/

void gd32_rcu_ckout0_config(uint32_t src, uint32_t div);

/****************************************************************************
 * Name: gd32_rcu_ckout1_config
*
 * Description:
 *   Configure the CK_OUT0 clock source and divider. CK_OUT0 is connected
 *   to PC9. PC9 should be configured in alternate function mode.
 ****************************************************************************/

void gd32_rcu_ckout1_config(uint32_t src, uint32_t div);

/****************************************************************************
 * Name: gd32_board_clockconfig
 *
 * Description:
 *   Any GD32 board may replace the "standard" board clock configuration
 *   logic with its own, custom clock configuration logic.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BOARD_GD32F4_CUSTOM_CLOCKCONFIG
void gd32_board_clock_config(void);
#endif

/****************************************************************************
 * Name: gd32_clockenable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings in
 *   board.h.
 *   This function is only available to support low-power modes of operation:
 *   When re-awakening from deep-sleep modes, it is necessary to re-enable/
 *   re-start the PLL
 *
 *   This functional performs a subset of the operations performed by
 *   gd32_clockconfig():  It does not reset any devices, and it does not
 *   reset the currently enabled peripheral clocks.
 *
 *   If CONFIG_ARCH_BOARD_GD32F4_CUSTOM_CLOCKCONFIG is defined, then clocking
 *   will be enabled by an externally provided, board-specific function
 *   called gd32_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void gd32_clock_enable(void);
#endif

/****************************************************************************
 * Name: gd32_rcu_lxtal_enable
 *
 * Description:
 *   Enable the External Low Speed crystal oscillator (LXTAL).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_rcu_lxtal_enable(void);

/****************************************************************************
 * Name: gd32_rcu_irc16_enable
 *
 * Description:
 *   Enable the Internal 16M RC oscillator (IRC16M).
 *
 ****************************************************************************/

void gd32_rcu_irc16m_enable(void);

/****************************************************************************
 * Name: gd32_rcu_irc16m_disable
 *
 * Description:
 *   Disable the Internal 16M RC oscillator (IRC16M).
 *
 ****************************************************************************/

void gd32_rcu_irc16m_disable(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32F4_GD32F4XX_RCU_H */
