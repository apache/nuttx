/****************************************************************************
 * boards/arm/stm32/olimex-stm32-p207/include/board.h
 *
 *   Copyright (C) 2009, 2012, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __BOARDS_ARM_STM32_OLIMEX_STM32_P207_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_OLIMEX_STM32_P207_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif
#include "stm32_rcc.h"
#include "stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *************************************************************************/

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC (30-60KHz, uncalibrated)
 * HSE - On-board crystal frequency is 25MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        25000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (25,000,000 / 25) * 240
 *         = 240,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 240,000,000 / 2 = 120,000,000
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 240,000,000 / 5 = 48,000,000
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(25)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(240)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(5)

#define STM32_SYSCLK_FREQUENCY  120000000ul

/* AHB clock (HCLK) is SYSCLK (120MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (30MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 (60Mhz)*/

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (60MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 (120Mhz)*/

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM3_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM4_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM5_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM6_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM7_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM8_FREQUENCY    STM32_HCLK_FREQUENCY

/* LED definitions ******************************************************************/
/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
 * way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_LED4        3
#define BOARD_NLEDS       4

#define BOARD_LED_GREEN1  BOARD_LED1
#define BOARD_LED_YELLOW  BOARD_LED2
#define BOARD_LED_RED     BOARD_LED3
#define BOARD_LED_GREEN2  BOARD_LED4

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)
#define BOARD_LED4_BIT    (1 << BOARD_LED4)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 4 LEDs on board the
 * Olimex STM32-P207.  The following definitions describe how NuttX controls the LEDs:
 */

#define LED_STARTED       0  /* LED1 */
#define LED_HEAPALLOCATE  1  /* LED2 */
#define LED_IRQSENABLED   2  /* LED1 + LED2 */
#define LED_STACKCREATED  3  /* LED3 */
#define LED_INIRQ         4  /* LED1 + LED3 */
#define LED_SIGNAL        5  /* LED2 + LED3 */
#define LED_ASSERTION     6  /* LED1 + LED2 + LED3 */
#define LED_PANIC         7  /* N/C  + N/C  + N/C + LED4 */

/* Button definitions ***************************************************************/
/* The Olimex STM32-P207 supports seven buttons: */

#define BUTTON_TAMPER     0
#define BUTTON_WKUP       1
#define BUTTON_RIGHT      2
#define BUTTON_UP         3
#define BUTTON_LEFT       4
#define BUTTON_DOWN       5
#define BUTTON_CENTER     6

#define NUM_BUTTONS       7

#define BUTTON_TAMPER_BIT (1 << BUTTON_TAMPER)
#define BUTTON_WKUP_BIT   (1 << BUTTON_WKUP)
#define BUTTON_RIGHT_BIT  (1 << BUTTON_RIGHT)
#define BUTTON_UP_BIT     (1 << BUTTON_UP)
#define BUTTON_LEFT_BIT   (1 << BUTTON_LEFT)
#define BUTTON_DOWN_BIT   (1 << BUTTON_DOWN)
#define BUTTON_CENTER_BIT (1 << BUTTON_CENTER)

/* Alternate function pin selections ************************************************/

//USART3:
#define GPIO_USART3_RX    GPIO_USART3_RX_3  //PD9
#define GPIO_USART3_TX    GPIO_USART3_TX_3  //PD8
#define GPIO_USART3_CTS   GPIO_USART3_CTS_2 //PD11
#define GPIO_USART3_RTS   GPIO_USART3_RTS_2 //PD12

//CAN:
#define GPIO_CAN1_RX      GPIO_CAN1_RX_2 //PB8
#define GPIO_CAN1_TX      GPIO_CAN1_TX_2 //PB9

//Ethernet:
/*
 * - PA2  is ETH_MDIO
 * - PC1  is ETH_MDC
 * - PB5  is ETH_PPS_OUT     - NC (not connected)
 * - PA0  is ETH_MII_CRS     - NC
 * - PA3  is ETH_MII_COL     - NC
 * - PB10 is ETH_MII_RX_ER   - NC
 * - PB0  is ETH_MII_RXD2    - NC
 * - PH7  is ETH_MII_RXD3    - NC
 * - PC3  is ETH_MII_TX_CLK  - NC
 * - PC2  is ETH_MII_TXD2    - NC
 * - PB8  is ETH_MII_TXD3    - NC
 * - PA1  is ETH_MII_RX_CLK/ETH_RMII_REF_CLK
 * - PA7  is ETH_MII_RX_DV/ETH_RMII_CRS_DV
 * - PC4  is ETH_MII_RXD0/ETH_RMII_RXD0
 * - PC5  is ETH_MII_RXD1/ETH_RMII_RXD1
 * - PB11 is ETH_MII_TX_EN/ETH_RMII_TX_EN
 * - PG13 is ETH_MII_TXD0/ETH_RMII_TXD0
 * - PG14 is ETH_MII_TXD1/ETH_RMII_TXD1
 */

#define GPIO_ETH_PPS_OUT    GPIO_ETH_PPS_OUT_1
#define GPIO_ETH_MII_CRS    GPIO_ETH_MII_CRS_1
#define GPIO_ETH_MII_COL    GPIO_ETH_MII_COL_1
#define GPIO_ETH_MII_RX_ER  GPIO_ETH_MII_RX_ER_1
#define GPIO_ETH_MII_RXD2   GPIO_ETH_MII_RXD2_1
#define GPIO_ETH_MII_RXD3   GPIO_ETH_MII_RXD3_1
#define GPIO_ETH_MII_TXD3   GPIO_ETH_MII_TXD3_1
#define GPIO_ETH_MII_TX_EN  GPIO_ETH_MII_TX_EN_2
#define GPIO_ETH_MII_TXD0   GPIO_ETH_MII_TXD0_2
#define GPIO_ETH_MII_TXD1   GPIO_ETH_MII_TXD1_2
#define GPIO_ETH_RMII_TX_EN GPIO_ETH_RMII_TX_EN_1
#define GPIO_ETH_RMII_TXD0  GPIO_ETH_RMII_TXD0_2
#define GPIO_ETH_RMII_TXD1  GPIO_ETH_RMII_TXD1_2

#endif /* __BOARDS_ARM_STM32_OLIMEX_STM32_P207_INCLUDE_BOARD_H */
