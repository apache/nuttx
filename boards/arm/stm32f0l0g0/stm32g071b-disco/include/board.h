/****************************************************************************
 * boards/arm/stm32f0l0g0/stm32g071b-disco/include/board.h
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

#ifndef __BOARDS_ARM_STM32F0L0G0_STM32G071B_DISCO_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32F0L0G0_STM32G071B_DISCO_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - Internal 16 MHz RC Oscillator
 * LSI - 32 KHz RC
 * HSE - 8 MHz from MCO output of ST-LINK
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000            /* Between 30kHz and 60kHz */
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768            /* X2 on board */

/* Main PLL Configuration.
 *
 * PLL source is HSI = 16,000,000
 *
 * PLL_VCOx = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 * Subject to:
 *
 *   1 <= PLLM <= 8
 *   8 <= PLLN <= 86
 *   4 MHz <= PLL_IN <= 16MHz
 *   64 MHz <= PLL_VCO <= 344MHz
 *   SYSCLK  = PLLRCLK = PLL_VCO / PLLR
 *
 */

/* PLL source is HSI, PLLN=50, PLLM=4
 * PLLP enable, PLLQ enable, PLLR enable
 *
 *   2 <= PLLP <= 32
 *   2 <= PLLQ <= 8
 *   2 <= PLLR <= 8
 *
 *   PLLR <= 64MHz
 *   PLLQ <= 128MHz
 *   PLLP <= 128MHz
 *
 *   PLL_VCO = (16,000,000 / 4) * 50 = 200 MHz
 *
 *   PLLP = PLL_VCO/4 = 200 MHz / 4 = 40 MHz
 *   PLLQ = PLL_VCO/4 = 200 MHz / 4 = 40 MHz
 *   PLLR = PLL_VCO/4 = 200 MHz / 4 = 40 MHz
 */

#define STM32_PLLCFG_PLLSRC     RCC_PLLCFG_PLLSRC_HSI
#define STM32_PLLCFG_PLLCFG     (RCC_PLLCFG_PLLPEN | \
                                 RCC_PLLCFG_PLLQEN | \
                                 RCC_PLLCFG_PLLREN)

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(4)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(50)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP(4)
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(4)
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(4)

#define STM32_VCO_FREQUENCY     ((STM32_HSE_FREQUENCY / 2) * 50)
#define STM32_PLLP_FREQUENCY    (STM32_VCO_FREQUENCY / 4)
#define STM32_PLLQ_FREQUENCY    (STM32_VCO_FREQUENCY / 4)
#define STM32_PLLR_FREQUENCY    (STM32_VCO_FREQUENCY / 4)

/* Use the PLL and set the SYSCLK source to be the PLLR (40MHz) */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY  (STM32_PLLR_FREQUENCY)
#define STM32_SYSCLK_FREQUENCY  (STM32_PLLR_FREQUENCY)

/* AHB clock (HCLK) is SYSCLK (40MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/2 (20MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* LED definitions **********************************************************/

/* LED index values for use with board_userled() */

#define BOARD_LEDSINK   0 /* LD4: SINK mode LED */
#define BOARD_LEDSOURCE 1 /* LD5: SOURCE mode LED */
#define BOARD_LEDSPY    2 /* LD6: SPY mode LED */
#define BOARD_LEDCC     3 /* LD7: CC mode LED */
#define BOARD_NLEDS     4

/* LED bits for use with board_userled_all() */

#define BOARD_LEDSINK_BIT   (1 << BOARD_LEDSINK)
#define BOARD_LEDSOURCE_BIT (1 << BOARD_LEDSOURCE)
#define BOARD_LEDSPY_BIT    (1 << BOARD_LEDSPY)
#define BOARD_LEDCC_BIT     (1 << BOARD_LEDCC)

/* Button definitions *******************************************************/

/* The STM32G071B-DISO supports one buttons:
 *
 *   B1 RESET: push button connected to NRST is used to RESET the
 *             STM32G071RB.
 *
 * and a Joystick:
 *
 *   Joystick center - PC0
 *   Joystick down   - PC2
 *   Joystick left   - PC1
 *   Joystick right  - PC3
 *   Joystick up     - PC4
 */

/* Alternate function pin selections ****************************************/

/* USART */

/* By default the USART3 is connected to STLINK Virtual COM Port:
 * USART3_RX - PC11
 * USART3_TX - PC10
 */

#define GPIO_USART3_RX GPIO_USART3_RX_6 /* PC11 */
#define GPIO_USART3_TX GPIO_USART3_TX_6 /* PC10 */

/* I2C1
 *   I2C1_SCL - PB6
 *   I2C1_SDA - PB7
 */

#define GPIO_I2C1_SCL GPIO_I2C1_SCL_2 /* PB6 */
#define GPIO_I2C1_SDA GPIO_I2C1_SDA_2 /* PB7 */

/* SPI1 - OLED display
 *   SPI1_MISO - not used
 *   SPI1_MOSI - PA2
 *   SPI1_SCK  - PA1
 */

#undef  GPIO_SPI1_MISO                  /* Not used */
#define GPIO_SPI1_MOSI GPIO_SPI1_MOSI_1 /* PA2 */
#define GPIO_SPI1_SCK  GPIO_SPI1_SCK_1  /* PA1 */

#endif /* __BOARDS_ARM_STM32F0L0G0_STM32G071B_DISCO_INCLUDE_BOARD_H */
