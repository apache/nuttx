/****************************************************************************
 * boards/arm/stm32/b-g474e-dpow1/include/board.h
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32_B_G474E_DPOW1_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_B_G474E_DPOW1_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#undef STM32_BOARD_XTAL                                    /* Not installed by default */

#define STM32_HSI_FREQUENCY            16000000ul          /* 16MHz */
#define STM32_LSI_FREQUENCY            32000               /* 32kHz */
#undef STM32_HSE_FREQUENCY                                 /* Not installed by default */
#undef STM32_LSE_FREQUENCY                                 /* Not available on this board */

/* Main PLL Configuration.
 *
 * PLL source is HSI = 16MHz
 * PLLN = 85, PLLM = 4, PLLP = 10, PLLQ = 2, PLLR = 2
 *
 * f(VCO Clock) = f(PLL Clock Input) x (PLLN / PLLM)
 * f(PLL_P) = f(VCO Clock) / PLLP
 * f(PLL_Q) = f(VCO Clock) / PLLQ
 * f(PLL_R) = f(VCO Clock) / PLLR
 *
 * Where:
 * 8 <= PLLN <= 127
 * 1 <= PLLM <= 16
 * PLLP = 2 through 31
 * PLLQ = 2, 4, 6, or 8
 * PLLR = 2, 4, 6, or 8
 *
 * Do not exceed 170MHz on f(PLL_P), f(PLL_Q), or f(PLL_R).
 * 64MHz <= f(VCO Clock) <= 344MHz.
 *
 * Given the above:
 *
 * f(VCO Clock) = HSI   x PLLN / PLLM
 *              = 16MHz x 85   / 4
 *              = 340MHz
 *
 * PLLPCLK      = f(VCO Clock) / PLLP
 *              = 340MHz       / 10
 *              = 34MHz
 *                (May be used for ADC)
 *
 * PLLQCLK      = f(VCO Clock) / PLLQ
 *              = 340MHz       / 2
 *              = 170MHz
 *                (May be used for QUADSPI, FDCAN, SAI1, I2S3. If set to
 *                48MHz, may be used for USB, RNG.)
 *
 * PLLRCLK      = f(VCO Clock) / PLLR
 *              = 340MHz       / 2
 *              = 170MHz
 *                (May be used for SYSCLK and most peripherals.)
 */

#define STM32_PLLCFGR_PLLSRC           RCC_PLLCFGR_PLLSRC_HSI
#define STM32_PLLCFGR_PLLCFG           (RCC_PLLCFGR_PLLPEN | \
                                       RCC_PLLCFGR_PLLQEN | \
                                       RCC_PLLCFGR_PLLREN)

#define STM32_PLLCFGR_PLLN             RCC_PLLCFGR_PLLN(85)
#define STM32_PLLCFGR_PLLM             RCC_PLLCFGR_PLLM(4)
#define STM32_PLLCFGR_PLLP             RCC_PLLCFGR_PLLPDIV(10)
#define STM32_PLLCFGR_PLLQ             RCC_PLLCFGR_PLLQ_2
#define STM32_PLLCFGR_PLLR             RCC_PLLCFGR_PLLR_2

#define STM32_VCO_FREQUENCY            ((STM32_HSI_FREQUENCY / 4) * 85)
#define STM32_PLLP_FREQUENCY           (STM32_VCO_FREQUENCY / 10)
#define STM32_PLLQ_FREQUENCY           (STM32_VCO_FREQUENCY / 2)
#define STM32_PLLR_FREQUENCY           (STM32_VCO_FREQUENCY / 2)

/* Use the PLL and set the SYSCLK source to be PLLR (170MHz) */

#define STM32_SYSCLK_SW                RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS               RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY         STM32_PLLR_FREQUENCY

/* AHB clock (HCLK) is SYSCLK (170MHz) */

#define STM32_RCC_CFGR_HPRE            RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY           STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK (170MHz) */

#define STM32_RCC_CFGR_PPRE1           RCC_CFGR_PPRE1_HCLK
#define STM32_PCLK1_FREQUENCY          STM32_HCLK_FREQUENCY

/* APB2 clock (PCLK2) is HCLK (170MHz) */

#define STM32_RCC_CFGR_PPRE2           RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY          STM32_HCLK_FREQUENCY

/* LED definitions **********************************************************/

/* The B-G474E-DPOW1 Discovery kit has four user LEDs.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1                     0 /* User LD2 (Blue) */
#define BOARD_LED2                     1 /* User LD3 (Orange) */
#define BOARD_LED3                     2 /* User LD4 (Green) */
#define BOARD_LED4                     3 /* User LD5 (Red)*/
#define BOARD_NLEDS                    4

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT                 (1 << BOARD_LED1)
#define BOARD_LED2_BIT                 (1 << BOARD_LED2)
#define BOARD_LED3_BIT                 (1 << BOARD_LED3)
#define BOARD_LED4_BIT                 (1 << BOARD_LED4)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 4 user LEDs
 * on the board.  The following definitions describe how NuttX controls the
 * LEDs:
 *
 * |--------------------|-------------------------|------------|
 * | SYMBOL             | Meaning                 | LED states |
 * |--------------------|-------------------------|------------|
 * | LED_STARTED        | NuttX has been started  | 0 0 0 0    |
 * | LED_HEAPALLOCATE   | Heap has been allocated | 0 0 0 0    |
 * | LED_IRQSENABLED    | Interrupts enabled      | 0 0 0 0    |
 * | LED_STACKCREATED   | Idle stack created      | 1 0 0 0    |
 * | LED_INIRQ          | In an interrupt         | No change  |
 * | LED_SIGNAL         | In a signal handler     | No change  |
 * | LED_ASSERTION      | An assertion failed     | No change  |
 * | LED_PANIC          | The system has crashed  | 0 B 0 0    |
 * | LED_IDLE           | STM32 is is sleep mode  | Not used   |
 * |--------------------|-------------------------|------------|
 *
 * LED states legend:
 * 0 = off
 * 1 = on
 * B = blink
 */

#define LED_STARTED                    0
#define LED_HEAPALLOCATE               0
#define LED_IRQSENABLED                0
#define LED_STACKCREATED               1
#define LED_INIRQ                      2
#define LED_SIGNAL                     2
#define LED_ASSERTION                  2
#define LED_PANIC                      3

/* Button definitions *******************************************************/

/* Alternate function pin selections ****************************************/

/* USART3 (ST LINK V3E Virtual Console) */

#define GPIO_USART3_TX     GPIO_USART3_TX_3 /* PC10 */
#define GPIO_USART3_RX     GPIO_USART3_RX_3 /* PC11 */

/* Pin Multiplexing Disambiguation ******************************************/

#endif /* __BOARDS_ARM_STM32_B_G474E_DPOW1_INCLUDE_BOARD_H */
