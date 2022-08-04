/****************************************************************************
 * boards/arm/stm32/nucleo-g431kb/include/board.h
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

#ifndef __BOARDS_ARM_STM32_NUCLEO_G431KB_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_NUCLEO_G431KB_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The Nucleo-G431KB supports four ways to configure high-speed clock
 *
 *   - HSI configuration (default): 16 MHz high-speed internal RC oscillator.
 *   - HSE bypass configuration (from ST-LINK): The input clock is the
 *     ST-LINK MCO output. The frequency is fixed to 25 MHz, and connected
 *     to the PF0-OSC_IN of the STM32G4 microcontroller.
 *   - HSE bypass configuration (from ARDUINO D7): The clock is coming from
 *     an external oscillator through the pin PF0 (ARDUINO D7 pin 10 of the
 *     CN4 connector).
 *   - HSE oscillator configuration: The clock is provided by an external
 *     24MHz crystal (X2) available in the PCB.
 */

#define STM32_BOARD_XTAL               24000000ul          /* 24MHz */

#define STM32_HSI_FREQUENCY            16000000ul          /* 16MHz */
#define STM32_LSI_FREQUENCY            32000               /* 32kHz */
#define STM32_HSE_FREQUENCY            STM32_BOARD_XTAL
#undef  STM32_LSE_FREQUENCY                                /* Not available on this board */

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

/* APB2 timers 1, 8, 20 and 15-17 will receive PCLK2. */

/* Timers driven from APB2 will be PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB1_TIM15_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB1_TIM16_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB1_TIM17_CLKIN  (STM32_PCLK2_FREQUENCY)

/* APB1 timers 2-7 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (STM32_PCLK1_FREQUENCY)

/* USB divider -- Divide PLL clock by 1.5 */

#define STM32_CFGR_USBPRE       0

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 */

#define BOARD_TIM1_FREQUENCY   (STM32_PCLK2_FREQUENCY)
#define BOARD_TIM2_FREQUENCY   (STM32_PCLK1_FREQUENCY)
#define BOARD_TIM3_FREQUENCY   (STM32_PCLK1_FREQUENCY)
#define BOARD_TIM4_FREQUENCY   (STM32_PCLK1_FREQUENCY)
#define BOARD_TIM5_FREQUENCY   (STM32_PCLK1_FREQUENCY)
#define BOARD_TIM6_FREQUENCY   (STM32_PCLK1_FREQUENCY)
#define BOARD_TIM7_FREQUENCY   (STM32_PCLK1_FREQUENCY)
#define BOARD_TIM8_FREQUENCY   (STM32_PCLK2_FREQUENCY)
#define BOARD_TIM15_FREQUENCY  (STM32_PCLK2_FREQUENCY)
#define BOARD_TIM16_FREQUENCY  (STM32_PCLK2_FREQUENCY)
#define BOARD_TIM17_FREQUENCY  (STM32_PCLK2_FREQUENCY)
#define BOARD_TIM20_FREQUENCY  (STM32_PCLK2_FREQUENCY)

/* LED definitions **********************************************************/

/* The Nucleo-G431KB board has only one user LED, LD2. LD2 is a green LED
 * connected to the following STM32G4 pins
 *   - PB8 (default)
 *   - PB3
 * It is also connected to Arduino signal D13.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control this LED in
 * any way. The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED2                     0 /* User LD2 */
#define BOARD_NLEDS                    1

/* LED bits for use with board_userled_all() */

#define BOARD_LED2_BIT                 (1 << BOARD_LED2)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the Nucleo-G431KB. The following definitions describe how NuttX controls
 * the LED:
 *
 *   SYMBOL              Meaning                  LED1 state
 *   ------------------  -----------------------  ----------
 *   LED_STARTED         NuttX has been started   OFF
 *   LED_HEAPALLOCATE    Heap has been allocated  OFF
 *   LED_IRQSENABLED     Interrupts enabled       OFF
 *   LED_STACKCREATED    Idle stack created       ON
 *   LED_INIRQ           In an interrupt          No change
 *   LED_SIGNAL          In a signal handler      No change
 *   LED_ASSERTION       An assertion failed      No change
 *   LED_PANIC           The system has crashed   Blinking
 *   LED_IDLE            STM32 is is sleep mode   Not used
 */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED 1
#define LED_INIRQ        2
#define LED_SIGNAL       2
#define LED_ASSERTION    2
#define LED_PANIC        1

/* Button definitions *******************************************************/

/* The Nucleo G431KB don't have buttons that are controllable by software:
 *
 *   B1 RESET: push button connected to NRST is used to RESET the
 *             STM32G431KB.
 */

/* Alternate function pin selections ****************************************/

/* USART2 (STLINK Virtual COM Port) */

#define GPIO_USART2_TX     GPIO_USART2_TX_1 /* PA2 */
#define GPIO_USART2_RX     GPIO_USART2_RX_1 /* PA3 */

/* PWM configuration ********************************************************/

/* TIM1 PWM */

#define GPIO_TIM1_CH1OUT  GPIO_TIM1_CH1OUT_1  /* PA8 */

/* Comparators configuration ************************************************/

#define GPIO_COMP2_OUT    GPIO_COMP2_OUT_3  /* PA12 */
#define GPIO_COMP2_INP    GPIO_COMP2_INP_2  /* PA7 */
#define GPIO_COMP2_INM    GPIO_COMP2_INM_2  /* PA5 check solder bridge SB2 */

/* DMA channels *************************************************************/

/* USART2 */

#define DMACHAN_USART2_TX DMAMAP_DMA12_USART2TX_0 /* DMA1 */
#define DMACHAN_USART2_RX DMAMAP_DMA12_USART2RX_0 /* DMA1 */

#endif /* __BOARDS_ARM_STM32_NUCLEO_G431KB_INCLUDE_BOARD_H */
