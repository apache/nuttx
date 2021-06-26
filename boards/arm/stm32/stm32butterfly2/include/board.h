/****************************************************************************
 * boards/arm/stm32/stm32butterfly2/include/board.h
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

#ifndef __BOARDS_ARM_STM32_STM32_BUTTERFLY2_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_STM32_BUTTERFLY2_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "stm32_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - 8 MHz RC factory-trimmed
 * LSI - 40 KHz RC (30-60KHz, uncalibrated)
 * HSE - On-board crystal frequency is 14.7456MHz
 * LSE - LSE is not connected
 */

#define STM32_BOARD_XTAL        14745600ul

#define STM32_HSI_FREQUENCY     8000000ul
#define STM32_LSI_FREQUENCY     40000u
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     0

/* PLL output is 71.8848MHz */

#define STM32_PLL_PREDIV2       RCC_CFGR2_PREDIV2d4
#define STM32_PLL_PLL2MUL       RCC_CFGR2_PLL2MULx12
#define STM32_PLL_PREDIV1       RCC_CFGR2_PREDIV1d4
#define STM32_PLL_PLLMUL        RCC_CFGR_PLLMUL_CLKx65
#define STM32_PLL_FREQUENCY     71884800ul

/* SYSCLK and HCLK adre the PLL frequency */

#define STM32_SYSCLK_FREQUENCY  STM32_PLL_FREQUENCY
#define STM32_HCLK_FREQUENCY    STM32_PLL_FREQUENCY

/* USB clock output is 47.9232MHz */

#define STM32_CFGR_OTGFSPRE     RCC_CFGR_OTGFSPREd3

/* APB2 clock (PCLK2) is HCLK */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_APB2_CLKIN        STM32_PCLK2_FREQUENCY

#define STM32APB_TIM1_CLKIN     STM32_PCLK2_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/2 (35.9424MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)

/* LED definitions **********************************************************/

/* There are four LEDs on stm32butterfly2 board that can be controlled by
 * software.
 * All pulled high and van be illuminated by driving the output low.
 *
 *   LED1 PB0
 *   LED2 PB1
 *   LED3 PC4
 *   LED4 PC5
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1      0
#define BOARD_LED2      1
#define BOARD_LED3      2
#define BOARD_LED4      3
#define BOARD_NLEDS     4

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT  (1 << BOARD_LED1)
#define BOARD_LED2_BIT  (1 << BOARD_LED2)
#define BOARD_LED3_BIT  (1 << BOARD_LED3)
#define BOARD_LED4_BIT  (1 << BOARD_LED4)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In thath case, the usage by the board port is defined in
 * include/board.h and src/stm32_leds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *      SYMBOL            Val    Meaning                     LED state
 *                                                       LED1 LED2 LED3 LED4
 *      ----------------- ---   -----------------------  ---- ---- ---- ----
 */

#define LED_STARTED       0  /* NuttX has been started   ON   OFF  OFF  OFF   */
#define LED_HEAPALLOCATE  1  /* Heap has been allocated  OFF  ON   OFF  OFF   */
#define LED_IRQSENABLED   2  /* Interrupts enabled       OFF  OFF  ON   OFF   */
#define LED_STACKCREATED  3  /* Idle stack created       OFF  OFF  OFF  ON    */
#define LED_INIRQ         5  /* In an interrupt          N/C  N/C  N/C  GLOW  */
#define LED_SIGNAL        6  /* In a signal handler      N/C  N/C  N/C  GLOW  */
#define LED_ASSERTION     7  /* An assertion failed      N/C  N/C  N/C  GLOW  */
#define LED_PANIC         8  /* The system has crashed   N/C  N/C  N/C  FLASH */
#undef  LED_IDLE             /* MCU is is sleep mode         Not used         */

/* After booting, LED1-3 are not longer used by the system and can be used
 * for other purposes by the application (Of course, all LEDs are available
 * to the application if CONFIG_ARCH_LEDS is not defined.
 */

/* ADC configuration. Right now only ADC12_IN10 is supported
 * (potentiometer)
 */

#ifdef CONFIG_STM32_ADC2
#  error "CONFIG_STM32_ADC2 is not supported"
#endif

/* SPI configuration. Only SPI1 is supported */

#ifdef CONFIG_STM32_SPI2
#  error "CONFIG_STM32_SPI2 is not supported"
#endif

#endif /* __BOARDS_ARM_STM32_STM32_BUTTERFLY2_INCLUDE_BOARD_H */
