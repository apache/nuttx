/****************************************************************************
 * boards/arm/stm32f0l0g0/nucleo-g070rb/include/board.h
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

#ifndef __BOARDS_ARM_STM32F0L0G0_NUCLEO_G070RB_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32F0L0G0_NUCLEO_G070RB_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - Internal 16 MHz RC Oscillator
 * LSI - 32 KHz RC
 * HSE - 8 MHz from MCO output of ST-LINK (disabled by default)
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul          /* 8MHz */

#define STM32_HSI_FREQUENCY     16000000ul         /* 16MHz */
#define STM32_LSI_FREQUENCY     32000              /* Between 30kHz and 60kHz */
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768              /* X2 on board */

/* Main PLL Configuration.
 *
 * PLL_VCO = (PLL_SOURCE_FREQUENCY / PLL_M) * PLL_N
 * SYSCLK  = PLLCLK = PLLR = PLL_VCO / PLL_DIV_R
 * PLLP    = PLL_VCO / PLL_DIV_P
 *
 * Subject to:
 *
 * PLL_SOURCE_FREQUENCY is one of {STM32_HSE_FREQUENCY, STM32_HSI_FREQUENCY}
 *
 *       1 <= PLL_DIV_M <= 8
 *       8 <= PLL_DIV_N <= 86
 *       2 <= PLL_DIV_P <= 32
 *       2 <= PLL_DIV_R <= 8
 *   4 MHz <= PLL_SOURCE_FREQUENCY <= 48 MHz
 *  96 MHz <= PLL_VCO <= 344MHz
 */

/* Considering:
 * - PLL_SOURCE_FREQUENCY = STM32_HSI_FREQUENCY = 16,000,000
 * - PLL_DIV_M = 1
 * - PLL_DIV_N = 8
 * - PLL_DIV_R = 2
 * - PLL_DIV_P = 2
 *
 *   PLL_VCO = (16,000,000 / 1) * 8 = 128 MHz
 *   PLLP    =    (PLL_VCO / 2)     = 64 MHz
 *   PLLR    =    (PLL_VCO / 2)     = 64 MHz
 */

#define STM32_PLLCFG_PLLSRC     RCC_PLLCFG_PLLSRC_HSI
#define STM32_PLLCFG_PLLCFG     (RCC_PLLCFG_PLLPEN | \
                                 RCC_PLLCFG_PLLREN)

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(1)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(8)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP(2)
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(2)

#define STM32_VCO_FREQUENCY     ((STM32_HSI_FREQUENCY / 1) * 8)
#define STM32_PLLP_FREQUENCY    (STM32_VCO_FREQUENCY / 2)
#define STM32_PLLR_FREQUENCY    (STM32_VCO_FREQUENCY / 2)

/* Use the PLL and set the SYSCLK source to be the PLLR (64 MHz) */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY  (STM32_PLLR_FREQUENCY)

/* AHB clock (HCLK) is SYSCLK (64 MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK (64 MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK
#define STM32_PCLK1_FREQUENCY   STM32_HCLK_FREQUENCY

/* Timer clock frequencies */

/* Timers driven from APB1. Frequency = PCLK1 */

#define STM32_APB1_TIM3_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (STM32_PCLK1_FREQUENCY)

/* Timers driven from APB2 is equal to PCLK1 */

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB2_TIM14_CLKIN  (STM32_PCLK1_FREQUENCY)
#define STM32_APB2_TIM15_CLKIN  (STM32_PCLK1_FREQUENCY)
#define STM32_APB2_TIM16_CLKIN  (STM32_PCLK1_FREQUENCY)
#define STM32_APB2_TIM17_CLKIN  (STM32_PCLK1_FREQUENCY)

/* LED definitions **********************************************************/

/* Nucleo G070RB board has three LEDs. Two of these are controlled by
 * logic on the board and are not available for software control:
 *
 * LD1 COM:  LD1 default status is red.  LD1 turns to green to indicate that
 *           communications are in progress between the PC and the
 *           ST-LINK/V2-1.
 * LD3 5V_PWR:  green LED indicates that the board is powered by a 5V source.
 *
 * And one can be controlled by software:
 *
 * User LD4: green LED is a user LED connected to STM32 I/O PA5.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LED in
 * any way.  The following definition is used to access the LED.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1       0 /* User LD4 */
#define BOARD_NLEDS      1

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT   (1 << BOARD_LED1)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on the
 * board. The following definitions describe how NuttX controls
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

/* Nucleo G070RB board supports two buttons; only one button is controllable
 * by software:
 *
 *   B1 USER:  user button connected to STM32 I/O PC13.
 *   B2 RESET: push button connected to NRST; used to RESET the MCU.
 */

#define BUTTON_USER      0  /* User B1 */
#define NUM_BUTTONS      1

#define BUTTON_USER_BIT  (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/* I2C */

#define GPIO_I2C1_SCL       (GPIO_I2C1_SCL_3|GPIO_SPEED_LOW)      /* PB8  */
#define GPIO_I2C1_SDA       (GPIO_I2C1_SDA_3|GPIO_SPEED_LOW)      /* PB9  */

/* TIM */

#define GPIO_TIM1_CH1OUT    (GPIO_TIM1_CH1OUT_1|GPIO_SPEED_LOW)   /* PA8  */
#define GPIO_TIM1_CH2OUT    (GPIO_TIM1_CH2OUT_2|GPIO_SPEED_LOW)   /* PB3  */
#define GPIO_TIM1_CH3OUT    (GPIO_TIM1_CH3OUT_2|GPIO_SPEED_LOW)   /* PB6  */
#define GPIO_TIM1_CH4OUT    (GPIO_TIM1_CH4OUT_1|GPIO_SPEED_LOW)   /* PA11 */
#define GPIO_TIM1_CH1NOUT   (GPIO_TIM1_CH1NOUT_2|GPIO_SPEED_LOW)  /* PB13 */
#define GPIO_TIM1_CH2NOUT   (GPIO_TIM1_CH2NOUT_2|GPIO_SPEED_LOW)  /* PB14 */
#define GPIO_TIM1_CH3NOUT   (GPIO_TIM1_CH3NOUT_2|GPIO_SPEED_LOW)  /* PB15 */

#define GPIO_TIM3_CH1OUT    (GPIO_TIM3_CH1OUT_2|GPIO_SPEED_LOW)   /* PB4  */
#define GPIO_TIM3_CH2OUT    (GPIO_TIM3_CH2OUT_2|GPIO_SPEED_LOW)   /* PB5  */
#define GPIO_TIM3_CH3OUT    (GPIO_TIM3_CH3OUT_1|GPIO_SPEED_LOW)   /* PB0  */
#define GPIO_TIM3_CH4OUT    (GPIO_TIM3_CH4OUT_1|GPIO_SPEED_LOW)   /* PB1  */

#define GPIO_TIM14_CH1OUT   (GPIO_TIM14_CH1OUT_2|GPIO_SPEED_LOW)  /* PA7  */

#define GPIO_TIM15_CH1OUT   (GPIO_TIM15_CH1OUT_3|GPIO_SPEED_LOW)  /* PC1  */
#define GPIO_TIM15_CH2OUT   (GPIO_TIM15_CH2OUT_3|GPIO_SPEED_LOW)  /* PC2  */
#define GPIO_TIM15_CH1NOUT  (GPIO_TIM15_CH1NOUT_1|GPIO_SPEED_LOW) /* PA1  */

#define GPIO_TIM16_CH1OUT   (GPIO_TIM16_CH1OUT_3|GPIO_SPEED_LOW)  /* PD0  */

#define GPIO_TIM17_CH1OUT   (GPIO_TIM17_CH1OUT_2|GPIO_SPEED_LOW)  /* PD1  */

/* USART */

/* By default the USART2 is connected to STLINK Virtual COM Port:
 * USART2_RX - PA3
 * USART2_TX - PA2
 */

#define GPIO_USART2_RX      (GPIO_USART2_RX_1|GPIO_SPEED_HIGH)    /* PA3 */
#define GPIO_USART2_TX      (GPIO_USART2_TX_1|GPIO_SPEED_HIGH)    /* PA2 */

/* DMA channels *************************************************************/

/* ADC */

/* TODO ADC */

#endif /* __BOARDS_ARM_STM32F0L0G0_NUCLEO_G070RB_INCLUDE_BOARD_H */
