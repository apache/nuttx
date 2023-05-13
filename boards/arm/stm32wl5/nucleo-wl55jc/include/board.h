/****************************************************************************
 * boards/arm/stm32wl5/nucleo-wl55jc/include/board.h
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

#ifndef __BOARDS_ARM_STM32WL5_NUCLEO_WL55JC_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32WL5_NUCLEO_WL55JC_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/* Clocking *****************************************************************/

/* nucleo-wl55jc has installed 32Mhz HSE oscillator */

#define STM32WL5_XTAL_FREQ           32000000ul

/* Use the HSE */

#define STM32WL5_BOARD_USEHSE        1

/* HSE source is a TCXO crystal which needs to be first powered on */

#define STM32WL5_BOARD_USETCXO

/* Prescaler common to all PLL inputs */

#define STM32WL5_PLLCFG_PLLM         RCC_PLLCFG_PLLM(2)  /* 32MHz / 2 = 16MHz */

/* 'main' PLL config; we use this to generate our system clock */

/* disable unused pll clocks */

#define STM32WL5_PLLCFG_PLLP         0
#undef  STM32WL5_PLLCFG_PLLP_ENABLED
#define STM32WL5_PLLCFG_PLLQ         0
#undef  STM32WL5_PLLCFG_PLLQ_ENABLED

/* further multiplicate source for system clock */

#define STM32WL5_PLLCFG_PLLN         RCC_PLLCFG_PLLN(6)  /* 16MHz * 6 = 96MHz */
#define STM32WL5_PLLCFG_PLLR         RCC_PLLCFG_PLLR(2)  /* 96MHz / 2 = 48MHz */
#define STM32WL5_PLLCFG_PLLR_ENABLED

/* Resulting system clock is 48MHz */
#define STM32WL5_SYSCLK_FREQUENCY    48000000ul

/* Configure the HCLK divisor (for the AHB bus, core, memory, and DMA */

#define STM32WL5_RCC_CFGR_HPRE       RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32WL5_HCLK_FREQUENCY      STM32WL5_SYSCLK_FREQUENCY

/* Configure the HCLK3 divisor (for flash and sram2) */
#define STM32WL5_RCC_CFGR_HPRE       RCC_CFGR_HPRE_SYSCLK      /* HCLK3  = SYSCLK / 1 */
#define STM32WL5_HCLK3_FREQUENCY     STM32WL5_SYSCLK_FREQUENCY

/* Configure the APB1 prescaler */

#define STM32WL5_RCC_CFGR_PPRE1      RCC_CFGR_PPRE1_HCLK       /* PCLK1 = HCLK / 1 */
#define STM32WL5_PCLK1_FREQUENCY     (STM32WL5_HCLK_FREQUENCY / 1)

/* The timer clock frequencies are automatically defined by hardware.
 * If the APB prescaler equals 1, the timer clock frequencies are set to the
 * same frequency as that of the APB domain. Otherwise they are set to twice.
 */

#define STM32WL5_APB1_TIM2_CLKIN     (STM32WL5_PCLK1_FREQUENCY)

/* Configure the APB2 prescaler */

#define STM32WL5_RCC_CFGR_PPRE2      RCC_CFGR_PPRE2_HCLK       /* PCLK2 = HCLK / 1 */
#define STM32WL5_PCLK2_FREQUENCY     (STM32WL5_HCLK_FREQUENCY / 1)

/* The timer clock frequencies are automatically defined by hardware.
 * If the APB prescaler equals 1, the timer clock frequencies are set to the
 * same frequency as that of the APB domain. Otherwise they are set to twice.
 */

#define STM32WL5_APB2_TIM1_CLKIN     STM32WL5_PCLK2_FREQUENCY
#define STM32WL5_APB2_TIM16_CLKIN    STM32WL5_PCLK2_FREQUENCY
#define STM32WL5_APB2_TIM17_CLKIN    STM32WL5_PCLK2_FREQUENCY

/* The timer clock frequencies are automatically defined by hardware. If the
 * APB prescaler equals 1, the timer clock frequencies are set to the same
 * frequency as that of the APB domain. Otherwise they are set to twice.
 * Note: TIM1,15,16 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY         STM32WL5_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY         STM32WL5_HCLK_FREQUENCY
#define BOARD_TIM16_FREQUENCY        STM32WL5_HCLK_FREQUENCY
#define BOARD_TIM17_FREQUENCY        STM32WL5_HCLK_FREQUENCY
#define BOARD_LPTIM1_FREQUENCY       STM32WL5_HCLK_FREQUENCY
#define BOARD_LPTIM2_FREQUENCY       STM32WL5_HCLK_FREQUENCY
#define BOARD_LPTIM3_FREQUENCY       STM32WL5_HCLK_FREQUENCY

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Alternate function pin selections ****************************************/

/* USART1:
 *   RXD: PB7   (D0 on arduino pinout)
 *   TXD: PB6   (D1 on arduino pinout)
 */

#define GPIO_USART1_RX GPIO_USART1_RX_2    /* PB7 */
#define GPIO_USART1_TX GPIO_USART1_TX_2    /* PB6 */

/* LPUART1
 *   Connected to virtual com port
 */

#define GPIO_LPUART1_RX GPIO_LPUART1_RX_1  /* PA3  */
#define GPIO_LPUART1_TX GPIO_LPUART1_TX_1  /* PA2 */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_2
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_2
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_2

#define GPIO_SPI2S2_MISO   GPIO_SPI2_MISO_2
#define GPIO_SPI2S2_MOSI   GPIO_SPI2_MOSI_2
#define GPIO_SPI2S2_SCK    GPIO_SPI2_SCK_3

/* user buttons
 *
 * There are 3 buttons provided for user to program
 *
 * PA0 - Button 1
 * PA1 - Button 2
 * PC6 - Button 3
 *
 * Buttons need to be pulled up internally by chip,
 * and button press will pull pin down to LOW level.
 */

#define BOARD_BUTTON1  0
#define BOARD_BUTTON2  1
#define BOARD_BUTTON3  2
#define BOARD_NBUTTONS 3

#define BUTTON1_BIT (1 << BOARD_BUTTON1)
#define BUTTON2_BIT (1 << BOARD_BUTTON2)
#define BUTTON3_BIT (1 << BOARD_BUTTON3)

/* LEDs
 *
 * The Nucleo wl55jc board provides 3 user leds
 *   PB15 - blue LED
 *   PB11 - red LED
 *   PB9  - green LED
 *
 * - When the I/O is HIGH value, the LED is on.
 * - When the I/O is LOW, the LED is off.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_BLUE   0
#define BOARD_LED_GREEN  1
#define BOARD_LED_RED    2
#define BOARD_NLEDS      3

/* LED bits for use with board_userled_all() */

#define BOARD_LED_BLUE_BIT    (1 << BOARD_LED_BLUE)
#define BOARD_LED_GREEN_BIT   (1 << BOARD_LED_GREEN)
#define BOARD_LED_RED_BIT     (1 << BOARD_LED_RED)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/stm32_autoleds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *      SYMBOL            Val    Meaning                     LED state
 *                                                       BLUE GREEN RED
 *      ----------------- ---   -----------------------  ---- ----- ----
 */

#define LED_STARTED       1  /* NuttX has been started   ON    OFF  OFF */
#define LED_HEAPALLOCATE  2  /* Heap has been allocated  OFF   ON   OFF */
#define LED_IRQSENABLED   3  /* Interrupts enabled       ON    ON   OFF */
#define LED_STACKCREATED  4  /* Idle stack created       OFF   OFF  ON  */
#define LED_INIRQ         5  /* In an interrupt          GLOW  N/C  N/C */
#define LED_SIGNAL        6  /* In a signal handler      GLOW  N/C  N/C */
#define LED_ASSERTION     7  /* An assertion failed      GLOW  N/C  N/C */
#define LED_PANIC         8  /* The system has crashed   ON    ON   ON  */
#undef  LED_IDLE             /* MCU is is sleep mode        Not used    */

/****************************************************************************
 * Public Data
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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wl5_board_initialize
 *
 * Description:
 *   All STM32WL5 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32wl5_board_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32WL5_NUCLEO_WL55JC_INCLUDE_BOARD_H */
