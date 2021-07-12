/****************************************************************************
 * boards/arm/stm32l5/nucleo-l552ze/include/board.h
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

#ifndef __BOARDS_ARM_STM32L5_NUCLEO_L552ZE_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32L5_NUCLEO_L552ZE_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The NUCLEO-L552ZE-Q supports both HSE and LSE crystals (X2 and X3).
 * However, as shipped, the X3 crystal is not populated.  Therefore the
 * Nucleo-L552ZE-Q will need to run off the 16MHz HSI clock, or the
 * 32kHz-synced MSI.  This configuration uses the MSI.
 *
 *   System Clock source : PLL (MSI)
 *   SYSCLK(Hz)          : 110000000   Determined by PLL configuration
 *   HCLK(Hz)            : 110000000    (STM32L5_RCC_CFGR_HPRE)  (Max 110MHz)
 *   AHB Prescaler       : 1            (STM32L5_RCC_CFGR_HPRE)  (Max 110MHz)
 *   APB1 Prescaler      : 1            (STM32L5_RCC_CFGR_PPRE1) (Max 110MHz)
 *   APB2 Prescaler      : 1            (STM32L5_RCC_CFGR_PPRE2) (Max 110MHz)
 *   MSI Frequency(Hz)   : 4000000      (nominal)
 *   PLLM                : 1            (STM32L5_PLLCFG_PLLM)
 *   PLLN                : 55           (STM32L5_PLLCFG_PLLN)
 *   PLLP                : 0            (STM32L5_PLLCFG_PLLP)
 *   PLLQ                : 0            (STM32L5_PLLCFG_PLLQ)
 *   PLLR                : 2            (STM32L5_PLLCFG_PLLR)
 *   Flash Latency(WS)   : 5
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * MSI - 4 MHz, autotrimmed via LSE
 * HSE - not installed
 * LSE - 32.768 kHz installed
 */

#define STM32L5_HSI_FREQUENCY     16000000ul
#define STM32L5_LSI_FREQUENCY     32000
#define STM32L5_LSE_FREQUENCY     32768

#define STM32L5_BOARD_USEMSI      1
#define STM32L5_BOARD_MSIRANGE    RCC_CR_MSIRANGE_4M

/* prescaler common to all PLL inputs */

#define STM32L5_PLLCFG_PLLM             RCC_PLLCFG_PLLM(1)

/* 'main' PLL config; we use this to generate our system clock */

#define STM32L5_PLLCFG_PLLN             RCC_PLLCFG_PLLN(55)
#define STM32L5_PLLCFG_PLLP             0
#undef  STM32L5_PLLCFG_PLLP_ENABLED
#define STM32L5_PLLCFG_PLLQ             0
#undef STM32L5_PLLCFG_PLLQ_ENABLED
#define STM32L5_PLLCFG_PLLR             RCC_PLLCFG_PLLR_2
#define STM32L5_PLLCFG_PLLR_ENABLED

/* 'SAIPLL1' is not used in this application */

#define STM32L5_PLLSAI1CFG_PLLN         RCC_PLLSAI1CFG_PLLN(24)
#define STM32L5_PLLSAI1CFG_PLLP         0
#undef  STM32L5_PLLSAI1CFG_PLLP_ENABLED
#define STM32L5_PLLSAI1CFG_PLLQ         0
#undef STM32L5_PLLSAI1CFG_PLLQ_ENABLED
#define STM32L5_PLLSAI1CFG_PLLR         0
#undef  STM32L5_PLLSAI1CFG_PLLR_ENABLED

/* 'SAIPLL2' is not used in this application */

#define STM32L5_PLLSAI2CFG_PLLN         RCC_PLLSAI2CFG_PLLN(8)
#define STM32L5_PLLSAI2CFG_PLLP         0
#undef  STM32L5_PLLSAI2CFG_PLLP_ENABLED
#define STM32L5_PLLSAI2CFG_PLLR         0
#undef  STM32L5_PLLSAI2CFG_PLLR_ENABLED

#define STM32L5_SYSCLK_FREQUENCY  110000000ul

/* Enable CLK48; get it from HSI48 */

#if defined(CONFIG_STM32L5_USBFS) || defined(CONFIG_STM32L5_RNG)
#  define STM32L5_USE_CLK48       1
#  define STM32L5_CLK48_SEL       RCC_CCIPR_CLK48SEL_HSI48
#  define STM32L5_HSI48_SYNCSRC   SYNCSRC_NONE
#endif

/* Enable LSE (for the RTC and for MSI autotrimming) */

#define STM32L5_USE_LSE           1

/* Configure the HCLK divisor (for the AHB bus, core, memory, and DMA */

#define STM32L5_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32L5_HCLK_FREQUENCY    STM32L5_SYSCLK_FREQUENCY

/* Configure the APB1 prescaler */

#define STM32L5_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK       /* PCLK1 = HCLK / 1 */
#define STM32L5_PCLK1_FREQUENCY   (STM32L5_HCLK_FREQUENCY / 1)

#define STM32L5_APB1_TIM2_CLKIN   (STM32L5_PCLK1_FREQUENCY)
#define STM32L5_APB1_TIM3_CLKIN   (STM32L5_PCLK1_FREQUENCY)
#define STM32L5_APB1_TIM4_CLKIN   (STM32L5_PCLK1_FREQUENCY)
#define STM32L5_APB1_TIM5_CLKIN   (STM32L5_PCLK1_FREQUENCY)
#define STM32L5_APB1_TIM6_CLKIN   (STM32L5_PCLK1_FREQUENCY)
#define STM32L5_APB1_TIM7_CLKIN   (STM32L5_PCLK1_FREQUENCY)

/* Configure the APB2 prescaler */

#define STM32L5_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK       /* PCLK2 = HCLK / 1 */
#define STM32L5_PCLK2_FREQUENCY   (STM32L5_HCLK_FREQUENCY / 1)

#define STM32L5_APB2_TIM1_CLKIN   (STM32L5_PCLK2_FREQUENCY)
#define STM32L5_APB2_TIM15_CLKIN  (STM32L5_PCLK2_FREQUENCY)
#define STM32L5_APB2_TIM16_CLKIN  (STM32L5_PCLK2_FREQUENCY)

/* The timer clock frequencies are automatically defined by hardware.  If the
 * APB prescaler equals 1, the timer clock frequencies are set to the same
 * frequency as that of the APB domain. Otherwise they are set to twice.
 * Note: TIM1,15,16 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32L5_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    STM32L5_HCLK_FREQUENCY
#define BOARD_TIM3_FREQUENCY    STM32L5_HCLK_FREQUENCY
#define BOARD_TIM4_FREQUENCY    STM32L5_HCLK_FREQUENCY
#define BOARD_TIM5_FREQUENCY    STM32L5_HCLK_FREQUENCY
#define BOARD_TIM6_FREQUENCY    STM32L5_HCLK_FREQUENCY
#define BOARD_TIM7_FREQUENCY    STM32L5_HCLK_FREQUENCY
#define BOARD_TIM15_FREQUENCY   STM32L5_HCLK_FREQUENCY
#define BOARD_TIM16_FREQUENCY   STM32L5_HCLK_FREQUENCY
#define BOARD_LPTIM1_FREQUENCY  STM32L5_HCLK_FREQUENCY
#define BOARD_LPTIM2_FREQUENCY  STM32L5_HCLK_FREQUENCY

/* DMA Channel/Stream Selections ********************************************/

/* Alternate function pin selections ****************************************/

/* USART3: Connected to Arduino connector D0/D1 (or to STLink VCP if solder
 * bridges SB123 to SB130 are re-worked accordingly).
 */

#define GPIO_USART3_RX   GPIO_USART3_RX_4    /* PD9 */
#define GPIO_USART3_TX   GPIO_USART3_TX_4    /* PD8 */

/* LED definitions **********************************************************/

/* The Nucleo-144 board has numerous LEDs but only three, LD1 a Green LED,
 * LD2 a Blue LED and LD3 a Red LED, that can be controlled by software.
 * The following definitions assume the default Solder Bridges are installed.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs
 * in any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_NLEDS       3

#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_BLUE    BOARD_LED2
#define BOARD_LED_RED     BOARD_LED3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/stm32_autoleds.c. The LEDs are used to encode OS-
 * related events as follows:
 *
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                        Red   Green Blue
 *   ----------------------  --------------------------  ------ ------ ----
 */
#define LED_STARTED        0 /* NuttX has been started   OFF    OFF   OFF  */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  OFF    OFF   ON   */
#define LED_IRQSENABLED    2 /* Interrupts enabled       OFF    ON    OFF  */
#define LED_STACKCREATED   3 /* Idle stack created       OFF    ON    ON   */
#define LED_INIRQ          4 /* In an interrupt          N/C    N/C   GLOW */
#define LED_SIGNAL         5 /* In a signal handler      N/C    GLOW  N/C  */
#define LED_ASSERTION      6 /* An assertion failed      GLOW   N/C   GLOW */
#define LED_PANIC          7 /* The system has crashed   Blink  OFF   N/C  */
#define LED_IDLE           8 /* MCU is is sleep mode     ON     OFF   OFF  */

/* Thus if the Green LED is statically on, NuttX has successfully booted and
 * is, apparently, running normally.  If the Red LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 */

/* Button definitions *******************************************************/

/* The Nucleo-L552ZE supports one button:  Pushbutton B1, labeled "User", is
 * connected to GPIO PC13.
 * A high value will be sensed when the button is pressed.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1
#define BUTTON_USER_BIT    (1 << BUTTON_USER)

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
 * Name: stm32l5_board_initialize
 *
 * Description:
 *   All STM32L5 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices
 *   have been initialized.
 *
 ****************************************************************************/

void stm32l5_board_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __BOARDS_ARM_STM32L5_NUCLEO_L552ZE_INCLUDE_BOARD_H */
