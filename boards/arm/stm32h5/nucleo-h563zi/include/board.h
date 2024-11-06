/****************************************************************************
 * boards/arm/stm32h5/nucleo-h563zi/include/board.h
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

#ifndef __BOARDS_ARM_STM32H5_NUCLEO_H563ZI_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32H5_NUCLEO_H563ZI_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The NUCLEO-H563ZI-Q supports both HSE and LSE crystals (X2 and X3).
 * However, as shipped, the X3 crystal is not populated.  Therefore the
 * Nucleo-H563ZI-Q will need to run off the 32MHz HSI clock, or the
 * 4 MHz CSI clock.  This configuration uses the HSI.
 *
 *   System Clock source : PLL (CSI)
 *   SYSCLK(Hz)          : 250000000   Determined by PLL1 configuration
 *   HCLK(Hz)            : 250000000    (STM32_RCC_CFGR_HPRE)  (Max 250MHz)
 *   AHB Prescaler       : 1            (STM32_RCC_CFGR_HPRE)  (Max 250MHz)
 *   APB1 Prescaler      : 1            (STM32_RCC_CFGR_PPRE1) (Max 250MHz)
 *   APB2 Prescaler      : 1            (STM32_RCC_CFGR_PPRE2) (Max 250MHz)
 *   CSI Frequency(Hz)   : 4000000      (nominal)
 *   PLL1M               : 2            (STM32_PLL1CFGR_PLLM)
 *   PLL1N               : 31           (STM32_PLL1CFGR_PLLN)
 *   PLL1P               : 2            (STM32_PLL1CFGR_PLLP)
 *   PLL1Q               : 0            (STM32_PLL1CFGR_PLLQ)
 *   PLL1R               : 1            (STM32_PLL1CFGR_PLLR)
 *   PLL2M               : 2            (STM32_PLL2CFGR_PLLM)
 *   PLL2N               : 15           (STM32_PLL2CFGR_PLLN)
 *   PLL2P               : 0            (STM32_PLL2CFGR_PLLP)
 *   PLL2Q               : 0            (STM32_PLL2CFGR_PLLQ)
 *   PLL2R               : 1            (STM32_PLL2CFGR_PLLR)
 *   PLL3M               : 2            (STM32_PLL3CFGR_PLLM)
 *   PLL3N               : 15           (STM32_PLL3CFGR_PLLN)
 *   PLL3P               : 0            (STM32_PLL3CFGR_PLLP)
 *   PLL3Q               : 0            (STM32_PLL3CFGR_PLLQ)
 *   PLL3R               : 1            (STM32_PLL3CFGR_PLLR)
 *   Flash Latency(WS)   : 5
 */

/* HSI - 32 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * CSI - 4 MHz, autotrimmed via LSE
 * HSE - not installed
 * LSE - 32.768 kHz installed
 * SYSCLK = 250 MHz
 */

#define STM32_SYSCLK_FREQUENCY  250000000ul
#define STM32_HSI_FREQUENCY      32000000ul
#define STM32_LSI_FREQUENCY         32000
#define STM32_LSE_FREQUENCY         32768

#define STM32_BOARD_USEHSI       1
#define STM32_CR_HSIDIV          RCC_CR_HSIDIV(1)

/* prescaler common to all PLL inputs */

/* 'main' PLL1 config; we use this to generate our system clock */

/* Use 32 MHz HSI, set M to 2, N to 31, FRAC to 0x800 (2048), PLL1P to 2
 * SYSCLK = ((HSI / PLL1M) * (PLL1N + (PLL1FRACN/8192))) / PLL1P
 * SYSCLK = ((32000000 / 2) * (31 + (2048/8192))) / 2 = 250000000
 */

#define STM32_PLL1CFGR_PLL1FRACEN        RCC_PLL1CFGR_PLL1FRACEN 
#define STM32_PLL1CFGR_PLL1VCOSEL        0
#define STM32_PLL1CFGR_PLL1RGE           RCC_PLL1CFGR_PLL1RGE_8_16M 

#define STM32_PLL1CFGR_PLL1M             RCC_PLL1CFGR_PLL1M(2)
#define STM32_PLL1DIVR_PLL1N             RCC_PLL1DIVR_PLL1N(31)

#define STM32_PLL1DIVR_PLL1P             RCC_PLL1DIVR_PLL1P(2)
#define STM32_PLL1CFGR_PLL1P_ENABLED     1
#define STM32_PLL1P_FREQUENCY            250000000
#define STM32_PLL1DIVR_PLL1Q             0
#undef STM32_PLL1CFGR_PLL1Q_ENABLED
#define STM32_PLL1DIVR_PLL1R             0 
#undef STM32_PLL1CFGR_PLL1R_ENABLED

#define STM32_PLL1FRACR_PLL1FRACN        RCC_PLL1FRACR_PLL1FRACN(2048)

/* PLL2 config */

#define STM32_PLL2CFGR_PLL2M             RCC_PLL2CFGR_PLL2M(4)
#define STM32_PLL2CFGR_PLL2FRACEN        RCC_PLL2CFGR_PLL2FRACEN 
#define STM32_PLL2CFGR_PLL2VCOSEL        RCC_PLL2CFGR_PLL2VCOSEL
#define STM32_PLL2CFGR_PLL2RGE           RCC_PLL2CFGR_PLL2RGE_8_16M 

#define STM32_PLL2DIVR_PLL2N             RCC_PLL2DIVR_PLL2N(15)

#define STM32_PLL2DIVR_PLL2P             RCC_PLL2DIVR_PLL2P(1)
#define STM32_PLL2CFGR_PLL2P_ENABLED
#define STM32_PLL2DIVR_PLL2Q             0
#undef STM32_PLL2CFGR_PLL2Q_ENABLED
#define STM32_PLL2DIVR_PLL2R             0 
#undef STM32_PLL2CFGR_PLL2R_ENABLED

#define STM32_PLL2FRACR_PLL2FRACN        RCC_PLL2FRACR_PLL2FRACN(5120)

/* PLL3 config */

#define STM32_PLL3CFGR_PLL3M             RCC_PLL3CFGR_PLL3M(4)
#define STM32_PLL3CFGR_PLL3FRACEN        RCC_PLL3CFGR_PLL3FRACEN 
#define STM32_PLL3CFGR_PLL3VCOSEL        RCC_PLL3CFGR_PLL3VCOSEL
#define STM32_PLL3CFGR_PLL3RGE           RCC_PLL3CFGR_PLL3RGE_8_16M 

#define STM32_PLL3DIVR_PLL3N             RCC_PLL3DIVR_PLL3N(15)

#define STM32_PLL3DIVR_PLL3P             RCC_PLL3DIVR_PLL3P(1)
#define STM32_PLL3CFGR_PLL3P_ENABLED
#define STM32_PLL3DIVR_PLL3Q             0
#undef STM32_PLL3CFGR_PLL3Q_ENABLED
#define STM32_PLL3DIVR_PLL3R             0 
#undef STM32_PLL3CFGR_PLL3R_ENABLED

#define STM32_PLL3FRACR_PLL3FRACN        RCC_PLL3FRACR_PLL3FRACN(5120)

/* Enable CLK48; get it from HSI48 */

#if defined(CONFIG_STM32H5_USBFS) || defined(CONFIG_STM32H5_RNG)
#  define STM32H5_USE_CLK48       1
#  define STM32H5_CLKUSB_SEL      RCC_CCIPR4_USBSEL_HSI48KERCK 
#  define STM32H5_HSI48_SYNCSRC   SYNCSRC_NONE
#endif

/* Enable LSE (for the RTC) */

#define STM32_USE_LSE           1

/* Configure the HCLK divisor (for the AHB bus, core, memory, and DMA */

#define STM32_RCC_CFGR2_HPRE    RCC_CFGR2_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* Configure the APB1 prescaler */

#define STM32_RCC_CFGR2_PPRE1     RCC_CFGR2_PPRE1_HCLK1       /* PCLK1 = HCLK / 1 */
#define STM32_PCLK1_FREQUENCY    (STM32_HCLK_FREQUENCY / 1)

#define STM32_APB1_TIM2_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_LPTIM2_CLKIN  (STM32_PCLK1_FREQUENCY)

/* Configure the APB2 prescaler */

#define STM32_RCC_CFGR2_PPRE2    RCC_CFGR2_PPRE2_HCLK1       /* PCLK2 = HCLK / 1 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY / 1)

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM15_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM16_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM17_CLKIN  (STM32_PCLK2_FREQUENCY)

/* Configure the APB3 prescaler */

#define STM32_RCC_CFGR2_PPRE3    RCC_CFGR2_PPRE3_HCLK1       /* PCLK2 = HCLK / 1 */
#define STM32_PCLK3_FREQUENCY   (STM32_HCLK_FREQUENCY / 1)

#define STM32_APB3_LPTIM1_CLKIN  (STM32_PCLK3_FREQUENCY)
#define STM32_APB3_LPTIM3_CLKIN  (STM32_PCLK3_FREQUENCY)
#define STM32_APB3_LPTIM4_CLKIN  (STM32_PCLK3_FREQUENCY)
#define STM32_APB3_LPTIM5_CLKIN  (STM32_PCLK3_FREQUENCY)
#define STM32_APB3_LPTIM6_CLKIN  (STM32_PCLK3_FREQUENCY)
/* The timer clock frequencies are automatically defined by hardware.  If the
 * APB prescaler equals 1, the timer clock frequencies are set to the same
 * frequency as that of the APB domain. Otherwise they are set to twice.
 */

#define BOARD_TIM1_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM3_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM4_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM5_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM6_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM7_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM8_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM12_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM13_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM14_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM15_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM16_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_LPTIM1_FREQUENCY  STM32_HCLK_FREQUENCY
#define BOARD_LPTIM2_FREQUENCY  STM32_HCLK_FREQUENCY
#define BOARD_LPTIM3_FREQUENCY  STM32_HCLK_FREQUENCY
#define BOARD_LPTIM4_FREQUENCY  STM32_HCLK_FREQUENCY
#define BOARD_LPTIM5_FREQUENCY  STM32_HCLK_FREQUENCY
#define BOARD_LPTIM6_FREQUENCY  STM32_HCLK_FREQUENCY

/* Configure the Kernel clocks */

/* DMA Channel/Stream Selections ********************************************/

/* Alternate function pin selections ****************************************/

/* USART3: Connected to Arduino connector D0/D1 (or to STLink VCP if solder
 * bridges SB123 to SB130 are re-worked accordingly).
 */

#define GPIO_USART3_RX   GPIO_USART3_RX_4    /* PD9 */
#define GPIO_USART3_TX   GPIO_USART3_TX_4    /* PD8 */

/* LED definitions **********************************************************/

/* The Nucleo board has numerous LEDs but only three, LD1 a Green LED,
 * LD2 a Yellow LED, and LD3 a Red LED, that can be controlled by software.
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
#define BOARD_LED_YELLOW  BOARD_LED2
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

/* The Nucleo-H563ZI supports one button:  Pushbutton B1, labeled "User", is
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
 * Name: stm32_board_initialize
 *
 * Description:
 *   All STM32H5 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices
 *   have been initialized.
 *
 ****************************************************************************/

void stm32_board_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __BOARDS_ARM_STM32H5_NUCLEO_H563ZI_INCLUDE_BOARD_H */
