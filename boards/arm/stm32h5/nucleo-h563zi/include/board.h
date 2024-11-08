/****************************************************************************
 * boards/arm/stm32h5/nucleo-h563zi/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* The Nucleo-H563ZI-Q supports using a HSE crystal (X3). It is shipped with
 * the crystal populated, but requires solder bridge configuration to enable
 * it. Therefore, the Nucleo-H563ZI-Q will need to run off the 64MHz HSI
 * clock, or the 4 MHz CSI clock. This configuration uses the HSI.
 *
 *    System Clock Source : PLL1
 *    SYSCLK Freq (MHz)   : 250
 *    HCLK Freq   (MHz)   : 250
 *    PLL1 Freq   (MHz)   : 250
 *    Flash Latency (WS)  : 5
 *
 * NOTE : The STM32H5 requires PLL1P to be configured, as this is used as the
 * system clock source. A custom clock config function must be supplied to
 * use a different system clock source.
 */

#define STM32_SYSCLK_FREQUENCY  250000000ul
#define STM32_LSI_FREQUENCY         32000
#define STM32_LSE_FREQUENCY         32768

#ifdef CONFIG_STM32H5_USE_HSE

#define STM32_HSE_FREQUENCY     25000000ul
#define STM32_BOARD_USEHSE

/* PLL1 config: Use to generate 250 MHz system clock
 *  With HSE Freq = 25 MHz
 */

#define STM32_PLLCFG_PLL1CFG     (RCC_PLL1CFGR_PLL1SRC_HSE  | \
                                  RCC_PLL1CFGR_PLL1RGE_4_8M | \
                                  RCC_PLL1CFGR_PLL1M(5) | \
                                  RCC_PLL1CFGR_PLL1PEN | \
                                  RCC_PLL1CFGR_PLL1QEN | \
                                  RCC_PLL1CFGR_PLL1REN)
#define STM32_PLLCFG_PLL1N        RCC_PLL1DIVR_PLL1N(100)
#define STM32_PLLCFG_PLL1P        RCC_PLL1DIVR_PLL1P(2)
#define STM32_PLLCFG_PLL1Q        RCC_PLL1DIVR_PLL1Q(2)
#define STM32_PLLCFG_PLL1R        RCC_PLL1DIVR_PLL1R(2)
#define STM32_PLLCFG_PLL1DIVR     (STM32_PLLCFG_PLL1N | \
                                   STM32_PLLCFG_PLL1P | \
                                   STM32_PLLCFG_PLL1Q | \
                                   STM32_PLLCFG_PLL1R)

#define STM32_VC01_FRQ            ((STM32_HSE_FREQUENCY / 5) * 100)
#define STM32_PLL1P_FREQUENCY     (STM32_VCO1_FRQ / 2)
#define STM32_PLL1Q_FREQUENCY     (STM32_VCO1_FRQ / 2)
#define STM32_PLL1R_FREQUENCY     (STM32_VCO1_FRQ / 2)

/* PLL2 config: Need to use for max ADC speed. */

#define STM32_PLLCFG_PLL2CFG      (RCC_PLL2CFGR_PLL2SRC_HSE | \
                                   RCC_PLL2CFGR_PLL2RGE_4_8M | \
                                   RCC_PLL2CFGR_PLL2M(5) | \
                                   RCC_PLL2CFGR_PLL2REN)
#define STM32_PLLCFG_PLL2N         RCC_PLL2DIVR_PLL2N(60)
#define STM32_PLLCFG_PLL2R         RCC_PLL2DIVR_PLL2R(4)
#define STM32_PLLCFG_PLL2DIVR     (STM32_PLLCFG_PLL2N | \
                                   STM32_PLLCFG_PLL2R)

#define STM32_VCO2_FRQ            ((STM32_HSE_FREQUENCY / 5) * 60)
#define STM32_PLL2R_FREQUENCY     (STM32_VCO2_FRQ / 4)

#else

#define STM32_BOARD_USEHSI       1
#define STM32_BOARD_HSIDIV       RCC_CR_HSIDIV(1)
#define STM32_HSI_FREQUENCY      32000000ul

/* PLL1 config: Used to generate system clock
 *  PLL1DIVR expects N, P, Q, and R should be defined.
 *  With HSI Freq = 32 MHz, this gives 250 MHz pll1_y_ck output
 */

#define STM32_PLLCFG_PLL1CFG      (RCC_PLL1CFGR_PLL1SRC_HSI  | \
                                   RCC_PLL1CFGR_PLL1RGE_4_8M | \
                                   RCC_PLL1CFGR_PLL1M(8) | \
                                   RCC_PLL1CFGR_PLL1PEN | \
                                   RCC_PLL1CFGR_PLL1QEN | \
                                   RCC_PLL1CFGR_PLL1REN)
#define STM32_PLLCFG_PLL1N         RCC_PLL1DIVR_PLL1N(125)
#define STM32_PLLCFG_PLL1P         RCC_PLL1DIVR_PLL1P(2)
#define STM32_PLLCFG_PLL1Q         RCC_PLL1DIVR_PLL1Q(2)
#define STM32_PLLCFG_PLL1R         RCC_PLL1DIVR_PLL1R(2)
#define STM32_PLLCFG_PLL1DIVR     (STM32_PLLCFG_PLL1N | \
                                   STM32_PLLCFG_PLL1P | \
                                   STM32_PLLCFG_PLL1Q | \
                                   STM32_PLLCFG_PLL1R)

#define STM32_VCO1_FRQ            ((STM32_HSI_FREQUENCY / 8) * 125)
#define STM32_PLL1P_FREQUENCY     (STM32_VCO1_FRQ / 2)
#define STM32_PLL1Q_FREQUENCY     (STM32_VCO1_FRQ / 2)
#define STM32_PLL1R_FREQUENCY     (STM32_VCO1_FRQ / 2)

/* PLL2 config: Needed to use 2 ADC at max speed. */

#define STM32_PLLCFG_PLL2CFG      (RCC_PLL2CFGR_PLL2SRC_HSI | \
                                   RCC_PLL2CFGR_PLL2RGE_4_8M | \
                                   RCC_PLL2CFGR_PLL2M(8) | \
                                   RCC_PLL2CFGR_PLL2REN)
#define STM32_PLLCFG_PLL2N         RCC_PLL2DIVR_PLL2N(75)
#define STM32_PLLCFG_PLL2R         RCC_PLL2DIVR_PLL2R(4)
#define STM32_PLLCFG_PLL2DIVR     (STM32_PLLCFG_PLL2N | \
                                   STM32_PLLCFG_PLL2R)

#define STM32_VCO2_FRQ            ((STM32_HSI_FREQUENCY / 8) * 75)
#define STM32_PLL2R_FREQUENCY     (STM32_VCO2_FRQ / 4)

#endif /* CONFIG_STM32H5_USE_HSE*/

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

#define STM32_RCC_CFGR2_PPRE1     RCC_CFGR2_PPRE1_HCLK1      /* PCLK1 = HCLK / 1 */
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

#define STM32_RCC_CFGR2_PPRE3     RCC_CFGR2_PPRE3_HCLK1      /* PCLK2 = HCLK / 1 */
#define STM32_PCLK3_FREQUENCY    (STM32_HCLK_FREQUENCY / 1)

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

/* Ethernet definitions *****************************************************/

#define GPIO_ETH_MDC          (GPIO_ETH_MDC_0 | GPIO_SPEED_100MHZ)          /* PC1 */
#define GPIO_ETH_MDIO         (GPIO_ETH_MDIO_0 | GPIO_SPEED_100MHZ)         /* PA2 */
#define GPIO_ETH_RMII_RXD0    (GPIO_ETH_RMII_RXD0_0 | GPIO_SPEED_100MHZ)    /* PC4 */
#define GPIO_ETH_RMII_RXD1    (GPIO_ETH_RMII_RXD1_0 | GPIO_SPEED_100MHZ)    /* PC5 */
#define GPIO_ETH_RMII_TXD0    (GPIO_ETH_RMII_TXD0_3 | GPIO_SPEED_100MHZ)    /* PG13 */
#define GPIO_ETH_RMII_TXD1    (GPIO_ETH_RMII_TXD1_1 | GPIO_SPEED_100MHZ)    /* PB15 */
#define GPIO_ETH_RMII_TX_EN   (GPIO_ETH_RMII_TX_EN_3 | GPIO_SPEED_100MHZ)   /* PG11 */
#define GPIO_ETH_RMII_CRS_DV  (GPIO_ETH_RMII_CRS_DV_0 | GPIO_SPEED_100MHZ)  /* PA7 */
#define GPIO_ETH_RMII_REF_CLK (GPIO_ETH_RMII_REF_CLK_0 | GPIO_SPEED_100MHZ) /* PA1 */

/* ADC Clock Source *********************************************************/

#define STM32_RCC_CCIPR5_ADCDACSEL RCC_CCIPR5_ADCDACSEL_PLL2RCK
#define STM32_ADC_CLK_FREQUENCY    STM32_PLL2R_FREQUENCY

#define GPIO_ADC1_IN3   (GPIO_ADC1_IN3_0)
#define GPIO_ADC1_IN10  (GPIO_ADC1_IN10_0)

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
