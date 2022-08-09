/****************************************************************************
 * boards/arm/stm32l4/steval-stlcs01v1/include/board.h
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

#ifndef __BOARDS_ARM_STM32L4_STEVAL_STLCS01V1_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32L4_STEVAL_STLCS01V1_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/*   System Clock source   : PLL (HSI)
 *   SYSCLK(Hz)            : 80000000    Determined by PLL configuration
 *   HCLK(Hz)              : 80000000   (STM32L4_RCC_CFGR_HPRE)  (Max 80 MHz)
 *   AHB Prescaler         : 1          (STM32L4_RCC_CFGR_HPRE)  (Max 80 MHz)
 *   APB1 Prescaler        : 1          (STM32L4_RCC_CFGR_PPRE1) (Max 80 MHz)
 *   APB2 Prescaler        : 1          (STM32L4_RCC_CFGR_PPRE2) (Max 80 MHz)
 *   HSI Frequency(Hz)     : 16000000   (nominal)
 *   PLLM                  : 1          (STM32L4_PLLCFG_PLLM)
 *   PLLN                  : 10         (STM32L4_PLLCFG_PLLN)
 *   PLLP                  : 0          (STM32L4_PLLCFG_PLLP)
 *   PLLQ                  : 0          (STM32L4_PLLCFG_PLLQ)
 *   PLLR                  : 2          (STM32L4_PLLCFG_PLLR)
 *   PLLSAI1N              : 12
 *   PLLSAI1Q              : 4
 *   Flash Latency(WS)     : 4
 *   Prefetch Buffer       : OFF
 *   48MHz for USB OTG FS, : Doable if required using PLLSAI1 or MSI
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * MSI - variable up to 48 MHz, synchronized to LSE
 * HSE - not installed
 * LSE - 32.768 kHz installed
 */

#define STM32L4_HSI_FREQUENCY     16000000ul
#define STM32L4_LSI_FREQUENCY     32000
#define STM32L4_LSE_FREQUENCY     32768

#define STM32L4_BOARD_USEHSI      1

/* XXX sysclk mux = pllclk */

/* XXX pll source mux = hsi */

/* REVISIT: Trimming of the HSI and MSI is not yet supported. */

/* Main PLL Configuration.
 *
 * Formulae:
 *
 *   VCO input frequency        = PLL input clock frequency / PLLM,
 *                                1 <= PLLM <= 8
 *   VCO output frequency       = VCO input frequency × PLLN,
 *                                8 <= PLLN <= 86,
 *                                frequency range 64 to 344 MHz
 *   PLL output P (SAI3) clock frequency = VCO frequency / PLLP,
 *                                         PLLP = 7, or 17,
 *                                                or 0 to disable
 *   PLL output Q (48M1) clock frequency = VCO frequency / PLLQ,
 *                                         PLLQ = 2, 4, 6, or 8,
 *                                                or 0 to disable
 *   PLL output R (CLK)  clock frequency = VCO frequency / PLLR,
 *                                         PLLR = 2, 4, 6, or 8,
 *                                                or 0 to disable
 *
 * PLL output P is used for SAI
 * PLL output Q is used for OTG FS, SDMMC, RNG
 * PLL output R is used for SYSCLK
 * PLLP = 0 (not used)
 * PLLQ = 0 (not used)
 * PLLR = 2
 * PLLN = 10
 * PLLM = 1
 *
 * We will configure like this
 *
 *   PLL source is HSI
 *
 *   PLL_REF  = STM32L4_HSI_FREQUENCY / PLLM
 *            = 16,000,000 / 1
 *            = 16,000,000
 *
 *   PLL_VCO  = PLL_REF * PLLN
 *            = 16,000,000 * 10
 *            = 160,000,000
 *
 *   PLL_CLK  = PLL_VCO / PLLR
 *            = 160,000,000 / 2 = 80,000,000
 *   PLL_48M1 = disabled
 *   PLL_SAI3 = disabled
 *
 * ----------------------------------------
 *
 * PLLSAI1 Configuration
 *
 * The clock input and M divider are identical to the main PLL.
 * However the multiplier and postscalers are independent.
 * The PLLSAI1 is configured only if CONFIG_STM32L4_SAI1PLL is defined
 *
 *   SAI1VCO input frequency        = PLL input clock frequency
 *   SAI1VCO output frequency       = SAI1VCO input frequency × PLLSAI1N,
 *                                    8 <= PLLSAI1N <= 86,
 *                                    frequency range 64 to 344 MHz
 *   SAI1PLL output P (SAI1) clock frequency = SAI1VCO frequency / PLLSAI1P,
 *                                             PLLP = 7, or 17,
 *                                                    or 0 to disable
 *   SAI1PLL output Q (48M2) clock frequency = SAI1VCO frequency / PLLSAI1Q,
 *                                             PLLQ = 2, 4, 6, or 8,
 *                                                    or 0 to disable
 *   SAI1PLL output R (ADC1) clock frequency = SAI1VCO frequency / PLLSAI1R,
 *                                             PLLR = 2, 4, 6, or 8,
 *                                                    or 0 to disable
 *
 * ----------------------------------------
 *
 * PLLSAI2 Configuration
 *
 * The clock input and M divider are identical to the main PLL.
 * However the multiplier and postscalers are independent.
 * The PLLSAI2 is configured only if CONFIG_STM32L4_SAI2PLL is defined
 *
 *   SAI2VCO input frequency        = PLL input clock frequency
 *   SAI2VCO output frequency       = SAI2VCO input frequency × PLLSAI2N,
 *                                    8 <= PLLSAI1N <= 86,
 *                                    frequency range 64 to 344 MHz
 *   SAI2PLL output P (SAI2) clock frequency = SAI2VCO frequency / PLLSAI2P,
 *                                             PLLP = 7, or 17,
 *                                                    or 0 to disable
 *   SAI2PLL output R (ADC2) clock frequency = SAI2VCO frequency / PLLSAI2R,
 *                                             PLLR = 2, 4, 6, or 8,
 *                                                    or 0 to disable
 */

/* Prescaler common to all PLL inputs; will be 1 (XXX source is implicitly
 * as per comment above HSI) .
 */

#define STM32L4_PLLCFG_PLLM             RCC_PLLCFG_PLLM(1)

/* 'main' PLL config; we use this to generate our system clock via the R
 *  output.  We set it up as 16 MHz / 1 * 10 / 2 = 80 MHz
 *
 * XXX NOTE:
 * currently the main PLL is implicitly turned on and is implicitly
 * the system clock; this should be configurable since not all
 * applications may want things done this way.
 */

#define STM32L4_PLLCFG_PLLN             RCC_PLLCFG_PLLN(10)
#define STM32L4_PLLCFG_PLLP             0
#undef  STM32L4_PLLCFG_PLLP_ENABLED
#define STM32L4_PLLCFG_PLLQ             RCC_PLLCFG_PLLQ_2
#define STM32L4_PLLCFG_PLLQ_ENABLED
#define STM32L4_PLLCFG_PLLR             RCC_PLLCFG_PLLR(2)
#define STM32L4_PLLCFG_PLLR_ENABLED

/* 'SAIPLL1' is used to generate the 48 MHz clock, since we can't
 * do that with the main PLL's N value.  We set N = 12, and enable
 * the Q output (ultimately for CLK48) with /4.  So,
 * 16 MHz / 1 * 12 / 4 = 48 MHz
 *
 * XXX NOTE:  currently the SAIPLL /must/ be explicitly selected in the
 * menuconfig, or else all this is a moot point, and the various 48 MHz
 * peripherals will not work (RNG at present).  I would suggest removing
 * that option from Kconfig altogether, and simply making it an option
 * that is selected via a #define here, like all these other params.
 */

#define STM32L4_PLLSAI1CFG_PLLN         RCC_PLLSAI1CFG_PLLN(12)
#define STM32L4_PLLSAI1CFG_PLLP         0
#undef  STM32L4_PLLSAI1CFG_PLLP_ENABLED
#define STM32L4_PLLSAI1CFG_PLLQ         RCC_PLLSAI1CFG_PLLQ_4
#define STM32L4_PLLSAI1CFG_PLLQ_ENABLED
#define STM32L4_PLLSAI1CFG_PLLR         0
#undef  STM32L4_PLLSAI1CFG_PLLR_ENABLED

/* 'SAIPLL2' is not used in this application */

#define STM32L4_PLLSAI2CFG_PLLN         RCC_PLLSAI2CFG_PLLN(8)
#define STM32L4_PLLSAI2CFG_PLLP         0
#undef  STM32L4_PLLSAI2CFG_PLLP_ENABLED
#define STM32L4_PLLSAI2CFG_PLLR         0
#undef  STM32L4_PLLSAI2CFG_PLLR_ENABLED

#define STM32L4_SYSCLK_FREQUENCY  80000000ul

/* CLK48 will come from PLLSAI1 (implicitly Q) */

#define STM32L4_USE_CLK48
#define STM32L4_CLK48_SEL         RCC_CCIPR_CLK48SEL_PLLSAI1

/* enable the LSE oscillator, used automatically trim the MSI, and for RTC */

#define STM32L4_USE_LSE           1

/* AHB clock (HCLK) is SYSCLK (80MHz) */

#define STM32L4_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32L4_HCLK_FREQUENCY    STM32L4_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/1 (80MHz) */

#define STM32L4_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK       /* PCLK1 = HCLK / 1 */
#define STM32L4_PCLK1_FREQUENCY   (STM32L4_HCLK_FREQUENCY / 1)

/* Timers driven from APB1 will be twice PCLK1 */

/* REVISIT : this can be configured */

#define STM32L4_APB1_TIM2_CLKIN   (2 * STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM3_CLKIN   (2 * STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM4_CLKIN   (2 * STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM5_CLKIN   (2 * STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM6_CLKIN   (2 * STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM7_CLKIN   (2 * STM32L4_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK (80MHz) */

#define STM32L4_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK       /* PCLK2 = HCLK / 1 */
#define STM32L4_PCLK2_FREQUENCY   (STM32L4_HCLK_FREQUENCY / 1)

/* Timers driven from APB2 will be twice PCLK2 */

/* REVISIT : this can be configured */

#define STM32L4_APB2_TIM1_CLKIN   (2*STM32L4_PCLK2_FREQUENCY)
#define STM32L4_APB2_TIM8_CLKIN   (2*STM32L4_PCLK2_FREQUENCY)
#define STM32L4_APB2_TIM15_CLKIN  (2*STM32L4_PCLK2_FREQUENCY)
#define STM32L4_APB2_TIM16_CLKIN  (2*STM32L4_PCLK2_FREQUENCY)
#define STM32L4_APB2_TIM17_CLKIN  (2*STM32L4_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8,15,16,17 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32L4_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    (STM32L4_HCLK_FREQUENCY / 2)
#define BOARD_TIM3_FREQUENCY    (STM32L4_HCLK_FREQUENCY / 2)
#define BOARD_TIM4_FREQUENCY    (STM32L4_HCLK_FREQUENCY / 2)
#define BOARD_TIM5_FREQUENCY    (STM32L4_HCLK_FREQUENCY / 2)
#define BOARD_TIM6_FREQUENCY    (STM32L4_HCLK_FREQUENCY / 2)
#define BOARD_TIM7_FREQUENCY    (STM32L4_HCLK_FREQUENCY / 2)
#define BOARD_TIM8_FREQUENCY    STM32L4_HCLK_FREQUENCY
#define BOARD_TIM15_FREQUENCY   STM32L4_HCLK_FREQUENCY
#define BOARD_TIM16_FREQUENCY   STM32L4_HCLK_FREQUENCY
#define BOARD_TIM17_FREQUENCY   STM32L4_HCLK_FREQUENCY
#define STM32L4_LPTIM1_FREQUENCY  (STM32L4_HCLK_FREQUENCY / 2)
#define STM32L4_LPTIM2_FREQUENCY  (STM32L4_HCLK_FREQUENCY / 2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDs
 *
 * The STEVAL-STLCS01V1 board provides a single user LED, LD1.  LD1
 * is the red LED connected to MCU I/O PG12.
 *
 *   - When the I/O is HIGH value, the LED is on.
 *   - When the I/O is LOW, the LED is off.
 */

/* LED index values for use with board_userled() */

#define BOARD_LD1         0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_LD1_BIT     (1 << BOARD_LD1)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows when the red LED (PE24) is available:
 *
 *   SYMBOL                Meaning                   LD1
 *   -------------------  -----------------------  -----------
 *   LED_STARTED          NuttX has been started     OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF
 *   LED_STACKCREATED     Idle stack created         ON
 *   LED_INIRQ            In an interrupt            No change
 *   LED_SIGNAL           In a signal handler        No change
 *   LED_ASSERTION        An assertion failed        No change
 *   LED_PANIC            The system has crashed     Blinking
 *   LED_IDLE             MCU is is sleep mode       Not used
 *
 * Thus if LD1, NuttX has successfully booted and is, apparently, running
 * normally.  If LD1 is flashing at approximately 2Hz, then a fatal error
 * has been detected and the system has halted.
 */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED 1
#define LED_INIRQ        1
#define LED_SIGNAL       2
#define LED_ASSERTION    2
#define LED_PANIC        1

/* Alternate function pin selections ****************************************/

/* I2C3 - sensors */

#define GPIO_I2C3_SCL  GPIO_I2C3_SCL_1  /* PC0 */
#define GPIO_I2C3_SDA  GPIO_I2C3_SDA_1  /* PC1 */

/* SPI1 - BlueNRG */

#define GPIO_SPI1_SCK  GPIO_SPI1_SCK_1  /* PA5 */
#define GPIO_SPI1_MOSI GPIO_SPI1_MOSI_1 /* PA7 */
#define GPIO_SPI1_MISO GPIO_SPI1_MISO_1 /* PA6 */

/* SPI2 - senosrs */

#define GPIO_SPI2_SCK  GPIO_SPI2_SCK_2  /* PB13 */
#define GPIO_SPI2_MOSI GPIO_SPI2_MOSI_1 /* PB15 */
#define GPIO_SPI2_MISO 0                /* Not used in half-duplex */

#endif /* __BOARDS_ARM_STM32L4_STEVAL_STLCS01V1_INCLUDE_BOARD_H */
