/****************************************************************************
 * boards/arm/stm32l4/nucleo-l496zg/include/board.h
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

#ifndef __BOARDS_ARM_STM32L4_NUCLEO_L496ZG_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32L4_NUCLEO_L496ZG_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/* Do not include STM32 L4 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The Nucleo-144  board provides the following clock sources:
 *
 *   MCO: 8 MHz from MCO output of ST-LINK is used as input clock
 *   X2:  32.768 KHz crystal for LSE
 *   X3:  HSE crystal oscillator (not provided)
 *
 * So we have these clock source available within the STM32
 *
 *   HSI: 16 MHz RC factory-trimmed
 *   LSI: 32 KHz RC
 *   HSE: 8 MHz from MCO output of ST-LINK
 *   LSE: 32.768 kHz
 */

#define STM32L4_HSI_FREQUENCY     16000000ul
#define STM32L4_LSI_FREQUENCY     32000
#define STM32L4_HSE_FREQUENCY     8000000ul  /* 8 MHz from MCO output */
#define STM32L4_LSE_FREQUENCY     32768

#define MSI_CLOCK_CONFIG

#if defined(HSI_CLOCK_CONFIG)

#define STM32L4_BOARD_USEHSI

/* Prescaler common to all PLL inputs; will be 1 */

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
#define STM32L4_PLLCFG_PLLR             RCC_PLLCFG_PLLR_2
#define STM32L4_PLLCFG_PLLR_ENABLED

/* 'SAIPLL1' is used to generate the 48 MHz clock, since we can't
 * do that with the main PLL's N value.  We set N = 13, and enable
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

#if defined(CONFIG_STM32L4_OTGFS) || defined(STM32L4_SDMMC) || defined(CONFIG_STM32L4_RNG)
#  define STM32L4_USE_CLK48       1
#  define STM32L4_CLK48_SEL       RCC_CCIPR_CLK48SEL_PLLSAI1
#  define STM32L4_HSI48_SYNCSRC   SYNCSRC_NONE
#endif

/* Enable the LSE oscillator, used automatically trim the MSI, and for RTC */

#define STM32L4_USE_LSE           1

/* AHB clock (HCLK) is SYSCLK (80MHz) */

#define STM32L4_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32L4_HCLK_FREQUENCY    STM32L4_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/1 (80MHz) */

#define STM32L4_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK       /* PCLK1 = HCLK / 1 */
#define STM32L4_PCLK1_FREQUENCY   (STM32L4_HCLK_FREQUENCY/1)

/* Timers driven from APB1 will be twice PCLK1 */

/* REVISIT : this can be configured */

#define STM32L4_APB1_TIM2_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM3_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM4_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM5_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM6_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM7_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK (80MHz) */

#define STM32L4_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK       /* PCLK2 = HCLK / 1 */
#define STM32L4_PCLK2_FREQUENCY   (STM32L4_HCLK_FREQUENCY/1)

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

/* REVISIT : this can be configured */

#elif defined(HSE_CLOCK_CONFIG)

#define STM32L4_BOARD_USEHSE

/* Prescaler common to all PLL inputs; will be 1 */

#define STM32L4_PLLCFG_PLLM             RCC_PLLCFG_PLLM(1)

/* 'main' PLL config; we use this to generate our system clock via the R
 *  output.  We set it up as 8 MHz / 1 * 20 / 2 = 80 MHz
 *
 * XXX NOTE:
 * currently the main PLL is implicitly turned on and is implicitly
 * the system clock; this should be configurable since not all
 * applications may want things done this way.
 */

#define STM32L4_PLLCFG_PLLN             RCC_PLLCFG_PLLN(20)
#define STM32L4_PLLCFG_PLLP             0
#undef  STM32L4_PLLCFG_PLLP_ENABLED
#define STM32L4_PLLCFG_PLLQ             RCC_PLLCFG_PLLQ_2
#define STM32L4_PLLCFG_PLLQ_ENABLED
#define STM32L4_PLLCFG_PLLR             RCC_PLLCFG_PLLR_2
#define STM32L4_PLLCFG_PLLR_ENABLED

/* 'SAIPLL1' is used to generate the 48 MHz clock, since we can't
 * do that with the main PLL's N value.  We set N = 12, and enable
 * the Q output (ultimately for CLK48) with /4.  So,
 * 8 MHz / 1 * 12 / 2 = 48 MHz
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
#define STM32L4_PLLSAI1CFG_PLLQ         RCC_PLLSAI1CFG_PLLQ_2
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

#if defined(CONFIG_STM32L4_OTGFS) || defined(STM32L4_SDMMC) || defined(CONFIG_STM32L4_RNG)
#  define STM32L4_USE_CLK48       1
#  define STM32L4_CLK48_SEL       RCC_CCIPR_CLK48SEL_PLLSAI1
#  define STM32L4_HSI48_SYNCSRC   SYNCSRC_NONE
#endif

/* Enable the LSE oscillator, used automatically trim the MSI, and for RTC */

#define STM32L4_USE_LSE           1

/* AHB clock (HCLK) is SYSCLK (80MHz) */

#define STM32L4_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32L4_HCLK_FREQUENCY    STM32L4_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/1 (80MHz) */

#define STM32L4_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK       /* PCLK1 = HCLK / 1 */
#define STM32L4_PCLK1_FREQUENCY   (STM32L4_HCLK_FREQUENCY/1)

/* Timers driven from APB1 will be twice PCLK1 */

/* REVISIT : this can be configured */

#define STM32L4_APB1_TIM2_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM3_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM4_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM5_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM6_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM7_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK (80MHz) */

#define STM32L4_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK       /* PCLK2 = HCLK / 1 */
#define STM32L4_PCLK2_FREQUENCY   (STM32L4_HCLK_FREQUENCY/1)

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

/* REVISIT : this can be configured */

#elif defined(MSI_CLOCK_CONFIG)

#define STM32L4_BOARD_USEMSI
#define STM32L4_BOARD_MSIRANGE    RCC_CR_MSIRANGE_4M

/* Prescaler common to all PLL inputs; will be 1 */

#define STM32L4_PLLCFG_PLLM             RCC_PLLCFG_PLLM(1)

/* 'main' PLL config; we use this to generate our system clock via the R
 *  output.  We set it up as 4 MHz / 1 * 40 / 2 = 80 MHz
 *
 * XXX NOTE:
 * currently the main PLL is implicitly turned on and is implicitly
 * the system clock; this should be configurable since not all
 * applications may want things done this way.
 */

#define STM32L4_PLLCFG_PLLN             RCC_PLLCFG_PLLN(40)
#define STM32L4_PLLCFG_PLLP             0
#undef  STM32L4_PLLCFG_PLLP_ENABLED
#define STM32L4_PLLCFG_PLLQ             RCC_PLLCFG_PLLQ_2
#define STM32L4_PLLCFG_PLLQ_ENABLED
#define STM32L4_PLLCFG_PLLR             RCC_PLLCFG_PLLR_2
#define STM32L4_PLLCFG_PLLR_ENABLED

/* 'SAIPLL1' is used to generate the 48 MHz clock, since we can't
 * do that with the main PLL's N value.  We set N = 12, and enable
 * the Q output (ultimately for CLK48) with /4.  So,
 * 4 MHz / 1 * 24 / 2 = 48 MHz
 *
 * XXX NOTE:  currently the SAIPLL /must/ be explicitly selected in the
 * menuconfig, or else all this is a moot point, and the various 48 MHz
 * peripherals will not work (RNG at present).  I would suggest removing
 * that option from Kconfig altogether, and simply making it an option
 * that is selected via a #define here, like all these other params.
 */

#define STM32L4_PLLSAI1CFG_PLLN         RCC_PLLSAI1CFG_PLLN(24)
#define STM32L4_PLLSAI1CFG_PLLP         0
#undef  STM32L4_PLLSAI1CFG_PLLP_ENABLED
#define STM32L4_PLLSAI1CFG_PLLQ         RCC_PLLSAI1CFG_PLLQ_2
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

#if defined(CONFIG_STM32L4_OTGFS) || defined(STM32L4_SDMMC) || defined(CONFIG_STM32L4_RNG)
#  define STM32L4_USE_CLK48       1
#  define STM32L4_CLK48_SEL       RCC_CCIPR_CLK48SEL_PLLSAI1
#  define STM32L4_HSI48_SYNCSRC   SYNCSRC_NONE
#endif

/* Enable the LSE oscillator, used automatically trim the MSI, and for RTC */

#define STM32L4_USE_LSE           1

/* AHB clock (HCLK) is SYSCLK (80MHz) */

#define STM32L4_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32L4_HCLK_FREQUENCY    STM32L4_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/1 (80MHz) */

#define STM32L4_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK       /* PCLK1 = HCLK / 1 */
#define STM32L4_PCLK1_FREQUENCY   (STM32L4_HCLK_FREQUENCY/1)

/* Timers driven from APB1 will be twice PCLK1 */

/* REVISIT : this can be configured */

#define STM32L4_APB1_TIM2_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM3_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM4_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM5_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM6_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM7_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK (80MHz) */

#define STM32L4_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK       /* PCLK2 = HCLK / 1 */
#define STM32L4_PCLK2_FREQUENCY   (STM32L4_HCLK_FREQUENCY/1)

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

/* REVISIT : this can be configured */

#endif

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
#define BOARD_LPTIM1_FREQUENCY  (STM32L4_HCLK_FREQUENCY / 2)
#define BOARD_LPTIM2_FREQUENCY  (STM32L4_HCLK_FREQUENCY / 2)

/* SDMMC dividers.
 * Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(118+2)=400 KHz
 */

#define STM32_SDMMC_INIT_CLKDIV         (118 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(1+2)=16 MHz
 * DMA OFF: SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define STM32_SDMMC_MMCXFR_CLKDIV     (1 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_MMCXFR_CLKDIV     (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(1+2)=16 MHz
 * DMA OFF: SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define STM32_SDMMC_SDXFR_CLKDIV      (1 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_SDXFR_CLKDIV      (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

#if defined(CONFIG_STM32L4_SDMMC2)
#  define GPIO_SDMMC2_D0 GPIO_SDMMC2_D0_1
#  define GPIO_SDMMC2_D1 GPIO_SDMMC2_D1_1
#  define GPIO_SDMMC2_D2 GPIO_SDMMC2_D2_1
#  define GPIO_SDMMC2_D3 GPIO_SDMMC2_D3_1
#endif

/* DMA Channel/Stream Selections ********************************************/

/* Stream selections are arbitrary for now but might become important in the
 * future if we set aside more DMA channels/streams.
 *
 * SDMMC DMA is on DMA2
 *
 * SDMMC1 DMA
 *   DMAMAP_SDMMC_1 = Channel 4, Stream 7
 *   DMAMAP_SDMMC_2 = Channel 5, Stream 7
 *
 * SDMMC2 DMA
 *   DMAMAP_SDMMC2_1 = Channel 11, Stream 0
 *   DMAMAP_SDMMC3_2 = Channel 11, Stream 5
 */

#define DMAMAP_SDMMC1  DMACHAN_SDMMC_1
#define DMAMAP_SDMMC2  DMACHAN_SDMMC_2

/* FLASH wait states
 *
 *  --------- ---------- -----------
 *  VDD       MAX SYSCLK WAIT STATES
 *  --------- ---------- -----------
 *  1.7-2.1 V   180 MHz    8
 *  2.1-2.4 V   216 MHz    9
 *  2.4-2.7 V   216 MHz    8
 *  2.7-3.6 V   216 MHz    7
 *  --------- ---------- -----------
 */

#define BOARD_FLASH_WAITSTATES 7

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
 * include/board.h and src/stm32_autoleds.c. The LEDs are used to encode
 * OS-related events as follows:
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

/* The Nucleo-L496ZG supports one button:  Pushbutton B1, labeled "User", is
 * connected to GPIO PC13.
 * A high value will be sensed when the button is depressed.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1
#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

#define GPIO_USART2_TX     GPIO_USART2_TX_2
#define GPIO_USART2_RX     GPIO_USART2_RX_2
#define GPIO_USART3_TX     GPIO_USART3_TX_2
#define GPIO_USART3_RX     GPIO_USART3_RX_2

#if defined(CONFIG_NUCLEO_CONSOLE_ARDUINO)
/* USART6:
 *
 * These configurations assume that you are using a standard Arduino RS-232
 * shield with the serial interface with RX on pin D0 and TX on pin D1:
 *
 *   -------- ---------------
 *               STM32L4
 *   ARDUINO  FUNCTION  GPIO
 *   -- ----- --------- -----
 *   DO RX    USART6_RX PG9
 *   D1 TX    USART6_TX PG14
 *   -- ----- --------- -----
 */

 #  define GPIO_USART6_RX GPIO_USART6_RX_2
 #  define GPIO_USART6_TX GPIO_USART6_TX_2
#endif

/* USART3:
 * Use  USART3 and the USB virtual COM port
 */

/* LPUART1 is connector to Virtual COM port PG6 and PG7. */

#define GPIO_LPUART1_TX    GPIO_LPUART1_TX_3
#define GPIO_LPUART1_RX    GPIO_LPUART1_RX_3

/* DMA channels *************************************************************/

/* ADC */

#define ADC1_DMA_CHAN DMACHAN_ADC1_1
#define ADC2_DMA_CHAN DMACHAN_ADC2_2
#define ADC3_DMA_CHAN DMACHAN_ADC3_2

/* SPI
 *
 *
 *  PA6   SPI1_MISO CN12-13
 *  PA7   SPI1_MOSI CN12-15
 *  PA5   SPI1_SCK  CN12-11
 *
 *  PB14  SPI2_MISO CN12-28
 *  PB15  SPI2_MOSI CN12-26
 *  PB13  SPI2_SCK  CN12-30
 *
 *  PB4   SPI3_MISO CN12-27
 *  PB5   SPI3_MOSI CN12-29
 *  PB3   SPI3_SCK  CN12-31
 */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_1

#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1
#define GPIO_SPI2_SCK    GPIO_SPI2_SCK_3

#define GPIO_SPI3_MISO   GPIO_SPI3_MISO_1
#define GPIO_SPI3_MOSI   GPIO_SPI3_MOSI_2
#define GPIO_SPI3_SCK    GPIO_SPI3_SCK_1

/* I2C
 *
 *
 *  PB8   I2C1_SCL CN12-3
 *  PB9   I2C1_SDA CN12-5

 *  PB10   I2C2_SCL CN11-51
 *  PB11 I2C2_SDA CN12-18
 *
 *  PA8   I2C3_SCL CN12-23
 *  PC9   I2C3_SDA CN12-1
 *
 */

#define GPIO_I2C1_SCL GPIO_I2C1_SCL_2
#define GPIO_I2C1_SDA GPIO_I2C1_SDA_2

#define GPIO_I2C2_SCL GPIO_I2C2_SCL_1
#define GPIO_I2C2_SDA GPIO_I2C2_SDA_1

#define GPIO_I2C3_SCL GPIO_I2C3_SCL_1
#define GPIO_I2C3_SDA GPIO_I2C3_SDA_1

#define GPIO_I2C4_SCL GPIO_I2C4_SCL_1
#define GPIO_I2C4_SDA GPIO_I2C4_SDA_1

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
 * Name: stm32l4_board_initialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after all memory
 *   has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32l4_board_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32L4_NUCLEO_L496ZG_INCLUDE_BOARD_H */
