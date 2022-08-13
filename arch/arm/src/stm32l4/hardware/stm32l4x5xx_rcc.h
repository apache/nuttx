/****************************************************************************
 * arch/arm/src/stm32l4/hardware/stm32l4x5xx_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4X5XX_RCC_H
#define __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4X5XX_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_STM32L4_STM32L4X5)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32L4_RCC_CR_OFFSET         0x0000  /* Clock control register */
#define STM32L4_RCC_ICSCR_OFFSET      0x0004  /* Internal clock sources calibration register */
#define STM32L4_RCC_CFGR_OFFSET       0x0008  /* Clock configuration register */
#define STM32L4_RCC_PLLCFG_OFFSET     0x000c  /* PLL configuration register */
#define STM32L4_RCC_PLLSAI1CFG_OFFSET 0x0010  /* PLLSAI1 configuration register */
#define STM32L4_RCC_PLLSAI2CFG_OFFSET 0x0014  /* PLLSAI2 configuration register */
#define STM32L4_RCC_CIER_OFFSET       0x0018  /* Clock interrupt enable register */
#define STM32L4_RCC_CIFR_OFFSET       0x001c  /* Clock interrupt flag register */
#define STM32L4_RCC_CICR_OFFSET       0x0020  /* Clock interrupt clear register */
#define STM32L4_RCC_AHB1RSTR_OFFSET   0x0028  /* AHB1 peripheral reset register */
#define STM32L4_RCC_AHB2RSTR_OFFSET   0x002c  /* AHB2 peripheral reset register */
#define STM32L4_RCC_AHB3RSTR_OFFSET   0x0030  /* AHB3 peripheral reset register */
#define STM32L4_RCC_APB1RSTR1_OFFSET  0x0038  /* APB1 Peripheral reset register 1 */
#define STM32L4_RCC_APB1RSTR2_OFFSET  0x003c  /* APB1 Peripheral reset register 2 */
#define STM32L4_RCC_APB2RSTR_OFFSET   0x0040  /* APB2 Peripheral reset register */
#define STM32L4_RCC_AHB1ENR_OFFSET    0x0048  /* AHB1 Peripheral Clock enable register */
#define STM32L4_RCC_AHB2ENR_OFFSET    0x004c  /* AHB2 Peripheral Clock enable register */
#define STM32L4_RCC_AHB3ENR_OFFSET    0x0050  /* AHB3 Peripheral Clock enable register */
#define STM32L4_RCC_APB1ENR1_OFFSET   0x0058  /* APB1 Peripheral Clock enable register 1 */
#define STM32L4_RCC_APB1ENR2_OFFSET   0x005c  /* APB1 Peripheral Clock enable register 2 */
#define STM32L4_RCC_APB2ENR_OFFSET    0x0060  /* APB2 Peripheral Clock enable register */
#define STM32L4_RCC_AHB1SMENR_OFFSET  0x0068  /* RCC AHB1 low power mode peripheral clock enable register */
#define STM32L4_RCC_AHB2SMENR_OFFSET  0x006c  /* RCC AHB2 low power mode peripheral clock enable register */
#define STM32L4_RCC_AHB3SMENR_OFFSET  0x0070  /* RCC AHB3 low power mode peripheral clock enable register */
#define STM32L4_RCC_APB1SMENR1_OFFSET 0x0078  /* RCC APB1 low power mode peripheral clock enable register 1 */
#define STM32L4_RCC_APB1SMENR2_OFFSET 0x007c  /* RCC APB1 low power mode peripheral clock enable register 2 */
#define STM32L4_RCC_APB2SMENR_OFFSET  0x0080  /* RCC APB2 low power mode peripheral clock enable register */
#define STM32L4_RCC_CCIPR_OFFSET      0x0088  /* Peripherals independent clock configuration register 1 */
#define STM32L4_RCC_BDCR_OFFSET       0x0090  /* Backup domain control register */
#define STM32L4_RCC_CSR_OFFSET        0x0094  /* Control/status register */

/* Register Addresses *******************************************************/

#define STM32L4_RCC_CR                (STM32L4_RCC_BASE+STM32L4_RCC_CR_OFFSET)
#define STM32L4_RCC_ICSCR             (STM32L4_RCC_BASE+STM32L4_RCC_ICSCR_OFFSET)
#define STM32L4_RCC_CFGR              (STM32L4_RCC_BASE+STM32L4_RCC_CFGR_OFFSET)
#define STM32L4_RCC_PLLCFG            (STM32L4_RCC_BASE+STM32L4_RCC_PLLCFG_OFFSET)
#define STM32L4_RCC_PLLSAI1CFG        (STM32L4_RCC_BASE+STM32L4_RCC_PLLSAI1CFG_OFFSET)
#define STM32L4_RCC_PLLSAI2CFG        (STM32L4_RCC_BASE+STM32L4_RCC_PLLSAI2CFG_OFFSET)
#define STM32L4_RCC_CIER              (STM32L4_RCC_BASE+STM32L4_RCC_CIER_OFFSET)
#define STM32L4_RCC_CIFR              (STM32L4_RCC_BASE+STM32L4_RCC_CIFR_OFFSET)
#define STM32L4_RCC_CICR              (STM32L4_RCC_BASE+STM32L4_RCC_CICR_OFFSET)
#define STM32L4_RCC_AHB1RSTR          (STM32L4_RCC_BASE+STM32L4_RCC_AHB1RSTR_OFFSET)
#define STM32L4_RCC_AHB2RSTR          (STM32L4_RCC_BASE+STM32L4_RCC_AHB2RSTR_OFFSET)
#define STM32L4_RCC_AHB3RSTR          (STM32L4_RCC_BASE+STM32L4_RCC_AHB3RSTR_OFFSET)
#define STM32L4_RCC_APB1RSTR1         (STM32L4_RCC_BASE+STM32L4_RCC_APB1RSTR1_OFFSET)
#define STM32L4_RCC_APB1RSTR2         (STM32L4_RCC_BASE+STM32L4_RCC_APB1RSTR2_OFFSET)
#define STM32L4_RCC_APB2RSTR          (STM32L4_RCC_BASE+STM32L4_RCC_APB2RSTR_OFFSET)
#define STM32L4_RCC_AHB1ENR           (STM32L4_RCC_BASE+STM32L4_RCC_AHB1ENR_OFFSET)
#define STM32L4_RCC_AHB2ENR           (STM32L4_RCC_BASE+STM32L4_RCC_AHB2ENR_OFFSET)
#define STM32L4_RCC_AHB3ENR           (STM32L4_RCC_BASE+STM32L4_RCC_AHB3ENR_OFFSET)
#define STM32L4_RCC_APB1ENR1          (STM32L4_RCC_BASE+STM32L4_RCC_APB1ENR1_OFFSET)
#define STM32L4_RCC_APB1ENR2          (STM32L4_RCC_BASE+STM32L4_RCC_APB1ENR2_OFFSET)
#define STM32L4_RCC_APB2ENR           (STM32L4_RCC_BASE+STM32L4_RCC_APB2ENR_OFFSET)
#define STM32L4_RCC_AHB1SMENR         (STM32L4_RCC_BASE+STM32L4_RCC_AHB1SMENR_OFFSET)
#define STM32L4_RCC_AHB2SMENR         (STM32L4_RCC_BASE+STM32L4_RCC_AHB2SMENR_OFFSET)
#define STM32L4_RCC_AHB3SMENR         (STM32L4_RCC_BASE+STM32L4_RCC_AHB3SMENR_OFFSET)
#define STM32L4_RCC_APB1SMENR1        (STM32L4_RCC_BASE+STM32L4_RCC_APB1SMENR1_OFFSET)
#define STM32L4_RCC_APB1SMENR2        (STM32L4_RCC_BASE+STM32L4_RCC_APB1SMENR2_OFFSET)
#define STM32L4_RCC_APB2SMENR         (STM32L4_RCC_BASE+STM32L4_RCC_APB2SMENR_OFFSET)
#define STM32L4_RCC_CCIPR             (STM32L4_RCC_BASE+STM32L4_RCC_CCIPR_OFFSET)
#define STM32L4_RCC_BDCR              (STM32L4_RCC_BASE+STM32L4_RCC_BDCR_OFFSET)
#define STM32L4_RCC_CSR               (STM32L4_RCC_BASE+STM32L4_RCC_CSR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Clock control register */

#define RCC_CR_MSION                (1 << 0)  /* Bit 0: Internal Multi Speed clock enable */
#define RCC_CR_MSIRDY               (1 << 1)  /* Bit 1: Internal Multi Speed clock ready flag */
#define RCC_CR_MSIPLLEN             (1 << 2)  /* Bit 2: MSI clock PLL enable */
#define RCC_CR_MSIRGSEL             (1 << 3)  /* Bit 3: MSI clock range selection */
#define RCC_CR_MSIRANGE_SHIFT       (4)       /* Bits 7-4: MSI clock range */
#define RCC_CR_MSIRANGE_MASK        (0x0f << RCC_CR_MSIRANGE_SHIFT)
#  define RCC_CR_MSIRANGE_100K      (0  << RCC_CR_MSIRANGE_SHIFT) /* 0000: around 100 kHz */
#  define RCC_CR_MSIRANGE_200K      (1  << RCC_CR_MSIRANGE_SHIFT) /* 0001: around 200 kHz */
#  define RCC_CR_MSIRANGE_400K      (2  << RCC_CR_MSIRANGE_SHIFT) /* 0010: around 400 kHz */
#  define RCC_CR_MSIRANGE_800K      (3  << RCC_CR_MSIRANGE_SHIFT) /* 0011: around 800 kHz */
#  define RCC_CR_MSIRANGE_1M        (4  << RCC_CR_MSIRANGE_SHIFT) /* 0100: around 1 MHz */
#  define RCC_CR_MSIRANGE_2M        (5  << RCC_CR_MSIRANGE_SHIFT) /* 0101: around 2 MHz */
#  define RCC_CR_MSIRANGE_4M        (6  << RCC_CR_MSIRANGE_SHIFT) /* 0110: around 4 MHz */
#  define RCC_CR_MSIRANGE_8M        (7  << RCC_CR_MSIRANGE_SHIFT) /* 0111: around 8 MHz */
#  define RCC_CR_MSIRANGE_16M       (8  << RCC_CR_MSIRANGE_SHIFT) /* 1000: around 16 MHz */
#  define RCC_CR_MSIRANGE_24M       (9  << RCC_CR_MSIRANGE_SHIFT) /* 1001: around 24 MHz */
#  define RCC_CR_MSIRANGE_32M       (10 << RCC_CR_MSIRANGE_SHIFT) /* 1010: around 32 MHz */
#  define RCC_CR_MSIRANGE_48M       (11 << RCC_CR_MSIRANGE_SHIFT) /* 1011: around 48 MHz */

#define RCC_CR_HSION                (1 << 8)  /* Bit 8: Internal High Speed clock enable */
#define RCC_CR_HSIKERON             (1 << 9)  /* Bit 9: HSI16 always enable for peripheral kernels */
#define RCC_CR_HSIRDY               (1 << 10) /* Bit 10: Internal High Speed clock ready flag */
#define RCC_CR_HSIASFS              (1 << 11) /* Bit 11: HSI automatic start from stop */
#define RCC_CR_HSEON                (1 << 16) /* Bit 16: External High Speed clock enable */
#define RCC_CR_HSERDY               (1 << 17) /* Bit 17: External High Speed clock ready flag */
#define RCC_CR_HSEBYP               (1 << 18) /* Bit 18: External High Speed clock Bypass */
#define RCC_CR_CSSON                (1 << 19) /* Bit 19: Clock Security System enable */
#define RCC_CR_PLLON                (1 << 24) /* Bit 24: PLL enable */
#define RCC_CR_PLLRDY               (1 << 25) /* Bit 25: PLL clock ready flag */
#define RCC_CR_PLLSAI1ON            (1 << 26) /* Bit 26: PLLSAI1 enable */
#define RCC_CR_PLLSAI1RDY           (1 << 27) /* Bit 27: PLLSAI1 clock ready flag */
#define RCC_CR_PLLSAI2ON            (1 << 28) /* Bit 28: PLLSAI2 enable */
#define RCC_CR_PLLSAI2RDY           (1 << 29) /* Bit 29: PLLSAI2 clock ready flag */

/* Internal Clock Sources Calibration */

#define RCC_CR_MSICAL_SHIFT         (0)       /* Bits 7-0: Internal Multi Speed clock Calibration */
#define RCC_CR_MSICAL_MASK          (0xff << RCC_CR_MSICAL_SHIFT)
#define RCC_CR_MSITRIM_SHIFT        (8)       /* Bits 15-8: Internal Multi Speed clock trimming */
#define RCC_CR_MSITRIM_MASK         (0xff << RCC_CR_MSITRIM_SHIFT)
#define RCC_CR_HSICAL_SHIFT         (16)      /* Bits 23-16: Internal High Speed clock Calibration */
#define RCC_CR_HSICAL_MASK          (0xff << RCC_CR_HSICAL_SHIFT)
#define RCC_CR_HSITRIM_SHIFT        (24)       /* Bits 28-24: Internal High Speed clock trimming */
#define RCC_CR_HSITRIM_MASK         (0x1f << RCC_CR_HSITRIM_SHIFT)

/* Clock configuration register */

#define RCC_CFGR_SW_SHIFT           (0)       /* Bits 0-1: System clock Switch */
#define RCC_CFGR_SW_MASK            (3 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_MSI           (0 << RCC_CFGR_SW_SHIFT) /* 00: MSI selected as system clock */
#  define RCC_CFGR_SW_HSI           (1 << RCC_CFGR_SW_SHIFT) /* 01: HSI selected as system clock */
#  define RCC_CFGR_SW_HSE           (2 << RCC_CFGR_SW_SHIFT) /* 10: HSE selected as system clock */
#  define RCC_CFGR_SW_PLL           (3 << RCC_CFGR_SW_SHIFT) /* 11: PLL selected as system clock */

#define RCC_CFGR_SWS_SHIFT          (2)       /* Bits 2-3: System Clock Switch Status */
#define RCC_CFGR_SWS_MASK           (3 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_MSI          (0 << RCC_CFGR_SWS_SHIFT) /* 00: MSI oscillator used as system clock */
#  define RCC_CFGR_SWS_HSI          (1 << RCC_CFGR_SWS_SHIFT) /* 01: HSI oscillator used as system clock */
#  define RCC_CFGR_SWS_HSE          (2 << RCC_CFGR_SWS_SHIFT) /* 10: HSE oscillator used as system clock */
#  define RCC_CFGR_SWS_PLL          (3 << RCC_CFGR_SWS_SHIFT) /* 11: PLL used as system clock */

#define RCC_CFGR_HPRE_SHIFT         (4)       /* Bits 4-7: AHB prescaler */
#define RCC_CFGR_HPRE_MASK          (0x0f << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLK      (0 << RCC_CFGR_HPRE_SHIFT)  /* 0xxx: SYSCLK not divided */
#  define RCC_CFGR_HPRE_SYSCLKd2    (8 << RCC_CFGR_HPRE_SHIFT)  /* 1000: SYSCLK divided by 2 */
#  define RCC_CFGR_HPRE_SYSCLKd4    (9 << RCC_CFGR_HPRE_SHIFT)  /* 1001: SYSCLK divided by 4 */
#  define RCC_CFGR_HPRE_SYSCLKd8    (10 << RCC_CFGR_HPRE_SHIFT) /* 1010: SYSCLK divided by 8 */
#  define RCC_CFGR_HPRE_SYSCLKd16   (11 << RCC_CFGR_HPRE_SHIFT) /* 1011: SYSCLK divided by 16 */
#  define RCC_CFGR_HPRE_SYSCLKd64   (12 << RCC_CFGR_HPRE_SHIFT) /* 1100: SYSCLK divided by 64 */
#  define RCC_CFGR_HPRE_SYSCLKd128  (13 << RCC_CFGR_HPRE_SHIFT) /* 1101: SYSCLK divided by 128 */
#  define RCC_CFGR_HPRE_SYSCLKd256  (14 << RCC_CFGR_HPRE_SHIFT) /* 1110: SYSCLK divided by 256 */
#  define RCC_CFGR_HPRE_SYSCLKd512  (15 << RCC_CFGR_HPRE_SHIFT) /* 1111: SYSCLK divided by 512 */

#define RCC_CFGR_PPRE1_SHIFT        (8)       /* Bits 8-10: APB Low speed prescaler (APB1) */
#define RCC_CFGR_PPRE1_MASK         (7 << RCC_CFGR_PPRE1_SHIFT)
#  define RCC_CFGR_PPRE1_HCLK       (0 << RCC_CFGR_PPRE1_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE1_HCLKd2     (4 << RCC_CFGR_PPRE1_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE1_HCLKd4     (5 << RCC_CFGR_PPRE1_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE1_HCLKd8     (6 << RCC_CFGR_PPRE1_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE1_HCLKd16    (7 << RCC_CFGR_PPRE1_SHIFT) /* 111: HCLK divided by 16 */

#define RCC_CFGR_PPRE2_SHIFT        (11)      /* Bits 11-13: APB High speed prescaler (APB2) */
#define RCC_CFGR_PPRE2_MASK         (7 << RCC_CFGR_PPRE2_SHIFT)
#  define RCC_CFGR_PPRE2_HCLK       (0 << RCC_CFGR_PPRE2_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE2_HCLKd2     (4 << RCC_CFGR_PPRE2_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE2_HCLKd4     (5 << RCC_CFGR_PPRE2_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE2_HCLKd8     (6 << RCC_CFGR_PPRE2_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE2_HCLKd16    (7 << RCC_CFGR_PPRE2_SHIFT) /* 111: HCLK divided by 16 */

#define RCC_CFGR_STOPWUCK           (1 << 15) /* Bit 15: Wakeup from Stop and CSS backup clock selection */
#  define RCC_CFGR_STOPWUCK_MSI     (0 << 15) /* 0: MSI */
#  define RCC_CFGR_STOPWUCK_HSI     (1 << 15) /* 0: HSI */
#define RCC_CFGR_MCO_SHIFT          (24)      /* Bits 24-26: Microcontroller Clock Output */
#define RCC_CFGR_MCO_MASK           (7 << RCC_CFGR_MCO_SHIFT)
#  define RCC_CFGR_MCO_NONE         (0 << RCC_CFGR_MCO_SHIFT) /* 000: Disabled */
#  define RCC_CFGR_MCO_SYSCLK       (1 << RCC_CFGR_MCO_SHIFT) /* 001: SYSCLK system clock selected */
#  define RCC_CFGR_MCO_MSI          (2 << RCC_CFGR_MCO_SHIFT) /* 010: MSI clock selected */
#  define RCC_CFGR_MCO_HSI          (3 << RCC_CFGR_MCO_SHIFT) /* 011: HSI clock selected */
#  define RCC_CFGR_MCO_HSE          (4 << RCC_CFGR_MCO_SHIFT) /* 100: HSE clock selected */
#  define RCC_CFGR_MCO_PLL          (5 << RCC_CFGR_MCO_SHIFT) /* 101: Main PLL selected  */
#  define RCC_CFGR_MCO_LSI          (6 << RCC_CFGR_MCO_SHIFT) /* 110: LSI clock selected */
#  define RCC_CFGR_MCO_LSE          (7 << RCC_CFGR_MCO_SHIFT) /* 111: LSE clock selected */

#define RCC_CFGR_MCOPRE_SHIFT       (28)      /* Bits 28-30: MCO prescaler */
#define RCC_CFGR_MCOPRE_MASK        (7 << RCC_CFGR_MCOPRE_SHIFT)
#  define RCC_CFGR_MCOPRE_NONE      (0 << RCC_CFGR_MCOPRE_SHIFT) /* 000: no division */
#  define RCC_CFGR_MCOPRE_DIV2      (1 << RCC_CFGR_MCOPRE_SHIFT) /* 001: division by 2 */
#  define RCC_CFGR_MCOPRE_DIV4      (2 << RCC_CFGR_MCOPRE_SHIFT) /* 010: division by 4 */
#  define RCC_CFGR_MCOPRE_DIV8      (3 << RCC_CFGR_MCOPRE_SHIFT) /* 011: division by 8 */
#  define RCC_CFGR_MCOPRE_DIV16     (4 << RCC_CFGR_MCOPRE_SHIFT) /* 100: division by 16 */

/* PLL configuration register */

#define RCC_PLLCFG_PLLSRC_SHIFT     (0) /* Bit 0-1: Main PLL(PLL) and audio PLLs (PLLSAIx)
                                         *          entry clock source */
#define RCC_PLLCFG_PLLSRC_MASK      (3 << RCC_PLLCFG_PLLSRC_SHIFT)
#  define RCC_PLLCFG_PLLSRC_NONE    (0 << RCC_PLLCFG_PLLSRC_SHIFT) /* 000: No clock sent to PLLs */
#  define RCC_PLLCFG_PLLSRC_MSI     (1 << RCC_PLLCFG_PLLSRC_SHIFT) /* 001: MSI selected as PLL source */
#  define RCC_PLLCFG_PLLSRC_HSI     (2 << RCC_PLLCFG_PLLSRC_SHIFT) /* 010: HSI selected as PLL source */
#  define RCC_PLLCFG_PLLSRC_HSE     (3 << RCC_PLLCFG_PLLSRC_SHIFT) /* 011: HSE selected as PLL source */

#define RCC_PLLCFG_PLLM_SHIFT       (4)      /* Bits 4-6: Main PLL (PLL) input clock divider */
#define RCC_PLLCFG_PLLM_MASK        (0x07 << RCC_PLLCFG_PLLM_SHIFT)
#  define RCC_PLLCFG_PLLM(n)        ((n-1) << RCC_PLLCFG_PLLM_SHIFT) /* m = 1..8 */

#define RCC_PLLCFG_PLLN_SHIFT       (8)      /* Bits 8-14: Main PLL (PLL) VCO multiplier */
#define RCC_PLLCFG_PLLN_MASK        (0x7f << RCC_PLLCFG_PLLN_SHIFT)
#  define RCC_PLLCFG_PLLN(n)        ((n) << RCC_PLLCFG_PLLN_SHIFT) /* n = 8..86 */

#define RCC_PLLCFG_PLLPEN           (1 << 16) /* Bit 16: Main PLL PLLSAI3CLK output enable */
#define RCC_PLLCFG_PLLP             (1 << 17) /* Bit 17: Main PLL div factor for PLLSAI3CLK */

#  define RCC_PLLCFG_PLLP_7         0               /* 0: PLLP = 7 */
#  define RCC_PLLCFG_PLLP_17        RCC_PLLCFG_PLLP /* 1: PLLP = 17 */

#define RCC_PLLCFG_PLLQEN           (1 << 20) /* Bit 20: Main PLL PLL48M1CLK output enable */
#define RCC_PLLCFG_PLLQ_SHIFT       (21)
#define RCC_PLLCFG_PLLQ_MASK        (3 << RCC_PLLCFG_PLLQ_SHIFT)
#  define RCC_PLLCFG_PLLQ(n)        ((((n)>>1)-1)<< RCC_PLLCFG_PLLQ_SHIFT) /* n=2,4,6,8 */

#  define RCC_PLLCFG_PLLQ_2         (0 << RCC_PLLCFG_PLLQ_SHIFT) /* 00: PLLQ = 2 */
#  define RCC_PLLCFG_PLLQ_4         (1 << RCC_PLLCFG_PLLQ_SHIFT) /* 01: PLLQ = 4 */
#  define RCC_PLLCFG_PLLQ_6         (2 << RCC_PLLCFG_PLLQ_SHIFT) /* 10: PLLQ = 6 */
#  define RCC_PLLCFG_PLLQ_8         (3 << RCC_PLLCFG_PLLQ_SHIFT) /* 11: PLLQ = 8 */

#define RCC_PLLCFG_PLLREN           (1 << 24) /* Bit 24: Main PLL PLLCLK output enable */
#define RCC_PLLCFG_PLLR_SHIFT       (25)
#define RCC_PLLCFG_PLLR_MASK        (3 << RCC_PLLCFG_PLLR_SHIFT)
#  define RCC_PLLCFG_PLLR(n)        ((((n)>>1)-1)<< RCC_PLLCFG_PLLR_SHIFT) /* n=2,4,6,8 */

#  define RCC_PLLCFG_PLLR_2         (0 << RCC_PLLCFG_PLLR_SHIFT) /* 00: PLLR = 2 */
#  define RCC_PLLCFG_PLLR_4         (1 << RCC_PLLCFG_PLLR_SHIFT) /* 01: PLLR = 4 */
#  define RCC_PLLCFG_PLLR_6         (2 << RCC_PLLCFG_PLLR_SHIFT) /* 10: PLLR = 6 */
#  define RCC_PLLCFG_PLLR_8         (3 << RCC_PLLCFG_PLLR_SHIFT) /* 11: PLLR = 8 */

#define RCC_PLLCFG_RESET            (0x00001000) /* PLLCFG reset value */

/* PLLSAI1 Configuration register */

#define RCC_PLLSAI1CFG_PLLN_SHIFT   (8)      /* Bits 6-14: SAI1 PLL (PLLSAI1) VCO multiplier */
#define RCC_PLLSAI1CFG_PLLN_MASK    (0x7f << RCC_PLLSAI1CFG_PLLN_SHIFT)
#  define RCC_PLLSAI1CFG_PLLN(n)    ((n) << RCC_PLLSAI1CFG_PLLN_SHIFT) /* n = 8..86 */

#define RCC_PLLSAI1CFG_PLLPEN       (1 << 16) /* Bit 16: SAI1 PLL PLLSAI1CLK output enable */
#define RCC_PLLSAI1CFG_PLLP         (1 << 17) /* Bit 17: Main PLL div factor for PLLSAI1CLK */

#  define RCC_PLLSAI1CFG_PLLP_7     0                   /* 0: PLLP = 7 */
#  define RCC_PLLSAI1CFG_PLLP_17    RCC_PLLSAI1CFG_PLLP /* 1: PLLP = 17 */

#define RCC_PLLSAI1CFG_PLLQEN       (1 << 20) /* Bit 20: Main PLL PLL48M2CLK output enable */
#define RCC_PLLSAI1CFG_PLLQ_SHIFT   (21)
#define RCC_PLLSAI1CFG_PLLQ_MASK    (3 << RCC_PLLSAI1CFG_PLLQ_SHIFT)
#  define RCC_PLLSAI1CFG_PLLQ(n)    ((((n)>>1)-1)<< RCC_PLLSAI1CFG_PLLQ_SHIFT) /* n=2,4,6,8 */

#  define RCC_PLLSAI1CFG_PLLQ_2     (0 << RCC_PLLSAI1CFG_PLLQ_SHIFT) /* 00: PLLQ = 2 */
#  define RCC_PLLSAI1CFG_PLLQ_4     (1 << RCC_PLLSAI1CFG_PLLQ_SHIFT) /* 01: PLLQ = 4 */
#  define RCC_PLLSAI1CFG_PLLQ_6     (2 << RCC_PLLSAI1CFG_PLLQ_SHIFT) /* 10: PLLQ = 6 */
#  define RCC_PLLSAI1CFG_PLLQ_8     (3 << RCC_PLLSAI1CFG_PLLQ_SHIFT) /* 11: PLLQ = 8 */

#define RCC_PLLSAI1CFG_PLLREN       (1 << 24) /* Bit 24: SAI1 PLL PLLADC1CLK output enable */
#define RCC_PLLSAI1CFG_PLLR_SHIFT   (25)
#define RCC_PLLSAI1CFG_PLLR_MASK    (3 << RCC_PLLSAI1CFG_PLLR_SHIFT)
#  define RCC_PLLSAI1CFG_PLLR(n)    ((((n)>>1)-1)<< RCC_PLLSAI1CFG_PLLR_SHIFT) /* n=2,4,6,8 */

#  define RCC_PLLSAI1CFG_PLLR_2     (0 << RCC_PLLSAI1CFG_PLLR_SHIFT) /* 00: PLLR = 2 */
#  define RCC_PLLSAI1CFG_PLLR_4     (1 << RCC_PLLSAI1CFG_PLLR_SHIFT) /* 01: PLLR = 4 */
#  define RCC_PLLSAI1CFG_PLLR_6     (2 << RCC_PLLSAI1CFG_PLLR_SHIFT) /* 10: PLLR = 6 */
#  define RCC_PLLSAI1CFG_PLLR_8     (3 << RCC_PLLSAI1CFG_PLLR_SHIFT) /* 11: PLLR = 8 */

/* PLLSAI2 Configuration register */

#define RCC_PLLSAI2CFG_PLLN_SHIFT   (8)      /* Bits 6-14: SAI2 PLL (PLLSAI2) VCO multiplier */
#define RCC_PLLSAI2CFG_PLLN_MASK    (0x7f << RCC_PLLSAI2CFG_PLLN_SHIFT)
#  define RCC_PLLSAI2CFG_PLLN(n)    ((n) << RCC_PLLSAI2CFG_PLLN_SHIFT) /* n = 8..86 */

#define RCC_PLLSAI2CFG_PLLPEN       (1 << 16) /* Bit 16: SAI1 PLL PLLSAI2CLK output enable */
#define RCC_PLLSAI2CFG_PLLP         (1 << 17) /* Bit 17: Main PLL div factor for PLLSAI2CLK */

#  define RCC_PLLSAI2CFG_PLLP_7     0                   /* 0: PLLP = 7 */
#  define RCC_PLLSAI2CFG_PLLP_17    RCC_PLLSAI2CFG_PLLP /* 1: PLLP = 17 */

#define RCC_PLLSAI2CFG_PLLREN       (1 << 24) /* Bit 24: SAI2 PLL PLLADC2CLK output enable */
#define RCC_PLLSAI2CFG_PLLR_SHIFT   (25)
#define RCC_PLLSAI2CFG_PLLR_MASK    (3 << RCC_PLLSAI2CFG_PLLR_SHIFT)
#  define RCC_PLLSAI2CFG_PLLR(n)    ((((n)>>1)-1)<< RCC_PLLSAI2CFG_PLLR_SHIFT) /* n=2,4,6,8 */

#  define RCC_PLLSAI2CFG_PLLR_2     (0 << RCC_PLLSAI2CFG_PLLR_SHIFT) /* 00: PLLR = 2 */
#  define RCC_PLLSAI2CFG_PLLR_4     (1 << RCC_PLLSAI2CFG_PLLR_SHIFT) /* 01: PLLR = 4 */
#  define RCC_PLLSAI2CFG_PLLR_6     (2 << RCC_PLLSAI2CFG_PLLR_SHIFT) /* 10: PLLR = 6 */
#  define RCC_PLLSAI2CFG_PLLR_8     (3 << RCC_PLLSAI2CFG_PLLR_SHIFT) /* 11: PLLR = 8 */

/* Clock interrupt enable register */

#define RCC_CIR_LSIRDYIE            (1 << 0)  /* Bit 0: LSI Ready Interrupt Enable */
#define RCC_CIR_LSERDYIE            (1 << 1)  /* Bit 1: LSE Ready Interrupt Enable */
#define RCC_CIR_MSIRDYIE            (1 << 2)  /* Bit 2: MSI Ready Interrupt Enable */
#define RCC_CIR_HSIRDYIE            (1 << 3)  /* Bit 3: HSI Ready Interrupt Enable */
#define RCC_CIR_HSERDYIE            (1 << 4)  /* Bit 4: HSE Ready Interrupt Enable */
#define RCC_CIR_PLLRDYIE            (1 << 5)  /* Bit 5: PLL Ready Interrupt Enable */
#define RCC_CIR_PLLSAI1RDYIE        (1 << 6)  /* Bit 6: PLLSAI1 Ready Interrupt enable */
#define RCC_CIR_PLLSAI2RDYIE        (1 << 7)  /* Bit 7: PLLSAI2 Ready Interrupt enable */
#define RCC_CIR_LSECSSIE            (1 << 9)  /* Bit 9: LSE Clock Security System Interrupt Enable */

/* Clock interrupt flag register */

#define RCC_CIR_LSIRDYIF            (1 << 0)  /* Bit 0: LSI Ready Interrupt Flag */
#define RCC_CIR_LSERDYIF            (1 << 1)  /* Bit 1: LSE Ready Interrupt Flag */
#define RCC_CIR_MSIRDYIF            (1 << 2)  /* Bit 2: MSI Ready Interrupt Flag */
#define RCC_CIR_HSIRDYIF            (1 << 3)  /* Bit 3: HSI Ready Interrupt Flag */
#define RCC_CIR_HSERDYIF            (1 << 4)  /* Bit 4: HSE Ready Interrupt Flag */
#define RCC_CIR_PLLRDYIF            (1 << 5)  /* Bit 5: PLL Ready Interrupt Flag */
#define RCC_CIR_PLLSAI1RDYIF        (1 << 6)  /* Bit 6: PLLSAI1 Ready Interrupt Flag */
#define RCC_CIR_PLLSAI2RDYIF        (1 << 7)  /* Bit 7: PLLSAI2 Ready Interrupt Flag */
#define RCC_CIR_CSSF                (1 << 8)  /* Bit 8: Clock Security System Interrupt Flag */
#define RCC_CIR_LSECSSIF            (1 << 9)  /* Bit 9: LSE Clock Security System Interrupt Flag */
#define RCC_CIR_HSI48RDYIF          (1 << 10) /* Bit 10: HSI48 Ready Interrupt Flag */

/* Clock interrupt clear register */

#define RCC_CIR_LSIRDYIC            (1 << 0)  /* Bit 0: LSI Ready Interrupt Clear */
#define RCC_CIR_LSERDYIC            (1 << 1)  /* Bit 1: LSE Ready Interrupt Clear */
#define RCC_CIR_MSIRDYIC            (1 << 2)  /* Bit 2: MSI Ready Interrupt Clear */
#define RCC_CIR_HSIRDYIC            (1 << 3)  /* Bit 3: HSI Ready Interrupt Clear */
#define RCC_CIR_HSERDYIC            (1 << 4)  /* Bit 4: HSE Ready Interrupt Clear */
#define RCC_CIR_PLLRDYIC            (1 << 5)  /* Bit 5: PLL Ready Interrupt Clear */
#define RCC_CIR_PLLSAI1RDYIC        (1 << 6)  /* Bit 6: PLLSAI1 Ready Interrupt Clear */
#define RCC_CIR_PLLSAI2RDYIC        (1 << 7)  /* Bit 7: PLLSAI2 Ready Interrupt Clear */
#define RCC_CIR_CSSC                (1 << 8)  /* Bit 8: Clock Security System Interrupt Clear */
#define RCC_CIR_LSECSSIC            (1 << 9)  /* Bit 9: LSE Clock Security System Interrupt Clear */

/* AHB1 peripheral reset register */

#define RCC_AHB1RSTR_DMA1RST        (1 << 0)  /* Bit 0:  DMA1 reset */
#define RCC_AHB1RSTR_DMA2RST        (1 << 1)  /* Bit 1:  DMA2 reset */
#define RCC_AHB1RSTR_FLASHRST       (1 << 8)  /* Bit 8:  Flash memory interface reset */
#define RCC_AHB1RSTR_CRCRST         (1 << 12) /* Bit 12: CRC reset */
#define RCC_AHB1RSTR_TSCRST         (1 << 16) /* Bit 16: Touch Sensing Controller reset */

/* AHB2 peripheral reset register */

#define RCC_AHB2RSTR_GPIORST(n)     (1 << (n))
#define RCC_AHB2RSTR_GPIOARST       (1 << 0)  /* Bit 0:  IO port A reset */
#define RCC_AHB2RSTR_GPIOBRST       (1 << 1)  /* Bit 1:  IO port B reset */
#define RCC_AHB2RSTR_GPIOCRST       (1 << 2)  /* Bit 2:  IO port C reset */
#define RCC_AHB2RSTR_GPIODRST       (1 << 3)  /* Bit 3:  IO port D reset */
#define RCC_AHB2RSTR_GPIOERST       (1 << 4)  /* Bit 4:  IO port E reset */
#define RCC_AHB2RSTR_GPIOFRST       (1 << 5)  /* Bit 5:  IO port F reset */
#define RCC_AHB2RSTR_GPIOGRST       (1 << 6)  /* Bit 6:  IO port G reset */
#define RCC_AHB2RSTR_GPIOHRST       (1 << 7)  /* Bit 7:  IO port H reset */
#define RCC_AHB2RSTR_OTGFSRST       (1 << 12) /* Bit 12: USB OTG FS module reset */
#define RCC_AHB2RSTR_ADCRST         (1 << 13) /* Bit 13: ADC interface reset (common to all ADCs) */
#define RCC_AHB2RSTR_RNGRST         (1 << 18) /* Bit 18: Random number generator module reset */

/* AHB3 peripheral reset register */

#define RCC_AHB3RSTR_FSMCRST        (1 << 0)  /* Bit 0: Flexible static memory controller module reset */
#define RCC_AHB3RSTR_QSPIRST        (1 << 8)  /* Bit 8: Quad SPI module reset */

/* APB1 Peripheral reset register 1 */

#define RCC_APB1RSTR1_TIM2RST       (1 << 0)  /* Bit 0:  TIM2 reset */
#define RCC_APB1RSTR1_TIM3RST       (1 << 1)  /* Bit 1:  TIM3 reset */
#define RCC_APB1RSTR1_TIM4RST       (1 << 2)  /* Bit 2:  TIM4 reset */
#define RCC_APB1RSTR1_TIM5RST       (1 << 3)  /* Bit 3:  TIM5 reset */
#define RCC_APB1RSTR1_TIM6RST       (1 << 4)  /* Bit 4:  TIM6 reset */
#define RCC_APB1RSTR1_TIM7RST       (1 << 5)  /* Bit 5:  TIM7 reset */
#define RCC_APB1RSTR1_SPI2RST       (1 << 14) /* Bit 14: SPI2 reset */
#define RCC_APB1RSTR1_SPI3RST       (1 << 15) /* Bit 15: SPI3 reset */
#define RCC_APB1RSTR1_USART2RST     (1 << 17) /* Bit 17: USART2 reset */
#define RCC_APB1RSTR1_USART3RST     (1 << 18) /* Bit 18: USART3 reset */
#define RCC_APB1RSTR1_UART4RST      (1 << 19) /* Bit 19: USART4 reset */
#define RCC_APB1RSTR1_UART5RST      (1 << 20) /* Bit 20: USART5 reset */
#define RCC_APB1RSTR1_I2C1RST       (1 << 21) /* Bit 21: I2C1 reset */
#define RCC_APB1RSTR1_I2C2RST       (1 << 22) /* Bit 22: I2C2 reset */
#define RCC_APB1RSTR1_I2C3RST       (1 << 23) /* Bit 23: I2C3 reset */
#define RCC_APB1RSTR1_CAN1RST       (1 << 25) /* Bit 25: CAN1 reset */
#define RCC_APB1RSTR1_PWRRST        (1 << 28) /* Bit 28: Power interface reset */
#define RCC_APB1RSTR1_DAC1RST       (1 << 29) /* Bit 29: DAC1 reset */
#define RCC_APB1RSTR1_OPAMPRST      (1 << 30) /* Bit 30: OPAMP reset */
#define RCC_APB1RSTR1_LPTIM1RST     (1 << 31) /* Bit 31: Low-power Timer 1 reset */

/* APB1 Peripheral reset register 2 */

#define RCC_APB1RSTR2_LPUART1RST    (1 << 0)  /* Bit 0:  Low-power UART 1 reset */
#define RCC_APB1RSTR2_SWPMI1RST     (1 << 2)  /* Bit 2:  Single Wire Protocol reset */
#define RCC_APB1RSTR2_LPTIM2RST     (1 << 5)  /* Bit 5:  Low-power Timer 2 reset */

/* APB2 Peripheral reset register */

#define RCC_APB2RSTR_SYSCFGRST      (1 << 0)  /* Bit 0:  System configuration controller reset */
#define RCC_APB2RSTR_SDMMCRST       (1 << 10) /* Bit 10: SDMMC reset */
#define RCC_APB2RSTR_TIM1RST        (1 << 11) /* Bit 11: TIM1 reset */
#define RCC_APB2RSTR_SPI1RST        (1 << 12) /* Bit 12: SPI1 reset */
#define RCC_APB2RSTR_TIM8RST        (1 << 13) /* Bit 13: TIM8 reset */
#define RCC_APB2RSTR_USART1RST      (1 << 14) /* Bit 14: USART1 reset */
#define RCC_APB2RSTR_TIM15RST       (1 << 16) /* Bit 16: TIM15 reset */
#define RCC_APB2RSTR_TIM16RST       (1 << 17) /* Bit 17: TIM16 reset */
#define RCC_APB2RSTR_TIM17RST       (1 << 18) /* Bit 18: TIM17 reset */
#define RCC_APB2RSTR_SAI1RST        (1 << 21) /* Bit 21: SAI1 reset */
#define RCC_APB2RSTR_SAI2RST        (1 << 22) /* Bit 22: SAI2 reset */
#define RCC_APB2RSTR_DFSDMRST       (1 << 24) /* Bit 24: DFSDM reset */

/* AHB1 Peripheral Clock enable register */

#define RCC_AHB1ENR_DMA1EN          (1 << 0)  /* Bit 0:  DMA1 enable */
#define RCC_AHB1ENR_DMA2EN          (1 << 1)  /* Bit 1:  DMA2 enable */
#define RCC_AHB1ENR_FLASHEN         (1 << 8)  /* Bit 8:  Flash memory interface enable */
#define RCC_AHB1ENR_CRCEN           (1 << 12) /* Bit 12: CRC enable */
#define RCC_AHB1ENR_TSCEN           (1 << 16) /* Bit 16: Touch Sensing Controller enable */

/* AHB2 Peripheral Clock enable register */

#define RCC_AHB2ENR_GPIOEN(n)       (1 << (n))
#define RCC_AHB2ENR_GPIOAEN         (1 << 0)  /* Bit 0:  IO port A enable */
#define RCC_AHB2ENR_GPIOBEN         (1 << 1)  /* Bit 1:  IO port B enable */
#define RCC_AHB2ENR_GPIOCEN         (1 << 2)  /* Bit 2:  IO port C enable */
#define RCC_AHB2ENR_GPIODEN         (1 << 3)  /* Bit 3:  IO port D enable */
#define RCC_AHB2ENR_GPIOEEN         (1 << 4)  /* Bit 4:  IO port E enable */
#define RCC_AHB2ENR_GPIOFEN         (1 << 5)  /* Bit 5:  IO port F enable */
#define RCC_AHB2ENR_GPIOGEN         (1 << 6)  /* Bit 6:  IO port G enable */
#define RCC_AHB2ENR_GPIOHEN         (1 << 7)  /* Bit 7:  IO port H enable */
#define RCC_AHB2ENR_OTGFSEN         (1 << 12) /* Bit 12: USB OTG FS module enable */
#define RCC_AHB2ENR_ADCEN           (1 << 13) /* Bit 13: ADC interface enable (common to all ADCs) */
#define RCC_AHB2ENR_RNGEN           (1 << 18) /* Bit 18: Random number generator module enable */

/* AHB3 Peripheral Clock enable register */

#define RCC_AHB3ENR_FSMCEN          (1 << 0)  /* Bit 0: Flexible static memory controller module enable */
#define RCC_AHB3ENR_QSPIEN          (1 << 8)  /* Bit 8: Quad SPI module enable */

/* APB1 Peripheral Clock enable register 1 */

#define RCC_APB1ENR1_TIM2EN         (1 << 0)  /* Bit 0:  TIM2 enable */
#define RCC_APB1ENR1_TIM3EN         (1 << 1)  /* Bit 1:  TIM3 enable */
#define RCC_APB1ENR1_TIM4EN         (1 << 2)  /* Bit 2:  TIM4 enable */
#define RCC_APB1ENR1_TIM5EN         (1 << 3)  /* Bit 3:  TIM5 enable */
#define RCC_APB1ENR1_TIM6EN         (1 << 4)  /* Bit 4:  TIM6 enable */
#define RCC_APB1ENR1_TIM7EN         (1 << 5)  /* Bit 5:  TIM7 enable */
#define RCC_APB1ENR1_WWDGEN         (1 << 11) /* Bit 11: Windowed Watchdog enable */
#define RCC_APB1ENR1_SPI2EN         (1 << 14) /* Bit 14: SPI2 enable */
#define RCC_APB1ENR1_SPI3EN         (1 << 15) /* Bit 15: SPI3 enable */
#define RCC_APB1ENR1_USART2EN       (1 << 17) /* Bit 17: USART2 enable */
#define RCC_APB1ENR1_USART3EN       (1 << 18) /* Bit 18: USART3 enable */
#define RCC_APB1ENR1_UART4EN        (1 << 19) /* Bit 19: USART4 enable */
#define RCC_APB1ENR1_UART5EN        (1 << 20) /* Bit 20: USART5 enable */
#define RCC_APB1ENR1_I2C1EN         (1 << 21) /* Bit 21: I2C1 enable */
#define RCC_APB1ENR1_I2C2EN         (1 << 22) /* Bit 22: I2C2 enable */
#define RCC_APB1ENR1_I2C3EN         (1 << 23) /* Bit 23: I2C3 enable */
#define RCC_APB1ENR1_CAN1EN         (1 << 25) /* Bit 25: CAN1 enable */
#define RCC_APB1ENR1_PWREN          (1 << 28) /* Bit 28: Power interface enable */
#define RCC_APB1ENR1_DAC1EN         (1 << 29) /* Bit 29: DAC1 enable */
#define RCC_APB1ENR1_OPAMPEN        (1 << 30) /* Bit 30: OPAMP enable */
#define RCC_APB1ENR1_LPTIM1EN       (1 << 31) /* Bit 31: Low-power Timer 1 enable */

/* APB1 Peripheral Clock enable register 2 */

#define RCC_APB1ENR2_LPUART1EN      (1 << 0)  /* Bit 0:  Low-power UART 1 enable */
#define RCC_APB1ENR2_SWPMI1EN       (1 << 2)  /* Bit 2:  Single Wire Protocol enable */
#define RCC_APB1ENR2_LPTIM2EN       (1 << 5)  /* Bit 5:  Low-power Timer 2 enable */

/* APB2 Peripheral Clock enable register */

#define RCC_APB2ENR_SYSCFGEN        (1 << 0)  /* Bit 0:  System configuration controller enable */
#define RCC_APB2ENR_FWEN            (1 << 7)  /* Bit 7:  Firewall enable */
#define RCC_APB2ENR_SDMMCEN         (1 << 10) /* Bit 10: SDMMC enable */
#define RCC_APB2ENR_TIM1EN          (1 << 11) /* Bit 11: TIM1 enable */
#define RCC_APB2ENR_SPI1EN          (1 << 12) /* Bit 12: SPI1 enable */
#define RCC_APB2ENR_TIM8EN          (1 << 13) /* Bit 13: TIM8 enable */
#define RCC_APB2ENR_USART1EN        (1 << 14) /* Bit 14: USART1 enable */
#define RCC_APB2ENR_TIM15EN         (1 << 16) /* Bit 16: TIM15 enable */
#define RCC_APB2ENR_TIM16EN         (1 << 17) /* Bit 17: TIM16 enable */
#define RCC_APB2ENR_TIM17EN         (1 << 18) /* Bit 18: TIM17 enable */
#define RCC_APB2ENR_SAI1EN          (1 << 21) /* Bit 21: SAI1 enable */
#define RCC_APB2ENR_SAI2EN          (1 << 22) /* Bit 22: SAI2 enable */
#define RCC_APB2ENR_DFSDMEN         (1 << 24) /* Bit 24: DFSDM enable */

/* RCC AHB1 low power mode peripheral clock enable register */

#define RCC_AHB1SMENR_DMA1LPSMEN    (1 << 0)  /* Bit 0:  DMA1 enable during Sleep mode */
#define RCC_AHB1SMENR_DMA2LPSMEN    (1 << 1)  /* Bit 1:  DMA2 enable during Sleep mode */
#define RCC_AHB1SMENR_FLASHLPSMEN   (1 << 8)  /* Bit 8:  Flash memory interface enable during Sleep mode */
#define RCC_AHB1SMENR_SRAM1SMEN     (1 << 9)  /* Bit 9:  SRAM1 enable during Sleep mode */
#define RCC_AHB1SMENR_CRCLPSMEN     (1 << 12) /* Bit 12: CRC enable during Sleep mode */
#define RCC_AHB1SMENR_TSCLPSMEN     (1 << 16) /* Bit 16: Touch Sensing Controller enable during Sleep mode */

/* RCC AHB2 low power mode peripheral clock enable register */

#define RCC_AHB2SMENR_GPIOASMEN     (1 << 0)  /* Bit 0:  IO port A enable during Sleep mode */
#define RCC_AHB2SMENR_GPIOBSMEN     (1 << 1)  /* Bit 1:  IO port B enable during Sleep mode */
#define RCC_AHB2SMENR_GPIOCSMEN     (1 << 2)  /* Bit 2:  IO port C enable during Sleep mode */
#define RCC_AHB2SMENR_GPIODSMEN     (1 << 3)  /* Bit 3:  IO port D enable during Sleep mode */
#define RCC_AHB2SMENR_GPIOESMEN     (1 << 4)  /* Bit 4:  IO port E enable during Sleep mode */
#define RCC_AHB2SMENR_GPIOFSMEN     (1 << 5)  /* Bit 5:  IO port F enable during Sleep mode */
#define RCC_AHB2SMENR_GPIOGSMEN     (1 << 6)  /* Bit 6:  IO port G enable during Sleep mode */
#define RCC_AHB2SMENR_GPIOHSMEN     (1 << 7)  /* Bit 7:  IO port H enable during Sleep mode */
#define RCC_AHB2SMENR_SRAM2SMEN     (1 << 9)  /* Bit 9:  SRAM2 enable during Sleep mode */
#define RCC_AHB2SMENR_OTGFSSMEN     (1 << 12) /* Bit 12: USB OTG FS module enable during Sleep mode */
#define RCC_AHB2SMENR_ADCSMEN       (1 << 13) /* Bit 13: ADC interface enable during Sleep mode (common to all ADCs) */
#define RCC_AHB2SMENR_RNGSMEN       (1 << 18) /* Bit 18: Random number generator module enable during Sleep mode */

/* RCC AHB3 low power mode peripheral clock enable register */

#define RCC_AHB3SMENR_FSMCSMEN      (1 << 0)  /* Bit 0: Flexible static memory controller module enable during Sleep mode */
#define RCC_AHB3SMENR_QSPISMEN      (1 << 8)  /* Bit 8: Quad SPI module enable during Sleep mode */

/* RCC APB1 low power mode peripheral clock enable register 1 */

#define RCC_APB1SMENR1_TIM2SMEN     (1 << 0)  /* Bit 0:  TIM2 enable during Sleep mode */
#define RCC_APB1SMENR1_TIM3SMEN     (1 << 1)  /* Bit 1:  TIM3 enable during Sleep mode */
#define RCC_APB1SMENR1_TIM4SMEN     (1 << 2)  /* Bit 2:  TIM4 enable during Sleep mode */
#define RCC_APB1SMENR1_TIM5SMEN     (1 << 3)  /* Bit 3:  TIM5 enable during Sleep mode */
#define RCC_APB1SMENR1_TIM6SMEN     (1 << 4)  /* Bit 4:  TIM6 enable during Sleep mode */
#define RCC_APB1SMENR1_TIM7SMEN     (1 << 5)  /* Bit 5:  TIM7 enable during Sleep mode */
#define RCC_APB1SMENR1_WWDGSMEN     (1 << 11) /* Bit 11: Windowed Watchdog enable during Sleep mode */
#define RCC_APB1SMENR1_SPI2SMEN     (1 << 14) /* Bit 14: SPI2 enable during Sleep mode */
#define RCC_APB1SMENR1_SPI3SMEN     (1 << 15) /* Bit 15: SPI3 enable during Sleep mode */
#define RCC_APB1SMENR1_USART2SMEN   (1 << 17) /* Bit 17: USART2 enable during Sleep mode */
#define RCC_APB1SMENR1_USART3SMEN   (1 << 18) /* Bit 18: USART3 enable during Sleep mode */
#define RCC_APB1SMENR1_UART4SMEN    (1 << 19) /* Bit 19: USART4 enable during Sleep mode */
#define RCC_APB1SMENR1_UART5SMEN    (1 << 20) /* Bit 20: USART5 enable during Sleep mode */
#define RCC_APB1SMENR1_I2C1SMEN     (1 << 21) /* Bit 21: I2C1 enable during Sleep mode */
#define RCC_APB1SMENR1_I2C2SMEN     (1 << 22) /* Bit 22: I2C2 enable during Sleep mode */
#define RCC_APB1SMENR1_I2C3SMEN     (1 << 23) /* Bit 23: I2C3 enable during Sleep mode */
#define RCC_APB1SMENR1_CAN1SMEN     (1 << 25) /* Bit 25: CAN1 enable during Sleep mode */
#define RCC_APB1SMENR1_PWRSMEN      (1 << 28) /* Bit 28: Power interface enable during Sleep mode */
#define RCC_APB1SMENR1_DAC1SMEN     (1 << 29) /* Bit 29: DAC1 enable during Sleep mode */
#define RCC_APB1SMENR1_OPAMPSMEN    (1 << 30) /* Bit 30: OPAMP enable during Sleep mode */
#define RCC_APB1SMENR1_LPTIM1SMEN   (1 << 31) /* Bit 31: Low-power Timer 1 enable during Sleep mode */

/* RCC APB1 low power modeperipheral clock enable register 2 */

#define RCC_APB1SMENR2_LPUART1SMEN  (1 << 0)  /* Bit 0:  Low-power UART 1 enable during Sleep mode */
#define RCC_APB1SMENR2_SWPMI1SMEN   (1 << 2)  /* Bit 2:  Single Wire Protocol enable during Sleep mode */
#define RCC_APB1SMENR2_LPTIM2SMEN   (1 << 5)  /* Bit 5:  Low-power Timer 2 enable during Sleep mode */

/* RCC APB2 low power mode peripheral clock enable register */

#define RCC_APB2SMENR_SYSCFGSMEN    (1 << 0)  /* Bit 0:  System configuration controller enable during Sleep mode */
#define RCC_APB2SMENR_SDMMCSMEN     (1 << 10) /* Bit 10: SDMMC enable during Sleep mode */
#define RCC_APB2SMENR_TIM1SMEN      (1 << 11) /* Bit 11: TIM1 enable during Sleep mode */
#define RCC_APB2SMENR_SPI1SMEN      (1 << 12) /* Bit 12: SPI1 enable during Sleep mode */
#define RCC_APB2SMENR_TIM8SMEN      (1 << 13) /* Bit 13: TIM8 enable during Sleep mode */
#define RCC_APB2SMENR_USART1SMEN    (1 << 14) /* Bit 14: USART1 enable during Sleep mode */
#define RCC_APB2SMENR_TIM15SMEN     (1 << 16) /* Bit 16: TIM15 enable during Sleep mode */
#define RCC_APB2SMENR_TIM16SMEN     (1 << 17) /* Bit 17: TIM16 enable during Sleep mode */
#define RCC_APB2SMENR_TIM17SMEN     (1 << 18) /* Bit 18: TIM17 enable during Sleep mode */
#define RCC_APB2SMENR_SAI1SMEN      (1 << 21) /* Bit 21: SAI1 enable during Sleep mode */
#define RCC_APB2SMENR_SAI2SMEN      (1 << 22) /* Bit 22: SAI2 enable during Sleep mode */
#define RCC_APB2SMENR_DFSDMSMEN     (1 << 24) /* Bit 24: DFSDM enable during Sleep mode */

/* Peripheral Independent Clock Configuration register */

#define RCC_CCIPR_USART1SEL_SHIFT     (0)
#define RCC_CCIPR_USART1SEL_MASK      (3 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_PCLK    (0 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_SYSCLK  (1 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_HSI     (2 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_LSE     (3 << RCC_CCIPR_USART1SEL_SHIFT)
#define RCC_CCIPR_USART2SEL_SHIFT     (2)
#define RCC_CCIPR_USART2SEL_MASK      (3 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_PCLK    (0 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_SYSCLK  (1 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_HSI     (2 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_LSE     (3 << RCC_CCIPR_USART2SEL_SHIFT)
#define RCC_CCIPR_USART3SEL_SHIFT     (4)
#define RCC_CCIPR_USART3SEL_MASK      (3 << RCC_CCIPR_USART3SEL_SHIFT)
#  define RCC_CCIPR_USART3SEL_PCLK    (0 << RCC_CCIPR_USART3SEL_SHIFT)
#  define RCC_CCIPR_USART3SEL_SYSCLK  (1 << RCC_CCIPR_USART3SEL_SHIFT)
#  define RCC_CCIPR_USART3SEL_HSI     (2 << RCC_CCIPR_USART3SEL_SHIFT)
#  define RCC_CCIPR_USART3SEL_LSE     (3 << RCC_CCIPR_USART3SEL_SHIFT)
#define RCC_CCIPR_UART4SEL_SHIFT      (6)
#define RCC_CCIPR_UART4SEL_MASK       (3 << RCC_CCIPR_UART4SEL_SHIFT)
#  define RCC_CCIPR_UART4SEL_PCLK     (0 << RCC_CCIPR_UART4SEL_SHIFT)
#  define RCC_CCIPR_UART4SEL_SYSCLK   (1 << RCC_CCIPR_UART4SEL_SHIFT)
#  define RCC_CCIPR_UART4SEL_HSI      (2 << RCC_CCIPR_UART4SEL_SHIFT)
#  define RCC_CCIPR_UART4SEL_LSE      (3 << RCC_CCIPR_UART4SEL_SHIFT)
#define RCC_CCIPR_UART5SEL_SHIFT      (8)
#define RCC_CCIPR_UART5SEL_MASK       (3 << RCC_CCIPR_UART5SEL_SHIFT)
#  define RCC_CCIPR_UART5SEL_PCLK     (0 << RCC_CCIPR_UART5SEL_SHIFT)
#  define RCC_CCIPR_UART5SEL_SYSCLK   (1 << RCC_CCIPR_UART5SEL_SHIFT)
#  define RCC_CCIPR_UART5SEL_HSI      (2 << RCC_CCIPR_UART5SEL_SHIFT)
#  define RCC_CCIPR_UART5SEL_LSE      (3 << RCC_CCIPR_UART5SEL_SHIFT)
#define RCC_CCIPR_LPUART1SEL_SHIFT    (10)
#define RCC_CCIPR_LPUART1SEL_MASK     (3 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_PCLK   (0 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_SYSCLK (1 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_HSI    (2 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_LSE    (3 << RCC_CCIPR_LPUART1SEL_SHIFT)
#define RCC_CCIPR_I2C1SEL_SHIFT       (12)
#define RCC_CCIPR_I2C1SEL_MASK        (3 << RCC_CCIPR_I2C1SEL_SHIFT)
#  define RCC_CCIPR_I2C1SEL_PCLK      (0 << RCC_CCIPR_I2C1SEL_SHIFT)
#  define RCC_CCIPR_I2C1SEL_SYSCLK    (1 << RCC_CCIPR_I2C1SEL_SHIFT)
#  define RCC_CCIPR_I2C1SEL_HSI       (2 << RCC_CCIPR_I2C1SEL_SHIFT)
#define RCC_CCIPR_I2C2SEL_SHIFT       (14)
#define RCC_CCIPR_I2C2SEL_MASK        (3 << RCC_CCIPR_I2C2SEL_SHIFT)
#  define RCC_CCIPR_I2C2SEL_PCLK      (0 << RCC_CCIPR_I2C2SEL_SHIFT)
#  define RCC_CCIPR_I2C2SEL_SYSCLK    (1 << RCC_CCIPR_I2C2SEL_SHIFT)
#  define RCC_CCIPR_I2C2SEL_HSI       (2 << RCC_CCIPR_I2C2SEL_SHIFT)
#define RCC_CCIPR_I2C3SEL_SHIFT       (16)
#define RCC_CCIPR_I2C3SEL_MASK        (3 << RCC_CCIPR_I2C3SEL_SHIFT)
#  define RCC_CCIPR_I2C3SEL_PCLK      (0 << RCC_CCIPR_I2C3SEL_SHIFT)
#  define RCC_CCIPR_I2C3SEL_SYSCLK    (1 << RCC_CCIPR_I2C3SEL_SHIFT)
#  define RCC_CCIPR_I2C3SEL_HSI       (2 << RCC_CCIPR_I2C3SEL_SHIFT)
#define RCC_CCIPR_LPTIM1SEL_SHIFT     (18)
#define RCC_CCIPR_LPTIM1SEL_MASK      (3 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_PCLK    (0 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_LSI     (1 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_HSI     (2 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_LSE     (3 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#define RCC_CCIPR_LPTIM2SEL_SHIFT     (20)
#define RCC_CCIPR_LPTIM2SEL_MASK      (3 << RCC_CCIPR_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR_LPTIM2SEL_PCLK    (0 << RCC_CCIPR_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR_LPTIM2SEL_LSI     (1 << RCC_CCIPR_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR_LPTIM2SEL_HSI     (2 << RCC_CCIPR_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR_LPTIM2SEL_LSE     (3 << RCC_CCIPR_LPTIM2SEL_SHIFT)
#define RCC_CCIPR_SAI1SEL_SHIFT       (22)
#define RCC_CCIPR_SAI1SEL_MASK        (3 << RCC_CCIPR_SAI1SEL_SHIFT)
#  define RCC_CCIPR_SAI1SEL_PLLSAI1   (0 << RCC_CCIPR_SAI1SEL_SHIFT)
#  define RCC_CCIPR_SAI1SEL_PLLSAI2   (1 << RCC_CCIPR_SAI1SEL_SHIFT)
#  define RCC_CCIPR_SAI1SEL_PLLMAIN   (2 << RCC_CCIPR_SAI1SEL_SHIFT)
#  define RCC_CCIPR_SAI1SEL_EXTCLK    (3 << RCC_CCIPR_SAI1SEL_SHIFT)
#define RCC_CCIPR_SAI2SEL_SHIFT       (24)
#define RCC_CCIPR_SAI2SEL_MASK        (3 << RCC_CCIPR_SAI2SEL_SHIFT)
#  define RCC_CCIPR_SAI2SEL_PLLSAI1   (0 << RCC_CCIPR_SAI2SEL_SHIFT)
#  define RCC_CCIPR_SAI2SEL_PLLSAI2   (1 << RCC_CCIPR_SAI2SEL_SHIFT)
#  define RCC_CCIPR_SAI2SEL_PLLMAIN   (2 << RCC_CCIPR_SAI2SEL_SHIFT)
#  define RCC_CCIPR_SAI2SEL_EXTCLK    (3 << RCC_CCIPR_SAI2SEL_SHIFT)
#define RCC_CCIPR_CLK48SEL_SHIFT      (26)
#define RCC_CCIPR_CLK48SEL_MASK       (3 << RCC_CCIPR_CLK48SEL_SHIFT)
#  define RCC_CCIPR_CLK48SEL_NONE     (0 << RCC_CCIPR_CLK48SEL_SHIFT)
#  define RCC_CCIPR_CLK48SEL_PLLSAI1  (1 << RCC_CCIPR_CLK48SEL_SHIFT)
#  define RCC_CCIPR_CLK48SEL_PLLMAIN  (2 << RCC_CCIPR_CLK48SEL_SHIFT)
#  define RCC_CCIPR_CLK48SEL_MSI      (3 << RCC_CCIPR_CLK48SEL_SHIFT)
#define RCC_CCIPR_ADCSEL_SHIFT        (28)
#define RCC_CCIPR_ADCSEL_MASK         (3 << RCC_CCIPR_ADCSEL_SHIFT)
#  define RCC_CCIPR_ADCSEL_NONE       (0 << RCC_CCIPR_ADCSEL_SHIFT)
#  define RCC_CCIPR_ADCSEL_PLLSAI1    (1 << RCC_CCIPR_ADCSEL_SHIFT)
#  define RCC_CCIPR_ADCSEL_PLLSAI2    (2 << RCC_CCIPR_ADCSEL_SHIFT)
#  define RCC_CCIPR_ADCSEL_SYSCLK     (3 << RCC_CCIPR_ADCSEL_SHIFT)
#define RCC_CCIPR_SWPMI1SEL           (1 << 30)
#  define RCC_CCIPR_SWPMI1SEL_PCLK    0
#  define RCC_CCIPR_SWPMI1SEL_HSI     RCC_CCIPR_SWPMI1SEL
#define RCC_CCIPR_DFSDMSEL            (1 << 31)
#  define RCC_CCIPR_DFSDMSEL_PCLK     0
#  define RCC_CCIPR_DFSDMSEL_SYSCLK   RCC_CCIPR_DFSDMSEL

/* Backup domain control register */

#define RCC_BDCR_LSEON              (1 << 0)  /* Bit 0: External Low Speed oscillator enable */
#define RCC_BDCR_LSERDY             (1 << 1)  /* Bit 1: External Low Speed oscillator Ready */
#define RCC_BDCR_LSEBYP             (1 << 2)  /* Bit 2: External Low Speed oscillator Bypass */
#define RCC_BDCR_LSEDRV_SHIFT       (3)       /* Bits 3-4: LSE oscillator drive capability */
#define RCC_BDCR_LSEDRV_MASK        (3 << RCC_BDCR_LSEDRV_SHIFT)
#  define RCC_BDCR_LSEDRV_LOW       (0 << RCC_BDCR_LSEDRV_SHIFT) /* 00: Lower driving capability */
#  define RCC_BDCR_LSEDRV_MEDLO     (1 << RCC_BDCR_LSEDRV_SHIFT) /* 01: Medium Low driving capability */
#  define RCC_BDCR_LSEDRV_MEDHI     (2 << RCC_BDCR_LSEDRV_SHIFT) /* 10: Medium High driving capability*/
#  define RCC_BDCR_LSEDRV_HIGH      (3 << RCC_BDCR_LSEDRV_SHIFT) /* 11: Higher driving capability */

#define RCC_BDCR_LSECSSON           (1 << 5) /* Bit 5: CSS on LSE enable */
#define RCC_BDCR_LSECSSD            (1 << 6) /* Bit 6: CSS on LSE failure Detection */
#define RCC_BDCR_RTCSEL_SHIFT       (8)      /* Bits 9:8: RTC clock source selection */
#define RCC_BDCR_RTCSEL_MASK        (3 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_NOCLK     (0 << RCC_BDCR_RTCSEL_SHIFT) /* 00: No clock */
#  define RCC_BDCR_RTCSEL_LSE       (1 << RCC_BDCR_RTCSEL_SHIFT) /* 01: LSE oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_LSI       (2 << RCC_BDCR_RTCSEL_SHIFT) /* 10: LSI oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_HSE       (3 << RCC_BDCR_RTCSEL_SHIFT) /* 11: HSE oscillator clock divided by 32 used as RTC clock */

#define RCC_BDCR_RTCEN              (1 << 15) /* Bit 15: RTC clock enable */
#define RCC_BDCR_BDRST              (1 << 16) /* Bit 16: Backup domain software reset */
#define RCC_BDCR_LSCOEN             (1 << 24) /* Bit 24: Low speed clock output enable */
#define RCC_BDCR_LSCOSEL            (1 << 25) /* Bit 25: Low speed clock output selection */

#  define RCC_BCDR_LSCOSEL_LSI      0                 /* LSI selected */
#  define RCC_BDCR_LSCOSEL_LSE      RCC_BDCR_LSCOSEL  /* LSE selected */

/* Control/status register */

#define RCC_CSR_LSION               (1 << 0)  /* Bit 0: Internal Low Speed oscillator enable */
#define RCC_CSR_LSIRDY              (1 << 1)  /* Bit 1: Internal Low Speed oscillator Ready */
#define RCC_CSR_MSISRANGE_SHIFT     8
#  define RCC_CSR_MSISRANGE_MASK    (0x0F << RCC_CSR_MSISRANGE_SHIFT) /* MSI range after Standby mode */
#  define RCC_CSR_MSISRANGE_1M      (4    << RCC_CSR_MSISRANGE_SHIFT) /* 0100: around 1 MHz */
#  define RCC_CSR_MSISRANGE_2M      (5    << RCC_CSR_MSISRANGE_SHIFT) /* 0101: around 2 MHz */
#  define RCC_CSR_MSISRANGE_4M      (6    << RCC_CSR_MSISRANGE_SHIFT) /* 0110: around 4 MHz */
#  define RCC_CSR_MSISRANGE_8M      (7    << RCC_CSR_MSISRANGE_SHIFT) /* 0111: around 8 MHz */

#define RCC_CSR_RMVF                (1 << 23) /* Bit 23: Remove reset flag */
#define RCC_CSR_FWRSTF              (1 << 24) /* Bit 24: Firewall reset flag */
#define RCC_CSR_OBLRSTF             (1 << 25) /* Bit 25: Option byte loader reset flag */
#define RCC_CSR_PINRSTF             (1 << 26) /* Bit 26: PIN reset flag */
#define RCC_CSR_BORRSTF             (1 << 27) /* Bit 27: BOR reset flag */
#define RCC_CSR_SFTRSTF             (1 << 28) /* Bit 28: Software Reset flag */
#define RCC_CSR_IWDGRSTF            (1 << 29) /* Bit 29: Independent Watchdog reset flag */
#define RCC_CSR_WWDGRSTF            (1 << 30) /* Bit 30: Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF            (1 << 31) /* Bit 31: Low-Power reset flag */

#endif /* CONFIG_STM32L4_STM32L4X5 */
#endif /* __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4X5XX_RCC_H */
