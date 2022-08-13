/****************************************************************************
 * arch/arm/src/stm32wl5/hardware/stm32wl5_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL5_RCC_H
#define __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL5_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32WL5_RCC_CR_OFFSET           0x0000  /* Clock control register */
#define STM32WL5_RCC_ICSCR_OFFSET        0x0004  /* Internal clock sources calibration register */
#define STM32WL5_RCC_CFGR_OFFSET         0x0008  /* Clock configuration register */
#define STM32WL5_RCC_PLLCFG_OFFSET       0x000c  /* PLL configuration register */
#define STM32WL5_RCC_CIER_OFFSET         0x0018  /* Clock interrupt enable register */
#define STM32WL5_RCC_CIFR_OFFSET         0x001c  /* Clock interrupt flag register */
#define STM32WL5_RCC_CICR_OFFSET         0x0020  /* Clock interrupt clear register */
#define STM32WL5_RCC_AHB1RSTR_OFFSET     0x0028  /* AHB1 peripheral reset register */
#define STM32WL5_RCC_AHB2RSTR_OFFSET     0x002c  /* AHB2 peripheral reset register */
#define STM32WL5_RCC_AHB3RSTR_OFFSET     0x0030  /* AHB3 peripheral reset register */
#define STM32WL5_RCC_APB1RSTR1_OFFSET    0x0038  /* APB1 Peripheral reset register 1 */
#define STM32WL5_RCC_APB1RSTR2_OFFSET    0x003c  /* APB1 Peripheral reset register 2 */
#define STM32WL5_RCC_APB2RSTR_OFFSET     0x0040  /* APB2 Peripheral reset register */
#define STM32WL5_RCC_AHB1ENR_OFFSET      0x0048  /* AHB1 Peripheral Clock enable register */
#define STM32WL5_RCC_AHB2ENR_OFFSET      0x004c  /* AHB2 Peripheral Clock enable register */
#define STM32WL5_RCC_AHB3ENR_OFFSET      0x0050  /* AHB3 Peripheral Clock enable register */
#define STM32WL5_RCC_APB1ENR1_OFFSET     0x0058  /* APB1 Peripheral Clock enable register 1 */
#define STM32WL5_RCC_APB1ENR2_OFFSET     0x005c  /* APB1 Peripheral Clock enable register 2 */
#define STM32WL5_RCC_APB2ENR_OFFSET      0x0060  /* APB2 Peripheral Clock enable register */
#define STM32WL5_RCC_AHB1SMENR_OFFSET    0x0068  /* RCC AHB1 low power mode peripheral clock enable register */
#define STM32WL5_RCC_AHB2SMENR_OFFSET    0x006c  /* RCC AHB2 low power mode peripheral clock enable register */
#define STM32WL5_RCC_AHB3SMENR_OFFSET    0x0070  /* RCC AHB3 low power mode peripheral clock enable register */
#define STM32WL5_RCC_APB1SMENR1_OFFSET   0x0078  /* RCC APB1 low power mode peripheral clock enable register 1 */
#define STM32WL5_RCC_APB1SMENR2_OFFSET   0x007c  /* RCC APB1 low power mode peripheral clock enable register 2 */
#define STM32WL5_RCC_APB2SMENR_OFFSET    0x0080  /* RCC APB2 low power mode peripheral clock enable register */
#define STM32WL5_RCC_CCIPR_OFFSET        0x0088  /* Peripherals independent clock configuration register 1 */
#define STM32WL5_RCC_BDCR_OFFSET         0x0090  /* Backup domain control register */
#define STM32WL5_RCC_CSR_OFFSET          0x0094  /* Control/status register */
#define STM32WL5_RCC_EXTCFGR_OFFSET      0x0108
#define STM32WL5_RCC_C2AHB1ENR_OFFSET    0x0148  /* CPU2 AHB1 Peripheral Clock enable register */
#define STM32WL5_RCC_C2AHB2ENR_OFFSET    0x014c  /* CPU2 AHB2 Peripheral Clock enable register */
#define STM32WL5_RCC_C2AHB3ENR_OFFSET    0x0150  /* CPU2 AHB3 Peripheral Clock enable register */
#define STM32WL5_RCC_C2APB1ENR1_OFFSET   0x0158  /* CPU2 APB1 Peripheral Clock enable register 1 */
#define STM32WL5_RCC_C2APB1ENR2_OFFSET   0x015c  /* CPU2 APB1 Peripheral Clock enable register 2 */
#define STM32WL5_RCC_C2APB2ENR_OFFSET    0x0160  /* CPU2 APB2 Peripheral Clock enable register */
#define STM32WL5_RCC_C2APB3ENR_OFFSET    0x0164  /* CPU2 APB3 Peripheral Clock enable register */
#define STM32WL5_RCC_C2AHB1SMENR_OFFSET  0x0168  /* CPU2 RCC AHB1 low power mode peripheral clock enable register */
#define STM32WL5_RCC_C2AHB2SMENR_OFFSET  0x016c  /* CPU2 RCC AHB2 low power mode peripheral clock enable register */
#define STM32WL5_RCC_C2AHB3SMENR_OFFSET  0x0170  /* CPU2 RCC AHB3 low power mode peripheral clock enable register */
#define STM32WL5_RCC_C2APB1SMENR1_OFFSET 0x0178  /* CPU2 RCC APB1 low power mode peripheral clock enable register 1 */
#define STM32WL5_RCC_C2APB1SMENR2_OFFSET 0x017c  /* CPU2 RCC APB1 low power mode peripheral clock enable register 2 */
#define STM32WL5_RCC_C2APB2SMENR_OFFSET  0x0180  /* CPU2 RCC APB2 low power mode peripheral clock enable register */
#define STM32WL5_RCC_C2APB3SMENR_OFFSET  0x0184  /* CPU2 RCC APB3 low power mode peripheral clock enable register */

/* Register Addresses *******************************************************/

#define STM32WL5_RCC_CR           (STM32WL5_RCC_BASE + STM32WL5_RCC_CR_OFFSET)
#define STM32WL5_RCC_ICSCR        (STM32WL5_RCC_BASE + STM32WL5_RCC_ICSCR_OFFSET)
#define STM32WL5_RCC_CFGR         (STM32WL5_RCC_BASE + STM32WL5_RCC_CFGR_OFFSET)
#define STM32WL5_RCC_PLLCFG       (STM32WL5_RCC_BASE + STM32WL5_RCC_PLLCFG_OFFSET)
#define STM32WL5_RCC_CIER         (STM32WL5_RCC_BASE + STM32WL5_RCC_CIER_OFFSET)
#define STM32WL5_RCC_CIFR         (STM32WL5_RCC_BASE + STM32WL5_RCC_CIFR_OFFSET)
#define STM32WL5_RCC_CICR         (STM32WL5_RCC_BASE + STM32WL5_RCC_CICR_OFFSET)
#define STM32WL5_RCC_AHB1RSTR     (STM32WL5_RCC_BASE + STM32WL5_RCC_AHB1RSTR_OFFSET)
#define STM32WL5_RCC_AHB2RSTR     (STM32WL5_RCC_BASE + STM32WL5_RCC_AHB2RSTR_OFFSET)
#define STM32WL5_RCC_AHB3RSTR     (STM32WL5_RCC_BASE + STM32WL5_RCC_AHB3RSTR_OFFSET)
#define STM32WL5_RCC_APB1RSTR1    (STM32WL5_RCC_BASE + STM32WL5_RCC_APB1RSTR1_OFFSET)
#define STM32WL5_RCC_APB1RSTR2    (STM32WL5_RCC_BASE + STM32WL5_RCC_APB1RSTR2_OFFSET)
#define STM32WL5_RCC_APB2RSTR     (STM32WL5_RCC_BASE + STM32WL5_RCC_APB2RSTR_OFFSET)
#define STM32WL5_RCC_AHB1ENR      (STM32WL5_RCC_BASE + STM32WL5_RCC_AHB1ENR_OFFSET)
#define STM32WL5_RCC_AHB2ENR      (STM32WL5_RCC_BASE + STM32WL5_RCC_AHB2ENR_OFFSET)
#define STM32WL5_RCC_AHB3ENR      (STM32WL5_RCC_BASE + STM32WL5_RCC_AHB3ENR_OFFSET)
#define STM32WL5_RCC_APB1ENR1     (STM32WL5_RCC_BASE + STM32WL5_RCC_APB1ENR1_OFFSET)
#define STM32WL5_RCC_APB1ENR2     (STM32WL5_RCC_BASE + STM32WL5_RCC_APB1ENR2_OFFSET)
#define STM32WL5_RCC_APB2ENR      (STM32WL5_RCC_BASE + STM32WL5_RCC_APB2ENR_OFFSET)
#define STM32WL5_RCC_AHB1SMENR    (STM32WL5_RCC_BASE + STM32WL5_RCC_AHB1SMENR_OFFSET)
#define STM32WL5_RCC_AHB2SMENR    (STM32WL5_RCC_BASE + STM32WL5_RCC_AHB2SMENR_OFFSET)
#define STM32WL5_RCC_AHB3SMENR    (STM32WL5_RCC_BASE + STM32WL5_RCC_AHB3SMENR_OFFSET)
#define STM32WL5_RCC_APB1SMENR1   (STM32WL5_RCC_BASE + STM32WL5_RCC_APB1SMENR1_OFFSET)
#define STM32WL5_RCC_APB1SMENR2   (STM32WL5_RCC_BASE + STM32WL5_RCC_APB1SMENR2_OFFSET)
#define STM32WL5_RCC_APB2SMENR    (STM32WL5_RCC_BASE + STM32WL5_RCC_APB2SMENR_OFFSET)
#define STM32WL5_RCC_CCIPR        (STM32WL5_RCC_BASE + STM32WL5_RCC_CCIPR_OFFSET)
#define STM32WL5_RCC_BDCR         (STM32WL5_RCC_BASE + STM32WL5_RCC_BDCR_OFFSET)
#define STM32WL5_RCC_CSR          (STM32WL5_RCC_BASE + STM32WL5_RCC_CSR_OFFSET)
#define STM32WL5_RCC_EXTCFGR      (STM32WL5_RCC_BASE + STM32WL5_RCC_EXTCFGR_OFFSET)
#define STM32WL5_RCC_C2AHB1ENR    (STM32WL5_RCC_BASE + STM32WL5_RCC_C2AHB1ENR_OFFSET)
#define STM32WL5_RCC_C2AHB2ENR    (STM32WL5_RCC_BASE + STM32WL5_RCC_C2AHB2ENR_OFFSET)
#define STM32WL5_RCC_C2AHB3ENR    (STM32WL5_RCC_BASE + STM32WL5_RCC_C2AHB3ENR_OFFSET)
#define STM32WL5_RCC_C2APB1ENR1   (STM32WL5_RCC_BASE + STM32WL5_RCC_C2APB1ENR1_OFFSET)
#define STM32WL5_RCC_C2APB1ENR2   (STM32WL5_RCC_BASE + STM32WL5_RCC_C2APB1ENR2_OFFSET)
#define STM32WL5_RCC_C2APB2ENR    (STM32WL5_RCC_BASE + STM32WL5_RCC_C2APB2ENR_OFFSET)
#define STM32WL5_RCC_C2APB3ENR    (STM32WL5_RCC_BASE + STM32WL5_RCC_C2APB3ENR_OFFSET)
#define STM32WL5_RCC_C2AHB1SMENR  (STM32WL5_RCC_BASE + STM32WL5_RCC_C2AHB1SMENR_OFFSET)
#define STM32WL5_RCC_C2AHB2SMENR  (STM32WL5_RCC_BASE + STM32WL5_RCC_C2AHB2SMENR_OFFSET)
#define STM32WL5_RCC_C2AHB3SMENR  (STM32WL5_RCC_BASE + STM32WL5_RCC_C2AHB3SMENR_OFFSET)
#define STM32WL5_RCC_C2APB1SMENR1 (STM32WL5_RCC_BASE + STM32WL5_RCC_C2APB1SMENR1_OFFSET)
#define STM32WL5_RCC_C2APB1SMENR2 (STM32WL5_RCC_BASE + STM32WL5_RCC_C2APB1SMENR2_OFFSET)
#define STM32WL5_RCC_C2APB2SMENR  (STM32WL5_RCC_BASE + STM32WL5_RCC_C2APB2SMENR_OFFSET)
#define STM32WL5_RCC_C2APB3SMENR  (STM32WL5_RCC_BASE + STM32WL5_RCC_C2APB3SMENR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Clock control register */

#define RCC_CR_MSION                     (1 << 0)  /* Bit 0: Internal Multi Speed clock enable */
#define RCC_CR_MSIRDY                    (1 << 1)  /* Bit 1: Internal Multi Speed clock ready flag */
#define RCC_CR_MSIPLLEN                  (1 << 2)  /* Bit 2: MSI clock PLL enable */
#define RCC_CR_MSIRGSEL                  (1 << 3)  /* Bit 3: MSI clock range selection */
#define RCC_CR_MSIRANGE_SHIFT            (4)       /* Bits 7-4: MSI clock range */
#define RCC_CR_MSIRANGE_MASK             (0x0f << RCC_CR_MSIRANGE_SHIFT)
#  define RCC_CR_MSIRANGE_100K           (0  << RCC_CR_MSIRANGE_SHIFT) /* 0000: around 100 kHz */
#  define RCC_CR_MSIRANGE_200K           (1  << RCC_CR_MSIRANGE_SHIFT) /* 0001: around 200 kHz */
#  define RCC_CR_MSIRANGE_400K           (2  << RCC_CR_MSIRANGE_SHIFT) /* 0010: around 400 kHz */
#  define RCC_CR_MSIRANGE_800K           (3  << RCC_CR_MSIRANGE_SHIFT) /* 0011: around 800 kHz */
#  define RCC_CR_MSIRANGE_1M             (4  << RCC_CR_MSIRANGE_SHIFT) /* 0100: around 1 MHz */
#  define RCC_CR_MSIRANGE_2M             (5  << RCC_CR_MSIRANGE_SHIFT) /* 0101: around 2 MHz */
#  define RCC_CR_MSIRANGE_4M             (6  << RCC_CR_MSIRANGE_SHIFT) /* 0110: around 4 MHz */
#  define RCC_CR_MSIRANGE_8M             (7  << RCC_CR_MSIRANGE_SHIFT) /* 0111: around 8 MHz */
#  define RCC_CR_MSIRANGE_16M            (8  << RCC_CR_MSIRANGE_SHIFT) /* 1000: around 16 MHz */
#  define RCC_CR_MSIRANGE_24M            (9  << RCC_CR_MSIRANGE_SHIFT) /* 1001: around 24 MHz */
#  define RCC_CR_MSIRANGE_32M            (10 << RCC_CR_MSIRANGE_SHIFT) /* 1010: around 32 MHz */
#  define RCC_CR_MSIRANGE_48M            (11 << RCC_CR_MSIRANGE_SHIFT) /* 1011: around 48 MHz */

#define RCC_CR_HSION                     (1 << 8)  /* Bit 8: Internal High Speed clock enable */
#define RCC_CR_HSIKERON                  (1 << 9)  /* Bit 9: HSI16 always enable for peripheral kernels */
#define RCC_CR_HSIRDY                    (1 << 10) /* Bit 10: Internal High Speed clock ready flag */
#define RCC_CR_HSIASFS                   (1 << 11) /* Bit 11: HSI automatic start from stop */

#define RCC_CR_HSEON                     (1 << 16) /* Bit 16: External High Speed clock enable */
#define RCC_CR_HSERDY                    (1 << 17) /* Bit 17: External High Speed clock ready flag */
#define RCC_CR_CSSON                     (1 << 19) /* Bit 19: Clock Security System enable */
#define RCC_CR_HSEPRE                    (1 << 20) /* Bit 20: HSE prescaller */
#define RCC_CR_HSEBYPPWR                 (1 << 21) /* Bit 21: Enable HSE VDDTCXO */
#define RCC_CR_PLLON                     (1 << 24) /* Bit 24: PLL enable */
#define RCC_CR_PLLRDY                    (1 << 25) /* Bit 25: PLL clock ready flag */

/* Internal Clock Sources Calibration */

#define RCC_CR_HSITRIM_SHIFT             (24)      /* Bits 30-24: Internal High Speed clock trimming */
#define RCC_CR_HSITRIM_MASK              (0x7f << RCC_CR_HSITRIM_SHIFT)
#define RCC_CR_HSICAL_SHIFT              (16)      /* Bits 23-16: Internal High Speed clock Calibration */
#define RCC_CR_HSICAL_MASK               (0xff << RCC_CR_HSICAL_SHIFT)
#define RCC_CR_MSITRIM_SHIFT             (8)       /* Bits 15-8: Internal Multi Speed clock trimming */
#define RCC_CR_MSITRIM_MASK              (0xff << RCC_CR_MSITRIM_SHIFT)
#define RCC_CR_MSICAL_SHIFT              (0)       /* Bits 7-0: Internal Multi Speed clock Calibration */
#define RCC_CR_MSICAL_MASK               (0xff << RCC_CR_MSICAL_SHIFT)

/* Clock configuration register */

#define RCC_CFGR_SW_SHIFT                (0)       /* Bits 0-1: System clock Switch */
#define RCC_CFGR_SW_MASK                 (3 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_MSI                (0 << RCC_CFGR_SW_SHIFT) /* 00: MSI selected as system clock */
#  define RCC_CFGR_SW_HSI16              (1 << RCC_CFGR_SW_SHIFT) /* 01: HSI16 selected as system clock */
#  define RCC_CFGR_SW_HSE                (2 << RCC_CFGR_SW_SHIFT) /* 10: HSE selected as system clock */
#  define RCC_CFGR_SW_PLL                (3 << RCC_CFGR_SW_SHIFT) /* 11: PLL selected as system clock */

#define RCC_CFGR_SWS_SHIFT               (2)       /* Bits 2-3: System Clock Switch Status */
#define RCC_CFGR_SWS_MASK                (3 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_MSI               (0 << RCC_CFGR_SWS_SHIFT) /* 00: MSI oscillator used as system clock */
#  define RCC_CFGR_SWS_HSI16             (1 << RCC_CFGR_SWS_SHIFT) /* 01: HSI16 oscillator used as system clock */
#  define RCC_CFGR_SWS_HSE               (2 << RCC_CFGR_SWS_SHIFT) /* 10: HSE oscillator used as system clock */
#  define RCC_CFGR_SWS_PLL               (3 << RCC_CFGR_SWS_SHIFT) /* 11: PLL used as system clock */

#define RCC_CFGR_HPRE_SHIFT              (4)       /* Bits 4-7: AHB prescaler */
#define RCC_CFGR_HPRE_MASK               (0x0f << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLK           ( 0 << RCC_CFGR_HPRE_SHIFT) /* not-listed: SYSCLK not divided */
#  define RCC_CFGR_HPRE_SYSCLK_3         ( 1 << RCC_CFGR_HPRE_SHIFT) /* 0001: SYSCLK divided by 3 */
#  define RCC_CFGR_HPRE_SYSCLK_5         ( 2 << RCC_CFGR_HPRE_SHIFT) /* 0010: SYSCLK divided by 5 */
#  define RCC_CFGR_HPRE_SYSCLK_6         ( 5 << RCC_CFGR_HPRE_SHIFT) /* 0101: SYSCLK divided by 6 */
#  define RCC_CFGR_HPRE_SYSCLK_10        ( 6 << RCC_CFGR_HPRE_SHIFT) /* 0110: SYSCLK divided by 10 */
#  define RCC_CFGR_HPRE_SYSCLK_32        ( 7 << RCC_CFGR_HPRE_SHIFT) /* 0111: SYSCLK divided by 32 */
#  define RCC_CFGR_HPRE_SYSCLK_2         ( 8 << RCC_CFGR_HPRE_SHIFT) /* 1000: SYSCLK divided by 2 */
#  define RCC_CFGR_HPRE_SYSCLK_4         ( 9 << RCC_CFGR_HPRE_SHIFT) /* 1001: SYSCLK divided by 4 */
#  define RCC_CFGR_HPRE_SYSCLK_8         (10 << RCC_CFGR_HPRE_SHIFT) /* 1010: SYSCLK divided by 8 */
#  define RCC_CFGR_HPRE_SYSCLK_16        (11 << RCC_CFGR_HPRE_SHIFT) /* 1011: SYSCLK divided by 16 */
#  define RCC_CFGR_HPRE_SYSCLK_64        (12 << RCC_CFGR_HPRE_SHIFT) /* 1100: SYSCLK divided by 64 */
#  define RCC_CFGR_HPRE_SYSCLK_128       (13 << RCC_CFGR_HPRE_SHIFT) /* 1101: SYSCLK divided by 128 */
#  define RCC_CFGR_HPRE_SYSCLK_256       (14 << RCC_CFGR_HPRE_SHIFT) /* 1110: SYSCLK divided by 256 */
#  define RCC_CFGR_HPRE_SYSCLK_512       (15 << RCC_CFGR_HPRE_SHIFT) /* 1111: SYSCLK divided by 512 */

#define RCC_CFGR_PPRE1_SHIFT             (8)       /* Bits 8-10: APB Low speed prescaler (APB1) */
#define RCC_CFGR_PPRE1_MASK              (7 << RCC_CFGR_PPRE1_SHIFT)
#  define RCC_CFGR_PPRE1_HCLK            (0 << RCC_CFGR_PPRE1_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE1_HCLK_2          (4 << RCC_CFGR_PPRE1_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE1_HCLK_4          (5 << RCC_CFGR_PPRE1_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE1_HCLK_8          (6 << RCC_CFGR_PPRE1_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE1_HCLK_16         (7 << RCC_CFGR_PPRE1_SHIFT) /* 111: HCLK divided by 16 */

#define RCC_CFGR_PPRE2_SHIFT             (11)      /* Bits 11-13: APB High speed prescaler (APB2) */
#define RCC_CFGR_PPRE2_MASK              (7 << RCC_CFGR_PPRE2_SHIFT)
#  define RCC_CFGR_PPRE2_HCLK            (0 << RCC_CFGR_PPRE2_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE2_HCLK_2          (4 << RCC_CFGR_PPRE2_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE2_HCLK_4          (5 << RCC_CFGR_PPRE2_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE2_HCLK_8          (6 << RCC_CFGR_PPRE2_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE2_HCLK_16         (7 << RCC_CFGR_PPRE2_SHIFT) /* 111: HCLK divided by 16 */

#define RCC_CFGR_STOPWUCK                (1 << 15) /* Bit 15: Wakeup from Stop and CSS backup clock selection */
#  define RCC_CFGR_STOPWUCK_MSI          (0 << 15) /* 0: MSI */
#  define RCC_CFGR_STOPWUCK_HSI16        (1 << 15) /* 1: HSI16 */

#define RCC_CFGR_HPREF                   (1 << 16) /* Bit 16: HCLK1 prescaler flag (cpu1, ahb1, ahb2) */
#  define RCC_CFGR_HPREF_OFF             (0 << 16) /* 0: Prescaller value not applied */
#  define RCC_CFGR_HPREF_ON              (1 << 16) /* 1: Prescaller value applied */

#define RCC_CFGR_PPRE1F                  (1 << 17) /* Bit 17: PCLK1 prescaler flag (apb1) */
#  define RCC_CFGR_PPRE1F_OFF            (0 << 17) /* 0: Prescaller value not applied */
#  define RCC_CFGR_PPRE1F_ON             (1 << 17) /* 1: Prescaller value applied */

#define RCC_CFGR_PPRE2F                  (1 << 18) /* Bit 18: PCLK2 prescaler flag (apb2) */
#  define RCC_CFGR_PPRE2F_OFF            (0 << 18) /* 0: Prescaller value not applied */
#  define RCC_CFGR_PPRE2F_ON             (1 << 18) /* 1: Prescaller value applied */

#define RCC_CFGR_MCOSEL_SHIFT            (24)      /* Bits 24-27: Microcontroller Clock Output */
#define RCC_CFGR_MCOSEL_MASK             (0x0f << RCC_CFGR_MCOSEL_SHIFT)
#  define RCC_CFGR_MCOSEL_NONE           (0 << RCC_CFGR_MCOSEL_SHIFT) /* 0000: Disabled */
#  define RCC_CFGR_MCOSEL_SYSCLK         (1 << RCC_CFGR_MCOSEL_SHIFT) /* 0001: SYSCLK system clock selected */
#  define RCC_CFGR_MCOSEL_MSI            (2 << RCC_CFGR_MCOSEL_SHIFT) /* 0010: MSI clock selected */
#  define RCC_CFGR_MCOSEL_HSI16          (3 << RCC_CFGR_MCOSEL_SHIFT) /* 0011: HSI16 clock selected */
#  define RCC_CFGR_MCOSEL_HSE            (4 << RCC_CFGR_MCOSEL_SHIFT) /* 0100: HSE clock selected */
#  define RCC_CFGR_MCOSEL_PLL            (5 << RCC_CFGR_MCOSEL_SHIFT) /* 0101: Main PLL selected  */
#  define RCC_CFGR_MCOSEL_LSI            (6 << RCC_CFGR_MCOSEL_SHIFT) /* 0110: LSI clock selected */
#  define RCC_CFGR_MCOSEL_LSE            (7 << RCC_CFGR_MCOSEL_SHIFT) /* 0111: LSE clock selected */
#  define RCC_CFGR_MCOSEL_HSI48          (8 << RCC_CFGR_MCOSEL_SHIFT) /* 1000: HSI48 clock selected */

#define RCC_CFGR_MCOPRE_SHIFT            (28)      /* Bits 28-30: MCO prescaler */
#define RCC_CFGR_MCOPRE_MASK             (7 << RCC_CFGR_MCOPRE_SHIFT)
#  define RCC_CFGR_MCOPRE_NONE           (0 << RCC_CFGR_MCOPRE_SHIFT) /* 000: no division */
#  define RCC_CFGR_MCOPRE_DIV2           (1 << RCC_CFGR_MCOPRE_SHIFT) /* 001: division by 2 */
#  define RCC_CFGR_MCOPRE_DIV4           (2 << RCC_CFGR_MCOPRE_SHIFT) /* 010: division by 4 */
#  define RCC_CFGR_MCOPRE_DIV8           (3 << RCC_CFGR_MCOPRE_SHIFT) /* 011: division by 8 */
#  define RCC_CFGR_MCOPRE_DIV16          (4 << RCC_CFGR_MCOPRE_SHIFT) /* 100: division by 16 */

/* PLL configuration register */

#define RCC_PLLCFG_PLLSRC_SHIFT          (0)       /* Bit 0-1: Main PLL(PLL) and audio PLLs (PLLSAIx) entry clock source */
#define RCC_PLLCFG_PLLSRC_MASK           (3 << RCC_PLLCFG_PLLSRC_SHIFT)
#  define RCC_PLLCFG_PLLSRC_NONE         (0 << RCC_PLLCFG_PLLSRC_SHIFT) /* 000: No clock sent to PLLs */
#  define RCC_PLLCFG_PLLSRC_MSI          (1 << RCC_PLLCFG_PLLSRC_SHIFT) /* 001: MSI selected as PLL source */
#  define RCC_PLLCFG_PLLSRC_HSI16        (2 << RCC_PLLCFG_PLLSRC_SHIFT) /* 010: HSI16 selected as PLL source */
#  define RCC_PLLCFG_PLLSRC_HSE          (3 << RCC_PLLCFG_PLLSRC_SHIFT) /* 011: HSE selected as PLL source */

#define RCC_PLLCFG_PLLM_SHIFT            (4)       /* Bits 4-6: Main PLL (PLL) input clock divider */
#define RCC_PLLCFG_PLLM_MASK             (0x07 << RCC_PLLCFG_PLLM_SHIFT)
#  define RCC_PLLCFG_PLLM(n)             ((n-1) << RCC_PLLCFG_PLLM_SHIFT) /* m = 1..8 */

#define RCC_PLLCFG_PLLN_SHIFT            (8)       /* Bits 8-14: Main PLL (PLL) VCO multiplier */
#define RCC_PLLCFG_PLLN_MASK             (0x7f << RCC_PLLCFG_PLLN_SHIFT)
#  define RCC_PLLCFG_PLLN(n)             ((n) << RCC_PLLCFG_PLLN_SHIFT) /* n = 6..127 */

#define RCC_PLLCFG_PLLPEN                (1 << 16) /* Bit 16: Main PLL PLLSAI3CLK output enable */

#define RCC_PLLCFG_PLLP_SHIFT            (17)      /* Bit 17-21: Main PLL div factor for PLLPCLK */
#define RCC_PLLCFG_PLLP_MASK             (0x1f << RCC_PLLCFG_PLLP_SHIFT)
#  define RCC_PLLCFG_PLLP                (((n)-1) << RCC_PLLCFG_PLLP_SHIFT) /* 2..32 */

#define RCC_PLLCFG_PLLQEN                (1 << 24) /* Bit 24: Main PLL PLLQCLK output enable */

#define RCC_PLLCFG_PLLQ_SHIFT            (25)      /* Bits 25-27: Main PLL division factor for PLLQCLK (48 MHz clock) */
#define RCC_PLLCFG_PLLQ_MASK             (3 << RCC_PLLCFG_PLLQ_SHIFT)
#  define RCC_PLLCFG_PLLQ(n)             (((n)-1) << RCC_PLLCFG_PLLQ_SHIFT) /* n=2..8 */

#define RCC_PLLCFG_PLLREN                (1 << 28) /* Bit 24: Main PLL PLLRCLK output enable */

#define RCC_PLLCFG_PLLR_SHIFT            (29)      /* Bits 29-31: Main PLL division factor for PLLRCLK */
#define RCC_PLLCFG_PLLR_MASK             (3 << RCC_PLLCFG_PLLR_SHIFT)
#  define RCC_PLLCFG_PLLR(n)             (((n)-1) << RCC_PLLCFG_PLLR_SHIFT) /* n=2..8 */

/* Clock interrupt enable register */

#define RCC_CIER_LSIRDYIE                (1 << 0)  /* Bit 0: LSI Ready Interrupt Enable */
#define RCC_CIER_LSERDYIE                (1 << 1)  /* Bit 1: LSE Ready Interrupt Enable */
#define RCC_CIER_MSIRDYIE                (1 << 2)  /* Bit 2: MSI Ready Interrupt Enable */
#define RCC_CIER_HSIRDYIE                (1 << 3)  /* Bit 3: HSI Ready Interrupt Enable */
#define RCC_CIER_HSERDYIE                (1 << 4)  /* Bit 4: HSE Ready Interrupt Enable */
#define RCC_CIER_PLLRDYIE                (1 << 5)  /* Bit 5: PLL Ready Interrupt Enable */
#define RCC_CIER_PLESCCIE                (1 << 9)  /* Bit 9: LSE clock security system Interrupt Enable */

/* Clock interrupt flag register */

#define RCC_CIFR_LSIRDYIF                (1 << 0)  /* Bit 0: LSI Ready Interrupt Flag */
#define RCC_CIFR_LSERDYIF                (1 << 1)  /* Bit 1: LSE Ready Interrupt Flag */
#define RCC_CIFR_MSIRDYIF                (1 << 2)  /* Bit 2: MSI Ready Interrupt Flag */
#define RCC_CIFR_HSIRDYIF                (1 << 3)  /* Bit 3: HSI Ready Interrupt Flag */
#define RCC_CIFR_HSERDYIF                (1 << 4)  /* Bit 4: HSE Ready Interrupt Flag */
#define RCC_CIFR_PLLRDYIF                (1 << 5)  /* Bit 5: PLL Ready Interrupt Flag */
#define RCC_CIFR_CSSF                    (1 << 8)  /* Bit 8: Clock Security System Interrupt Flag */
#define RCC_CIFR_LSECSSF                 (1 << 9)  /* Bit 9: LSE CSS (clock security system) Interrupt Flag */

/* Clock interrupt clear register */

#define RCC_CICR_LSIRDYIC                (1 << 0)  /* Bit 0: LSI Ready Interrupt Clear */
#define RCC_CICR_LSERDYIC                (1 << 1)  /* Bit 1: LSE Ready Interrupt Clear */
#define RCC_CICR_MSIRDYIC                (1 << 2)  /* Bit 2: MSI Ready Interrupt Clear */
#define RCC_CICR_HSIRDYIC                (1 << 3)  /* Bit 3: HSI Ready Interrupt Clear */
#define RCC_CICR_HSERDYIC                (1 << 4)  /* Bit 4: HSE Ready Interrupt Clear */
#define RCC_CICR_PLLRDYIC                (1 << 5)  /* Bit 5: PLL Ready Interrupt Clear */
#define RCC_CICR_CSSC                    (1 << 8)  /* Bit 8: Clock Security System Interrupt Clear */
#define RCC_CICR_ LSECSSC                (1 << 9)  /* Bit 9: LSE CSS Interrupt Clear */

/* AHB1 peripheral reset register */

#define RCC_AHB1RSTR_DMA1RST             (1 << 0)  /* Bit 0:  DMA1 reset */
#define RCC_AHB1RSTR_DMA2RST             (1 << 1)  /* Bit 1:  DMA2 reset */
#define RCC_AHB1RSTR_DMAMUX1RST          (1 << 2)  /* Bit 2:  DMAMUX1 reset */
#define RCC_AHB1RSTR_CRCRST              (1 << 12) /* Bit 12: CRC reset */

/* AHB2 peripheral reset register */

#define RCC_AHB2RSTR_GPIORST(n)          (1 << (n))
#define RCC_AHB2RSTR_GPIOARST            (1 << 0)  /* Bit 0:  IO port A reset */
#define RCC_AHB2RSTR_GPIOBRST            (1 << 1)  /* Bit 1:  IO port B reset */
#define RCC_AHB2RSTR_GPIOCRST            (1 << 2)  /* Bit 2:  IO port C reset */
#define RCC_AHB2RSTR_GPIOHRST            (1 << 7)  /* Bit 7:  IO port H reset */

/* AHB3 peripheral reset register */

#define RCC_AHB3RSTR_PKARST              (1 << 16) /* Bit 16: PKA reset */
#define RCC_AHB3RSTR_AESRST              (1 << 17) /* Bit 16: AES reset */
#define RCC_AHB3RSTR_RNGRST              (1 << 18) /* Bit 16: RNG reset */
#define RCC_AHB3RSTR_HSEMRST             (1 << 19) /* Bit 16: HSEM reset */
#define RCC_AHB3RSTR_IPCCRST             (1 << 20) /* Bit 16: IPCC reset */
#define RCC_AHB3RSTR_FLASHRST            (1 << 25) /* Bit 16: FLASH reset */

/* APB1 Peripheral reset register 1 */

#define RCC_APB1RSTR1_TIM2RST            (1 << 0)  /* Bit 0:  TIM2 reset */
#define RCC_APB1RSTR1_SPI2RST            (1 << 14) /* Bit 14: SPI2 reset */
#define RCC_APB1RSTR1_USART2RST          (1 << 17) /* Bit 17: USART2 reset */
#define RCC_APB1RSTR1_I2C1RST            (1 << 21) /* Bit 21: I2C1 reset */
#define RCC_APB1RSTR1_I2C2RST            (1 << 22) /* Bit 22: I2C2 reset */
#define RCC_APB1RSTR1_I2C3RST            (1 << 23) /* Bit 23: I2C3 reset */
#define RCC_APB1RSTR1_DAC1RST            (1 << 29) /* Bit 29: DAC1 reset */
#define RCC_APB1RSTR1_LPTIM1RST          (1 << 31) /* Bit 31: Low-power Timer 1 reset */

/* APB1 Peripheral reset register 2 */

#define RCC_APB1RSTR2_LPUART1RST         (1 << 0)  /* Bit 0:  Low-power UART 1 reset */
#define RCC_APB1RSTR2_LPTIM2RST          (1 << 5)  /* Bit 5:  Low-power Timer 2 reset */
#define RCC_APB1RSTR2_LPTIM3RST          (1 << 6)  /* Bit 6:  Low-power Timer 3 reset */

/* APB2 Peripheral reset register */

#define RCC_APB2RSTR_ACDRST              (1 << 9)  /* Bit 9:  ADC1 reset */
#define RCC_APB2RSTR_TIM1RST             (1 << 11) /* Bit 11: TIM1 reset */
#define RCC_APB2RSTR_SPI1RST             (1 << 12) /* Bit 12: SPI1 reset */
#define RCC_APB2RSTR_USART1RST           (1 << 14) /* Bit 14: USART1 reset */
#define RCC_APB2RSTR_TIM16RST            (1 << 17) /* Bit 17: TIM16 reset */
#define RCC_APB2RSTR_TIM17RST            (1 << 18) /* Bit 18: TIM17 reset */

/* APB3 Peripheral reset register */

#define RCC_APB3RSTR_SUBGHZSPIRST        (1 << 0)  /* Bit 9:  SUBGHZSPI reset */

/* AHB1 Peripheral Clock enable register */

#define RCC_AHB1ENR_DMA1EN               (1 << 0)  /* Bit 0:  DMA1 enable */
#define RCC_AHB1ENR_DMA2EN               (1 << 1)  /* Bit 1:  DMA2 enable */
#define RCC_AHB1ENR_DMAMUX1EN            (1 << 2)  /* Bit 2:  DMAMUX1 enable */
#define RCC_AHB1ENR_CRCEN                (1 << 12) /* Bit 12: CRC enable */

/* AHB2 Peripheral Clock enable register */

#define RCC_AHB2ENR_GPIOEN(n)            (1 << (n))
#define RCC_AHB2ENR_GPIOAEN              (1 << 0)  /* Bit 0:  IO port A enable */
#define RCC_AHB2ENR_GPIOBEN              (1 << 1)  /* Bit 1:  IO port B enable */
#define RCC_AHB2ENR_GPIOCEN              (1 << 2)  /* Bit 2:  IO port C enable */
#define RCC_AHB2ENR_GPIOHEN              (1 << 7)  /* Bit 7:  IO port H enable */

/* AHB3 Peripheral Clock enable register */

#define RCC_AHB3ENR_PKAEN                (1 << 16) /* Bit 16: PKA clock enable */
#define RCC_AHB3ENR_AESEN                (1 << 17) /* Bit 17: AES Cryptographic module enable */
#define RCC_AHB3ENR_RNGEN                (1 << 18) /* Bit 18: Random number generator module enable */
#define RCC_AHB3ENR_HSEMEN               (1 << 19) /* Bit 19: HSEM enable */
#define RCC_AHB3ENR_IPCCEN               (1 << 20) /* Bit 20: IPCC enable */
#define RCC_AHB3ENR_FLASHEN              (1 << 25) /* Bit 25: FLASH enable */

/* APB1 Peripheral Clock enable register 1 */

#define RCC_APB1ENR1_TIM2EN              (1 << 0)  /* Bit 0:  TIM2 enable */
#define RCC_APB1ENR1_RTCAPBEN            (1 << 10) /* Bit 10: RTC APB clock enable */
#define RCC_APB1ENR1_WWDGEN              (1 << 11) /* Bit 11: Windowed Watchdog enable */
#define RCC_APB1ENR1_SPI2EN              (1 << 14) /* Bit 14: SPI2 enable */
#define RCC_APB1ENR1_USART2EN            (1 << 17) /* Bit 17: USART2 enable */
#define RCC_APB1ENR1_I2C1EN              (1 << 21) /* Bit 21: I2C1 enable */
#define RCC_APB1ENR1_I2C2EN              (1 << 22) /* Bit 22: I2C2 enable */
#define RCC_APB1ENR1_I2C3EN              (1 << 23) /* Bit 23: I2C3 enable */
#define RCC_APB1ENR1_DAC1EN              (1 << 29) /* Bit 29: DAC1 enable */
#define RCC_APB1ENR1_LPTIM1EN            (1 << 31) /* Bit 31: Low-power Timer 1 enable */

/* APB1 Peripheral Clock enable register 2 */

#define RCC_APB1ENR2_LPUART1EN           (1 << 0)  /* Bit 0:  Low-power UART 1 enable */
#define RCC_APB1ENR2_LPTIM2EN            (1 << 5)  /* Bit 5:  Low-power Timer 2 enable */
#define RCC_APB1ENR2_LPTIM3EN            (1 << 6)  /* Bit 6:  Low-power Timer 3 enable */

/* APB2 Peripheral Clock enable register */

#define RCC_APB2ENR_ADC1EN               (1 << 9)  /* Bit 9:  ADC1 enable */
#define RCC_APB2ENR_TIM1EN               (1 << 11) /* Bit 11: TIM1 enable */
#define RCC_APB2ENR_SPI1EN               (1 << 12) /* Bit 12: SPI1 enable */
#define RCC_APB2ENR_USART1EN             (1 << 14) /* Bit 14: USART1 enable */
#define RCC_APB2ENR_TIM16EN              (1 << 17) /* Bit 17: TIM16 enable */
#define RCC_APB2ENR_TIM17EN              (1 << 18) /* Bit 18: TIM17 enable */

/* APB3 Peripheral Clock enable register */

#define RCC_APB3ENR_SUBGHZSPIEN          (1 << 0)  /* Bit 0:  SUBGHZSPI enable */

/* RCC AHB1 Sleep and Stop modes peripheral clock enable register */

#define RCC_AHB1SMENR_DMA1SMEN           (1 << 0)  /* Bit 0:  DMA1 enable during Sleep mode */
#define RCC_AHB1SMENR_DMA2SMEN           (1 << 1)  /* Bit 1:  DMA2 enable during Sleep mode */
#define RCC_AHB1SMENR_DMAMUX1SMEN        (1 << 2)  /* Bit 2:  DMAMUX1 enable during Sleep mode */
#define RCC_AHB1SMENR_CRCSMEN            (1 << 12) /* Bit 12: CRC enable during Sleep mode */

/* RCC AHB2 low power mode peripheral clock enable register */

#define RCC_AHB2SMENR_GPIOASMEN          (1 << 0)  /* Bit 0:  IO port A enable during Sleep mode */
#define RCC_AHB2SMENR_GPIOBSMEN          (1 << 1)  /* Bit 1:  IO port B enable during Sleep mode */
#define RCC_AHB2SMENR_GPIOCSMEN          (1 << 2)  /* Bit 2:  IO port C enable during Sleep mode */
#define RCC_AHB2SMENR_GPIOHSMEN          (1 << 7)  /* Bit 7:  IO port H enable during Sleep mode */

/* RCC AHB3 low power mode peripheral clock enable register */

#define RCC_AHB2SMENR_PKASMEN            (1 << 16) /* Bit 16: PKA enable during cpu1 sleep mode */
#define RCC_AHB2SMENR_AESSMEN            (1 << 17) /* Bit 17: AES Cryptographic module enable during cpu1 Sleep mode */
#define RCC_AHB2SMENR_RNGSMEN            (1 << 18) /* Bit 18: Random number generator module enable during cpu1 Sleep mode */
#define RCC_AHB2SMENR_SRAM1SMEN          (1 << 23) /* Bit 23: SRAM1 enable during cpu1 Sleep mode */
#define RCC_AHB2SMENR_SRAM2SMEN          (1 << 24) /* Bit 24: SRAM2 enable during cpu1 Sleep mode */
#define RCC_AHB2SMENR_FLASHSMEN          (1 << 25) /* Bit 25: FLASH enable during cpu1 Sleep mode */

/* RCC APB1 low power mode peripheral clock enable register 1 */

#define RCC_APB1SMENR1_TIM2SMEN          (1 << 0)  /* Bit 0:  TIM2 enable during Sleep mode */
#define RCC_APB1SMENR1_RTCAPBSMEN        (1 << 10) /* Bit 10: RTC APB clock enable during Sleep mode */
#define RCC_APB1SMENR1_WWDGSMEN          (1 << 11) /* Bit 11: Windowed Watchdog enable during Sleep mode */
#define RCC_APB1SMENR1_SPI2SMEN          (1 << 14) /* Bit 14: SPI2 enable during Sleep mode */
#define RCC_APB1SMENR1_USART2SMEN        (1 << 17) /* Bit 17: USART2 enable during Sleep mode */
#define RCC_APB1SMENR1_I2C1SMEN          (1 << 21) /* Bit 21: I2C1 enable during Sleep mode */
#define RCC_APB1SMENR1_I2C2SMEN          (1 << 22) /* Bit 22: I2C2 enable during Sleep mode */
#define RCC_APB1SMENR1_I2C3SMEN          (1 << 23) /* Bit 23: I2C3 enable during Sleep mode */
#define RCC_APB1SMENR1_DAC1SMEN          (1 << 29) /* Bit 29: DAC1 enable during Sleep mode */
#define RCC_APB1SMENR1_LPTIM1SMEN        (1 << 31) /* Bit 31: Low-power Timer 1 enable during Sleep mode */

/* RCC APB1 low power modeperipheral clock enable register 2 */

#define RCC_APB1SMENR2_LPUART1SMEN       (1 << 0)  /* Bit 0:  Low-power UART 1 enable during Sleep mode */
#define RCC_APB1SMENR2_LPTIM2SMEN        (1 << 5)  /* Bit 5:  Low-power Timer 2 enable during Sleep mode */
#define RCC_APB1SMENR2_LPTIM3SMEN        (1 << 6)  /* Bit 6:  Low-power Timer 3 enable during Sleep mode */

/* RCC APB2 low power mode peripheral clock enable register */

#define RCC_APB2SMENR_ADC1SMEN           (1 << 9)  /* Bit 9:  ADC1 enable during Sleep mode */
#define RCC_APB2SMENR_TIM1SMEN           (1 << 11) /* Bit 11: TIM1 enable during Sleep mode */
#define RCC_APB2SMENR_SPI1SMEN           (1 << 12) /* Bit 12: SPI1 enable during Sleep mode */
#define RCC_APB2SMENR_USART1SMEN         (1 << 14) /* Bit 14: USART1 enable during Sleep mode */
#define RCC_APB2SMENR_TIM16SMEN          (1 << 17) /* Bit 17: TIM16 enable during Sleep mode */
#define RCC_APB2SMENR_TIM17SMEN          (1 << 18) /* Bit 18: TIM17 enable during Sleep mode */

/* RCC APB3 low power mode peripheral clock enable register */

#define RCC_APB3SMENR_SUBGHZSPISMEN      (1 << 0)  /* Bit 0: SUBGHZSPI enable during Sleep mode */

/* Peripheral Independent Clock Configuration register */

#define RCC_CCIPR_USART1SEL_SHIFT        (0)
#define RCC_CCIPR_USART1SEL_MASK         (3 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_PCLK1      (0 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_SYSCLK     (1 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_HSI16      (2 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_LSE        (3 << RCC_CCIPR_USART1SEL_SHIFT)

#define RCC_CCIPR_USART2SEL_SHIFT        (2)
#define RCC_CCIPR_USART2SEL_MASK         (3 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_PCLK1      (0 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_SYSCLK     (1 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_HSI16      (2 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_LSE        (3 << RCC_CCIPR_USART2SEL_SHIFT)

#define RCC_CCIPR_SPI2S2SEL_SHIFT        (8)
#define RCC_CCIPR_SPI2S2SEL_MASK         (3 << RCC_CCIPR_SPI2S2SEL_SHIFT)
#  define RCC_CCIPR_SPI2S2SEL_PCLK1      (0 << RCC_CCIPR_SPI2S2SEL_SHIFT)
#  define RCC_CCIPR_SPI2S2SEL_SYSCLK     (1 << RCC_CCIPR_SPI2S2SEL_SHIFT)
#  define RCC_CCIPR_SPI2S2SEL_HSI16      (2 << RCC_CCIPR_SPI2S2SEL_SHIFT)
#  define RCC_CCIPR_SPI2S2SEL_LSE        (3 << RCC_CCIPR_SPI2S2SEL_SHIFT)

#define RCC_CCIPR_LPUART1SEL_SHIFT       (10)
#define RCC_CCIPR_LPUART1SEL_MASK        (3 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_PCLK1     (0 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_SYSCLK    (1 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_HSI16     (2 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_LSE       (3 << RCC_CCIPR_LPUART1SEL_SHIFT)

#define RCC_CCIPR_I2C1SEL_SHIFT          (12)
#define RCC_CCIPR_I2C1SEL_MASK           (3 << RCC_CCIPR_I2C1SEL_SHIFT)
#  define RCC_CCIPR_I2C1SEL_PCLK1        (0 << RCC_CCIPR_I2C1SEL_SHIFT)
#  define RCC_CCIPR_I2C1SEL_SYSCLK       (1 << RCC_CCIPR_I2C1SEL_SHIFT)
#  define RCC_CCIPR_I2C1SEL_HSI16        (2 << RCC_CCIPR_I2C1SEL_SHIFT)

#define RCC_CCIPR_I2C2SEL_SHIFT          (14)
#define RCC_CCIPR_I2C2SEL_MASK           (3 << RCC_CCIPR_I2C2SEL_SHIFT)
#  define RCC_CCIPR_I2C2SEL_PCLK1        (0 << RCC_CCIPR_I2C2SEL_SHIFT)
#  define RCC_CCIPR_I2C2SEL_SYSCLK       (1 << RCC_CCIPR_I2C2SEL_SHIFT)
#  define RCC_CCIPR_I2C2SEL_HSI16        (2 << RCC_CCIPR_I2C2SEL_SHIFT)

#define RCC_CCIPR_I2C3SEL_SHIFT          (16)
#define RCC_CCIPR_I2C3SEL_MASK           (3 << RCC_CCIPR_I2C3SEL_SHIFT)
#  define RCC_CCIPR_I2C3SEL_PCLK1        (0 << RCC_CCIPR_I2C3SEL_SHIFT)
#  define RCC_CCIPR_I2C3SEL_SYSCLK       (1 << RCC_CCIPR_I2C3SEL_SHIFT)
#  define RCC_CCIPR_I2C3SEL_HSI16        (2 << RCC_CCIPR_I2C3SEL_SHIFT)

#define RCC_CCIPR_LPTIM1SEL_SHIFT        (18)
#define RCC_CCIPR_LPTIM1SEL_MASK         (3 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_PCLK1      (0 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_LSI        (1 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_HSI16      (2 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_LSE        (3 << RCC_CCIPR_LPTIM1SEL_SHIFT)

#define RCC_CCIPR_LPTIM2SEL_SHIFT        (20)
#define RCC_CCIPR_LPTIM2SEL_MASK         (3 << RCC_CCIPR_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR_LPTIM2SEL_PCLK1      (0 << RCC_CCIPR_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR_LPTIM2SEL_LSI        (1 << RCC_CCIPR_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR_LPTIM2SEL_HSI16      (2 << RCC_CCIPR_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR_LPTIM2SEL_LSE        (3 << RCC_CCIPR_LPTIM2SEL_SHIFT)

#define RCC_CCIPR_LPTIM3SEL_SHIFT        (22)
#define RCC_CCIPR_LPTIM3SEL_MASK         (3 << RCC_CCIPR_SAI1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM3SEL_PCLK1      (0 << RCC_CCIPR_SAI1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM3SEL_LSI        (1 << RCC_CCIPR_SAI1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM3SEL_HSI16      (2 << RCC_CCIPR_SAI1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM3SEL_LSE        (3 << RCC_CCIPR_SAI1SEL_SHIFT)

#define RCC_CCIPR_ADCSEL_SHIFT           (28)
#define RCC_CCIPR_ADCSEL_MASK            (3 << RCC_CCIPR_ADCSEL_SHIFT)
#  define RCC_CCIPR_ADCSEL_NONE          (0 << RCC_CCIPR_ADCSEL_SHIFT)
#  define RCC_CCIPR_ADCSEL_HSI16         (1 << RCC_CCIPR_ADCSEL_SHIFT)
#  define RCC_CCIPR_ADCSEL_PLLADC1CLK    (2 << RCC_CCIPR_ADCSEL_SHIFT)
#  define RCC_CCIPR_ADCSEL_SYSCLK        (3 << RCC_CCIPR_ADCSEL_SHIFT)

#define RCC_CCIPR_RNGSEL_SHIFT           (30)
#define RCC_CCIPR_RNGSEL_MASK            (3 << RCC_CCIPR_RNGSEL_SHIFT)
#  define RCC_CCIPR_RNGSEL_NONE          (0 << RCC_CCIPR_RNGSEL_SHIFT)
#  define RCC_CCIPR_RNGSEL_HSI16         (1 << RCC_CCIPR_RNGSEL_SHIFT)
#  define RCC_CCIPR_RNGSEL_PLLRNG1CLK    (2 << RCC_CCIPR_RNGSEL_SHIFT)
#  define RCC_CCIPR_RNGSEL_SYSCLK        (3 << RCC_CCIPR_RNGSEL_SHIFT)

/* Backup domain control register */

#define RCC_BDCR_LSEON                   (1 << 0)  /* Bit 0: External Low Speed oscillator enable */
#define RCC_BDCR_LSERDY                  (1 << 1)  /* Bit 1: External Low Speed oscillator Ready */
#define RCC_BDCR_LSEBYP                  (1 << 2)  /* Bit 2: External Low Speed oscillator Bypass */

#define RCC_BDCR_LSEDRV_SHIFT            (3)       /* Bits 3-4: LSE oscillator drive capability */
#define RCC_BDCR_LSEDRV_MASK             (3 << RCC_BDCR_LSEDRV_SHIFT)
#  define RCC_BDCR_LSEDRV_LOW            (0 << RCC_BDCR_LSEDRV_SHIFT) /* 00: Lower driving capability */
#  define RCC_BDCR_LSEDRV_MEDLO          (1 << RCC_BDCR_LSEDRV_SHIFT) /* 01: Medium Low driving capability */
#  define RCC_BDCR_LSEDRV_MEDHI          (2 << RCC_BDCR_LSEDRV_SHIFT) /* 10: Medium High driving capability*/
#  define RCC_BDCR_LSEDRV_HIGH           (3 << RCC_BDCR_LSEDRV_SHIFT) /* 11: Higher driving capability */

#define RCC_BDCR_LSECSSON                (1 << 5) /* Bit 5: CSS on LSE enable */
#define RCC_BDCR_LSECSSD                 (1 << 6) /* Bit 6: CSS on LSE failure Detection */
#define RCC_BDCR_LSESYSEN                (1 << 7) /* Bit 7: LSE system clock (LSESYS) enable */

#define RCC_BDCR_RTCSEL_SHIFT            (8)       /* Bits 9:8: RTC clock source selection */
#define RCC_BDCR_RTCSEL_MASK             (3 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_NOCLK          (0 << RCC_BDCR_RTCSEL_SHIFT) /* 00: No clock */
#  define RCC_BDCR_RTCSEL_LSE            (1 << RCC_BDCR_RTCSEL_SHIFT) /* 01: LSE oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_LSI            (2 << RCC_BDCR_RTCSEL_SHIFT) /* 10: LSI oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_HSE            (3 << RCC_BDCR_RTCSEL_SHIFT) /* 11: HSE oscillator clock divided by 32 used as RTC clock */

#define RCC_BDCR_LSESYSRDY               (1 << 11) /* Bit 11: LSE system clock (LSESYS) ready */

#define RCC_BDCR_RTCEN                   (1 << 15)         /* Bit 15: RTC clock enable */
#define RCC_BDCR_BDRST                   (1 << 16)         /* Bit 16: Backup domain software reset */
#define RCC_BDCR_LSCOEN                  (1 << 24)         /* Bit 24: Low speed clock output enable */
#define RCC_BDCR_LSCOSEL                 (1 << 25)         /* Bit 25: Low speed clock output selection */
#  define RCC_BCDR_LSCOSEL_LSI           0                 /* LSI selected */
#  define RCC_BDCR_LSCOSEL_LSE           RCC_BDCR_LSCOSEL  /* LSE selected */

/* Control/status register */

#define RCC_CSR_LSION                    (1 << 0)  /* Bit 0: Internal Low Speed oscillator enable */
#define RCC_CSR_LSIRDY                   (1 << 1)  /* Bit 1: Internal Low Speed oscillator Ready */

#define RCC_CSR_LSIPRE                   (1 << 4)  /* Bit 4: Internal Low Speed oscillator prescaler (LSI/128) enable */

#define RCC_CSR_MSISRANGE_SHIFT          8
#  define RCC_CSR_MSISRANGE_MASK         (0x0F << RCC_CSR_MSISRANGE_SHIFT) /* MSI range after Standby mode */
#  define RCC_CSR_MSISRANGE_1M           (4    << RCC_CSR_MSISRANGE_SHIFT) /* 0100: around 1 MHz */
#  define RCC_CSR_MSISRANGE_2M           (5    << RCC_CSR_MSISRANGE_SHIFT) /* 0101: around 2 MHz */
#  define RCC_CSR_MSISRANGE_4M           (6    << RCC_CSR_MSISRANGE_SHIFT) /* 0110: around 4 MHz */
#  define RCC_CSR_MSISRANGE_8M           (7    << RCC_CSR_MSISRANGE_SHIFT) /* 0111: around 8 MHz */

#define RCC_CSR_RFRSTF                   (1 << 14) /* Bit 14: Radio in reset status flag */
#define RCC_CSR_RFRST                    (1 << 15) /* Bit 15: Radio reset */
#define RCC_CSR_RMVF                     (1 << 23) /* Bit 23: Remove reset flag */
#define RCC_CSR_RFILARSTF                (1 << 24) /* Bit 24: Radio illegal command flag */
#define RCC_CSR_OBLRSTF                  (1 << 25) /* Bit 25: Option byte loader reset flag */
#define RCC_CSR_PINRSTF                  (1 << 26) /* Bit 26: PIN reset flag */
#define RCC_CSR_BORRSTF                  (1 << 27) /* Bit 27: BOR reset flag */
#define RCC_CSR_SFTRSTF                  (1 << 28) /* Bit 28: Software Reset flag */
#define RCC_CSR_IWDGRSTF                 (1 << 29) /* Bit 29: Independent Watchdog reset flag */
#define RCC_CSR_WWDGRSTF                 (1 << 30) /* Bit 30: Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF                 (1 << 31) /* Bit 31: Low-Power reset flag */

/* Extended Clock recovery RC register */

#define RCC_EXTCFGR_SHDHPRE_SHIFT        (0)       /* Bits 0-3: HCLK3 shared prescaler (ahb3, flash, sram1/2) */
#define RCC_EXTCFGR_SHDHPRE_MASK         (0x0f << RCC_EXTCFGR_SHDHPRE_SHIFT)
#  define RCC_EXTCFGR_SHDHPRE_SYSCLK     ( 0 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* not-listed: SYSCLK not divided */
#  define RCC_EXTCFGR_SHDHPRE_SYSCLK_3   ( 1 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 0001: SYSCLK divided by 3 */
#  define RCC_EXTCFGR_SHDHPRE_SYSCLK_5   ( 2 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 0010: SYSCLK divided by 5 */
#  define RCC_EXTCFGR_SHDHPRE_SYSCLK_6   ( 5 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 0101: SYSCLK divided by 6 */
#  define RCC_EXTCFGR_SHDHPRE_SYSCLK_10  ( 6 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 0110: SYSCLK divided by 10 */
#  define RCC_EXTCFGR_SHDHPRE_SYSCLK_32  ( 7 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 0111: SYSCLK divided by 32 */
#  define RCC_EXTCFGR_SHDHPRE_SYSCLK_2   ( 8 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 1000: SYSCLK divided by 2 */
#  define RCC_EXTCFGR_SHDHPRE_SYSCLK_4   ( 9 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 1001: SYSCLK divided by 4 */
#  define RCC_EXTCFGR_SHDHPRE_SYSCLK_8   (10 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 1010: SYSCLK divided by 8 */
#  define RCC_EXTCFGR_SHDHPRE_SYSCLK_16  (11 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 1011: SYSCLK divided by 16 */
#  define RCC_EXTCFGR_SHDHPRE_SYSCLK_64  (12 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 1100: SYSCLK divided by 64 */
#  define RCC_EXTCFGR_SHDHPRE_SYSCLK_128 (13 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 1101: SYSCLK divided by 128 */
#  define RCC_EXTCFGR_SHDHPRE_SYSCLK_256 (14 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 1110: SYSCLK divided by 256 */
#  define RCC_EXTCFGR_SHDHPRE_SYSCLK_512 (15 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 1111: SYSCLK divided by 512 */

#define RCC_EXTCFGR_C2HPRE_SHIFT         (4)       /* Bits 4-7: HCLK2 prescaler (cpu2) */
#define RCC_EXTCFGR_C2HPRE_MASK          (0x0f << RCC_EXTCFGR_C2HPRE_SHIFT)
#  define RCC_EXTCFGR_C2HPRE_SYSCLK      ( 0 << RCC_EXTCFGR_C2HPRE_SHIFT) /* not-listed: SYSCLK not divided */
#  define RCC_EXTCFGR_C2HPRE_SYSCLK_3    ( 1 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 0001: SYSCLK divided by 3 */
#  define RCC_EXTCFGR_C2HPRE_SYSCLK_5    ( 2 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 0010: SYSCLK divided by 5 */
#  define RCC_EXTCFGR_C2HPRE_SYSCLK_6    ( 5 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 0101: SYSCLK divided by 6 */
#  define RCC_EXTCFGR_C2HPRE_SYSCLK_10   ( 6 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 0110: SYSCLK divided by 10 */
#  define RCC_EXTCFGR_C2HPRE_SYSCLK_32   ( 7 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 0111: SYSCLK divided by 32 */
#  define RCC_EXTCFGR_C2HPRE_SYSCLK_2    ( 8 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 1000: SYSCLK divided by 2 */
#  define RCC_EXTCFGR_C2HPRE_SYSCLK_4    ( 9 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 1001: SYSCLK divided by 4 */
#  define RCC_EXTCFGR_C2HPRE_SYSCLK_8    (10 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 1010: SYSCLK divided by 8 */
#  define RCC_EXTCFGR_C2HPRE_SYSCLK_16   (11 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 1011: SYSCLK divided by 16 */
#  define RCC_EXTCFGR_C2HPRE_SYSCLK_64   (12 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 1100: SYSCLK divided by 64 */
#  define RCC_EXTCFGR_C2HPRE_SYSCLK_128  (13 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 1101: SYSCLK divided by 128 */
#  define RCC_EXTCFGR_C2HPRE_SYSCLK_256  (14 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 1110: SYSCLK divided by 256 */
#  define RCC_EXTCFGR_C2HPRE_SYSCLK_512  (15 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 1111: SYSCLK divided by 512 */

#define RCC_EXTCFGR_SHDHPREF             ( 1 << 16) /* Bit 16: Apply HCLK3 prescaller */
#define RCC_EXTCFGR_C2HPREF              ( 1 << 17) /* Bit 17: Apply HCLK2 prescaller (cpu2) */

#endif /* __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL562XX_RCC_H */
