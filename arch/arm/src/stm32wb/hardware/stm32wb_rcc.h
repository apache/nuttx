/****************************************************************************
 * arch/arm/src/stm32wb/hardware/stm32wb_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_RCC_H
#define __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32WB_RCC_CR_OFFSET           0x0000  /* Clock control register */
#define STM32WB_RCC_ICSCR_OFFSET        0x0004  /* Internal clock sources calibration register */
#define STM32WB_RCC_CFGR_OFFSET         0x0008  /* Clock configuration register */
#define STM32WB_RCC_PLLCFG_OFFSET       0x000c  /* PLL configuration register */
#define STM32WB_RCC_PLLSAI1CFG_OFFSET   0x0010  /* PLLSAI1 configuration register */
#define STM32WB_RCC_CIER_OFFSET         0x0018  /* Clock interrupt enable register */
#define STM32WB_RCC_CIFR_OFFSET         0x001c  /* Clock interrupt flag register */
#define STM32WB_RCC_CICR_OFFSET         0x0020  /* Clock interrupt clear register */
#define STM32WB_RCC_SMPSCR_OFFSET       0x0024  /* Step-down converter control register */
#define STM32WB_RCC_AHB1RSTR_OFFSET     0x0028  /* AHB1 peripheral reset register */
#define STM32WB_RCC_AHB2RSTR_OFFSET     0x002c  /* AHB2 peripheral reset register */
#define STM32WB_RCC_AHB3RSTR_OFFSET     0x0030  /* AHB3 peripheral reset register */
#define STM32WB_RCC_APB1RSTR1_OFFSET    0x0038  /* APB1 Peripheral reset register 1 */
#define STM32WB_RCC_APB1RSTR2_OFFSET    0x003c  /* APB1 Peripheral reset register 2 */
#define STM32WB_RCC_APB2RSTR_OFFSET     0x0040  /* APB2 Peripheral reset register */
#define STM32WB_RCC_APB3RSTR_OFFSET     0x0044  /* APB3 Peripheral reset register */
#define STM32WB_RCC_AHB1ENR_OFFSET      0x0048  /* AHB1 Peripheral Clock enable register */
#define STM32WB_RCC_AHB2ENR_OFFSET      0x004c  /* AHB2 Peripheral Clock enable register */
#define STM32WB_RCC_AHB3ENR_OFFSET      0x0050  /* AHB3 Peripheral Clock enable register */
#define STM32WB_RCC_APB1ENR1_OFFSET     0x0058  /* APB1 Peripheral Clock enable register 1 */
#define STM32WB_RCC_APB1ENR2_OFFSET     0x005c  /* APB1 Peripheral Clock enable register 2 */
#define STM32WB_RCC_APB2ENR_OFFSET      0x0060  /* APB2 Peripheral Clock enable register */
#define STM32WB_RCC_AHB1SMENR_OFFSET    0x0068  /* AHB1 clock enable in sleep and stop modes register */
#define STM32WB_RCC_AHB2SMENR_OFFSET    0x006c  /* AHB2 clock enable in sleep and stop modes register */
#define STM32WB_RCC_AHB3SMENR_OFFSET    0x0070  /* AHB3 clock enable in sleep and stop modes register */
#define STM32WB_RCC_APB1SMENR1_OFFSET   0x0078  /* APB1 clock enable in sleep and stop modes register 1 */
#define STM32WB_RCC_APB1SMENR2_OFFSET   0x007c  /* APB1 clock enable in sleep and stop modes register 2 */
#define STM32WB_RCC_APB2SMENR_OFFSET    0x0080  /* APB2 clock enable in sleep and stop modes register */
#define STM32WB_RCC_CCIPR_OFFSET        0x0088  /* Peripherals independent clock configuration register */
#define STM32WB_RCC_BDCR_OFFSET         0x0090  /* Backup domain control register */
#define STM32WB_RCC_CSR_OFFSET          0x0094  /* Control/status register */
#define STM32WB_RCC_CRRCR_OFFSET        0x0098  /* Clock recovery RC register */
#define STM32WB_RCC_HSECR_OFFSET        0x009c  /* Clock HSE register */
#define STM32WB_RCC_EXTCFGR_OFFSET      0x0108  /* Extended clock recovery register */
#define STM32WB_RCC_C2AHB1ENR_OFFSET    0x0148  /* CPU2 AHB1 Peripheral Clock enable register */
#define STM32WB_RCC_C2AHB2ENR_OFFSET    0x014c  /* CPU2 AHB2 Peripheral Clock enable register */
#define STM32WB_RCC_C2AHB3ENR_OFFSET    0x0150  /* CPU2 AHB3 Peripheral Clock enable register */
#define STM32WB_RCC_C2APB1ENR1_OFFSET   0x0158  /* CPU2 APB1 Peripheral Clock enable register 1 */
#define STM32WB_RCC_C2APB1ENR2_OFFSET   0x015c  /* CPU2 APB1 Peripheral Clock enable register 2 */
#define STM32WB_RCC_C2APB2ENR_OFFSET    0x0160  /* CPU2 APB2 Peripheral Clock enable register */
#define STM32WB_RCC_C2APB3ENR_OFFSET    0x0164  /* CPU2 APB3 Peripheral Clock enable register */
#define STM32WB_RCC_C2AHB1SMENR_OFFSET  0x0168  /* CPU2 AHB1 clock enable in sleep and stop modes register */
#define STM32WB_RCC_C2AHB2SMENR_OFFSET  0x016c  /* CPU2 AHB2 clock enable in sleep and stop modes register */
#define STM32WB_RCC_C2AHB3SMENR_OFFSET  0x0170  /* CPU2 AHB3 clock enable in sleep and stop modes register */
#define STM32WB_RCC_C2APB1SMENR1_OFFSET 0x0178  /* CPU2 APB1 clock enable in sleep and stop modes register 1 */
#define STM32WB_RCC_C2APB1SMENR2_OFFSET 0x017c  /* CPU2 APB1 clock enable in sleep and stop modes register 2 */
#define STM32WB_RCC_C2APB2SMENR_OFFSET  0x0180  /* CPU2 APB2 clock enable in sleep and stop modes register */
#define STM32WB_RCC_C2APB3SMENR_OFFSET  0x0184  /* CPU2 APB3 clock enable in sleep and stop modes register */

/* Register Addresses *******************************************************/

#define STM32WB_RCC_CR                (STM32WB_RCC_BASE + STM32WB_RCC_CR_OFFSET)
#define STM32WB_RCC_ICSCR             (STM32WB_RCC_BASE + STM32WB_RCC_ICSCR_OFFSET)
#define STM32WB_RCC_CFGR              (STM32WB_RCC_BASE + STM32WB_RCC_CFGR_OFFSET)
#define STM32WB_RCC_PLLCFG            (STM32WB_RCC_BASE + STM32WB_RCC_PLLCFG_OFFSET)
#define STM32WB_RCC_PLLSAI1CFG        (STM32WB_RCC_BASE + STM32WB_RCC_PLLSAI1CFG_OFFSET)
#define STM32WB_RCC_CIER              (STM32WB_RCC_BASE + STM32WB_RCC_CIER_OFFSET)
#define STM32WB_RCC_CIFR              (STM32WB_RCC_BASE + STM32WB_RCC_CIFR_OFFSET)
#define STM32WB_RCC_CICR              (STM32WB_RCC_BASE + STM32WB_RCC_CICR_OFFSET)
#define STM32WB_RCC_SMPSCR            (STM32WB_RCC_BASE + STM32WB_RCC_SMPSCR_OFFSET)
#define STM32WB_RCC_AHB1RSTR          (STM32WB_RCC_BASE + STM32WB_RCC_AHB1RSTR_OFFSET)
#define STM32WB_RCC_AHB2RSTR          (STM32WB_RCC_BASE + STM32WB_RCC_AHB2RSTR_OFFSET)
#define STM32WB_RCC_AHB3RSTR          (STM32WB_RCC_BASE + STM32WB_RCC_AHB3RSTR_OFFSET)
#define STM32WB_RCC_APB1RSTR1         (STM32WB_RCC_BASE + STM32WB_RCC_APB1RSTR1_OFFSET)
#define STM32WB_RCC_APB1RSTR2         (STM32WB_RCC_BASE + STM32WB_RCC_APB1RSTR2_OFFSET)
#define STM32WB_RCC_APB2RSTR          (STM32WB_RCC_BASE + STM32WB_RCC_APB2RSTR_OFFSET)
#define STM32WB_RCC_APB3RSTR          (STM32WB_RCC_BASE + STM32WB_RCC_APB3RSTR_OFFSET)
#define STM32WB_RCC_AHB1ENR           (STM32WB_RCC_BASE + STM32WB_RCC_AHB1ENR_OFFSET)
#define STM32WB_RCC_AHB2ENR           (STM32WB_RCC_BASE + STM32WB_RCC_AHB2ENR_OFFSET)
#define STM32WB_RCC_AHB3ENR           (STM32WB_RCC_BASE + STM32WB_RCC_AHB3ENR_OFFSET)
#define STM32WB_RCC_APB1ENR1          (STM32WB_RCC_BASE + STM32WB_RCC_APB1ENR1_OFFSET)
#define STM32WB_RCC_APB1ENR2          (STM32WB_RCC_BASE + STM32WB_RCC_APB1ENR2_OFFSET)
#define STM32WB_RCC_APB2ENR           (STM32WB_RCC_BASE + STM32WB_RCC_APB2ENR_OFFSET)
#define STM32WB_RCC_AHB1SMENR         (STM32WB_RCC_BASE + STM32WB_RCC_AHB1SMENR_OFFSET)
#define STM32WB_RCC_AHB2SMENR         (STM32WB_RCC_BASE + STM32WB_RCC_AHB2SMENR_OFFSET)
#define STM32WB_RCC_AHB3SMENR         (STM32WB_RCC_BASE + STM32WB_RCC_AHB3SMENR_OFFSET)
#define STM32WB_RCC_APB1SMENR1        (STM32WB_RCC_BASE + STM32WB_RCC_APB1SMENR1_OFFSET)
#define STM32WB_RCC_APB1SMENR2        (STM32WB_RCC_BASE + STM32WB_RCC_APB1SMENR2_OFFSET)
#define STM32WB_RCC_APB2SMENR         (STM32WB_RCC_BASE + STM32WB_RCC_APB2SMENR_OFFSET)
#define STM32WB_RCC_CCIPR             (STM32WB_RCC_BASE + STM32WB_RCC_CCIPR_OFFSET)
#define STM32WB_RCC_BDCR              (STM32WB_RCC_BASE + STM32WB_RCC_BDCR_OFFSET)
#define STM32WB_RCC_CSR               (STM32WB_RCC_BASE + STM32WB_RCC_CSR_OFFSET)
#define STM32WB_RCC_CRRCR             (STM32WB_RCC_BASE + STM32WB_RCC_CRRCR_OFFSET)
#define STM32WB_RCC_HSECR             (STM32WB_RCC_BASE + STM32WB_RCC_HSECR_OFFSET)
#define STM32WB_RCC_EXTCFGR           (STM32WB_RCC_BASE + STM32WB_RCC_EXTCFGR_OFFSET)
#define STM32WB_RCC_C2AHB1ENR         (STM32WB_RCC_BASE + STM32WB_RCC_C2AHB1ENR_OFFSET)
#define STM32WB_RCC_C2AHB2ENR         (STM32WB_RCC_BASE + STM32WB_RCC_C2AHB2ENR_OFFSET)
#define STM32WB_RCC_C2AHB3ENR         (STM32WB_RCC_BASE + STM32WB_RCC_C2AHB3ENR_OFFSET)
#define STM32WB_RCC_C2APB1ENR1        (STM32WB_RCC_BASE + STM32WB_RCC_C2APB1ENR1_OFFSET)
#define STM32WB_RCC_C2APB1ENR2        (STM32WB_RCC_BASE + STM32WB_RCC_C2APB1ENR2_OFFSET)
#define STM32WB_RCC_C2APB2ENR         (STM32WB_RCC_BASE + STM32WB_RCC_C2APB2ENR_OFFSET)
#define STM32WB_RCC_C2APB3ENR         (STM32WB_RCC_BASE + STM32WB_RCC_C2APB3ENR_OFFSET)
#define STM32WB_RCC_C2AHB1SMENR       (STM32WB_RCC_BASE + STM32WB_RCC_C2AHB1SMENR_OFFSET)
#define STM32WB_RCC_C2AHB2SMENR       (STM32WB_RCC_BASE + STM32WB_RCC_C2AHB2SMENR_OFFSET)
#define STM32WB_RCC_C2AHB3SMENR       (STM32WB_RCC_BASE + STM32WB_RCC_C2AHB3SMENR_OFFSET)
#define STM32WB_RCC_C2APB1SMENR1      (STM32WB_RCC_BASE + STM32WB_RCC_C2APB1SMENR1_OFFSET)
#define STM32WB_RCC_C2APB1SMENR2      (STM32WB_RCC_BASE + STM32WB_RCC_C2APB1SMENR2_OFFSET)
#define STM32WB_RCC_C2APB2SMENR       (STM32WB_RCC_BASE + STM32WB_RCC_C2APB2SMENR_OFFSET)
#define STM32WB_RCC_C2APB3SMENR       (STM32WB_RCC_BASE + STM32WB_RCC_C2APB3SMENR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Clock control register */

#define RCC_CR_MSION                  (1 << 0)  /* Bit 0: Internal Multi Speed clock enable */
#define RCC_CR_MSIRDY                 (1 << 1)  /* Bit 1: Internal Multi Speed clock ready flag */
#define RCC_CR_MSIPLLEN               (1 << 2)  /* Bit 2: MSI clock PLL enable */
#define RCC_CR_MSIRANGE_SHIFT         (4)       /* Bits 7-4: MSI clock range */
#define RCC_CR_MSIRANGE_MASK          (0x0f << RCC_CR_MSIRANGE_SHIFT)
#  define RCC_CR_MSIRANGE_100K        (0  << RCC_CR_MSIRANGE_SHIFT) /* 0000: around 100 kHz */
#  define RCC_CR_MSIRANGE_200K        (1  << RCC_CR_MSIRANGE_SHIFT) /* 0001: around 200 kHz */
#  define RCC_CR_MSIRANGE_400K        (2  << RCC_CR_MSIRANGE_SHIFT) /* 0010: around 400 kHz */
#  define RCC_CR_MSIRANGE_800K        (3  << RCC_CR_MSIRANGE_SHIFT) /* 0011: around 800 kHz */
#  define RCC_CR_MSIRANGE_1M          (4  << RCC_CR_MSIRANGE_SHIFT) /* 0100: around 1 MHz */
#  define RCC_CR_MSIRANGE_2M          (5  << RCC_CR_MSIRANGE_SHIFT) /* 0101: around 2 MHz */
#  define RCC_CR_MSIRANGE_4M          (6  << RCC_CR_MSIRANGE_SHIFT) /* 0110: around 4 MHz */
#  define RCC_CR_MSIRANGE_8M          (7  << RCC_CR_MSIRANGE_SHIFT) /* 0111: around 8 MHz */
#  define RCC_CR_MSIRANGE_16M         (8  << RCC_CR_MSIRANGE_SHIFT) /* 1000: around 16 MHz */
#  define RCC_CR_MSIRANGE_24M         (9  << RCC_CR_MSIRANGE_SHIFT) /* 1001: around 24 MHz */
#  define RCC_CR_MSIRANGE_32M         (10 << RCC_CR_MSIRANGE_SHIFT) /* 1010: around 32 MHz */
#  define RCC_CR_MSIRANGE_48M         (11 << RCC_CR_MSIRANGE_SHIFT) /* 1011: around 48 MHz */

#define RCC_CR_HSION                  (1 << 8)  /* Bit 8:  HSI16 clock enable */
#define RCC_CR_HSIKERON               (1 << 9)  /* Bit 9:  HSI16 always enable for peripheral kernels */
#define RCC_CR_HSIRDY                 (1 << 10) /* Bit 10: HSI16 clock ready flag */
#define RCC_CR_HSIASFS                (1 << 11) /* Bit 11: HSI16 automatic start from stop */
#define RCC_CR_HSEON                  (1 << 16) /* Bit 16: HSE clock enable */
#define RCC_CR_HSERDY                 (1 << 17) /* Bit 17: HSE clock ready flag */
#define RCC_CR_CSSON                  (1 << 19) /* Bit 19: Clock Security System enable */
#define RCC_CR_HSEPRE                 (1 << 20) /* Bit 20: HSE sysclk and PLL M divider prescaler */
#define RCC_CR_PLLON                  (1 << 24) /* Bit 24: PLL enable */
#define RCC_CR_PLLRDY                 (1 << 25) /* Bit 25: PLL clock ready flag */
#define RCC_CR_PLLSAI1ON              (1 << 26) /* Bit 26: PLLSAI1 enable */
#define RCC_CR_PLLSAI1RDY             (1 << 27) /* Bit 27: PLLSAI1 clock ready flag */

/* Internal Clock Sources Calibration */

#define RCC_CR_HSITRIM_SHIFT          (24)      /* Bits 30-24: HSI16 clock trimming */
#define RCC_CR_HSITRIM_MASK           (0x7f << RCC_CR_HSITRIM_SHIFT)
#define RCC_CR_HSICAL_SHIFT           (16)      /* Bits 23-16: HSI16 clock Calibration */
#define RCC_CR_HSICAL_MASK            (0xff << RCC_CR_HSICAL_SHIFT)
#define RCC_CR_MSITRIM_SHIFT          (8)       /* Bits 15-8:  Internal Multi Speed clock trimming */
#define RCC_CR_MSITRIM_MASK           (0xff << RCC_CR_MSITRIM_SHIFT)
#define RCC_CR_MSICAL_SHIFT           (0)       /* Bits 7-0:   Internal Multi Speed clock Calibration */
#define RCC_CR_MSICAL_MASK            (0xff << RCC_CR_MSICAL_SHIFT)

/* Clock configuration register */

#define RCC_CFGR_SW_SHIFT             (0)       /* Bits 0-1: System clock Switch */
#define RCC_CFGR_SW_MASK              (0x3 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_MSI             (0x0 << RCC_CFGR_SW_SHIFT) /* 00: MSI selected as system clock */
#  define RCC_CFGR_SW_HSI16           (0x1 << RCC_CFGR_SW_SHIFT) /* 00: HSI16 selected as system clock */
#  define RCC_CFGR_SW_HSE             (0x2 << RCC_CFGR_SW_SHIFT) /* 01: HSE selected as system clock */
#  define RCC_CFGR_SW_PLL             (0x3 << RCC_CFGR_SW_SHIFT) /* 10: PLL selected as system clock */

#define RCC_CFGR_SWS_SHIFT            (2)       /* Bits 2-3: System Clock Switch Status */
#define RCC_CFGR_SWS_MASK             (0x3 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_MSI            (0x0 << RCC_CFGR_SWS_SHIFT) /* 00: MSI oscillator used as system clock */
#  define RCC_CFGR_SWS_HSI16          (0x1 << RCC_CFGR_SWS_SHIFT) /* 00: HSI16 oscillator used as system clock */
#  define RCC_CFGR_SWS_HSE            (0x2 << RCC_CFGR_SWS_SHIFT) /* 01: HSE oscillator used as system clock */
#  define RCC_CFGR_SWS_PLL            (0x3 << RCC_CFGR_SWS_SHIFT) /* 10: PLL used as system clock */

#define RCC_CFGR_HPRE_SHIFT           (4)       /* Bits 4-7: HCLK1 prescaler (AHB1, AHB2, AHB3 and SRAM1) */
#define RCC_CFGR_HPRE_MASK            (0x0f << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLK        (0x00 << RCC_CFGR_HPRE_SHIFT) /* 0000: SYSCLK not divided */
#  define RCC_CFGR_HPRE_SYSCLKd3      (0x01 << RCC_CFGR_HPRE_SHIFT) /* 0001: SYSCLK divided by 3 */
#  define RCC_CFGR_HPRE_SYSCLKd5      (0x02 << RCC_CFGR_HPRE_SHIFT) /* 0010: SYSCLK divided by 5 */
#  define RCC_CFGR_HPRE_SYSCLKd6      (0x05 << RCC_CFGR_HPRE_SHIFT) /* 0101: SYSCLK divided by 6 */
#  define RCC_CFGR_HPRE_SYSCLKd10     (0x06 << RCC_CFGR_HPRE_SHIFT) /* 0110: SYSCLK divided by 10 */
#  define RCC_CFGR_HPRE_SYSCLKd32     (0x07 << RCC_CFGR_HPRE_SHIFT) /* 0111: SYSCLK divided by 32 */
#  define RCC_CFGR_HPRE_SYSCLKd2      (0x08 << RCC_CFGR_HPRE_SHIFT) /* 1000: SYSCLK divided by 2 */
#  define RCC_CFGR_HPRE_SYSCLKd4      (0x09 << RCC_CFGR_HPRE_SHIFT) /* 1001: SYSCLK divided by 4 */
#  define RCC_CFGR_HPRE_SYSCLKd8      (0x10 << RCC_CFGR_HPRE_SHIFT) /* 1010: SYSCLK divided by 8 */
#  define RCC_CFGR_HPRE_SYSCLKd16     (0x11 << RCC_CFGR_HPRE_SHIFT) /* 1011: SYSCLK divided by 16 */
#  define RCC_CFGR_HPRE_SYSCLKd64     (0x12 << RCC_CFGR_HPRE_SHIFT) /* 1100: SYSCLK divided by 64 */
#  define RCC_CFGR_HPRE_SYSCLKd128    (0x13 << RCC_CFGR_HPRE_SHIFT) /* 1101: SYSCLK divided by 128 */
#  define RCC_CFGR_HPRE_SYSCLKd256    (0x14 << RCC_CFGR_HPRE_SHIFT) /* 1110: SYSCLK divided by 256 */
#  define RCC_CFGR_HPRE_SYSCLKd512    (0x15 << RCC_CFGR_HPRE_SHIFT) /* 1111: SYSCLK divided by 512 */

#define RCC_CFGR_PPRE1_SHIFT          (8)       /* Bits 8-10: PCLK1 Low speed prescaler (APB1) */
#define RCC_CFGR_PPRE1_MASK           (0x7 << RCC_CFGR_PPRE1_SHIFT)
#  define RCC_CFGR_PPRE1_HCLK1        (0x0 << RCC_CFGR_PPRE1_SHIFT) /* 0xx: HCLK1 not divided */
#  define RCC_CFGR_PPRE1_HCLK1d2      (0x4 << RCC_CFGR_PPRE1_SHIFT) /* 100: HCLK1 divided by 2 */
#  define RCC_CFGR_PPRE1_HCLK1d4      (0x5 << RCC_CFGR_PPRE1_SHIFT) /* 101: HCLK1 divided by 4 */
#  define RCC_CFGR_PPRE1_HCLK1d8      (0x6 << RCC_CFGR_PPRE1_SHIFT) /* 110: HCLK1 divided by 8 */
#  define RCC_CFGR_PPRE1_HCLK1d16     (0x7 << RCC_CFGR_PPRE1_SHIFT) /* 111: HCLK1 divided by 16 */

#define RCC_CFGR_PPRE2_SHIFT          (11)      /* Bits 11-13: PCLK2 High speed prescaler (APB2) */
#define RCC_CFGR_PPRE2_MASK           (0x7 << RCC_CFGR_PPRE2_SHIFT)
#  define RCC_CFGR_PPRE2_HCLK1        (0x0 << RCC_CFGR_PPRE2_SHIFT) /* 0xx: HCLK1 not divided */
#  define RCC_CFGR_PPRE2_HCLK1d2      (0x4 << RCC_CFGR_PPRE2_SHIFT) /* 100: HCLK1 divided by 2 */
#  define RCC_CFGR_PPRE2_HCLK1d4      (0x5 << RCC_CFGR_PPRE2_SHIFT) /* 101: HCLK1 divided by 4 */
#  define RCC_CFGR_PPRE2_HCLK1d8      (0x6 << RCC_CFGR_PPRE2_SHIFT) /* 110: HCLK1 divided by 8 */
#  define RCC_CFGR_PPRE2_HCLK1d16     (0x7 << RCC_CFGR_PPRE2_SHIFT) /* 111: HCLK1 divided by 16 */

#define RCC_CFGR_STOPWUCK             (1 << 15) /* Bit 15: Wakeup from Stop and CSS backup clock selection */
#  define RCC_CFGR_STOPWUCK_MSI       (0 << 15) /* 0: MSI */
#  define RCC_CFGR_STOPWUCK_HSI16     (1 << 15) /* 0: HSI16 */

#define RCC_CFGR_HPREF                (1 << 16) /* Bit 16: HCLK1 prescaler flag (AHB1, AHB2, AHB3, SRAM1) */
#define RCC_CFGR_PPRE1F               (1 << 17) /* Bit 17: PCLK1 prescaler flag (APB1) */
#define RCC_CFGR_PPRE2F               (1 << 18) /* Bit 18: PCLK2 prescaler flag (APB2) */

#define RCC_CFGR_MCOSEL_SHIFT         (24)      /* Bits 24-27: Microcontroller Clock Output */
#define RCC_CFGR_MCOSEL_MASK          (0xf << RCC_CFGR_MCOSEL_SHIFT)
#  define RCC_CFGR_MCOSEL_DISABLED    (0x0 << RCC_CFGR_MCOSEL_SHIFT) /* 0000: Output disabled, no clock on MCO */
#  define RCC_CFGR_MCOSEL_SYSCLK      (0x1 << RCC_CFGR_MCOSEL_SHIFT) /* 0001: SYSCLK system clock selected */
#  define RCC_CFGR_MCOSEL_MSI         (0x2 << RCC_CFGR_MCOSEL_SHIFT) /* 0010: MSI clock selected */
#  define RCC_CFGR_MCOSEL_HSI16       (0x3 << RCC_CFGR_MCOSEL_SHIFT) /* 0011: HSI16 clock selected */
#  define RCC_CFGR_MCOSEL_HSE_AFT     (0x4 << RCC_CFGR_MCOSEL_SHIFT) /* 0100: HSE clock after stabilization */
#  define RCC_CFGR_MCOSEL_PLL         (0x5 << RCC_CFGR_MCOSEL_SHIFT) /* 0101: Main PLLRCLK selected  */
#  define RCC_CFGR_MCOSEL_LSI1        (0x6 << RCC_CFGR_MCOSEL_SHIFT) /* 0110: LSI1 clock selected */
#  define RCC_CFGR_MCOSEL_LSI2        (0x7 << RCC_CFGR_MCOSEL_SHIFT) /* 0111: LSI2 clock selected */
#  define RCC_CFGR_MCOSEL_LSE         (0x8 << RCC_CFGR_MCOSEL_SHIFT) /* 1000: LSE clock selected */
#  define RCC_CFGR_MCOSEL_HSI48       (0x9 << RCC_CFGR_MCOSEL_SHIFT) /* 1001: HSI48 clock selected */
#  define RCC_CFGR_MCOSEL_HSE_BFR     (0xc << RCC_CFGR_MCOSEL_SHIFT) /* 1100: HSE clock before stabilization */

#define RCC_CFGR_MCOPRE_SHIFT         (28)      /* Bits 28-30: MCO prescaler */
#define RCC_CFGR_MCOPRE_MASK          (0x7 << RCC_CFGR_MCOPRE_SHIFT)
#  define RCC_CFGR_MCOPRE_DIV1        (0x0 << RCC_CFGR_MCOPRE_SHIFT) /* 000: no division */
#  define RCC_CFGR_MCOPRE_DIV2        (0x1 << RCC_CFGR_MCOPRE_SHIFT) /* 001: division by 2 */
#  define RCC_CFGR_MCOPRE_DIV4        (0x2 << RCC_CFGR_MCOPRE_SHIFT) /* 010: division by 4 */
#  define RCC_CFGR_MCOPRE_DIV8        (0x3 << RCC_CFGR_MCOPRE_SHIFT) /* 011: division by 8 */
#  define RCC_CFGR_MCOPRE_DIV16       (0x4 << RCC_CFGR_MCOPRE_SHIFT) /* 100: division by 16 */

#define RCC_CFGR_RESET_MASK           (0x00070000)

/* PLL configuration register */

#define RCC_PLLCFG_PLLSRC_SHIFT       (0)       /* Bits 0-1: Main PLL and audio PLLSAI1 entry clock source */
#define RCC_PLLCFG_PLLSRC_MASK        (0x3 << RCC_PLLCFG_PLLSRC_SHIFT)
#  define RCC_PLLCFG_PLLSRC_NONE      (0x0 << RCC_PLLCFG_PLLSRC_SHIFT) /* 00: No clock sent to PLLs */
#  define RCC_PLLCFG_PLLSRC_MSI       (0x1 << RCC_PLLCFG_PLLSRC_SHIFT) /* 01: MSI selected as PLLs source */
#  define RCC_PLLCFG_PLLSRC_HSI16     (0x2 << RCC_PLLCFG_PLLSRC_SHIFT) /* 10: HSI16 selected as PLLs source */
#  define RCC_PLLCFG_PLLSRC_HSE       (0x3 << RCC_PLLCFG_PLLSRC_SHIFT) /* 11: HSE selected as PLLs source */

#define RCC_PLLCFG_PLLM_SHIFT         (4)       /* Bits 4-6: Main PLL and audio PLLSAI1 divider */
#define RCC_PLLCFG_PLLM_MASK          (0x07 << RCC_PLLCFG_PLLM_SHIFT)
#  define RCC_PLLCFG_PLLM(n)          (((n) - 1) << RCC_PLLCFG_PLLM_SHIFT) /* n = 1..8 */

#define RCC_PLLCFG_PLLN_SHIFT         (8)       /* Bits 8-14: Main PLL (PLL) VCO multiplier */
#define RCC_PLLCFG_PLLN_MASK          (0x7f << RCC_PLLCFG_PLLN_SHIFT)
#  define RCC_PLLCFG_PLLN(n)          ((n) << RCC_PLLCFG_PLLN_SHIFT) /* n = 6..127 */

#define RCC_PLLCFG_PLLPEN             (1 << 16) /* Bit 16: Main PLL PLLPCLK output enable */

#define RCC_PLLCFG_PLLP_SHIFT         (17)      /* Bits 17-21: Main PLL div factor for PLLPCLK */
#define RCC_PLLCFG_PLLP_MASK          (0x1f << RCC_PLLCFG_PLLP_SHIFT)
#  define RCC_PLLCFG_PLLP(n)          (((n) - 1) << RCC_PLLCFG_PLLP_SHIFT) /* n = 2..32 */

#define RCC_PLLCFG_PLLQEN             (1 << 24) /* Bit 24: Main PLL PLLQCLK output enable */

#define RCC_PLLCFG_PLLQ_SHIFT         (25)      /* Bits 25-27: Main PLL division factor for PLLQCLK */
#define RCC_PLLCFG_PLLQ_MASK          (0x7 << RCC_PLLCFG_PLLQ_SHIFT)
#  define RCC_PLLCFG_PLLQ(n)          (((n) - 1) << RCC_PLLCFG_PLLQ_SHIFT) /* n = 2..8 */

#define RCC_PLLCFG_PLLREN             (1 << 28) /* Bit 28: Main PLL PLLRCLK output enable */

#define RCC_PLLCFG_PLLR_SHIFT         (29)      /* Bits 29-31: Main PLL division factor for PLLRCLK */
#define RCC_PLLCFG_PLLR_MASK          (0x7 << RCC_PLLCFG_PLLR_SHIFT)
#  define RCC_PLLCFG_PLLR(n)          (((n) - 1) << RCC_PLLCFG_PLLR_SHIFT) /* n = 2..8 */

#define RCC_PLLCFG_RESET              (0x22040100) /* PLLCFG reset value */

/* PLLSAI1 Configuration register */

#define RCC_PLLSAI1CFG_PLLN_SHIFT     (8)       /* Bits 8-14: SAI1 PLL (PLLSAI1) VCO multiplier */
#define RCC_PLLSAI1CFG_PLLN_MASK      (0x7f << RCC_PLLSAI1CFG_PLLN_SHIFT)
#  define RCC_PLLSAI1CFG_PLLN(n)      ((n) << RCC_PLLSAI1CFG_PLLN_SHIFT) /* n = 4..86 */

#define RCC_PLLSAI1CFG_PLLPEN         (1 << 16) /* Bit 16: SAI1 PLL PLLSAI1CLK output enable */

#define RCC_PLLSAI1CFG_PLLP_SHIFT     (17)      /* Bit 17-21: Main PLL div factor for PLLSAI1CLK */
#define RCC_PLLSAI1CFG_PLLP_MASK      (0x1f << RCC_PLLSAI1CFG_PLLP_SHIFT)
#  define RCC_PLLSAI1CFG_PLLP(n)      (((n) - 1) << RCC_PLLSAI1CFG_PLLP_SHIFT) /* n = 2..32 */

#define RCC_PLLSAI1CFG_PLLQEN         (1 << 24) /* Bit 24: PLLSAI1QCLK output enable */

#define RCC_PLLSAI1CFG_PLLQ_SHIFT     (25)      /* Bits 25-27: PLLSAI1 division factor for PLLSAI1QCLK */
#define RCC_PLLSAI1CFG_PLLQ_MASK      (0x3 << RCC_PLLSAI1CFG_PLLQ_SHIFT)
#  define RCC_PLLSAI1CFG_PLLQ(n)      (((n) - 1) << RCC_PLLSAI1CFG_PLLQ_SHIFT) /* n = 2..8 */

#define RCC_PLLSAI1CFG_PLLREN         (1 << 28) /* Bit 28: SAI1 PLL PLLSAI1RCLK output enable */

#define RCC_PLLSAI1CFG_PLLR_SHIFT     (29)      /* Bits 29-31: PLLSAI1 division factor for PLLSAI1RCLK */
#define RCC_PLLSAI1CFG_PLLR_MASK      (0x3 << RCC_PLLSAI1CFG_PLLR_SHIFT)
#  define RCC_PLLSAI1CFG_PLLR(n)      (((n) - 1) << RCC_PLLSAI1CFG_PLLR_SHIFT) /* n = 2..8 */

#define RCC_PLLSAI1CFG_RESET          (0x22040100) /* PLLSAI1CFG reset value */

/* Clock interrupt enable register */

#define RCC_CIER_LSI1RDYIE            (1 << 0)  /* Bit 0: LSI1 Ready Interrupt Enable */
#define RCC_CIER_LSERDYIE             (1 << 1)  /* Bit 1: LSE Ready Interrupt Enable */
#define RCC_CIER_MSIRDYIE             (1 << 2)  /* Bit 2: MSI Ready Interrupt Enable */
#define RCC_CIER_HSI16RDYIE           (1 << 3)  /* Bit 3: HSI16 Ready Interrupt Enable */
#define RCC_CIER_HSERDYIE             (1 << 4)  /* Bit 4: HSE Ready Interrupt Enable */
#define RCC_CIER_PLLRDYIE             (1 << 5)  /* Bit 5: PLL Ready Interrupt Enable */
#define RCC_CIER_PLLSAI1RDYIE         (1 << 6)  /* Bit 6: PLLSAI1 Ready Interrupt enable */
#define RCC_CIER_LSECSSIE             (1 << 9)  /* Bit 9: LSE Clock Security System Interrupt Enable */
#define RCC_CIER_HSI48RDYIE           (1 << 10) /* Bit 10: HSI48 Ready Interrupt Enable */
#define RCC_CIER_LSI2RDYIE            (1 << 11) /* Bit 11: LSI2 Ready Interrupt Enable */

/* Clock interrupt flag register */

#define RCC_CIFR_LSI1RDYIF            (1 << 0)  /* Bit 0:  LSI1 Ready Interrupt Flag */
#define RCC_CIFR_LSERDYIF             (1 << 1)  /* Bit 1:  LSE Ready Interrupt Flag */
#define RCC_CIFR_MSIRDYIF             (1 << 2)  /* Bit 2:  MSI Ready Interrupt Flag */
#define RCC_CIFR_HSI16RDYIF           (1 << 3)  /* Bit 3:  HSI16 Ready Interrupt Flag */
#define RCC_CIFR_HSERDYIF             (1 << 4)  /* Bit 4:  HSE Ready Interrupt Flag */
#define RCC_CIFR_PLLRDYIF             (1 << 5)  /* Bit 5:  PLL Ready Interrupt Flag */
#define RCC_CIFR_PLLSAI1RDYIF         (1 << 6)  /* Bit 6:  PLLSAI1 Ready Interrupt Flag */
#define RCC_CIFR_CSSF                 (1 << 8)  /* Bit 8:  Clock Security System Interrupt Flag */
#define RCC_CIFR_LSECSSIF             (1 << 9)  /* Bit 9:  LSE Clock Security System Interrupt Flag */
#define RCC_CIFR_HSI48RDYIF           (1 << 10) /* Bit 10: HSI48 Ready Interrupt Flag */
#define RCC_CIFR_LSI2RDYIF            (1 << 11) /* Bit 11: LSI2 Ready Interrupt Flag */

/* Clock interrupt clear register */

#define RCC_CICR_LSI1RDYIC            (1 << 0)  /* Bit 0:  LSI1 Ready Interrupt Clear */
#define RCC_CICR_LSERDYIC             (1 << 1)  /* Bit 1:  LSE Ready Interrupt Clear */
#define RCC_CICR_MSIRDYIC             (1 << 2)  /* Bit 2:  MSI Ready Interrupt Clear */
#define RCC_CICR_HSI16RDYIC           (1 << 3)  /* Bit 3:  HSI16 Ready Interrupt Clear */
#define RCC_CICR_HSERDYIC             (1 << 4)  /* Bit 4:  HSE Ready Interrupt Clear */
#define RCC_CICR_PLLRDYIC             (1 << 5)  /* Bit 5:  PLL Ready Interrupt Clear */
#define RCC_CICR_PLLSAI1RDYIC         (1 << 6)  /* Bit 6:  PLLSAI1 Ready Interrupt Clear */
#define RCC_CICR_CSSC                 (1 << 8)  /* Bit 8:  Clock Security System Interrupt Clear */
#define RCC_CICR_LSECSSIC             (1 << 9)  /* Bit 9:  LSE Clock Security System Interrupt Clear */
#define RCC_CICR_HSI48RDYIC           (1 << 10) /* Bit 10: HSI48 Oscillator Ready Interrupt Clear */
#define RCC_CICR_LSI2RDYIC            (1 << 11) /* Bit 11: LSI2 Ready Interrupt Clear */

/* SMPS step-down converter control register */

#define RCC_SMPS_SMPSSEL_SHIFT        (0)       /* Bits 0-1: SMPS Clock Selection  */
#define RCC_SMPS_SMPSSEL_MASK         (0x3 << RCC_SMPS_SMPSSEL_SHIFT)
#  define RCC_SMPS_SMPSSEL_HSI16      (0x0 << RCC_SMPS_SMPSSEL_SHIFT) /* 00: HSI16 selected as SMPS clock */
#  define RCC_SMPS_SMPSSEL_MSI        (0x1 << RCC_SMPS_SMPSSEL_SHIFT) /* 01: MSI selected as SMPS clock */
#  define RCC_SMPS_SMPSSEL_HSE        (0x2 << RCC_SMPS_SMPSSEL_SHIFT) /* 10: HSE selected as SMPS clock */
#  define RCC_SMPS_SMPSSEL_NONE       (0x3 << RCC_SMPS_SMPSSEL_SHIFT) /* 11: no clock is used */

#define RCC_SMPS_SMPSDIV_SHIFT        (4)       /* Bits 4-5: SMPS Clock Division factor */
#define RCC_SMPS_SMPSDIV_MASK         (0x3 << RCC_SMPS_SMPSDIV_SHIFT)
#  define RCC_SMPS_SMPSDIVd1          (0x0 << RCC_SMPS_SMPSDIV_SHIFT) /* 00: Selected clock not divided */
#  define RCC_SMPS_SMPSDIVd2          (0x1 << RCC_SMPS_SMPSDIV_SHIFT) /* 01: Selected clock divided by 2 */

#define RCC_SMPS_SMPSSWS_SHIFT        (8)       /* Bits 8-9: SMPS Clock Switch Status */
#define RCC_SMPS_SMPSSWS_MASK         (0x3 << RCC_SMPS_SMPSSWS_SHIFT)
#  define RCC_SMPS_SMPSSWS_HSI16      (0x0 << RCC_SMPS_SMPSSWS_SHIFT) /* 00: HSI16 oscillator used as SMPS clock */
#  define RCC_SMPS_SMPSSWS_MSI        (0x1 << RCC_SMPS_SMPSSWS_SHIFT) /* 01: MSI oscillator used as SMPS clock */
#  define RCC_SMPS_SMPSSWS_HSE        (0x2 << RCC_SMPS_SMPSSWS_SHIFT) /* 10: HSE oscillator used as SMPS clock */
#  define RCC_SMPS_SMPSSWS_NONE       (0x3 << RCC_SMPS_SMPSSWS_SHIFT) /* 11: no clock is used */

/* AHB1 peripheral reset register */

#define RCC_AHB1RSTR_DMA1RST          (1 << 0)  /* Bit 0:  DMA1 reset */
#define RCC_AHB1RSTR_DMA2RST          (1 << 1)  /* Bit 1:  DMA2 reset */
#define RCC_AHB1RSTR_DMAMUX1RST       (1 << 2)  /* Bit 2:  DMAMUX1 reset */
#define RCC_AHB1RSTR_FLASHRST         (1 << 8)  /* Bit 8:  Flash memory interface reset */
#define RCC_AHB1RSTR_CRCRST           (1 << 12) /* Bit 12: CRC reset */
#define RCC_AHB1RSTR_TSCRST           (1 << 16) /* Bit 16: Touch Sensing Controller reset */

/* AHB2 peripheral reset register */

#define RCC_AHB2RSTR_GPIORST(n)       (1 << (n))
#define RCC_AHB2RSTR_GPIOARST         (1 << 0)  /* Bit 0:  IO port A reset */
#define RCC_AHB2RSTR_GPIOBRST         (1 << 1)  /* Bit 1:  IO port B reset */
#define RCC_AHB2RSTR_GPIOCRST         (1 << 2)  /* Bit 2:  IO port C reset */
#define RCC_AHB2RSTR_GPIODRST         (1 << 3)  /* Bit 3:  IO port D reset */
#define RCC_AHB2RSTR_GPIOERST         (1 << 4)  /* Bit 4:  IO port E reset */
#define RCC_AHB2RSTR_GPIOHRST         (1 << 7)  /* Bit 7:  IO port H reset */
#define RCC_AHB2RSTR_ADCRST           (1 << 13) /* Bit 13: ADC reset */
#define RCC_AHB2RSTR_AESRST           (1 << 16) /* Bit 16: AES reset */

/* AHB3 and AHB4 peripheral reset register */

#define RCC_AHB3RSTR_QSPIRST          (1 << 8)  /* Bit 8:  QSPI reset */
#define RCC_AHB3RSTR_PKARST           (1 << 16) /* Bit 16: PKA reset */
#define RCC_AHB3RSTR_AES2IRST         (1 << 17) /* Bit 17: AES2 reset */
#define RCC_AHB3RSTR_RNGRST           (1 << 18) /* Bit 18: RNG reset */
#define RCC_AHB3RSTR_HSEMRST          (1 << 19) /* Bit 19: HSEM reset */
#define RCC_AHB3RSTR_IPCCRST          (1 << 20) /* Bit 20: IPCC reset */
#define RCC_AHB3RSTR_FLASHRST         (1 << 25) /* Bit 25: FLASH reset */

/* APB1 Peripheral reset register 1 */

#define RCC_APB1RSTR1_TIM2RST         (1 << 0)  /* Bit 0:  TIM2 reset */
#define RCC_APB1RSTR1_LCDRST          (1 << 9)  /* Bit 9:  LCD reset */
#define RCC_APB1RSTR1_SPI2RST         (1 << 14) /* Bit 14: SPI2 reset */
#define RCC_APB1RSTR1_I2C1RST         (1 << 21) /* Bit 21: I2C1 reset */
#define RCC_APB1RSTR1_I2C3RST         (1 << 23) /* Bit 23: I2C3 reset */
#define RCC_APB1RSTR1_CRSRST          (1 << 24) /* Bit 24: CRS reset */
#define RCC_APB1RSTR1_USBRST          (1 << 26) /* Bit 26: USB reset */
#define RCC_APB1RSTR1_LPTIM1RST       (1 << 31) /* Bit 31: LPTIM1 reset */

/* APB1 Peripheral reset register 2 */

#define RCC_APB1RSTR2_LPUART1RST      (1 << 0)  /* Bit 0: LPUART1 reset */
#define RCC_APB1RSTR2_LPTIM2RST       (1 << 5)  /* Bit 5: LPTIM2 reset */

/* APB2 Peripheral reset register */

#define RCC_APB2RSTR_TIM1RST          (1 << 11) /* Bit 11: TIM1 reset */
#define RCC_APB2RSTR_SPI1RST          (1 << 12) /* Bit 12: SPI1 reset */
#define RCC_APB2RSTR_USART1RST        (1 << 14) /* Bit 14: USART1 reset */
#define RCC_APB2RSTR_TIM16RST         (1 << 17) /* Bit 17: TIM16 reset */
#define RCC_APB2RSTR_TIM17RST         (1 << 18) /* Bit 18: TIM17 reset */
#define RCC_APB2RSTR_SAI1RST          (1 << 21) /* Bit 21: SAI1 reset */

/* APB3 Peripheral reset register */

#define RCC_APB3RSTR_RFRST            (1 << 0)  /* Bit 0: Radio system BLE and 802.15.4 reset */

/* AHB1 Peripheral Clock enable register */

#define RCC_AHB1ENR_DMA1EN            (1 << 0)  /* Bit 0:  DMA1 Enable */
#define RCC_AHB1ENR_DMA2EN            (1 << 1)  /* Bit 1:  DMA2 Enable */
#define RCC_AHB1ENR_DMAMUX1EN         (1 << 2)  /* Bit 2:  DMAMUX1 Enable */
#define RCC_AHB1ENR_CRCEN             (1 << 12) /* Bit 12: CRC Enable */
#define RCC_AHB1ENR_TSCEN             (1 << 16) /* Bit 16: Touch Sensing Controller Enable */

/* AHB2 Peripheral Clock enable register */

#define RCC_AHB2ENR_GPIOEN(n)         (1 << (n))
#define RCC_AHB2ENR_GPIOAEN           (1 << 0)  /* Bit 0:  IO port A Enable */
#define RCC_AHB2ENR_GPIOBEN           (1 << 1)  /* Bit 1:  IO port B Enable */
#define RCC_AHB2ENR_GPIOCEN           (1 << 2)  /* Bit 2:  IO port C Enable */
#define RCC_AHB2ENR_GPIODEN           (1 << 3)  /* Bit 3:  IO port D Enable */
#define RCC_AHB2ENR_GPIOEEN           (1 << 4)  /* Bit 4:  IO port E Enable */
#define RCC_AHB2ENR_GPIOHEN           (1 << 7)  /* Bit 7:  IO port H Enable */
#define RCC_AHB2ENR_ADCEN             (1 << 13) /* Bit 13: ADC interface Enable */
#define RCC_AHB2ENR_AES1EN            (1 << 16) /* Bit 16: AES1 Enable */

/* AHB3 and AHB4 Peripheral Clock enable register */

#define RCC_AHB3ENR_QSPIEN            (1 << 8)  /* Bit 8:  QSPI Enable */
#define RCC_AHB3ENR_PKAEN             (1 << 16) /* Bit 16: PKA Enable */
#define RCC_AHB3ENR_AES2EN            (1 << 17) /* Bit 17: AES2 Enable */
#define RCC_AHB3ENR_RNGEN             (1 << 18) /* Bit 18: RNG Enable */
#define RCC_AHB3ENR_HSEMEN            (1 << 19) /* Bit 19: HSEM Enable */
#define RCC_AHB3ENR_IPCCEN            (1 << 20) /* Bit 20: IPCC Enable */
#define RCC_AHB3ENR_FLASHEN           (1 << 25) /* Bit 25: FLASH Enable */

/* APB1 Peripheral Clock enable register 1 */

#define RCC_APB1ENR1_TIM2EN           (1 << 0)  /* Bit 0:  TIM2 enable */
#define RCC_APB1ENR1_LCDEN            (1 << 9)  /* Bit 9:  LCD enable */
#define RCC_APB1ENR1_RTCAPBEN         (1 << 10) /* Bit 10: RTC APB clock enable */
#define RCC_APB1ENR1_WWDGEN           (1 << 11) /* Bit 11: Windowed Watchdog enable */
#define RCC_APB1ENR1_SPI2EN           (1 << 14) /* Bit 14: SPI2 enable */
#define RCC_APB1ENR1_I2C1EN           (1 << 21) /* Bit 21: I2C1 enable */
#define RCC_APB1ENR1_I2C3EN           (1 << 23) /* Bit 23: I2C3 enable */
#define RCC_APB1ENR1_CRSEN            (1 << 24) /* Bit 24: CRSEN enable */
#define RCC_APB1ENR1_USBEN            (1 << 26) /* Bit 26: USB enable */
#define RCC_APB1ENR1_LPTIM1EN         (1 << 31) /* Bit 31: LPTIM1 enable */

/* APB1 Peripheral Clock enable register 2 */

#define RCC_APB1ENR2_LPUART1EN        (1 << 0)  /* Bit 0:  LPUART1 enable */
#define RCC_APB1ENR2_LPTIM2EN         (1 << 5)  /* Bit 5:  LPTIM2 enable */

/* APB2 Peripheral Clock enable register */

#define RCC_APB2ENR_TIM1EN            (1 << 11) /* Bit 11: TIM1 enable */
#define RCC_APB2ENR_SPI1EN            (1 << 12) /* Bit 12: SPI1 enable */
#define RCC_APB2ENR_USART1EN          (1 << 14) /* Bit 14: USART1 enable */
#define RCC_APB2ENR_TIM16EN           (1 << 17) /* Bit 17: TIM16 enable */
#define RCC_APB2ENR_TIM17EN           (1 << 18) /* Bit 18: TIM17 enable */
#define RCC_APB2ENR_SAI1EN            (1 << 21) /* Bit 21: SAI1 enable */

/* AHB1 peripheral clock enable in sleep and stop modes register */

#define RCC_AHB1SMENR_DMA1LPSMEN      (1 << 0)  /* Bit 0:  DMA1 enable in sleep and stop modes */
#define RCC_AHB1SMENR_DMA2LPSMEN      (1 << 1)  /* Bit 1:  DMA2 enable in sleep and stop modes */
#define RCC_AHB1SMENR_DMAMUX1SMEN     (1 << 2)  /* Bit 2:  DMAMUX1 clock enable in sleep and stop modes */
#define RCC_AHB1SMENR_SRAM1SMEN       (1 << 9)  /* Bit 9:  SRAM1 enable in sleep and stop modes */
#define RCC_AHB1SMENR_CRCSMEN         (1 << 12) /* Bit 12: CRC enable in sleep and stop modes */
#define RCC_AHB1SMENR_TSCSMEN         (1 << 16) /* Bit 16: TSC enable in sleep and stop modes */

/* AHB2 peripheral clock enable in sleep and stop modes register */

#define RCC_AHB2SMENR_GPIOASMEN       (1 << 0)  /* Bit 0:  IO port A enable in sleep and stop modes */
#define RCC_AHB2SMENR_GPIOBSMEN       (1 << 1)  /* Bit 1:  IO port B enable in sleep and stop modes */
#define RCC_AHB2SMENR_GPIOCSMEN       (1 << 2)  /* Bit 2:  IO port C enable in sleep and stop modes */
#define RCC_AHB2SMENR_GPIODSMEN       (1 << 3)  /* Bit 3:  IO port D enable in sleep and stop modes */
#define RCC_AHB2SMENR_GPIOESMEN       (1 << 4)  /* Bit 4:  IO port E enable in sleep and stop modes */
#define RCC_AHB2SMENR_GPIOHSMEN       (1 << 7)  /* Bit 7:  IO port H enable in sleep and stop modes */
#define RCC_AHB2SMENR_ADCSMEN         (1 << 13) /* Bit 13: ADC enable in sleep and stop modes */
#define RCC_AHB2SMENR_AES1SMEN        (1 << 16) /* Bit 16: AES1 enable in sleep and stop modes */

/* AHB3 and AHB4 peripheral clock enable in sleep and stop modes register */

#define RCC_AHB3SMENR_QSPISMEN        (1 << 8)  /* Bit 8:  QSPI enable in sleep and stop modes */
#define RCC_AHB3SMENR_PKASMEN         (1 << 16) /* Bit 16: PKA enable in sleep and stop modes */
#define RCC_AHB3SMENR_AES2SMEN        (1 << 17) /* Bit 17: AES2 enable in sleep and stop modes */
#define RCC_AHB3SMENR_RNGSMEN         (1 << 18) /* Bit 18: RNG enable in sleep and stop modes */
#define RCC_AHB3SMENR_SRAM2SMEN       (1 << 24) /* Bit 24: SRAM2x enable in sleep and stop modes */
#define RCC_AHB3SMENR_FLASHSMEN       (1 << 25) /* Bit 25: FLASH memory enable in sleep and stop modes */

/* APB1 peripheral clock enable in sleep and stop modes register 1 */

#define RCC_APB1SMENR1_TIM2SMEN       (1 << 0)  /* Bit 0:  TIM2 enable in sleep and stop modes */
#define RCC_APB1SMENR1_LCDSMEN        (9 << 0)  /* Bit 9:  LCD enable in sleep and stop modes */
#define RCC_APB1SMENR1_RTCAPBSMEN     (1 << 10) /* Bit 10: RTC APB clock enable in sleep and stop modes */
#define RCC_APB1SMENR1_WWDGSMEN       (1 << 11) /* Bit 11: Windowed Watchdog enable in sleep and stop modes */
#define RCC_APB1SMENR1_SPI2SMEN       (1 << 14) /* Bit 14: SPI2 enable in sleep and stop modes */
#define RCC_APB1SMENR1_I2C1SMEN       (1 << 21) /* Bit 21: I2C1 enable in sleep and stop modes */
#define RCC_APB1SMENR1_I2C3SMEN       (1 << 23) /* Bit 23: I2C3 enable in sleep and stop modes */
#define RCC_APB1SMENR1_CRSSMEN        (1 << 24) /* Bit 24: CRS enable in sleep and stop modes */
#define RCC_APB1SMENR1_USBSMEN        (1 << 26) /* Bit 26: USB enable in sleep and stop modes */
#define RCC_APB1SMENR1_LPTIM1SMEN     (1 << 31) /* Bit 31: LPTIM1 enable in sleep and stop modes */

/* APB1 peripheral clock enable in sleep and stop modes register 2 */

#define RCC_APB1SMENR2_LPUART1SMEN    (1 << 0)  /* Bit 0:  LPUART1 enable in sleep and stop modes */
#define RCC_APB1SMENR2_LPTIM2SMEN     (1 << 5)  /* Bit 5:  LPTIM2 enable in sleep and stop modes */

/* APB2 peripheral clock enable in sleep and stop modes register */

#define RCC_APB2SMENR_TIM1SMEN        (1 << 11) /* Bit 11: TIM1 enable in sleep and stop modes */
#define RCC_APB2SMENR_SPI1SMEN        (1 << 12) /* Bit 12: SPI1 enable in sleep and stop modes */
#define RCC_APB2SMENR_USART1SMEN      (1 << 14) /* Bit 14: USART1 enable in sleep and stop modes */
#define RCC_APB2SMENR_TIM16SMEN       (1 << 17) /* Bit 17: TIM16 enable in sleep and stop modes */
#define RCC_APB2SMENR_TIM17SMEN       (1 << 18) /* Bit 18: TIM17 enable in sleep and stop modes */
#define RCC_APB2SMENR_SAI1SMEN        (1 << 21) /* Bit 21: SAI1 enable in sleep and stop modes */

/* Peripheral Independent Clock Configuration register */

#define RCC_CCIPR_USART1SEL_SHIFT     (0)     /* Bits 0-1: USART1 clock source selection */
#define RCC_CCIPR_USART1SEL_MASK      (0x3 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_PCLK    (0x0 << RCC_CCIPR_USART1SEL_SHIFT)  /* 00: PCLK selected */
#  define RCC_CCIPR_USART1SEL_SYSCLK  (0x1 << RCC_CCIPR_USART1SEL_SHIFT)  /* 01: System clock selected */
#  define RCC_CCIPR_USART1SEL_HSI16   (0x2 << RCC_CCIPR_USART1SEL_SHIFT)  /* 10: HSI16 clock selected */
#  define RCC_CCIPR_USART1SEL_LSE     (0x3 << RCC_CCIPR_USART1SEL_SHIFT)  /* 11: LSE clock selected */

#define RCC_CCIPR_LPUART1SEL_SHIFT    (10)    /* Bits 10-11: LPUART1 clock source selection */
#define RCC_CCIPR_LPUART1SEL_MASK     (0x3 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_PCLK   (0x0 << RCC_CCIPR_LPUART1SEL_SHIFT) /* 00: PCLK selected */
#  define RCC_CCIPR_LPUART1SEL_SYSCLK (0x1 << RCC_CCIPR_LPUART1SEL_SHIFT) /* 01: System clock selected */
#  define RCC_CCIPR_LPUART1SEL_HSI16  (0x2 << RCC_CCIPR_LPUART1SEL_SHIFT) /* 10: HSI16 clock selected */
#  define RCC_CCIPR_LPUART1SEL_LSE    (0x3 << RCC_CCIPR_LPUART1SEL_SHIFT) /* 11: LSE clock selected */

#define RCC_CCIPR_I2C1SEL_SHIFT       (12)    /* Bits 12-13: I2C1 clock source selection */
#define RCC_CCIPR_I2C1SEL_MASK        (0x3 << RCC_CCIPR_I2C1SEL_SHIFT)
#  define RCC_CCIPR_I2C1SEL_PCLK      (0x0 << RCC_CCIPR_I2C1SEL_SHIFT)    /* 00: PCLK selected */
#  define RCC_CCIPR_I2C1SEL_SYSCLK    (0x1 << RCC_CCIPR_I2C1SEL_SHIFT)    /* 01: System clock selected */
#  define RCC_CCIPR_I2C1SEL_HSI16     (0x2 << RCC_CCIPR_I2C1SEL_SHIFT)    /* 10: HSI16 clock selected */

#define RCC_CCIPR_I2C3SEL_SHIFT       (16)    /* Bits 16-17: I2C3 clock source selection */
#define RCC_CCIPR_I2C3SEL_MASK        (0x3 << RCC_CCIPR_I2C3SEL_SHIFT)
#  define RCC_CCIPR_I2C3SEL_PCLK      (0x0 << RCC_CCIPR_I2C3SEL_SHIFT)    /* 00: PCLK selected */
#  define RCC_CCIPR_I2C3SEL_SYSCLK    (0x1 << RCC_CCIPR_I2C3SEL_SHIFT)    /* 01: System clock selected */
#  define RCC_CCIPR_I2C3SEL_HSI16     (0x2 << RCC_CCIPR_I2C3SEL_SHIFT)    /* 10: HSI16 clock selected */

#define RCC_CCIPR_LPTIM1SEL_SHIFT     (18)    /* Bits 18-19: LPTIM1 clock source selection */
#define RCC_CCIPR_LPTIM1SEL_MASK      (0x3 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_PCLK    (0x0 << RCC_CCIPR_LPTIM1SEL_SHIFT)  /* 00: PCLK selected */
#  define RCC_CCIPR_LPTIM1SEL_LSI     (0x1 << RCC_CCIPR_LPTIM1SEL_SHIFT)  /* 01: LSI clock selected */
#  define RCC_CCIPR_LPTIM1SEL_HSI16   (0x2 << RCC_CCIPR_LPTIM1SEL_SHIFT)  /* 10: HSI16 clock selected */
#  define RCC_CCIPR_LPTIM1SEL_LSE     (0x3 << RCC_CCIPR_LPTIM1SEL_SHIFT)  /* 11: LSE clock selected */

#define RCC_CCIPR_LPTIM2SEL_SHIFT     (20)    /* Bits 20-21: LPTIM2 clock source selection */
#define RCC_CCIPR_LPTIM2SEL_MASK      (0x3 << RCC_CCIPR_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR_LPTIM2SEL_PCLK    (0x0 << RCC_CCIPR_LPTIM2SEL_SHIFT)  /* 00: PCLK selected */
#  define RCC_CCIPR_LPTIM2SEL_LSI     (0x1 << RCC_CCIPR_LPTIM2SEL_SHIFT)  /* 01: LSI clock selected */
#  define RCC_CCIPR_LPTIM2SEL_HSI16   (0x2 << RCC_CCIPR_LPTIM2SEL_SHIFT)  /* 10: HSI16 clock selected */
#  define RCC_CCIPR_LPTIM2SEL_LSE     (0x3 << RCC_CCIPR_LPTIM2SEL_SHIFT)  /* 11: LSE clock selected */

#define RCC_CCIPR_SAI1SEL_SHIFT       (22)    /* Bits 22-23: SAI1 clock source selection */
#define RCC_CCIPR_SAI1SEL_MASK        (0x3 << RCC_CCIPR_SAI1SEL_SHIFT)
#  define RCC_CCIPR_SAI1SEL_PLLSAI1   (0x0 << RCC_CCIPR_SAI1SEL_SHIFT)    /* 00: PLLSAI1 P clock selected */
#  define RCC_CCIPR_SAI1SEL_PLLP      (0x1 << RCC_CCIPR_SAI1SEL_SHIFT)    /* 01: PLL P clock selected */
#  define RCC_CCIPR_SAI1SEL_HSI16     (0x2 << RCC_CCIPR_SAI1SEL_SHIFT)    /* 10: HSI16 clock selected */
#  define RCC_CCIPR_SAI1SEL_EXTCLK    (0x3 << RCC_CCIPR_SAI1SEL_SHIFT)    /* 11: External input SAI1_EXTCLK */

#define RCC_CCIPR_CLK48SEL_SHIFT      (26)    /* Bits 26-27: 48 MHz clock source selection */
#define RCC_CCIPR_CLK48SEL_MASK       (0x3 << RCC_CCIPR_CLK48SEL_SHIFT)
#  define RCC_CCIPR_CLK48SEL_HSI48    (0x0 << RCC_CCIPR_CLK48SEL_SHIFT)   /* 00: HSI48 clock selected */
#  define RCC_CCIPR_CLK48SEL_PLLSAI1  (0x1 << RCC_CCIPR_CLK48SEL_SHIFT)   /* 01: PLLSAI1 Q clock selected */
#  define RCC_CCIPR_CLK48SEL_PLLMAIN  (0x2 << RCC_CCIPR_CLK48SEL_SHIFT)   /* 10: PLL Q clock selected */
#  define RCC_CCIPR_CLK48SEL_MSI      (0x3 << RCC_CCIPR_CLK48SEL_SHIFT)   /* 11: MSI clock selected */

#define RCC_CCIPR_ADCSEL_SHIFT        (28)    /* Bits 28-29: ADC clock source selection */
#define RCC_CCIPR_ADCSEL_MASK         (0x3 << RCC_CCIPR_ADCSEL_SHIFT)
#  define RCC_CCIPR_ADCSEL_NONE       (0x0 << RCC_CCIPR_ADCSEL_SHIFT)     /* 00: No clock */
#  define RCC_CCIPR_ADCSEL_PLLSAI1    (0x1 << RCC_CCIPR_ADCSEL_SHIFT)     /* 01: PLLSAI1 R clock selected */
#  define RCC_CCIPR_ADCSEL_PLLMAIN    (0x2 << RCC_CCIPR_ADCSEL_SHIFT)     /* 10: PLL P  clock selected */
#  define RCC_CCIPR_ADCSEL_SYSCLK     (0x3 << RCC_CCIPR_ADCSEL_SHIFT)     /* 11: System clock selected */

#define RCC_CCIPR_RNGSEL_SHIFT        (30)    /* Bits 30-31: RNG clock source selection */
#define RCC_CCIPR_RNGSEL_MASK         (0x3 << RCC_CCIPR_RNGSEL_SHIFT)
#  define RCC_CCIPR_RNGSEL_CLK48SEL   (0x0 << RCC_CCIPR_RNGSEL_SHIFT)    /* 00: Clock source selected by CLK48SEL */
#  define RCC_CCIPR_RNGSEL_LSI        (0x1 << RCC_CCIPR_RNGSEL_SHIFT)    /* 01: LSI clock selected */
#  define RCC_CCIPR_RNGSEL_LSE        (0x2 << RCC_CCIPR_RNGSEL_SHIFT)    /* 10: LSE clock selected */

/* Backup domain control register */

#define RCC_BDCR_LSEON                (1 << 0)  /* Bit 0: External Low Speed oscillator enable */
#define RCC_BDCR_LSERDY               (1 << 1)  /* Bit 1: External Low Speed oscillator Ready */
#define RCC_BDCR_LSEBYP               (1 << 2)  /* Bit 2: External Low Speed oscillator Bypass */

#define RCC_BDCR_LSEDRV_SHIFT         (3)       /* Bits 3-4: LSE oscillator drive capability */
#define RCC_BDCR_LSEDRV_MASK          (0x3 << RCC_BDCR_LSEDRV_SHIFT)
#  define RCC_BDCR_LSEDRV_LOW         (0x0 << RCC_BDCR_LSEDRV_SHIFT) /* 00: Lower driving capability */
#  define RCC_BDCR_LSEDRV_MEDLO       (0x1 << RCC_BDCR_LSEDRV_SHIFT) /* 01: Medium Low driving capability */
#  define RCC_BDCR_LSEDRV_MEDHI       (0x2 << RCC_BDCR_LSEDRV_SHIFT) /* 10: Medium High driving capability*/
#  define RCC_BDCR_LSEDRV_HIGH        (0x3 << RCC_BDCR_LSEDRV_SHIFT) /* 11: Higher driving capability */

#define RCC_BDCR_LSECSSON             (1 << 5)  /* Bit 5: CSS on LSE enable */
#define RCC_BDCR_LSECSSD              (1 << 6)  /* Bit 6: CSS on LSE failure Detection */

#define RCC_BDCR_RTCSEL_SHIFT         (8)       /* Bits 8-9: RTC clock source selection */
#define RCC_BDCR_RTCSEL_MASK          (0x3 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_NOCLK       (0x0 << RCC_BDCR_RTCSEL_SHIFT) /* 00: No clock */
#  define RCC_BDCR_RTCSEL_LSE         (0x1 << RCC_BDCR_RTCSEL_SHIFT) /* 01: LSE oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_LSI         (0x2 << RCC_BDCR_RTCSEL_SHIFT) /* 10: LSI oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_HSE         (0x3 << RCC_BDCR_RTCSEL_SHIFT) /* 11: HSE oscillator clock divided by 32 used as RTC clock */

#define RCC_BDCR_RTCEN                (1 << 15) /* Bit 15: RTC clock enable */
#define RCC_BDCR_BDRST                (1 << 16) /* Bit 16: Backup domain software reset */
#define RCC_BDCR_LSCOEN               (1 << 24) /* Bit 24: Low speed clock output enable */
#define RCC_BDCR_LSCOSEL              (1 << 25) /* Bit 25: Low speed clock output selection */
#  define RCC_BCDR_LSCOSEL_LSI        (0 << 25) /* 0: LSI selected */
#  define RCC_BDCR_LSCOSEL_LSE        (1 << 25) /* 1: LSE selected */

/* Control/status register */

#define RCC_CSR_LSI1ON                (1 << 0)  /* Bit 0: LSI1 Internal Low Speed oscillator enable */
#define RCC_CSR_LSI1RDY               (1 << 1)  /* Bit 1: LSI1 Internal Low Speed oscillator Ready */
#define RCC_CSR_LSI2ON                (1 << 2)  /* Bit 2: LSI2 Internal Low Speed oscillator enable */
#define RCC_CSR_LSI2RDY               (1 << 3)  /* Bit 3: LSI2 Internal Low Speed oscillator Ready */

#define RCC_CSR_LSI2TRIM_SHIFT        (8)       /* Bits: 8-11: LSI2 oscillator trim */
#define RCC_CSR_LSI2TRIM_MASK         (0x0f << RCC_CSR_LSI2TRIM_SHIFT)

#define RCC_CSR_RFWKPSEL_SHIFT        (14)      /* Bits: 14-15: RF system wakeup clock source selection */
#define RCC_CSR_RFWKPSEL_MASK         (0x0f << RCC_CSR_RFWKPSEL_SHIFT)
#  define RCC_CSR_RFWKPSEL_NOCLK      (0x0 << RCC_CSR_RFWKPSEL_SHIFT) /* 00: No clock */
#  define RCC_CSR_RFWKPSEL_LSE        (0x1 << RCC_CSR_RFWKPSEL_SHIFT) /* 01: LSE used as RF wakeup clock */
#  define RCC_CSR_RFWKPSEL_HSE        (0x3 << RCC_CSR_RFWKPSEL_SHIFT) /* 11: HSE divided by 1024 used as RF clock */

#define RCC_CSR_RFRSTS                (1 << 16) /* Bit 16: Radio system BLE and 802.15.4 reset status */
#define RCC_CSR_RMVF                  (1 << 23) /* Bit 23: Remove reset flag */
#define RCC_CSR_OBLRSTF               (1 << 25) /* Bit 25: Option byte loader reset flag */
#define RCC_CSR_PINRSTF               (1 << 26) /* Bit 26: PIN reset flag */
#define RCC_CSR_BORRSTF               (1 << 27) /* Bit 27: BOR reset flag */
#define RCC_CSR_SFTRSTF               (1 << 28) /* Bit 28: Software Reset flag */
#define RCC_CSR_IWWGRSTF              (1 << 29) /* Bit 29: Independent window watchdog reset flag */
#define RCC_CSR_WWDGRSTF              (1 << 30) /* Bit 30: Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF              (1 << 31) /* Bit 31: Low-power reset flag */

/* Clock recovery RC register */

#define RCC_CRRCR_HSI48ON             (1 << 0)  /* Bit 0: HSI48 clock enable */
#define RCC_CRRCR_HSI48RDY            (1 << 1)  /* Bit 1: HSI48 clock ready flag */
#define RCC_CRRCR_HSI48CAL_SHIFT      (7)       /* Bits 7-15: HSI48 clock calibration */
#define RCC_CRRCR_HSI48CAL_MASK       (0x01ff << RCC_CRRCR_HSI48CAL_SHIFT)

/*  Clock HSE register */

#define RCC_HSECR_UNLOCKED            (1 << 0)  /* Bit 0: HSE clock control register unlocked */
#define RCC_HSECR_HSES                (1 << 3)  /* Bit 3: HSE sense amplifier threshold */
#  define RCC_HSECR_HSES_1d2          (0 << 3)  /* 0: HSE bias current factor 1/2 */
#  define RCC_HSECR_HSES_3d4          (1 << 3)  /* 1: HSE bias current factor 3/4 */

#define RCC_HSECR_HSEGMC_SHIFT        (4)       /* Bits 4-6: HSE current control */
#define RCC_HSECR_HSEGMC_MASK         (0x7 << RCC_HSECR_HSEGMC_SHIFT)
#  define RCC_HSECR_HSEGMC_018        (0x0 << RCC_HSECR_HSEGMC_SHIFT) /* 000: current max limit 0.18 mA/V */
#  define RCC_HSECR_HSEGMC_057        (0x1 << RCC_HSECR_HSEGMC_SHIFT) /* 001: current max limit 0.57 mA/V */
#  define RCC_HSECR_HSEGMC_078        (0x2 << RCC_HSECR_HSEGMC_SHIFT) /* 010: current max limit 0.78 mA/V */
#  define RCC_HSECR_HSEGMC_113        (0x3 << RCC_HSECR_HSEGMC_SHIFT) /* 011: current max limit 1.13 mA/V */
#  define RCC_HSECR_HSEGMC_061        (0x4 << RCC_HSECR_HSEGMC_SHIFT) /* 100: current max limit 0.61 mA/V */
#  define RCC_HSECR_HSEGMC_165        (0x5 << RCC_HSECR_HSEGMC_SHIFT) /* 101: current max limit 1.65 mA/V */
#  define RCC_HSECR_HSEGMC_212        (0x6 << RCC_HSECR_HSEGMC_SHIFT) /* 110: current max limit 2.12 mA/V */
#  define RCC_HSECR_HSEGMC_284        (0x7 << RCC_HSECR_HSEGMC_SHIFT) /* 111: current max limit 2.84 mA/V */

#define RCC_HSECR_HSETUNE_SHIFT       (8)       /* Bits 8-13: HSE capacitor tuning */
#define RCC_HSECR_HSETUNE_MASK        (0x7 << RCC_HSECR_HSETUNE_SHIFT)

#define RCC_HSECR_SHDHPREF            (1 << 16) /* Bit 16: HCLK4 shared prescaler flag (AHB4, FLASH, SRAM2) */
#define RCC_HSECR_C2HPREF             (1 << 17) /* Bit 17: CPU2 HCLK2 prescaler flag */

#define RCC_HSECR_RFCSS               (1 << 20) /* Bit 20: Radio system HCLK5 and APB3 selected clock */
#  define RCC_HSECR_RFCSS_HSI16       (0 << 20) /* HSI16 used for Radio system HCLK5 and APB3 clock */
#  define RCC_HSECR_RFCSS_HSE2        (1 << 20) /* HSE divided by 2 used for Radio system HCLK5 and APB3 clock */

/* Extended clock recovery register */

#define RCC_EXTCFGR_SHDHPRE_SHIFT     (0)       /* Bits: 0-3: HCLK4 shared prescaler (AHB4, FLASH, SRAM2) */
#define RCC_EXTCFGR_SHDHPRE_MASK      (0xf)
#  define RCC_EXTCFGR_SHDHPRE_1       (0x0 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 0000: SYSCLK not divided */
#  define RCC_EXTCFGR_SHDHPRE_3       (0x1 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 0001: SYSCLK divided by 3 */
#  define RCC_EXTCFGR_SHDHPRE_5       (0x2 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 0010: SYSCLK divided by 5 */
#  define RCC_EXTCFGR_SHDHPRE_6       (0x5 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 0101: SYSCLK divided by 6 */
#  define RCC_EXTCFGR_SHDHPRE_10      (0x6 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 0110: SYSCLK divided by 10 */
#  define RCC_EXTCFGR_SHDHPRE_32      (0x7 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 0111: SYSCLK divided by 32 */
#  define RCC_EXTCFGR_SHDHPRE_2       (0x8 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 1000: SYSCLK divided by 2 */
#  define RCC_EXTCFGR_SHDHPRE_4       (0x9 << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 1001: SYSCLK divided by 4 */
#  define RCC_EXTCFGR_SHDHPRE_8       (0xa << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 1010: SYSCLK divided by 8 */
#  define RCC_EXTCFGR_SHDHPRE_16      (0xb << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 1011: SYSCLK divided by 16 */
#  define RCC_EXTCFGR_SHDHPRE_64      (0xc << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 1100: SYSCLK divided by 64 */
#  define RCC_EXTCFGR_SHDHPRE_128     (0xd << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 1101: SYSCLK divided by 128 */
#  define RCC_EXTCFGR_SHDHPRE_256     (0xe << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 1110: SYSCLK divided by 256 */
#  define RCC_EXTCFGR_SHDHPRE_512     (0xf << RCC_EXTCFGR_SHDHPRE_SHIFT) /* 1111: SYSCLK divided by 512 */

#define RCC_EXTCFGR_C2HPRE_SHIFT      (4)       /* Bits: 4-7: CPU2 HCLK2 prescaler */
#define RCC_EXTCFGR_C2HPRE_MASK       (0xf)
#  define RCC_EXTCFGR_C2HPRE_1        (0x0 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 0000: SYSCLK not divided */
#  define RCC_EXTCFGR_C2HPRE_3        (0x1 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 0001: SYSCLK divided by 3 */
#  define RCC_EXTCFGR_C2HPRE_5        (0x2 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 0010: SYSCLK divided by 5 */
#  define RCC_EXTCFGR_C2HPRE_6        (0x5 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 0101: SYSCLK divided by 6 */
#  define RCC_EXTCFGR_C2HPRE_10       (0x6 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 0110: SYSCLK divided by 10 */
#  define RCC_EXTCFGR_C2HPRE_32       (0x7 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 0111: SYSCLK divided by 32 */
#  define RCC_EXTCFGR_C2HPRE_2        (0x8 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 1000: SYSCLK divided by 2 */
#  define RCC_EXTCFGR_C2HPRE_4        (0x9 << RCC_EXTCFGR_C2HPRE_SHIFT) /* 1001: SYSCLK divided by 4 */
#  define RCC_EXTCFGR_C2HPRE_8        (0xa << RCC_EXTCFGR_C2HPRE_SHIFT) /* 1010: SYSCLK divided by 8 */
#  define RCC_EXTCFGR_C2HPRE_16       (0xb << RCC_EXTCFGR_C2HPRE_SHIFT) /* 1011: SYSCLK divided by 16 */
#  define RCC_EXTCFGR_C2HPRE_64       (0xc << RCC_EXTCFGR_C2HPRE_SHIFT) /* 1100: SYSCLK divided by 64 */
#  define RCC_EXTCFGR_C2HPRE_128      (0xd << RCC_EXTCFGR_C2HPRE_SHIFT) /* 1101: SYSCLK divided by 128 */
#  define RCC_EXTCFGR_C2HPRE_256      (0xe << RCC_EXTCFGR_C2HPRE_SHIFT) /* 1110: SYSCLK divided by 256 */
#  define RCC_EXTCFGR_C2HPRE_512      (0xf << RCC_EXTCFGR_C2HPRE_SHIFT) /* 1111: SYSCLK divided by 512 */

/* CPU2 AHB1 Peripheral Clock enable register */

#define RCC_C2AHB1ENR_DMA1EN          (1 << 0)  /* Bit 0:  CPU2 DMA1 Enable */
#define RCC_C2AHB1ENR_DMA2EN          (1 << 1)  /* Bit 1:  CPU2 DMA2 Enable */
#define RCC_C2AHB1ENR_DMAMUX1EN       (1 << 2)  /* Bit 2:  CPU2 DMAMUX1 Enable */
#define RCC_C2AHB1ENR_SRAM1EN         (1 << 9)  /* Bit 9:  CPU2 SRAM1 Enable */
#define RCC_C2AHB1ENR_CRCEN           (1 << 12) /* Bit 12: CPU2 CRC Enable */
#define RCC_C2AHB1ENR_TSCEN           (1 << 16) /* Bit 16: CPU2 Touch Sensing Controller Enable */

/* CPU2 AHB2 Peripheral Clock enable register */

#define RCC_C2AHB2ENR_GPIOEN(n)       (1 << (n))
#define RCC_C2AHB2ENR_GPIOAEN         (1 << 0)  /* Bit 0:  CPU2 IO port A Enable */
#define RCC_C2AHB2ENR_GPIOBEN         (1 << 1)  /* Bit 1:  CPU2 IO port B Enable */
#define RCC_C2AHB2ENR_GPIOCEN         (1 << 2)  /* Bit 2:  CPU2 IO port C Enable */
#define RCC_C2AHB2ENR_GPIODEN         (1 << 3)  /* Bit 3:  CPU2 IO port D Enable */
#define RCC_C2AHB2ENR_GPIOEEN         (1 << 4)  /* Bit 4:  CPU2 IO port E Enable */
#define RCC_C2AHB2ENR_GPIOHEN         (1 << 7)  /* Bit 7:  CPU2 IO port H Enable */
#define RCC_C2AHB2ENR_ADCEN           (1 << 13) /* Bit 13: CPU2 ADC interface Enable */
#define RCC_C2AHB2ENR_AES1EN          (1 << 16) /* Bit 16: CPU2 AES1 Enable */

/* CPU2 AHB3 and AHB4 Peripheral Clock enable register */

#define RCC_C2AHB3ENR_PKAEN           (1 << 16) /* Bit 16: CPU2 PKA Enable */
#define RCC_C2AHB3ENR_AES2EN          (1 << 17) /* Bit 17: CPU2 AES2 Enable */
#define RCC_C2AHB3ENR_RNGEN           (1 << 18) /* Bit 18: CPU2 RNG Enable */
#define RCC_C2AHB3ENR_HSEMEN          (1 << 19) /* Bit 19: CPU2 HSEM Enable */
#define RCC_C2AHB3ENR_IPCCEN          (1 << 20) /* Bit 20: CPU2 IPCC Enable */
#define RCC_C2AHB3ENR_FLASHEN         (1 << 25) /* Bit 25: CPU2 FLASH Enable */

/* CPU2 APB1 Peripheral Clock enable register 1 */

#define RCC_C2APB1ENR1_TIM2EN         (1 << 0)  /* Bit 0:  CPU2 TIM2 enable */
#define RCC_C2APB1ENR1_LCDEN          (1 << 9)  /* Bit 9:  CPU2 LCD enable */
#define RCC_C2APB1ENR1_RTCAPBEN       (1 << 10) /* Bit 10: CPU2 RTC APB clock enable */
#define RCC_C2APB1ENR1_SPI2EN         (1 << 14) /* Bit 14: CPU2 SPI2 enable */
#define RCC_C2APB1ENR1_I2C1EN         (1 << 21) /* Bit 21: CPU2 I2C1 enable */
#define RCC_C2APB1ENR1_I2C3EN         (1 << 23) /* Bit 23: CPU2 I2C3 enable */
#define RCC_C2APB1ENR1_CRSEN          (1 << 24) /* Bit 24: CPU2 CRSEN enable */
#define RCC_C2APB1ENR1_USBEN          (1 << 26) /* Bit 26: CPU2 USB enable */
#define RCC_C2APB1ENR1_LPTIM1EN       (1 << 31) /* Bit 31: CPU2 LPTIM1 enable */

/* CPU2 APB1 Peripheral Clock enable register 2 */

#define RCC_C2APB1ENR2_LPUART1EN      (1 << 0)  /* Bit 0:  CPU2 LPUART1 enable */
#define RCC_C2APB1ENR2_LPTIM2EN       (1 << 5)  /* Bit 5:  CPU2 LPTIM2 enable */

/* CPU2 APB2 Peripheral Clock enable register */

#define RCC_C2APB2ENR_TIM1EN          (1 << 11) /* Bit 11: CPU2 TIM1 enable */
#define RCC_C2APB2ENR_SPI1EN          (1 << 12) /* Bit 12: CPU2 SPI1 enable */
#define RCC_C2APB2ENR_USART1EN        (1 << 14) /* Bit 14: CPU2 USART1 enable */
#define RCC_C2APB2ENR_TIM16EN         (1 << 17) /* Bit 17: CPU2 TIM16 enable */
#define RCC_C2APB2ENR_TIM17EN         (1 << 18) /* Bit 18: CPU2 TIM17 enable */
#define RCC_C2APB2ENR_SAI1EN          (1 << 21) /* Bit 21: CPU2 SAI1 enable */

/* CPU2 APB3 Peripheral Clock enable register */

#define RCC_C2APB3ENR_BLEEN           (1 << 0)  /* Bit 0: CPU2 LPUART1 enable */
#define RCC_C2APB3ENR_802EN           (1 << 1)  /* Bit 1: CPU2 LPTIM2 enable */

/* CPU2 AHB1 peripheral clock enable in sleep and stop modes register */

#define RCC_C2AHB1SMENR_DMA1LPSMEN    (1 << 0)  /* Bit 0:  CPU2 DMA1 enable in sleep and stop modes */
#define RCC_C2AHB1SMENR_DMA2LPSMEN    (1 << 1)  /* Bit 1:  CPU2 DMA2 enable in sleep and stop modes */
#define RCC_C2AHB1SMENR_DMAMUX1SMEN   (1 << 2)  /* Bit 2:  CPU2 DMAMUX1 clock enable in sleep and stop modes */
#define RCC_C2AHB1SMENR_SRAM1SMEN     (1 << 9)  /* Bit 9:  CPU2 SRAM1 enable in sleep and stop modes */
#define RCC_C2AHB1SMENR_CRCSMEN       (1 << 12) /* Bit 12: CPU2 CRC enable in sleep and stop modes */
#define RCC_C2AHB1SMENR_TSCSMEN       (1 << 16) /* Bit 16: CPU2 TSC enable in sleep and stop modes */

/* CPU2 AHB2 peripheral clock enable in sleep and stop modes register */

#define RCC_C2AHB2SMENR_GPIOASMEN     (1 << 0)  /* Bit 0:  CPU2 IO port A enable in sleep and stop modes */
#define RCC_C2AHB2SMENR_GPIOBSMEN     (1 << 1)  /* Bit 1:  CPU2 IO port B enable in sleep and stop modes */
#define RCC_C2AHB2SMENR_GPIOCSMEN     (1 << 2)  /* Bit 2:  CPU2 IO port C enable in sleep and stop modes */
#define RCC_C2AHB2SMENR_GPIODSMEN     (1 << 3)  /* Bit 3:  CPU2 IO port D enable in sleep and stop modes */
#define RCC_C2AHB2SMENR_GPIOESMEN     (1 << 4)  /* Bit 4:  CPU2 IO port E enable in sleep and stop modes */
#define RCC_C2AHB2SMENR_GPIOHSMEN     (1 << 7)  /* Bit 7:  CPU2 IO port H enable in sleep and stop modes */
#define RCC_C2AHB2SMENR_ADCSMEN       (1 << 13) /* Bit 13: CPU2 ADC enable in sleep and stop modes */
#define RCC_C2AHB2SMENR_AES1SMEN      (1 << 16) /* Bit 16: CPU2 AES1 enable in sleep and stop modes */

/* CPU2 AHB3 and AHB4 peripheral clock enable in sleep and stop modes */

#define RCC_C2AHB3SMENR_PKASMEN       (1 << 16) /* Bit 16: CPU2 PKA enable in sleep and stop modes */
#define RCC_C2AHB3SMENR_AES2SMEN      (1 << 17) /* Bit 17: CPU2 AES2 enable in sleep and stop modes */
#define RCC_C2AHB3SMENR_RNGSMEN       (1 << 18) /* Bit 18: CPU2 RNG enable in sleep and stop modes */
#define RCC_C2AHB3SMENR_SRAM2SMEN     (1 << 24) /* Bit 24: CPU2 SRAM2x enable in sleep and stop modes */
#define RCC_C2AHB3SMENR_FLASHSMEN     (1 << 25) /* Bit 25: CPU2 FLASH memory enable in sleep and stop modes */

/* CPU2 APB1 peripheral clock enable in sleep and stop modes register 1 */

#define RCC_C2APB1SMENR1_TIM2SMEN     (1 << 0)  /* Bit 0:  CPU2 TIM2 enable in sleep and stop modes */
#define RCC_C2APB1SMENR1_LCDSMEN      (9 << 0)  /* Bit 9:  CPU2 LCD enable in sleep and stop modes */
#define RCC_C2APB1SMENR1_RTCAPBSMEN   (1 << 10) /* Bit 10: CPU2 RTC APB clock enable in sleep and stop modes */
#define RCC_C2APB1SMENR1_SPI2SMEN     (1 << 14) /* Bit 14: CPU2 SPI2 enable in sleep and stop modes */
#define RCC_C2APB1SMENR1_I2C1SMEN     (1 << 21) /* Bit 21: CPU2 I2C1 enable in sleep and stop modes */
#define RCC_C2APB1SMENR1_I2C3SMEN     (1 << 23) /* Bit 23: CPU2 I2C3 enable in sleep and stop modes */
#define RCC_C2APB1SMENR1_CRSSMEN      (1 << 24) /* Bit 24: CPU2 CRS enable in sleep and stop modes */
#define RCC_C2APB1SMENR1_USBSMEN      (1 << 26) /* Bit 26: CPU2 USB enable in sleep and stop modes */
#define RCC_C2APB1SMENR1_LPTIM1SMEN   (1 << 31) /* Bit 31: CPU2 LPTIM1 enable in sleep and stop modes */

/* CPU2 APB1 peripheral clock enable in sleep and stop modes register 2 */

#define RCC_C2APB1SMENR2_LPUART1SMEN  (1 << 0) /* Bit 0:  CPU2 LPUART1 enable in sleep and stop modes */
#define RCC_C2APB1SMENR2_LPTIM2SMEN   (1 << 5) /* Bit 5:  CPU2 LPTIM2 enable in sleep and stop modes */

/* CPU2 APB2 peripheral clock enable in sleep and stop modes register */

#define RCC_C2APB2SMENR_TIM1SMEN      (1 << 11) /* Bit 11: CPU2 TIM1 enable in sleep and stop modes */
#define RCC_C2APB2SMENR_SPI1SMEN      (1 << 12) /* Bit 12: CPU2 SPI1 enable in sleep and stop modes */
#define RCC_C2APB2SMENR_USART1SMEN    (1 << 14) /* Bit 14: CPU2 USART1 enable in sleep and stop modes */
#define RCC_C2APB2SMENR_TIM16SMEN     (1 << 17) /* Bit 17: CPU2 TIM16 enable in sleep and stop modes */
#define RCC_C2APB2SMENR_TIM17SMEN     (1 << 18) /* Bit 18: CPU2 TIM17 enable in sleep and stop modes */
#define RCC_C2APB2SMENR_SAI1SMEN      (1 << 21) /* Bit 21: CPU2 SAI1 enable in sleep and stop modes */

/* CPU2 APB3 Peripheral Clock enable in sleep and stop modes register */

#define RCC_C2APB3ENR_BLEEN           (1 << 0)  /* Bit 0: CPU2 LPUART1 enable in sleep and stop modes */
#define RCC_C2APB3ENR_802EN           (1 << 1)  /* Bit 1: CPU2 LPTIM2 enable in sleep and stop modes */

#endif /* __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_RCC_H */
