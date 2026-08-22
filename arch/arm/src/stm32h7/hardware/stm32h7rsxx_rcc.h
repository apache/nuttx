/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32h7rsxx_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_RCC_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_RCC_CR_OFFSET             0x0000 /* Clock control */
#define STM32_RCC_HSICFGR_OFFSET        0x0004 /* HSI configuration */
#define STM32_RCC_CRRCR_OFFSET          0x0008 /* HSI48 configuration */
#define STM32_RCC_CSICFGR_OFFSET        0x000c /* CSI configuration */
#define STM32_RCC_CFGR_OFFSET           0x0010 /* Clock configuration */
#define STM32_RCC_CDCFGR_OFFSET         0x0018 /* CPU domain configuration */
#define STM32_RCC_BMCFGR_OFFSET         0x001c /* Bus matrix configuration */
#define STM32_RCC_APBCFGR_OFFSET        0x0020 /* APB configuration */
#define STM32_RCC_PLLCKSELR_OFFSET      0x0028 /* PLL clock selection */
#define STM32_RCC_PLLCFGR_OFFSET        0x002c /* PLL configuration */
#define STM32_RCC_PLL1DIVR1_OFFSET      0x0030 /* PLL1 dividers N/P/Q/R */
#define STM32_RCC_PLL1FRACR_OFFSET      0x0034 /* PLL1 fraction */
#define STM32_RCC_PLL2DIVR1_OFFSET      0x0038 /* PLL2 dividers N/P/Q/R */
#define STM32_RCC_PLL2FRACR_OFFSET      0x003c /* PLL2 fraction */
#define STM32_RCC_PLL3DIVR1_OFFSET      0x0040 /* PLL3 dividers N/P/Q/R */
#define STM32_RCC_PLL3FRACR_OFFSET      0x0044 /* PLL3 fraction */
#define STM32_RCC_CCIPR1_OFFSET         0x004c /* Kernel clocks 1 */
#define STM32_RCC_CCIPR2_OFFSET         0x0050 /* Kernel clocks 2 */
#define STM32_RCC_CCIPR3_OFFSET         0x0054 /* Kernel clocks 3 */
#define STM32_RCC_CCIPR4_OFFSET         0x0058 /* Kernel clocks 4 */
#define STM32_RCC_CIER_OFFSET           0x0060 /* Clock interrupt enable */
#define STM32_RCC_CIFR_OFFSET           0x0064 /* Clock interrupt flags */
#define STM32_RCC_CICR_OFFSET           0x0068 /* Clock interrupt clear */
#define STM32_RCC_BDCR_OFFSET           0x0070 /* Backup domain control */
#define STM32_RCC_CSR_OFFSET            0x0074 /* Clock control/status */
#define STM32_RCC_AHB5RSTR_OFFSET       0x007c /* AHB5 reset */
#define STM32_RCC_AHB1RSTR_OFFSET       0x0080 /* AHB1 reset */
#define STM32_RCC_AHB2RSTR_OFFSET       0x0084 /* AHB2 reset */
#define STM32_RCC_AHB4RSTR_OFFSET       0x0088 /* AHB4 reset */
#define STM32_RCC_APB5RSTR_OFFSET       0x008c /* APB5 reset */
#define STM32_RCC_APB1RSTR1_OFFSET      0x0090 /* APB1 reset 1 */
#define STM32_RCC_APB1RSTR2_OFFSET      0x0094 /* APB1 reset 2 */
#define STM32_RCC_APB2RSTR_OFFSET       0x0098 /* APB2 reset */
#define STM32_RCC_APB4RSTR_OFFSET       0x009c /* APB4 reset */
#define STM32_RCC_AHB3RSTR_OFFSET       0x00a4 /* AHB3 reset */
#define STM32_RCC_CKGDISR_OFFSET        0x00b0 /* Clock generator disable */
#define STM32_RCC_PLL1DIVR2_OFFSET      0x00c0 /* PLL1 divider S */
#define STM32_RCC_PLL2DIVR2_OFFSET      0x00c4 /* PLL2 dividers S/T */
#define STM32_RCC_PLL3DIVR2_OFFSET      0x00c8 /* PLL3 divider S */
#define STM32_RCC_PLL1SSCGR_OFFSET      0x00cc /* PLL1 spread spectrum */
#define STM32_RCC_PLL2SSCGR_OFFSET      0x00d0 /* PLL2 spread spectrum */
#define STM32_RCC_PLL3SSCGR_OFFSET      0x00d4 /* PLL3 spread spectrum */
#define STM32_RCC_CKPROTR_OFFSET        0x0100 /* Clock protection */
#define STM32_RCC_RSR_OFFSET            0x0130 /* Reset status */
#define STM32_RCC_AHB5ENR_OFFSET        0x0134 /* AHB5 clock enable */
#define STM32_RCC_AHB1ENR_OFFSET        0x0138 /* AHB1 clock enable */
#define STM32_RCC_AHB2ENR_OFFSET        0x013c /* AHB2 clock enable */
#define STM32_RCC_AHB4ENR_OFFSET        0x0140 /* AHB4 clock enable */
#define STM32_RCC_APB5ENR_OFFSET        0x0144 /* APB5 clock enable */
#define STM32_RCC_APB1ENR1_OFFSET       0x0148 /* APB1 clock enable 1 */
#define STM32_RCC_APB1ENR2_OFFSET       0x014c /* APB1 clock enable 2 */
#define STM32_RCC_APB2ENR_OFFSET        0x0150 /* APB2 clock enable */
#define STM32_RCC_APB4ENR_OFFSET        0x0154 /* APB4 clock enable */
#define STM32_RCC_AHB3ENR_OFFSET        0x0158 /* AHB3 clock enable */
#define STM32_RCC_AHB5LPENR_OFFSET      0x015c /* AHB5 sleep enable */
#define STM32_RCC_AHB1LPENR_OFFSET      0x0160 /* AHB1 sleep enable */
#define STM32_RCC_AHB2LPENR_OFFSET      0x0164 /* AHB2 sleep enable */
#define STM32_RCC_AHB4LPENR_OFFSET      0x0168 /* AHB4 sleep enable */
#define STM32_RCC_AHB3LPENR_OFFSET      0x016c /* AHB3 sleep enable */
#define STM32_RCC_APB1LPENR1_OFFSET     0x0170 /* APB1 sleep enable 1 */
#define STM32_RCC_APB1LPENR2_OFFSET     0x0174 /* APB1 sleep enable 2 */
#define STM32_RCC_APB2LPENR_OFFSET      0x0178 /* APB2 sleep enable */
#define STM32_RCC_APB4LPENR_OFFSET      0x017c /* APB4 sleep enable */
#define STM32_RCC_APB5LPENR_OFFSET      0x0180 /* APB5 sleep enable */

/* Register Addresses *******************************************************/

#define STM32_RCC_CR                    (STM32_RCC_BASE + STM32_RCC_CR_OFFSET)
#define STM32_RCC_HSICFGR               (STM32_RCC_BASE + STM32_RCC_HSICFGR_OFFSET)
#define STM32_RCC_CRRCR                 (STM32_RCC_BASE + STM32_RCC_CRRCR_OFFSET)
#define STM32_RCC_CSICFGR               (STM32_RCC_BASE + STM32_RCC_CSICFGR_OFFSET)
#define STM32_RCC_CFGR                  (STM32_RCC_BASE + STM32_RCC_CFGR_OFFSET)
#define STM32_RCC_CDCFGR                (STM32_RCC_BASE + STM32_RCC_CDCFGR_OFFSET)
#define STM32_RCC_BMCFGR                (STM32_RCC_BASE + STM32_RCC_BMCFGR_OFFSET)
#define STM32_RCC_APBCFGR               (STM32_RCC_BASE + STM32_RCC_APBCFGR_OFFSET)
#define STM32_RCC_PLLCKSELR             (STM32_RCC_BASE + STM32_RCC_PLLCKSELR_OFFSET)
#define STM32_RCC_PLLCFGR               (STM32_RCC_BASE + STM32_RCC_PLLCFGR_OFFSET)
#define STM32_RCC_PLL1DIVR1             (STM32_RCC_BASE + STM32_RCC_PLL1DIVR1_OFFSET)
#define STM32_RCC_PLL1FRACR             (STM32_RCC_BASE + STM32_RCC_PLL1FRACR_OFFSET)
#define STM32_RCC_PLL2DIVR1             (STM32_RCC_BASE + STM32_RCC_PLL2DIVR1_OFFSET)
#define STM32_RCC_PLL2FRACR             (STM32_RCC_BASE + STM32_RCC_PLL2FRACR_OFFSET)
#define STM32_RCC_PLL3DIVR1             (STM32_RCC_BASE + STM32_RCC_PLL3DIVR1_OFFSET)
#define STM32_RCC_PLL3FRACR             (STM32_RCC_BASE + STM32_RCC_PLL3FRACR_OFFSET)
#define STM32_RCC_CCIPR1                (STM32_RCC_BASE + STM32_RCC_CCIPR1_OFFSET)
#define STM32_RCC_CCIPR2                (STM32_RCC_BASE + STM32_RCC_CCIPR2_OFFSET)
#define STM32_RCC_CCIPR3                (STM32_RCC_BASE + STM32_RCC_CCIPR3_OFFSET)
#define STM32_RCC_CCIPR4                (STM32_RCC_BASE + STM32_RCC_CCIPR4_OFFSET)
#define STM32_RCC_CIER                  (STM32_RCC_BASE + STM32_RCC_CIER_OFFSET)
#define STM32_RCC_CIFR                  (STM32_RCC_BASE + STM32_RCC_CIFR_OFFSET)
#define STM32_RCC_CICR                  (STM32_RCC_BASE + STM32_RCC_CICR_OFFSET)
#define STM32_RCC_BDCR                  (STM32_RCC_BASE + STM32_RCC_BDCR_OFFSET)
#define STM32_RCC_CSR                   (STM32_RCC_BASE + STM32_RCC_CSR_OFFSET)
#define STM32_RCC_AHB5RSTR              (STM32_RCC_BASE + STM32_RCC_AHB5RSTR_OFFSET)
#define STM32_RCC_AHB1RSTR              (STM32_RCC_BASE + STM32_RCC_AHB1RSTR_OFFSET)
#define STM32_RCC_AHB2RSTR              (STM32_RCC_BASE + STM32_RCC_AHB2RSTR_OFFSET)
#define STM32_RCC_AHB4RSTR              (STM32_RCC_BASE + STM32_RCC_AHB4RSTR_OFFSET)
#define STM32_RCC_APB5RSTR              (STM32_RCC_BASE + STM32_RCC_APB5RSTR_OFFSET)
#define STM32_RCC_APB1RSTR1             (STM32_RCC_BASE + STM32_RCC_APB1RSTR1_OFFSET)
#define STM32_RCC_APB1RSTR2             (STM32_RCC_BASE + STM32_RCC_APB1RSTR2_OFFSET)
#define STM32_RCC_APB2RSTR              (STM32_RCC_BASE + STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_APB4RSTR              (STM32_RCC_BASE + STM32_RCC_APB4RSTR_OFFSET)
#define STM32_RCC_AHB3RSTR              (STM32_RCC_BASE + STM32_RCC_AHB3RSTR_OFFSET)
#define STM32_RCC_CKGDISR               (STM32_RCC_BASE + STM32_RCC_CKGDISR_OFFSET)
#define STM32_RCC_PLL1DIVR2             (STM32_RCC_BASE + STM32_RCC_PLL1DIVR2_OFFSET)
#define STM32_RCC_PLL2DIVR2             (STM32_RCC_BASE + STM32_RCC_PLL2DIVR2_OFFSET)
#define STM32_RCC_PLL3DIVR2             (STM32_RCC_BASE + STM32_RCC_PLL3DIVR2_OFFSET)
#define STM32_RCC_PLL1SSCGR             (STM32_RCC_BASE + STM32_RCC_PLL1SSCGR_OFFSET)
#define STM32_RCC_PLL2SSCGR             (STM32_RCC_BASE + STM32_RCC_PLL2SSCGR_OFFSET)
#define STM32_RCC_PLL3SSCGR             (STM32_RCC_BASE + STM32_RCC_PLL3SSCGR_OFFSET)
#define STM32_RCC_CKPROTR               (STM32_RCC_BASE + STM32_RCC_CKPROTR_OFFSET)
#define STM32_RCC_RSR                   (STM32_RCC_BASE + STM32_RCC_RSR_OFFSET)
#define STM32_RCC_AHB5ENR               (STM32_RCC_BASE + STM32_RCC_AHB5ENR_OFFSET)
#define STM32_RCC_AHB1ENR               (STM32_RCC_BASE + STM32_RCC_AHB1ENR_OFFSET)
#define STM32_RCC_AHB2ENR               (STM32_RCC_BASE + STM32_RCC_AHB2ENR_OFFSET)
#define STM32_RCC_AHB4ENR               (STM32_RCC_BASE + STM32_RCC_AHB4ENR_OFFSET)
#define STM32_RCC_APB5ENR               (STM32_RCC_BASE + STM32_RCC_APB5ENR_OFFSET)
#define STM32_RCC_APB1ENR1              (STM32_RCC_BASE + STM32_RCC_APB1ENR1_OFFSET)
#define STM32_RCC_APB1ENR2              (STM32_RCC_BASE + STM32_RCC_APB1ENR2_OFFSET)
#define STM32_RCC_APB2ENR               (STM32_RCC_BASE + STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_APB4ENR               (STM32_RCC_BASE + STM32_RCC_APB4ENR_OFFSET)
#define STM32_RCC_AHB3ENR               (STM32_RCC_BASE + STM32_RCC_AHB3ENR_OFFSET)
#define STM32_RCC_AHB5LPENR             (STM32_RCC_BASE + STM32_RCC_AHB5LPENR_OFFSET)
#define STM32_RCC_AHB1LPENR             (STM32_RCC_BASE + STM32_RCC_AHB1LPENR_OFFSET)
#define STM32_RCC_AHB2LPENR             (STM32_RCC_BASE + STM32_RCC_AHB2LPENR_OFFSET)
#define STM32_RCC_AHB4LPENR             (STM32_RCC_BASE + STM32_RCC_AHB4LPENR_OFFSET)
#define STM32_RCC_AHB3LPENR             (STM32_RCC_BASE + STM32_RCC_AHB3LPENR_OFFSET)
#define STM32_RCC_APB1LPENR1            (STM32_RCC_BASE + STM32_RCC_APB1LPENR1_OFFSET)
#define STM32_RCC_APB1LPENR2            (STM32_RCC_BASE + STM32_RCC_APB1LPENR2_OFFSET)
#define STM32_RCC_APB2LPENR             (STM32_RCC_BASE + STM32_RCC_APB2LPENR_OFFSET)
#define STM32_RCC_APB4LPENR             (STM32_RCC_BASE + STM32_RCC_APB4LPENR_OFFSET)
#define STM32_RCC_APB5LPENR             (STM32_RCC_BASE + STM32_RCC_APB5LPENR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Clock control register ***************************************************/

#define RCC_CR_HSION                    (1 << 0)  /* Internal high-speed clock enable */
#define RCC_CR_HSIKERON                 (1 << 1)  /* HSI kernel clock in stop mode */
#define RCC_CR_HSIRDY                   (1 << 2)  /* Internal high-speed clock ready */
#define RCC_CR_HSIDIV_SHIFT             (3)       /* Bits 3-4: HSI clock divider */
#define RCC_CR_HSIDIV_MASK              (3 << RCC_CR_HSIDIV_SHIFT)
#  define RCC_CR_HSIDIV_1               (0 << RCC_CR_HSIDIV_SHIFT)
#  define RCC_CR_HSIDIV_2               (1 << RCC_CR_HSIDIV_SHIFT)
#  define RCC_CR_HSIDIV_4               (2 << RCC_CR_HSIDIV_SHIFT)
#  define RCC_CR_HSIDIV_8               (3 << RCC_CR_HSIDIV_SHIFT)
#define RCC_CR_HSIDIVF                  (1 << 5)  /* HSI divider flag */
#define RCC_CR_CSION                    (1 << 7)  /* Internal CSI clock enable */
#define RCC_CR_CSIRDY                   (1 << 8)  /* Internal CSI clock ready */
#define RCC_CR_CSIKERON                 (1 << 9)  /* CSI kernel clock in stop mode */
#define RCC_CR_HSI48ON                  (1 << 12) /* Internal HSI48 clock enable */
#define RCC_CR_HSI48RDY                 (1 << 13) /* Internal HSI48 clock ready */
#define RCC_CR_HSEON                    (1 << 16) /* External high-speed clock enable */
#define RCC_CR_HSERDY                   (1 << 17) /* External high-speed clock ready */
#define RCC_CR_HSEBYP                   (1 << 18) /* External clock bypass */
#define RCC_CR_HSEEXT                   (1 << 19) /* HSE external source selection */
#define RCC_CR_HSECSSON                 (1 << 20) /* HSE clock security enable */
#define RCC_CR_PLL1ON                   (1 << 24) /* PLL1 enable */
#define RCC_CR_PLL1RDY                  (1 << 25) /* PLL1 ready */
#define RCC_CR_PLL2ON                   (1 << 26) /* PLL2 enable */
#define RCC_CR_PLL2RDY                  (1 << 27) /* PLL2 ready */
#define RCC_CR_PLL3ON                   (1 << 28) /* PLL3 enable */
#define RCC_CR_PLL3RDY                  (1 << 29) /* PLL3 ready */

/* Oscillator configuration registers ***************************************/

#define RCC_HSICFGR_HSICAL_SHIFT        (0)  /* Bits 0-11: HSI calibration */
#define RCC_HSICFGR_HSICAL_MASK         (0xfff << RCC_HSICFGR_HSICAL_SHIFT)
#define RCC_HSICFGR_HSITRIM_SHIFT       (24) /* Bits 24-30: HSI trimming */
#define RCC_HSICFGR_HSITRIM_MASK        (0x7f << RCC_HSICFGR_HSITRIM_SHIFT)
#define RCC_HSICFGR_HSITRIM(n)          ((n) << RCC_HSICFGR_HSITRIM_SHIFT)
#define RCC_CRRCR_HSI48CAL_SHIFT        (0)  /* Bits 0-9: HSI48 calibration */
#define RCC_CRRCR_HSI48CAL_MASK         (0x3ff << RCC_CRRCR_HSI48CAL_SHIFT)
#define RCC_CSICFGR_CSICAL_SHIFT        (0)  /* Bits 0-9: CSI calibration */
#define RCC_CSICFGR_CSICAL_MASK         (0x3ff << RCC_CSICFGR_CSICAL_SHIFT)
#define RCC_CSICFGR_CSITRIM_SHIFT       (24) /* Bits 24-28: CSI trimming */
#define RCC_CSICFGR_CSITRIM_MASK        (31 << RCC_CSICFGR_CSITRIM_SHIFT)
#define RCC_CSICFGR_CSITRIM(n)          ((n) << RCC_CSICFGR_CSITRIM_SHIFT)

/* Clock configuration registers ********************************************/

#define RCC_CFGR_SW_SHIFT               (0)       /* Bits 0-2: System clock switch */
#define RCC_CFGR_SW_MASK                (7 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_HSI               (0 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_CSI               (1 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_HSE               (2 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_PLL1              (3 << RCC_CFGR_SW_SHIFT)
#define RCC_CFGR_SWS_SHIFT              (3)       /* Bits 3-5: System clock status */
#define RCC_CFGR_SWS_MASK               (7 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_HSI              (0 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_CSI              (1 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_HSE              (2 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_PLL1             (3 << RCC_CFGR_SWS_SHIFT)
#define RCC_CFGR_STOPWUCK               (1 << 6)  /* System wakeup clock select */
#define RCC_CFGR_STOPKERWUCK            (1 << 7)  /* Kernel wakeup clock select */
#define RCC_CFGR_RTCPRE_SHIFT           (8)       /* Bits 8-13: HSE RTC prescaler */
#define RCC_CFGR_RTCPRE_MASK            (0x3f << RCC_CFGR_RTCPRE_SHIFT)
#define RCC_CFGR_RTCPRE(n)              ((n) << RCC_CFGR_RTCPRE_SHIFT)
#define RCC_CFGR_TIMPRE                 (1 << 15) /* Timer clock prescaler */
#define RCC_CFGR_MCO1PRE_SHIFT          (18)      /* Bits 18-21: MCO1 prescaler */
#define RCC_CFGR_MCO1PRE_MASK           (15 << RCC_CFGR_MCO1PRE_SHIFT)
#define RCC_CFGR_MCO1PRE(n)             ((n) << RCC_CFGR_MCO1PRE_SHIFT)
#define RCC_CFGR_MCO1_SHIFT             (22)      /* Bits 22-24: MCO1 source */
#define RCC_CFGR_MCO1_MASK              (7 << RCC_CFGR_MCO1_SHIFT)
#define RCC_CFGR_MCO2PRE_SHIFT          (25)      /* Bits 25-28: MCO2 prescaler */
#define RCC_CFGR_MCO2PRE_MASK           (15 << RCC_CFGR_MCO2PRE_SHIFT)
#define RCC_CFGR_MCO2PRE(n)             ((n) << RCC_CFGR_MCO2PRE_SHIFT)
#define RCC_CFGR_MCO2_SHIFT             (29)      /* Bits 29-31: MCO2 source */
#define RCC_CFGR_MCO2_MASK              (7u << RCC_CFGR_MCO2_SHIFT)

#define RCC_CDCFGR_CPRE_SHIFT           (0) /* CPU clock prescaler */
#define RCC_CDCFGR_CPRE_MASK            (15 << RCC_CDCFGR_CPRE_SHIFT)
#  define RCC_CDCFGR_CPRE_SYSCLK        (0 << RCC_CDCFGR_CPRE_SHIFT)
#  define RCC_CDCFGR_CPRE_SYSCLKd2      (8 << RCC_CDCFGR_CPRE_SHIFT)
#  define RCC_CDCFGR_CPRE_SYSCLKd4      (9 << RCC_CDCFGR_CPRE_SHIFT)
#  define RCC_CDCFGR_CPRE_SYSCLKd8      (10 << RCC_CDCFGR_CPRE_SHIFT)
#  define RCC_CDCFGR_CPRE_SYSCLKd16     (11 << RCC_CDCFGR_CPRE_SHIFT)
#  define RCC_CDCFGR_CPRE_SYSCLKd64     (12 << RCC_CDCFGR_CPRE_SHIFT)
#  define RCC_CDCFGR_CPRE_SYSCLKd128    (13 << RCC_CDCFGR_CPRE_SHIFT)
#  define RCC_CDCFGR_CPRE_SYSCLKd256    (14 << RCC_CDCFGR_CPRE_SHIFT)
#  define RCC_CDCFGR_CPRE_SYSCLKd512    (15 << RCC_CDCFGR_CPRE_SHIFT)

#define RCC_BMCFGR_HPRE_SHIFT           (0) /* AHB clock prescaler */
#define RCC_BMCFGR_HPRE_MASK            (15 << RCC_BMCFGR_HPRE_SHIFT)
#  define RCC_BMCFGR_HPRE_SYSCLK        (0 << RCC_BMCFGR_HPRE_SHIFT)
#  define RCC_BMCFGR_HPRE_SYSCLKd2      (8 << RCC_BMCFGR_HPRE_SHIFT)
#  define RCC_BMCFGR_HPRE_SYSCLKd4      (9 << RCC_BMCFGR_HPRE_SHIFT)
#  define RCC_BMCFGR_HPRE_SYSCLKd8      (10 << RCC_BMCFGR_HPRE_SHIFT)
#  define RCC_BMCFGR_HPRE_SYSCLKd16     (11 << RCC_BMCFGR_HPRE_SHIFT)
#  define RCC_BMCFGR_HPRE_SYSCLKd64     (12 << RCC_BMCFGR_HPRE_SHIFT)
#  define RCC_BMCFGR_HPRE_SYSCLKd128    (13 << RCC_BMCFGR_HPRE_SHIFT)
#  define RCC_BMCFGR_HPRE_SYSCLKd256    (14 << RCC_BMCFGR_HPRE_SHIFT)
#  define RCC_BMCFGR_HPRE_SYSCLKd512    (15 << RCC_BMCFGR_HPRE_SHIFT)

#define RCC_APBCFGR_PPRE1_SHIFT         (0)  /* APB1 clock prescaler */
#define RCC_APBCFGR_PPRE1_MASK          (7 << RCC_APBCFGR_PPRE1_SHIFT)
#  define RCC_APBCFGR_PPRE1_HCLK        (0 << RCC_APBCFGR_PPRE1_SHIFT)
#  define RCC_APBCFGR_PPRE1_HCLKd2      (4 << RCC_APBCFGR_PPRE1_SHIFT)
#  define RCC_APBCFGR_PPRE1_HCLKd4      (5 << RCC_APBCFGR_PPRE1_SHIFT)
#  define RCC_APBCFGR_PPRE1_HCLKd8      (6 << RCC_APBCFGR_PPRE1_SHIFT)
#  define RCC_APBCFGR_PPRE1_HCLKd16     (7 << RCC_APBCFGR_PPRE1_SHIFT)
#define RCC_APBCFGR_PPRE2_SHIFT         (4)  /* APB2 clock prescaler */
#define RCC_APBCFGR_PPRE2_MASK          (7 << RCC_APBCFGR_PPRE2_SHIFT)
#  define RCC_APBCFGR_PPRE2_HCLK        (0 << RCC_APBCFGR_PPRE2_SHIFT)
#  define RCC_APBCFGR_PPRE2_HCLKd2      (4 << RCC_APBCFGR_PPRE2_SHIFT)
#  define RCC_APBCFGR_PPRE2_HCLKd4      (5 << RCC_APBCFGR_PPRE2_SHIFT)
#  define RCC_APBCFGR_PPRE2_HCLKd8      (6 << RCC_APBCFGR_PPRE2_SHIFT)
#  define RCC_APBCFGR_PPRE2_HCLKd16     (7 << RCC_APBCFGR_PPRE2_SHIFT)
#define RCC_APBCFGR_PPRE4_SHIFT         (8)  /* APB4 clock prescaler */
#define RCC_APBCFGR_PPRE4_MASK          (7 << RCC_APBCFGR_PPRE4_SHIFT)
#  define RCC_APBCFGR_PPRE4_HCLK        (0 << RCC_APBCFGR_PPRE4_SHIFT)
#  define RCC_APBCFGR_PPRE4_HCLKd2      (4 << RCC_APBCFGR_PPRE4_SHIFT)
#  define RCC_APBCFGR_PPRE4_HCLKd4      (5 << RCC_APBCFGR_PPRE4_SHIFT)
#  define RCC_APBCFGR_PPRE4_HCLKd8      (6 << RCC_APBCFGR_PPRE4_SHIFT)
#  define RCC_APBCFGR_PPRE4_HCLKd16     (7 << RCC_APBCFGR_PPRE4_SHIFT)
#define RCC_APBCFGR_PPRE5_SHIFT         (12) /* APB5 clock prescaler */
#define RCC_APBCFGR_PPRE5_MASK          (7 << RCC_APBCFGR_PPRE5_SHIFT)
#  define RCC_APBCFGR_PPRE5_HCLK        (0 << RCC_APBCFGR_PPRE5_SHIFT)
#  define RCC_APBCFGR_PPRE5_HCLKd2      (4 << RCC_APBCFGR_PPRE5_SHIFT)
#  define RCC_APBCFGR_PPRE5_HCLKd4      (5 << RCC_APBCFGR_PPRE5_SHIFT)
#  define RCC_APBCFGR_PPRE5_HCLKd8      (6 << RCC_APBCFGR_PPRE5_SHIFT)
#  define RCC_APBCFGR_PPRE5_HCLKd16     (7 << RCC_APBCFGR_PPRE5_SHIFT)

/* PLL registers ************************************************************/

#define RCC_PLLCKSELR_PLLSRC_SHIFT      (0)  /* PLL clock source */
#define RCC_PLLCKSELR_PLLSRC_MASK       (3 << RCC_PLLCKSELR_PLLSRC_SHIFT)
#  define RCC_PLLCKSELR_PLLSRC_HSI      (0 << RCC_PLLCKSELR_PLLSRC_SHIFT)
#  define RCC_PLLCKSELR_PLLSRC_CSI      (1 << RCC_PLLCKSELR_PLLSRC_SHIFT)
#  define RCC_PLLCKSELR_PLLSRC_HSE      (2 << RCC_PLLCKSELR_PLLSRC_SHIFT)
#define RCC_PLLCKSELR_DIVM1_SHIFT       (4)  /* PLL1 M prescaler */
#define RCC_PLLCKSELR_DIVM2_SHIFT       (12) /* PLL2 M prescaler */
#define RCC_PLLCKSELR_DIVM3_SHIFT       (20) /* PLL3 M prescaler */
#define RCC_PLLCKSELR_DIVM_MASK(s)      (0x3f << (s))
#  define RCC_PLLCKSELR_DIVM(n,s)       ((n) << (s))
#  define RCC_PLLCKSELR_DIVM1(n)        RCC_PLLCKSELR_DIVM(n, RCC_PLLCKSELR_DIVM1_SHIFT)
#  define RCC_PLLCKSELR_DIVM2(n)        RCC_PLLCKSELR_DIVM(n, RCC_PLLCKSELR_DIVM2_SHIFT)
#  define RCC_PLLCKSELR_DIVM3(n)        RCC_PLLCKSELR_DIVM(n, RCC_PLLCKSELR_DIVM3_SHIFT)

#define RCC_PLLCFGR_PLL1FRACEN          (1 << 0)  /* PLL1 fractional enable */
#define RCC_PLLCFGR_PLL1VCOSEL          (1 << 1)  /* PLL1 VCO selection */
#define RCC_PLLCFGR_PLL1SSCGEN          (1 << 2)
#define RCC_PLLCFGR_PLL1RGE_SHIFT       (3)       /* PLL1 input freq range */
#define RCC_PLLCFGR_PLL1RGE_MASK        (3 << RCC_PLLCFGR_PLL1RGE_SHIFT)
#  define RCC_PLLCFGR_PLL1RGE_1_2       (0 << RCC_PLLCFGR_PLL1RGE_SHIFT)
#  define RCC_PLLCFGR_PLL1RGE_2_4       (1 << RCC_PLLCFGR_PLL1RGE_SHIFT)
#  define RCC_PLLCFGR_PLL1RGE_4_8       (2 << RCC_PLLCFGR_PLL1RGE_SHIFT)
#  define RCC_PLLCFGR_PLL1RGE_8_16      (3 << RCC_PLLCFGR_PLL1RGE_SHIFT)
#define RCC_PLLCFGR_PLL1PEN             (1 << 5)  /* PLL1 P output enable */
#define RCC_PLLCFGR_PLL1QEN             (1 << 6)  /* PLL1 Q output enable */
#define RCC_PLLCFGR_PLL1REN             (1 << 7)  /* PLL1 R output enable */
#define RCC_PLLCFGR_PLL1SEN             (1 << 8)  /* PLL1 S output enable */
#define RCC_PLLCFGR_PLL2FRACEN          (1 << 11) /* PLL2 fractional enable */
#define RCC_PLLCFGR_PLL2VCOSEL          (1 << 12) /* PLL2 VCO selection */
#define RCC_PLLCFGR_PLL2SSCGEN          (1 << 13)
#define RCC_PLLCFGR_PLL2RGE_SHIFT       (14)      /* PLL2 input freq range */
#define RCC_PLLCFGR_PLL2RGE_MASK        (3 << RCC_PLLCFGR_PLL2RGE_SHIFT)
#define RCC_PLLCFGR_PLL2PEN             (1 << 16) /* PLL2 P output enable */
#define RCC_PLLCFGR_PLL2QEN             (1 << 17) /* PLL2 Q output enable */
#define RCC_PLLCFGR_PLL2REN             (1 << 18) /* PLL2 R output enable */
#define RCC_PLLCFGR_PLL2SEN             (1 << 19) /* PLL2 S output enable */
#define RCC_PLLCFGR_PLL2TEN             (1 << 20) /* PLL2 T output enable */
#define RCC_PLLCFGR_PLL3FRACEN          (1 << 22) /* PLL3 fractional enable */
#define RCC_PLLCFGR_PLL3VCOSEL          (1 << 23) /* PLL3 VCO selection */
#define RCC_PLLCFGR_PLL3SSCGEN          (1 << 24)
#define RCC_PLLCFGR_PLL3RGE_SHIFT       (25)      /* PLL3 input freq range */
#define RCC_PLLCFGR_PLL3RGE_MASK        (3 << RCC_PLLCFGR_PLL3RGE_SHIFT)
#define RCC_PLLCFGR_PLL3PEN             (1 << 27) /* PLL3 P output enable */
#define RCC_PLLCFGR_PLL3QEN             (1 << 28) /* PLL3 Q output enable */
#define RCC_PLLCFGR_PLL3REN             (1 << 29) /* PLL3 R output enable */
#define RCC_PLLCFGR_PLL3SEN             (1 << 30) /* PLL3 S output enable */

#define RCC_PLLDIVR_DIVN_SHIFT          (0)  /* PLL N multiplier */
#define RCC_PLLDIVR_DIVP_SHIFT          (9)  /* PLL P divider */
#define RCC_PLLDIVR_DIVQ_SHIFT          (16) /* PLL Q divider */
#define RCC_PLLDIVR_DIVR_SHIFT          (24) /* PLL R divider */
#  define RCC_PLLDIVR_DIVN(n)           (((n) - 1) << RCC_PLLDIVR_DIVN_SHIFT)
#  define RCC_PLLDIVR_DIVP(n)           (((n) - 1) << RCC_PLLDIVR_DIVP_SHIFT)
#  define RCC_PLLDIVR_DIVQ(n)           (((n) - 1) << RCC_PLLDIVR_DIVQ_SHIFT)
#  define RCC_PLLDIVR_DIVR(n)           (((n) - 1) << RCC_PLLDIVR_DIVR_SHIFT)
#define RCC_PLL1DIVR1_N(n)              RCC_PLLDIVR_DIVN(n)
#define RCC_PLL1DIVR1_P(n)              RCC_PLLDIVR_DIVP(n)
#define RCC_PLL1DIVR1_Q(n)              RCC_PLLDIVR_DIVQ(n)
#define RCC_PLL1DIVR1_R(n)              RCC_PLLDIVR_DIVR(n)
#define RCC_PLL2DIVR1_N(n)              RCC_PLLDIVR_DIVN(n)
#define RCC_PLL2DIVR1_P(n)              RCC_PLLDIVR_DIVP(n)
#define RCC_PLL2DIVR1_Q(n)              RCC_PLLDIVR_DIVQ(n)
#define RCC_PLL2DIVR1_R(n)              RCC_PLLDIVR_DIVR(n)
#define RCC_PLL3DIVR1_N(n)              RCC_PLLDIVR_DIVN(n)
#define RCC_PLL3DIVR1_P(n)              RCC_PLLDIVR_DIVP(n)
#define RCC_PLL3DIVR1_Q(n)              RCC_PLLDIVR_DIVQ(n)
#define RCC_PLL3DIVR1_R(n)              RCC_PLLDIVR_DIVR(n)
#define RCC_PLLFRACR_FRACN_SHIFT        (3)  /* PLL fractional value */
#define RCC_PLLFRACR_FRACN_MASK         (0x1fff << RCC_PLLFRACR_FRACN_SHIFT)
#  define RCC_PLLFRACR_FRACN(n)         ((n) << RCC_PLLFRACR_FRACN_SHIFT)

/* Secondary PLL dividers and spread-spectrum registers *********************/

#define RCC_PLL1DIVR2_DIVS_SHIFT        (0) /* PLL1 S divider */
#define RCC_PLL1DIVR2_DIVS_MASK         (7 << RCC_PLL1DIVR2_DIVS_SHIFT)
#  define RCC_PLL1DIVR2_DIVS(n)         (((n) - 1) << RCC_PLL1DIVR2_DIVS_SHIFT)
#define RCC_PLL2DIVR2_DIVS_SHIFT        (0) /* PLL2 S divider */
#define RCC_PLL2DIVR2_DIVS_MASK         (7 << RCC_PLL2DIVR2_DIVS_SHIFT)
#  define RCC_PLL2DIVR2_DIVS(n)         (((n) - 1) << RCC_PLL2DIVR2_DIVS_SHIFT)
#define RCC_PLL2DIVR2_DIVT_SHIFT        (8) /* PLL2 T divider */
#define RCC_PLL2DIVR2_DIVT_MASK         (7 << RCC_PLL2DIVR2_DIVT_SHIFT)
#  define RCC_PLL2DIVR2_DIVT(n)         (((n) - 1) << RCC_PLL2DIVR2_DIVT_SHIFT)
#define RCC_PLL3DIVR2_DIVS_SHIFT        (0) /* PLL3 S divider */
#define RCC_PLL3DIVR2_DIVS_MASK         (7 << RCC_PLL3DIVR2_DIVS_SHIFT)
#  define RCC_PLL3DIVR2_DIVS(n)         (((n) - 1) << RCC_PLL3DIVR2_DIVS_SHIFT)

#define RCC_PLL1SSCGR_MODPER_SHIFT      (0)  /* Modulation period */
#define RCC_PLL1SSCGR_MODPER_MASK       (0x1fff << RCC_PLL1SSCGR_MODPER_SHIFT)
#  define RCC_PLL1SSCGR_MODPER(n)       ((n) << RCC_PLL1SSCGR_MODPER_SHIFT)
#define RCC_PLL1SSCGR_TPDFNDIS          (1 << 13)
#define RCC_PLL1SSCGR_RPDFNDIS          (1 << 14)
#define RCC_PLL1SSCGR_SPREADSEL         (1 << 15)
#define RCC_PLL1SSCGR_INCSTEP_SHIFT     (16) /* Increment step */
#define RCC_PLL1SSCGR_INCSTEP_MASK      (0x7fff << RCC_PLL1SSCGR_INCSTEP_SHIFT)
#  define RCC_PLL1SSCGR_INCSTEP(n)      ((n) << RCC_PLL1SSCGR_INCSTEP_SHIFT)

#define RCC_PLL2SSCGR_MODPER_SHIFT      (0)  /* Modulation period */
#define RCC_PLL2SSCGR_MODPER_MASK       (0x1fff << RCC_PLL2SSCGR_MODPER_SHIFT)
#  define RCC_PLL2SSCGR_MODPER(n)       ((n) << RCC_PLL2SSCGR_MODPER_SHIFT)
#define RCC_PLL2SSCGR_TPDFNDIS          (1 << 13)
#define RCC_PLL2SSCGR_RPDFNDIS          (1 << 14)
#define RCC_PLL2SSCGR_SPREADSEL         (1 << 15)
#define RCC_PLL2SSCGR_INCSTEP_SHIFT     (16) /* Increment step */
#define RCC_PLL2SSCGR_INCSTEP_MASK      (0x7fff << RCC_PLL2SSCGR_INCSTEP_SHIFT)
#  define RCC_PLL2SSCGR_INCSTEP(n)      ((n) << RCC_PLL2SSCGR_INCSTEP_SHIFT)

#define RCC_PLL3SSCGR_MODPER_SHIFT      (0)  /* Modulation period */
#define RCC_PLL3SSCGR_MODPER_MASK       (0x1fff << RCC_PLL3SSCGR_MODPER_SHIFT)
#  define RCC_PLL3SSCGR_MODPER(n)       ((n) << RCC_PLL3SSCGR_MODPER_SHIFT)
#define RCC_PLL3SSCGR_TPDFNDIS          (1 << 13)
#define RCC_PLL3SSCGR_RPDFNDIS          (1 << 14)
#define RCC_PLL3SSCGR_SPREADSEL         (1 << 15)
#define RCC_PLL3SSCGR_INCSTEP_SHIFT     (16) /* Increment step */
#define RCC_PLL3SSCGR_INCSTEP_MASK      (0x7fff << RCC_PLL3SSCGR_INCSTEP_SHIFT)
#  define RCC_PLL3SSCGR_INCSTEP(n)      ((n) << RCC_PLL3SSCGR_INCSTEP_SHIFT)

/* Kernel clock selection registers *****************************************/

#define RCC_CCIPR1_FMCSEL_SHIFT         (0)  /* FMC clock source */
#define RCC_CCIPR1_FMCSEL_MASK          (3 << RCC_CCIPR1_FMCSEL_SHIFT)
#  define RCC_CCIPR1_FMCSEL(n)          ((n) << RCC_CCIPR1_FMCSEL_SHIFT)
#define RCC_CCIPR1_SDMMC12SEL_SHIFT     (2)  /* SDMMC12 clock source */
#define RCC_CCIPR1_SDMMC12SEL_MASK      (1 << RCC_CCIPR1_SDMMC12SEL_SHIFT)
#  define RCC_CCIPR1_SDMMC12SEL(n)      ((n) << RCC_CCIPR1_SDMMC12SEL_SHIFT)
#define RCC_CCIPR1_XSPI1SEL_SHIFT       (4)  /* XSPI1 clock source */
#define RCC_CCIPR1_XSPI1SEL_MASK        (3 << RCC_CCIPR1_XSPI1SEL_SHIFT)
#  define RCC_CCIPR1_XSPI1SEL(n)        ((n) << RCC_CCIPR1_XSPI1SEL_SHIFT)
#  define RCC_CCIPR1_XSPI1SEL_HCLK      (0 << RCC_CCIPR1_XSPI1SEL_SHIFT)
#define RCC_CCIPR1_XSPI2SEL_SHIFT       (6)  /* XSPI2 clock source */
#define RCC_CCIPR1_XSPI2SEL_MASK        (3 << RCC_CCIPR1_XSPI2SEL_SHIFT)
#  define RCC_CCIPR1_XSPI2SEL(n)        ((n) << RCC_CCIPR1_XSPI2SEL_SHIFT)
#  define RCC_CCIPR1_XSPI2SEL_HCLK      (0 << RCC_CCIPR1_XSPI2SEL_SHIFT)
#define RCC_CCIPR1_USBREFCKSEL_SHIFT    (8)  /* USBREFCK clock source */
#define RCC_CCIPR1_USBREFCKSEL_MASK     (15 << RCC_CCIPR1_USBREFCKSEL_SHIFT)
#  define RCC_CCIPR1_USBREFCKSEL(n)     ((n) << RCC_CCIPR1_USBREFCKSEL_SHIFT)
#define RCC_CCIPR1_USBPHYCSEL_SHIFT     (12) /* USBPHYC clock source */
#define RCC_CCIPR1_USBPHYCSEL_MASK      (3 << RCC_CCIPR1_USBPHYCSEL_SHIFT)
#  define RCC_CCIPR1_USBPHYCSEL(n)      ((n) << RCC_CCIPR1_USBPHYCSEL_SHIFT)
#define RCC_CCIPR1_OTGFSSEL_SHIFT       (14) /* OTGFS clock source */
#define RCC_CCIPR1_OTGFSSEL_MASK        (3 << RCC_CCIPR1_OTGFSSEL_SHIFT)
#  define RCC_CCIPR1_OTGFSSEL(n)        ((n) << RCC_CCIPR1_OTGFSSEL_SHIFT)
#define RCC_CCIPR1_ETH1REFCKSEL_SHIFT   (16) /* ETH1REFCK clock source */
#define RCC_CCIPR1_ETH1REFCKSEL_MASK    (3 << RCC_CCIPR1_ETH1REFCKSEL_SHIFT)
#  define RCC_CCIPR1_ETH1REFCKSEL(n)    ((n) << RCC_CCIPR1_ETH1REFCKSEL_SHIFT)
#define RCC_CCIPR1_ETH1PHYCKSEL_SHIFT   (18) /* ETH1PHYCK clock source */
#define RCC_CCIPR1_ETH1PHYCKSEL_MASK    (1 << RCC_CCIPR1_ETH1PHYCKSEL_SHIFT)
#  define RCC_CCIPR1_ETH1PHYCKSEL(n)    ((n) << RCC_CCIPR1_ETH1PHYCKSEL_SHIFT)
#define RCC_CCIPR1_ADF1SEL_SHIFT        (20) /* ADF1 clock source */
#define RCC_CCIPR1_ADF1SEL_MASK         (7 << RCC_CCIPR1_ADF1SEL_SHIFT)
#  define RCC_CCIPR1_ADF1SEL(n)         ((n) << RCC_CCIPR1_ADF1SEL_SHIFT)
#define RCC_CCIPR1_ADCSEL_SHIFT         (24) /* ADC clock source */
#define RCC_CCIPR1_ADCSEL_MASK          (3 << RCC_CCIPR1_ADCSEL_SHIFT)
#  define RCC_CCIPR1_ADCSEL(n)          ((n) << RCC_CCIPR1_ADCSEL_SHIFT)
#define RCC_CCIPR1_PSSISEL_SHIFT        (27) /* PSSI clock source */
#define RCC_CCIPR1_PSSISEL_MASK         (1 << RCC_CCIPR1_PSSISEL_SHIFT)
#  define RCC_CCIPR1_PSSISEL(n)         ((n) << RCC_CCIPR1_PSSISEL_SHIFT)
#define RCC_CCIPR1_CKPERSEL_SHIFT       (28) /* CKPER clock source */
#define RCC_CCIPR1_CKPERSEL_MASK        (3 << RCC_CCIPR1_CKPERSEL_SHIFT)
#  define RCC_CCIPR1_CKPERSEL(n)        ((n) << RCC_CCIPR1_CKPERSEL_SHIFT)

#define RCC_CCIPR2_UART234578SEL_SHIFT  (0)  /* UART234578 clock source */
#define RCC_CCIPR2_UART234578SEL_MASK   (7 << RCC_CCIPR2_UART234578SEL_SHIFT)
#  define RCC_CCIPR2_UART234578SEL(n)   ((n) << RCC_CCIPR2_UART234578SEL_SHIFT)
#define RCC_CCIPR2_SPI23SEL_SHIFT       (4)  /* SPI23 clock source */
#define RCC_CCIPR2_SPI23SEL_MASK        (7 << RCC_CCIPR2_SPI23SEL_SHIFT)
#  define RCC_CCIPR2_SPI23SEL(n)        ((n) << RCC_CCIPR2_SPI23SEL_SHIFT)
#define RCC_CCIPR2_I2C23SEL_SHIFT       (8)  /* I2C23 clock source */
#define RCC_CCIPR2_I2C23SEL_MASK        (3 << RCC_CCIPR2_I2C23SEL_SHIFT)
#  define RCC_CCIPR2_I2C23SEL(n)        ((n) << RCC_CCIPR2_I2C23SEL_SHIFT)
#define RCC_CCIPR2_I2C1_I3C1SEL_SHIFT   (12) /* I2C1/I3C1 clock source */
#define RCC_CCIPR2_I2C1_I3C1SEL_MASK    (3 << RCC_CCIPR2_I2C1_I3C1SEL_SHIFT)
#  define RCC_CCIPR2_I2C1_I3C1SEL(n)    ((n) << RCC_CCIPR2_I2C1_I3C1SEL_SHIFT)
#define RCC_CCIPR2_LPTIM1SEL_SHIFT      (16) /* LPTIM1 clock source */
#define RCC_CCIPR2_LPTIM1SEL_MASK       (7 << RCC_CCIPR2_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM1SEL(n)       ((n) << RCC_CCIPR2_LPTIM1SEL_SHIFT)
#define RCC_CCIPR2_FDCANSEL_SHIFT       (22) /* FDCAN clock source */
#define RCC_CCIPR2_FDCANSEL_MASK        (3 << RCC_CCIPR2_FDCANSEL_SHIFT)
#  define RCC_CCIPR2_FDCANSEL(n)        ((n) << RCC_CCIPR2_FDCANSEL_SHIFT)
#define RCC_CCIPR2_SPDIFRXSEL_SHIFT     (24) /* SPDIFRX clock source */
#define RCC_CCIPR2_SPDIFRXSEL_MASK      (3 << RCC_CCIPR2_SPDIFRXSEL_SHIFT)
#  define RCC_CCIPR2_SPDIFRXSEL(n)      ((n) << RCC_CCIPR2_SPDIFRXSEL_SHIFT)
#define RCC_CCIPR2_CECSEL_SHIFT         (28) /* CEC clock source */
#define RCC_CCIPR2_CECSEL_MASK          (3 << RCC_CCIPR2_CECSEL_SHIFT)
#  define RCC_CCIPR2_CECSEL(n)          ((n) << RCC_CCIPR2_CECSEL_SHIFT)

#define RCC_CCIPR3_USART1SEL_SHIFT      (0)  /* USART1 clock source */
#define RCC_CCIPR3_USART1SEL_MASK       (7 << RCC_CCIPR3_USART1SEL_SHIFT)
#  define RCC_CCIPR3_USART1SEL(n)       ((n) << RCC_CCIPR3_USART1SEL_SHIFT)
#define RCC_CCIPR3_SPI45SEL_SHIFT       (4)  /* SPI45 clock source */
#define RCC_CCIPR3_SPI45SEL_MASK        (7 << RCC_CCIPR3_SPI45SEL_SHIFT)
#  define RCC_CCIPR3_SPI45SEL(n)        ((n) << RCC_CCIPR3_SPI45SEL_SHIFT)
#define RCC_CCIPR3_SPI1SEL_SHIFT        (8)  /* SPI1 clock source */
#define RCC_CCIPR3_SPI1SEL_MASK         (7 << RCC_CCIPR3_SPI1SEL_SHIFT)
#  define RCC_CCIPR3_SPI1SEL(n)         ((n) << RCC_CCIPR3_SPI1SEL_SHIFT)
#define RCC_CCIPR3_SAI1SEL_SHIFT        (16) /* SAI1 clock source */
#define RCC_CCIPR3_SAI1SEL_MASK         (7 << RCC_CCIPR3_SAI1SEL_SHIFT)
#  define RCC_CCIPR3_SAI1SEL(n)         ((n) << RCC_CCIPR3_SAI1SEL_SHIFT)
#define RCC_CCIPR3_SAI2SEL_SHIFT        (20) /* SAI2 clock source */
#define RCC_CCIPR3_SAI2SEL_MASK         (7 << RCC_CCIPR3_SAI2SEL_SHIFT)
#  define RCC_CCIPR3_SAI2SEL(n)         ((n) << RCC_CCIPR3_SAI2SEL_SHIFT)

#define RCC_CCIPR4_LPUART1SEL_SHIFT     (0)  /* LPUART1 clock source */
#define RCC_CCIPR4_LPUART1SEL_MASK      (7 << RCC_CCIPR4_LPUART1SEL_SHIFT)
#  define RCC_CCIPR4_LPUART1SEL(n)      ((n) << RCC_CCIPR4_LPUART1SEL_SHIFT)
#define RCC_CCIPR4_SPI6SEL_SHIFT        (4)  /* SPI6 clock source */
#define RCC_CCIPR4_SPI6SEL_MASK         (7 << RCC_CCIPR4_SPI6SEL_SHIFT)
#  define RCC_CCIPR4_SPI6SEL(n)         ((n) << RCC_CCIPR4_SPI6SEL_SHIFT)
#define RCC_CCIPR4_LPTIM23SEL_SHIFT     (8)  /* LPTIM23 clock source */
#define RCC_CCIPR4_LPTIM23SEL_MASK      (7 << RCC_CCIPR4_LPTIM23SEL_SHIFT)
#  define RCC_CCIPR4_LPTIM23SEL(n)      ((n) << RCC_CCIPR4_LPTIM23SEL_SHIFT)
#define RCC_CCIPR4_LPTIM45SEL_SHIFT     (12) /* LPTIM45 clock source */
#define RCC_CCIPR4_LPTIM45SEL_MASK      (7 << RCC_CCIPR4_LPTIM45SEL_SHIFT)
#  define RCC_CCIPR4_LPTIM45SEL(n)      ((n) << RCC_CCIPR4_LPTIM45SEL_SHIFT)

/* Clock interrupts *********************************************************/

#define RCC_CI_LSIRDY                   (1 << 0)  /* LSI ready */
#define RCC_CI_LSERDY                   (1 << 1)  /* LSE ready */
#define RCC_CI_HSIRDY                   (1 << 2)  /* HSI ready */
#define RCC_CI_HSERDY                   (1 << 3)  /* HSE ready */
#define RCC_CI_CSIRDY                   (1 << 4)  /* CSI ready */
#define RCC_CI_HSI48RDY                 (1 << 5)  /* HSI48 ready */
#define RCC_CI_PLL1RDY                  (1 << 6)  /* PLL1 ready */
#define RCC_CI_PLL2RDY                  (1 << 7)  /* PLL2 ready */
#define RCC_CI_PLL3RDY                  (1 << 8)  /* PLL3 ready */
#define RCC_CI_LSECSS                   (1 << 9)  /* LSE CSS failure */
#define RCC_CI_HSECSS                   (1 << 10) /* HSE CSS failure */

#define RCC_CIER_LSIRDYIE               RCC_CI_LSIRDY
#define RCC_CIER_LSERDYIE               RCC_CI_LSERDY
#define RCC_CIER_HSIRDYIE               RCC_CI_HSIRDY
#define RCC_CIER_HSERDYIE               RCC_CI_HSERDY
#define RCC_CIER_CSIRDYIE               RCC_CI_CSIRDY
#define RCC_CIER_HSI48RDYIE             RCC_CI_HSI48RDY
#define RCC_CIER_PLL1RDYIE              RCC_CI_PLL1RDY
#define RCC_CIER_PLL2RDYIE              RCC_CI_PLL2RDY
#define RCC_CIER_PLL3RDYIE              RCC_CI_PLL3RDY
#define RCC_CIER_LSECSSIE               RCC_CI_LSECSS
#define RCC_CIFR_LSIRDYF                RCC_CI_LSIRDY
#define RCC_CIFR_LSERDYF                RCC_CI_LSERDY
#define RCC_CIFR_HSIRDYF                RCC_CI_HSIRDY
#define RCC_CIFR_HSERDYF                RCC_CI_HSERDY
#define RCC_CIFR_CSIRDYF                RCC_CI_CSIRDY
#define RCC_CIFR_HSI48RDYF              RCC_CI_HSI48RDY
#define RCC_CIFR_PLL1RDYF               RCC_CI_PLL1RDY
#define RCC_CIFR_PLL2RDYF               RCC_CI_PLL2RDY
#define RCC_CIFR_PLL3RDYF               RCC_CI_PLL3RDY
#define RCC_CIFR_LSECSSF                RCC_CI_LSECSS
#define RCC_CIFR_HSECSSF                RCC_CI_HSECSS
#define RCC_CICR_LSIRDYC                RCC_CI_LSIRDY
#define RCC_CICR_LSERDYC                RCC_CI_LSERDY
#define RCC_CICR_HSIRDYC                RCC_CI_HSIRDY
#define RCC_CICR_HSERDYC                RCC_CI_HSERDY
#define RCC_CICR_CSIRDYC                RCC_CI_CSIRDY
#define RCC_CICR_HSI48RDYC              RCC_CI_HSI48RDY
#define RCC_CICR_PLL1RDYC               RCC_CI_PLL1RDY
#define RCC_CICR_PLL2RDYC               RCC_CI_PLL2RDY
#define RCC_CICR_PLL3RDYC               RCC_CI_PLL3RDY
#define RCC_CICR_LSECSSC                RCC_CI_LSECSS
#define RCC_CICR_HSECSSC                RCC_CI_HSECSS

/* Backup domain and reset status *******************************************/

#define RCC_BDCR_LSEON                  (1 << 0)  /* LSE enable */
#define RCC_BDCR_LSERDY                 (1 << 1)  /* LSE ready */
#define RCC_BDCR_LSEBYP                 (1 << 2)  /* LSE bypass */
#define RCC_BDCR_LSEDRV_SHIFT           (3)       /* LSE drive capability */
#define RCC_BDCR_LSEDRV_MASK            (3 << RCC_BDCR_LSEDRV_SHIFT)
#define RCC_BDCR_LSECSSON               (1 << 5)  /* LSE CSS enable */
#define RCC_BDCR_LSECSSD                (1 << 6)  /* LSE CSS failure flag */
#define RCC_BDCR_LSEEXT                 (1 << 7)  /* LSE external source */
#define RCC_BDCR_RTCSEL_SHIFT           (8)       /* RTC clock source */
#define RCC_BDCR_RTCSEL_MASK            (3 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_NONE          (0 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_LSE           (1 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_LSI           (2 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_HSE           (3 << RCC_BDCR_RTCSEL_SHIFT)
#define RCC_BDCR_LSECSSRA               (1 << 12)
#define RCC_BDCR_RTCEN                  (1 << 15) /* RTC clock enable */
#define RCC_BDCR_VSWRST                 (1 << 16) /* VSwitch domain reset */
#define RCC_CSR_LSION                   (1 << 0)  /* LSI enable */
#define RCC_CSR_LSIRDY                  (1 << 1)  /* LSI ready */

/* Clock gating and protection registers ************************************/

#define RCC_CKGDISR_AXICKG              (1 << 0)
#define RCC_CKGDISR_AHBMCKG             (1 << 1)
#define RCC_CKGDISR_SDMMC1CKG           (1 << 2)
#define RCC_CKGDISR_HPDMA1CKG           (1 << 3)
#define RCC_CKGDISR_CPUCKG              (1 << 4)
#define RCC_CKGDISR_GPU2DS0CKG          (1 << 5)
#define RCC_CKGDISR_GPU2DS1CKG          (1 << 6)
#define RCC_CKGDISR_GPU2DCLCKG          (1 << 7)
#define RCC_CKGDISR_DCMIPPCKG           (1 << 8)
#define RCC_CKGDISR_DMA2DCKG            (1 << 9)
#define RCC_CKGDISR_GFXMMUSCKG          (1 << 10)
#define RCC_CKGDISR_LTDCCKG             (1 << 11)
#define RCC_CKGDISR_GFXMMUMCKG          (1 << 12)
#define RCC_CKGDISR_AHBSCKG             (1 << 13)
#define RCC_CKGDISR_FMCCKG              (1 << 14)
#define RCC_CKGDISR_XSPI1CKG            (1 << 15)
#define RCC_CKGDISR_XSPI2CKG            (1 << 16)
#define RCC_CKGDISR_AXISRAM4CKG         (1 << 17)
#define RCC_CKGDISR_AXISRAM3CKG         (1 << 18)
#define RCC_CKGDISR_AXISRAM2CKG         (1 << 19)
#define RCC_CKGDISR_AXISRAM1CKG         (1 << 20)
#define RCC_CKGDISR_FLASHCKG            (1 << 21)
#define RCC_CKGDISR_EXTICKG             (1 << 30)
#define RCC_CKGDISR_JTAGCKG             (1 << 31)

#define RCC_CKPROTR_XSPICKP             (1 << 0) /* XSPI clock protection */
#define RCC_CKPROTR_FMCCKP              (1 << 1) /* FMC clock protection */
#define RCC_CKPROTR_XSPI1SWP_SHIFT      (4)      /* XSPI1 clock switch */
#define RCC_CKPROTR_XSPI1SWP_MASK       (7 << RCC_CKPROTR_XSPI1SWP_SHIFT)
#  define RCC_CKPROTR_XSPI1SWP(n)       ((n) << RCC_CKPROTR_XSPI1SWP_SHIFT)
#define RCC_CKPROTR_XSPI2SWP_SHIFT      (8)      /* XSPI2 clock switch */
#define RCC_CKPROTR_XSPI2SWP_MASK       (7 << RCC_CKPROTR_XSPI2SWP_SHIFT)
#  define RCC_CKPROTR_XSPI2SWP(n)       ((n) << RCC_CKPROTR_XSPI2SWP_SHIFT)
#define RCC_CKPROTR_FMCSWP_SHIFT        (12)     /* FMC clock switch */
#define RCC_CKPROTR_FMCSWP_MASK         (7 << RCC_CKPROTR_FMCSWP_SHIFT)
#  define RCC_CKPROTR_FMCSWP(n)         ((n) << RCC_CKPROTR_FMCSWP_SHIFT)

/* Reset status register ****************************************************/

#define RCC_RSR_RMVF                    (1 << 16) /* Remove reset flag */
#define RCC_RSR_OBLRSTF                 (1 << 17) /* OBL reset flag */
#define RCC_RSR_BORRSTF                 (1 << 21) /* BOR reset flag */
#define RCC_RSR_PINRSTF                 (1 << 22) /* Pin reset flag */
#define RCC_RSR_PORRSTF                 (1 << 23) /* POR/PDR reset flag */
#define RCC_RSR_SFTRSTF                 (1 << 24) /* Software reset flag */
#define RCC_RSR_IWDGRSTF                (1 << 26) /* IWDG reset flag */
#define RCC_RSR_WWDGRSTF                (1 << 28) /* WWDG reset flag */
#define RCC_RSR_LPWRRSTF                (1 << 30) /* Low-power reset flag */

/* Peripheral reset registers ***********************************************/

#define RCC_AHB5RSTR_HPDMA1RST          (1 << 0)  /* HPDMA1 reset */
#define RCC_AHB5RSTR_DMA2DRST           (1 << 1)  /* DMA2D reset */
#define RCC_AHB5RSTR_JPEGRST            (1 << 3)  /* JPEG reset */
#define RCC_AHB5RSTR_FMCRST             (1 << 4)  /* FMC reset */
#define RCC_AHB5RSTR_XSPI1RST           (1 << 5)  /* XSPI1 reset */
#define RCC_AHB5RSTR_SDMMC1RST          (1 << 8)  /* SDMMC1 reset */
#define RCC_AHB5RSTR_XSPI2RST           (1 << 12) /* XSPI2 reset */
#define RCC_AHB5RSTR_XSPIMRST           (1 << 14) /* XSPIM reset */
#define RCC_AHB5RSTR_GFXMMURST          (1 << 19) /* GFXMMU reset */
#define RCC_AHB5RSTR_GPU2DRST           (1 << 20) /* GPU2D reset */

#define RCC_AHB1RSTR_GPDMA1RST          (1 << 4)  /* GPDMA1 reset */
#define RCC_AHB1RSTR_ADC12RST           (1 << 5)  /* ADC12 reset */
#define RCC_AHB1RSTR_ETH1MACRST         (1 << 15) /* ETH1 reset */
#define RCC_AHB1RSTR_OTGHSRST           (1 << 25) /* OTGHS reset */
#define RCC_AHB1RSTR_USBPHYCRST         (1 << 26) /* USBPHYC reset */
#define RCC_AHB1RSTR_OTGFSRST           (1 << 27) /* OTGFS reset */
#define RCC_AHB1RSTR_ADF1RST            (1 << 31) /* ADF1 reset */

#define RCC_AHB2RSTR_PSSIRST            (1 << 1)  /* PSSI reset */
#define RCC_AHB2RSTR_SDMMC2RST          (1 << 9)  /* SDMMC2 reset */
#define RCC_AHB2RSTR_CORDICRST          (1 << 14) /* CORDIC reset */

#define RCC_AHB4RSTR_GPIOARST           (1 << 0)  /* GPIOA reset */
#define RCC_AHB4RSTR_GPIOBRST           (1 << 1)  /* GPIOB reset */
#define RCC_AHB4RSTR_GPIOCRST           (1 << 2)  /* GPIOC reset */
#define RCC_AHB4RSTR_GPIODRST           (1 << 3)  /* GPIOD reset */
#define RCC_AHB4RSTR_GPIOERST           (1 << 4)  /* GPIOE reset */
#define RCC_AHB4RSTR_GPIOFRST           (1 << 5)  /* GPIOF reset */
#define RCC_AHB4RSTR_GPIOGRST           (1 << 6)  /* GPIOG reset */
#define RCC_AHB4RSTR_GPIOHRST           (1 << 7)  /* GPIOH reset */
#define RCC_AHB4RSTR_GPIOMRST           (1 << 12) /* GPIOM reset */
#define RCC_AHB4RSTR_GPIONRST           (1 << 13) /* GPION reset */
#define RCC_AHB4RSTR_GPIOORST           (1 << 14) /* GPIOO reset */
#define RCC_AHB4RSTR_GPIOPRST           (1 << 15) /* GPIOP reset */
#define RCC_AHB4RSTR_CRCRST             (1 << 19) /* CRC reset */

#define RCC_APB5RSTR_LTDCRST            (1 << 1) /* LTDC reset */
#define RCC_APB5RSTR_DCMIPPRST          (1 << 2) /* DCMIPP reset */
#define RCC_APB5RSTR_GFXTIMRST          (1 << 4) /* GFXTIM reset */

#define RCC_APB1RSTR1_TIM2RST           (1 << 0)  /* TIM2 reset */
#define RCC_APB1RSTR1_TIM3RST           (1 << 1)  /* TIM3 reset */
#define RCC_APB1RSTR1_TIM4RST           (1 << 2)  /* TIM4 reset */
#define RCC_APB1RSTR1_TIM5RST           (1 << 3)  /* TIM5 reset */
#define RCC_APB1RSTR1_TIM6RST           (1 << 4)  /* TIM6 reset */
#define RCC_APB1RSTR1_TIM7RST           (1 << 5)  /* TIM7 reset */
#define RCC_APB1RSTR1_TIM12RST          (1 << 6)  /* TIM12 reset */
#define RCC_APB1RSTR1_TIM13RST          (1 << 7)  /* TIM13 reset */
#define RCC_APB1RSTR1_TIM14RST          (1 << 8)  /* TIM14 reset */
#define RCC_APB1RSTR1_LPTIM1RST         (1 << 9)  /* LPTIM1 reset */
#define RCC_APB1RSTR1_SPI2RST           (1 << 14) /* SPI2 reset */
#define RCC_APB1RSTR1_SPI3RST           (1 << 15) /* SPI3 reset */
#define RCC_APB1RSTR1_SPDIFRXRST        (1 << 16) /* SPDIFRX reset */
#define RCC_APB1RSTR1_USART2RST         (1 << 17) /* USART2 reset */
#define RCC_APB1RSTR1_USART3RST         (1 << 18) /* USART3 reset */
#define RCC_APB1RSTR1_UART4RST          (1 << 19) /* UART4 reset */
#define RCC_APB1RSTR1_UART5RST          (1 << 20) /* UART5 reset */
#define RCC_APB1RSTR1_I2C1_I3C1RST      (1 << 21) /* I2C1/I3C1 reset */
#define RCC_APB1RSTR1_I2C2RST           (1 << 22) /* I2C2 reset */
#define RCC_APB1RSTR1_I2C3RST           (1 << 23) /* I2C3 reset */
#define RCC_APB1RSTR1_CECRST            (1 << 27) /* CEC reset */
#define RCC_APB1RSTR1_UART7RST          (1 << 30) /* UART7 reset */
#define RCC_APB1RSTR1_UART8RST          (1 << 31) /* UART8 reset */
#define RCC_APB1RSTR2_CRSRST            (1 << 1)  /* CRS reset */
#define RCC_APB1RSTR2_MDIOSRST          (1 << 5)  /* MDIOS reset */
#define RCC_APB1RSTR2_FDCANRST          (1 << 8)  /* FDCAN reset */
#define RCC_APB1RSTR2_UCPD1RST          (1 << 27) /* UCPD1 reset */

#define RCC_APB2RSTR_TIM1RST            (1 << 0)  /* TIM1 reset */
#define RCC_APB2RSTR_USART1RST          (1 << 4)  /* USART1 reset */
#define RCC_APB2RSTR_SPI1RST            (1 << 12) /* SPI1 reset */
#define RCC_APB2RSTR_SPI4RST            (1 << 13) /* SPI4 reset */
#define RCC_APB2RSTR_TIM15RST           (1 << 16) /* TIM15 reset */
#define RCC_APB2RSTR_TIM16RST           (1 << 17) /* TIM16 reset */
#define RCC_APB2RSTR_TIM17RST           (1 << 18) /* TIM17 reset */
#define RCC_APB2RSTR_TIM9RST            (1 << 19) /* TIM9 reset */
#define RCC_APB2RSTR_SPI5RST            (1 << 20) /* SPI5 reset */
#define RCC_APB2RSTR_SAI1RST            (1 << 22) /* SAI1 reset */
#define RCC_APB2RSTR_SAI2RST            (1 << 23) /* SAI2 reset */

#define RCC_APB4RSTR_SBSRST             (1 << 1)  /* SBS reset */
#define RCC_APB4RSTR_LPUART1RST         (1 << 3)  /* LPUART1 reset */
#define RCC_APB4RSTR_SPI6RST            (1 << 5)  /* SPI6 reset */
#define RCC_APB4RSTR_LPTIM2RST          (1 << 9)  /* LPTIM2 reset */
#define RCC_APB4RSTR_LPTIM3RST          (1 << 10) /* LPTIM3 reset */
#define RCC_APB4RSTR_LPTIM4RST          (1 << 11) /* LPTIM4 reset */
#define RCC_APB4RSTR_LPTIM5RST          (1 << 12) /* LPTIM5 reset */
#define RCC_APB4RSTR_VREFRST            (1 << 15) /* VREF reset */
#define RCC_APB4RSTR_DTSRST             (1 << 26) /* DTS reset */

#define RCC_AHB3RSTR_RNGRST             (1 << 0) /* RNG reset */
#define RCC_AHB3RSTR_HASHRST            (1 << 1) /* HASH reset */
#define RCC_AHB3RSTR_CRYPRST            (1 << 2) /* CRYP reset */
#define RCC_AHB3RSTR_SAESRST            (1 << 4) /* SAES reset */
#define RCC_AHB3RSTR_PKARST             (1 << 6) /* PKA reset */

/* Peripheral clock enable registers ****************************************/

#define RCC_AHB5ENR_HPDMA1EN            (1 << 0)  /* HPDMA1 clock enable */
#define RCC_AHB5ENR_DMA2DEN             (1 << 1)  /* DMA2D clock enable */
#define RCC_AHB5ENR_JPEGEN              (1 << 3)  /* JPEG clock enable */
#define RCC_AHB5ENR_FMCEN               (1 << 4)  /* FMC clock enable */
#define RCC_AHB5ENR_XSPI1EN             (1 << 5)  /* XSPI1 clock enable */
#define RCC_AHB5ENR_SDMMC1EN            (1 << 8)  /* SDMMC1 clock enable */
#define RCC_AHB5ENR_XSPI2EN             (1 << 12) /* XSPI2 clock enable */
#define RCC_AHB5ENR_XSPIMEN             (1 << 14) /* XSPIM clock enable */
#define RCC_AHB5ENR_GFXMMUEN            (1 << 19) /* GFXMMU clock enable */
#define RCC_AHB5ENR_GPU2DEN             (1 << 20) /* GPU2D clock enable */

#define RCC_AHB1ENR_GPDMA1EN            (1 << 4)  /* GPDMA1 clock enable */
#define RCC_AHB1ENR_ADC12EN             (1 << 5)  /* ADC12 clock enable */
#define RCC_AHB1ENR_ETH1MACEN           (1 << 15) /* ETH1MAC clock enable */
#define RCC_AHB1ENR_ETH1TXEN            (1 << 16) /* ETH1TX clock enable */
#define RCC_AHB1ENR_ETH1RXEN            (1 << 17) /* ETH1RX clock enable */
#define RCC_AHB1ENR_OTGHSEN             (1 << 25) /* OTGHS clock enable */
#define RCC_AHB1ENR_USBPHYCEN           (1 << 26) /* USBPHYC clock enable */
#define RCC_AHB1ENR_OTGFSEN             (1 << 27) /* OTGFS clock enable */
#define RCC_AHB1ENR_ADF1EN              (1 << 31) /* ADF1 clock enable */

#define RCC_AHB2ENR_PSSIEN              (1 << 1)  /* PSSI clock enable */
#define RCC_AHB2ENR_SDMMC2EN            (1 << 9)  /* SDMMC2 clock enable */
#define RCC_AHB2ENR_CORDICEN            (1 << 14) /* CORDIC clock enable */
#define RCC_AHB2ENR_SRAM1EN             (1 << 29) /* SRAM1 clock enable */
#define RCC_AHB2ENR_SRAM2EN             (1 << 30) /* SRAM2 clock enable */

#define RCC_AHB4ENR_GPIOAEN             (1 << 0)  /* GPIOA clock enable */
#define RCC_AHB4ENR_GPIOBEN             (1 << 1)  /* GPIOB clock enable */
#define RCC_AHB4ENR_GPIOCEN             (1 << 2)  /* GPIOC clock enable */
#define RCC_AHB4ENR_GPIODEN             (1 << 3)  /* GPIOD clock enable */
#define RCC_AHB4ENR_GPIOEEN             (1 << 4)  /* GPIOE clock enable */
#define RCC_AHB4ENR_GPIOFEN             (1 << 5)  /* GPIOF clock enable */
#define RCC_AHB4ENR_GPIOGEN             (1 << 6)  /* GPIOG clock enable */
#define RCC_AHB4ENR_GPIOHEN             (1 << 7)  /* GPIOH clock enable */
#define RCC_AHB4ENR_GPIOMEN             (1 << 12) /* GPIOM clock enable */
#define RCC_AHB4ENR_GPIONEN             (1 << 13) /* GPION clock enable */
#define RCC_AHB4ENR_GPIOOEN             (1 << 14) /* GPIOO clock enable */
#define RCC_AHB4ENR_GPIOPEN             (1 << 15) /* GPIOP clock enable */
#define RCC_AHB4ENR_CRCEN               (1 << 19) /* CRC clock enable */
#define RCC_AHB4ENR_BKPRAMEN            (1 << 28) /* BKPRAM clock enable */

#define RCC_APB5ENR_LTDCEN              (1 << 1) /* LTDC clock enable */
#define RCC_APB5ENR_DCMIPPEN            (1 << 2) /* DCMIPP clock enable */
#define RCC_APB5ENR_GFXTIMEN            (1 << 4) /* GFXTIM clock enable */

#define RCC_APB1ENR1_TIM2EN             (1 << 0)  /* TIM2 clock enable */
#define RCC_APB1ENR1_TIM3EN             (1 << 1)  /* TIM3 clock enable */
#define RCC_APB1ENR1_TIM4EN             (1 << 2)  /* TIM4 clock enable */
#define RCC_APB1ENR1_TIM5EN             (1 << 3)  /* TIM5 clock enable */
#define RCC_APB1ENR1_TIM6EN             (1 << 4)  /* TIM6 clock enable */
#define RCC_APB1ENR1_TIM7EN             (1 << 5)  /* TIM7 clock enable */
#define RCC_APB1ENR1_TIM12EN            (1 << 6)  /* TIM12 clock enable */
#define RCC_APB1ENR1_TIM13EN            (1 << 7)  /* TIM13 clock enable */
#define RCC_APB1ENR1_TIM14EN            (1 << 8)  /* TIM14 clock enable */
#define RCC_APB1ENR1_LPTIM1EN           (1 << 9)  /* LPTIM1 clock enable */
#define RCC_APB1ENR1_WWDGEN             (1 << 11) /* WWDG clock enable */
#define RCC_APB1ENR1_SPI2EN             (1 << 14) /* SPI2 clock enable */
#define RCC_APB1ENR1_SPI3EN             (1 << 15) /* SPI3 clock enable */
#define RCC_APB1ENR1_SPDIFRXEN          (1 << 16) /* SPDIFRX clock enable */
#define RCC_APB1ENR1_USART2EN           (1 << 17) /* USART2 clock enable */
#define RCC_APB1ENR1_USART3EN           (1 << 18) /* USART3 clock enable */
#define RCC_APB1ENR1_UART4EN            (1 << 19) /* UART4 clock enable */
#define RCC_APB1ENR1_UART5EN            (1 << 20) /* UART5 clock enable */
#define RCC_APB1ENR1_I2C1_I3C1EN        (1 << 21) /* I2C1/I3C1 clock enable */
#define RCC_APB1ENR1_I2C2EN             (1 << 22) /* I2C2 clock enable */
#define RCC_APB1ENR1_I2C3EN             (1 << 23) /* I2C3 clock enable */
#define RCC_APB1ENR1_CECEN              (1 << 27) /* CEC clock enable */
#define RCC_APB1ENR1_UART7EN            (1 << 30) /* UART7 clock enable */
#define RCC_APB1ENR1_UART8EN            (1 << 31) /* UART8 clock enable */
#define RCC_APB1ENR2_CRSEN              (1 << 1)  /* CRS clock enable */
#define RCC_APB1ENR2_MDIOSEN            (1 << 5)  /* MDIOS clock enable */
#define RCC_APB1ENR2_FDCANEN            (1 << 8)  /* FDCAN clock enable */
#define RCC_APB1ENR2_UCPD1EN            (1 << 27) /* UCPD1 clock enable */

#define RCC_APB2ENR_TIM1EN              (1 << 0)  /* TIM1 clock enable */
#define RCC_APB2ENR_USART1EN            (1 << 4)  /* USART1 clock enable */
#define RCC_APB2ENR_SPI1EN              (1 << 12) /* SPI1 clock enable */
#define RCC_APB2ENR_SPI4EN              (1 << 13) /* SPI4 clock enable */
#define RCC_APB2ENR_TIM15EN             (1 << 16) /* TIM15 clock enable */
#define RCC_APB2ENR_TIM16EN             (1 << 17) /* TIM16 clock enable */
#define RCC_APB2ENR_TIM17EN             (1 << 18) /* TIM17 clock enable */
#define RCC_APB2ENR_TIM9EN              (1 << 19) /* TIM9 clock enable */
#define RCC_APB2ENR_SPI5EN              (1 << 20) /* SPI5 clock enable */
#define RCC_APB2ENR_SAI1EN              (1 << 22) /* SAI1 clock enable */
#define RCC_APB2ENR_SAI2EN              (1 << 23) /* SAI2 clock enable */

#define RCC_APB4ENR_SBSEN               (1 << 1)  /* SBS clock enable */
#define RCC_APB4ENR_LPUART1EN           (1 << 3)  /* LPUART1 clock enable */
#define RCC_APB4ENR_SPI6EN              (1 << 5)  /* SPI6 clock enable */
#define RCC_APB4ENR_LPTIM2EN            (1 << 9)  /* LPTIM2 clock enable */
#define RCC_APB4ENR_LPTIM3EN            (1 << 10) /* LPTIM3 clock enable */
#define RCC_APB4ENR_LPTIM4EN            (1 << 11) /* LPTIM4 clock enable */
#define RCC_APB4ENR_LPTIM5EN            (1 << 12) /* LPTIM5 clock enable */
#define RCC_APB4ENR_VREFEN              (1 << 15) /* VREF clock enable */
#define RCC_APB4ENR_RTCAPBEN            (1 << 16) /* RTCAPB clock enable */
#define RCC_APB4ENR_DTSEN               (1 << 26) /* DTS clock enable */

#define RCC_AHB3ENR_RNGEN               (1 << 0) /* RNG clock enable */
#define RCC_AHB3ENR_HASHEN              (1 << 1) /* HASH clock enable */
#define RCC_AHB3ENR_CRYPEN              (1 << 2) /* CRYP clock enable */
#define RCC_AHB3ENR_SAESEN              (1 << 4) /* SAES clock enable */
#define RCC_AHB3ENR_PKAEN               (1 << 6) /* PKA clock enable */

/* Peripheral clock sleep enable registers **********************************/

#define RCC_AHB1LPENR_GPDMA1LPEN        (1 << 4)  /* GPDMA1 sleep enable */
#define RCC_AHB1LPENR_ADC12LPEN         (1 << 5)  /* ADC12 sleep enable */
#define RCC_AHB1LPENR_ETH1MACLPEN       (1 << 15) /* ETH1MAC sleep enable */
#define RCC_AHB1LPENR_ETH1TXLPEN        (1 << 16) /* ETH1TX sleep enable */
#define RCC_AHB1LPENR_ETH1RXLPEN        (1 << 17) /* ETH1RX sleep enable */
#define RCC_AHB1LPENR_UCPDCTRL          (1 << 24)
#define RCC_AHB1LPENR_OTGHSLPEN         (1 << 25) /* OTGHS sleep enable */
#define RCC_AHB1LPENR_USBPHYCLPEN       (1 << 26) /* USBPHYC sleep enable */
#define RCC_AHB1LPENR_OTGFSLPEN         (1 << 27) /* OTGFS sleep enable */
#define RCC_AHB1LPENR_ADF1LPEN          (1 << 31) /* ADF1 sleep enable */

#define RCC_AHB2LPENR_PSSILPEN          (1 << 1)  /* PSSI sleep enable */
#define RCC_AHB2LPENR_SDMMC2LPEN        (1 << 9)  /* SDMMC2 sleep enable */
#define RCC_AHB2LPENR_CORDICLPEN        (1 << 14) /* CORDIC sleep enable */
#define RCC_AHB2LPENR_SRAM1LPEN         (1 << 29) /* SRAM1 sleep enable */
#define RCC_AHB2LPENR_SRAM2LPEN         (1 << 30) /* SRAM2 sleep enable */

#define RCC_AHB3LPENR_RNGLPEN           (1 << 0) /* RNG sleep enable */
#define RCC_AHB3LPENR_HASHLPEN          (1 << 1) /* HASH sleep enable */
#define RCC_AHB3LPENR_CRYPLPEN          (1 << 2) /* CRYP sleep enable */
#define RCC_AHB3LPENR_SAESLPEN          (1 << 4) /* SAES sleep enable */
#define RCC_AHB3LPENR_PKALPEN           (1 << 6) /* PKA sleep enable */

#define RCC_AHB4LPENR_GPIOALPEN         (1 << 0)  /* GPIOA sleep enable */
#define RCC_AHB4LPENR_GPIOBLPEN         (1 << 1)  /* GPIOB sleep enable */
#define RCC_AHB4LPENR_GPIOCLPEN         (1 << 2)  /* GPIOC sleep enable */
#define RCC_AHB4LPENR_GPIODLPEN         (1 << 3)  /* GPIOD sleep enable */
#define RCC_AHB4LPENR_GPIOELPEN         (1 << 4)  /* GPIOE sleep enable */
#define RCC_AHB4LPENR_GPIOFLPEN         (1 << 5)  /* GPIOF sleep enable */
#define RCC_AHB4LPENR_GPIOGLPEN         (1 << 6)  /* GPIOG sleep enable */
#define RCC_AHB4LPENR_GPIOHLPEN         (1 << 7)  /* GPIOH sleep enable */
#define RCC_AHB4LPENR_GPIOMLPEN         (1 << 12) /* GPIOM sleep enable */
#define RCC_AHB4LPENR_GPIONLPEN         (1 << 13) /* GPION sleep enable */
#define RCC_AHB4LPENR_GPIOOLPEN         (1 << 14) /* GPIOO sleep enable */
#define RCC_AHB4LPENR_GPIOPLPEN         (1 << 15) /* GPIOP sleep enable */
#define RCC_AHB4LPENR_CRCLPEN           (1 << 19) /* CRC sleep enable */
#define RCC_AHB4LPENR_BKPRAMLPEN        (1 << 28) /* BKPRAM sleep enable */

#define RCC_AHB5LPENR_HPDMA1LPEN        (1 << 0)  /* HPDMA1 sleep enable */
#define RCC_AHB5LPENR_DMA2DLPEN         (1 << 1)  /* DMA2D sleep enable */
#define RCC_AHB5LPENR_FLASHLPEN         (1 << 2)  /* FLASH sleep enable */
#define RCC_AHB5LPENR_JPEGLPEN          (1 << 3)  /* JPEG sleep enable */
#define RCC_AHB5LPENR_FMCLPEN           (1 << 4)  /* FMC sleep enable */
#define RCC_AHB5LPENR_XSPI1LPEN         (1 << 5)  /* XSPI1 sleep enable */
#define RCC_AHB5LPENR_SDMMC1LPEN        (1 << 8)  /* SDMMC1 sleep enable */
#define RCC_AHB5LPENR_XSPI2LPEN         (1 << 12) /* XSPI2 sleep enable */
#define RCC_AHB5LPENR_XSPIMLPEN         (1 << 14) /* XSPIM sleep enable */
#define RCC_AHB5LPENR_GFXMMULPEN        (1 << 19) /* GFXMMU sleep enable */
#define RCC_AHB5LPENR_GPU2DLPEN         (1 << 20) /* GPU2D sleep enable */
#define RCC_AHB5LPENR_DTCM1LPEN         (1 << 28) /* DTCM1 sleep enable */
#define RCC_AHB5LPENR_DTCM2LPEN         (1 << 29) /* DTCM2 sleep enable */
#define RCC_AHB5LPENR_ITCMLPEN          (1 << 30) /* ITCM sleep enable */
#define RCC_AHB5LPENR_AXISRAMLPEN       (1 << 31) /* AXISRAM sleep enable */

#define RCC_APB1LPENR1_TIM2LPEN         (1 << 0)  /* TIM2 sleep enable */
#define RCC_APB1LPENR1_TIM3LPEN         (1 << 1)  /* TIM3 sleep enable */
#define RCC_APB1LPENR1_TIM4LPEN         (1 << 2)  /* TIM4 sleep enable */
#define RCC_APB1LPENR1_TIM5LPEN         (1 << 3)  /* TIM5 sleep enable */
#define RCC_APB1LPENR1_TIM6LPEN         (1 << 4)  /* TIM6 sleep enable */
#define RCC_APB1LPENR1_TIM7LPEN         (1 << 5)  /* TIM7 sleep enable */
#define RCC_APB1LPENR1_TIM12LPEN        (1 << 6)  /* TIM12 sleep enable */
#define RCC_APB1LPENR1_TIM13LPEN        (1 << 7)  /* TIM13 sleep enable */
#define RCC_APB1LPENR1_TIM14LPEN        (1 << 8)  /* TIM14 sleep enable */
#define RCC_APB1LPENR1_LPTIM1LPEN       (1 << 9)  /* LPTIM1 sleep enable */
#define RCC_APB1LPENR1_WWDGLPEN         (1 << 11) /* WWDG sleep enable */
#define RCC_APB1LPENR1_SPI2LPEN         (1 << 14) /* SPI2 sleep enable */
#define RCC_APB1LPENR1_SPI3LPEN         (1 << 15) /* SPI3 sleep enable */
#define RCC_APB1LPENR1_SPDIFRXLPEN      (1 << 16) /* SPDIFRX sleep enable */
#define RCC_APB1LPENR1_USART2LPEN       (1 << 17) /* USART2 sleep enable */
#define RCC_APB1LPENR1_USART3LPEN       (1 << 18) /* USART3 sleep enable */
#define RCC_APB1LPENR1_UART4LPEN        (1 << 19) /* UART4 sleep enable */
#define RCC_APB1LPENR1_UART5LPEN        (1 << 20) /* UART5 sleep enable */
#define RCC_APB1LPENR1_I2C1_I3C1LPEN    (1 << 21) /* I2C1/I3C1 sleep enable */
#define RCC_APB1LPENR1_I2C2LPEN         (1 << 22) /* I2C2 sleep enable */
#define RCC_APB1LPENR1_I2C3LPEN         (1 << 23) /* I2C3 sleep enable */
#define RCC_APB1LPENR1_CECLPEN          (1 << 27) /* CEC sleep enable */
#define RCC_APB1LPENR1_UART7LPEN        (1 << 30) /* UART7 sleep enable */
#define RCC_APB1LPENR1_UART8LPEN        (1 << 31) /* UART8 sleep enable */

#define RCC_APB1LPENR2_CRSLPEN          (1 << 1)  /* CRS sleep enable */
#define RCC_APB1LPENR2_MDIOSLPEN        (1 << 5)  /* MDIOS sleep enable */
#define RCC_APB1LPENR2_FDCANLPEN        (1 << 8)  /* FDCAN sleep enable */
#define RCC_APB1LPENR2_UCPD1LPEN        (1 << 27) /* UCPD1 sleep enable */

#define RCC_APB2LPENR_TIM1LPEN          (1 << 0)  /* TIM1 sleep enable */
#define RCC_APB2LPENR_USART1LPEN        (1 << 4)  /* USART1 sleep enable */
#define RCC_APB2LPENR_SPI1LPEN          (1 << 12) /* SPI1 sleep enable */
#define RCC_APB2LPENR_SPI4LPEN          (1 << 13) /* SPI4 sleep enable */
#define RCC_APB2LPENR_TIM15LPEN         (1 << 16) /* TIM15 sleep enable */
#define RCC_APB2LPENR_TIM16LPEN         (1 << 17) /* TIM16 sleep enable */
#define RCC_APB2LPENR_TIM17LPEN         (1 << 18) /* TIM17 sleep enable */
#define RCC_APB2LPENR_TIM9LPEN          (1 << 19) /* TIM9 sleep enable */
#define RCC_APB2LPENR_SPI5LPEN          (1 << 20) /* SPI5 sleep enable */
#define RCC_APB2LPENR_SAI1LPEN          (1 << 22) /* SAI1 sleep enable */
#define RCC_APB2LPENR_SAI2LPEN          (1 << 23) /* SAI2 sleep enable */

#define RCC_APB4LPENR_SBSLPEN           (1 << 1)  /* SBS sleep enable */
#define RCC_APB4LPENR_LPUART1LPEN       (1 << 3)  /* LPUART1 sleep enable */
#define RCC_APB4LPENR_SPI6LPEN          (1 << 5)  /* SPI6 sleep enable */
#define RCC_APB4LPENR_LPTIM2LPEN        (1 << 9)  /* LPTIM2 sleep enable */
#define RCC_APB4LPENR_LPTIM3LPEN        (1 << 10) /* LPTIM3 sleep enable */
#define RCC_APB4LPENR_LPTIM4LPEN        (1 << 11) /* LPTIM4 sleep enable */
#define RCC_APB4LPENR_LPTIM5LPEN        (1 << 12) /* LPTIM5 sleep enable */
#define RCC_APB4LPENR_VREFLPEN          (1 << 15) /* VREF sleep enable */
#define RCC_APB4LPENR_RTCAPBLPEN        (1 << 16) /* RTCAPB sleep enable */
#define RCC_APB4LPENR_DTSLPEN           (1 << 26) /* DTS sleep enable */

#define RCC_APB5LPENR_LTDCLPEN          (1 << 1) /* LTDC sleep enable */
#define RCC_APB5LPENR_DCMIPPLPEN        (1 << 2) /* DCMIPP sleep enable */
#define RCC_APB5LPENR_GFXTIMLPEN        (1 << 4) /* GFXTIM sleep enable */

/* Legacy H7 register names used by common drivers **************************/

#define STM32_RCC_APB1LENR              STM32_RCC_APB1ENR1
#define STM32_RCC_APB1LRSTR             STM32_RCC_APB1RSTR1
#define RCC_APB1LENR_TIM2EN             RCC_APB1ENR1_TIM2EN
#define RCC_APB1LENR_TIM3EN             RCC_APB1ENR1_TIM3EN
#define RCC_APB1LENR_TIM4EN             RCC_APB1ENR1_TIM4EN
#define RCC_APB1LENR_TIM5EN             RCC_APB1ENR1_TIM5EN
#define RCC_APB1LENR_TIM6EN             RCC_APB1ENR1_TIM6EN
#define RCC_APB1LENR_TIM7EN             RCC_APB1ENR1_TIM7EN
#define RCC_APB1LENR_LPTIM1EN           RCC_APB1ENR1_LPTIM1EN
#define RCC_APB1LENR_SPI2EN             RCC_APB1ENR1_SPI2EN
#define RCC_APB1LENR_SPI3EN             RCC_APB1ENR1_SPI3EN
#define RCC_APB1LENR_USART2EN           RCC_APB1ENR1_USART2EN
#define RCC_APB1LENR_USART3EN           RCC_APB1ENR1_USART3EN
#define RCC_APB1LENR_UART4EN            RCC_APB1ENR1_UART4EN
#define RCC_APB1LENR_UART5EN            RCC_APB1ENR1_UART5EN
#define RCC_APB1LENR_I2C1EN             RCC_APB1ENR1_I2C1_I3C1EN
#define RCC_APB1LENR_I2C2EN             RCC_APB1ENR1_I2C2EN
#define RCC_APB1LENR_I2C3EN             RCC_APB1ENR1_I2C3EN
#define RCC_APB1LENR_UART7EN            RCC_APB1ENR1_UART7EN
#define RCC_APB1LENR_UART8EN            RCC_APB1ENR1_UART8EN
#define RCC_APB1LRSTR_I2C1RST           RCC_APB1RSTR1_I2C1_I3C1RST
#define RCC_APB1LRSTR_I2C2RST           RCC_APB1RSTR1_I2C2RST
#define RCC_APB1LRSTR_I2C3RST           RCC_APB1RSTR1_I2C3RST

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_RCC_H */
