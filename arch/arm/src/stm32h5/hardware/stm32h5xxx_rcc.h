/****************************************************************************
 * arch/arm/src/stm32h5/hardware/stm32h5xxx_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32H5XXX_RCC_H
#define __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32H5XXX_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_STM32H5_STM32H5XXXX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_RCC_CR_OFFSET           0x0000  /* Clock control register */
#define STM32_RCC_HSICFGR_OFFSET      0x0010  /* HSI Calibration Register */
#define STM32_RCC_CRRCR_OFFSET        0x0014  /* RCC clock recovery RC register */
#define STM32_RCC_CFGR1_OFFSET        0x001c  /* RCC clock configuration register 1 */
#define STM32_RCC_CFGR2_OFFSET        0x0020  /* RCC clock configuration register 2 */
#define STM32_RCC_PLL1CFGR_OFFSET     0x0028  /* RCC PLL1 configuration register */
#define STM32_RCC_PLL2CFGR_OFFSET     0x002c  /* RCC PLL2 configuration register */
#define STM32_RCC_PLL3CFGR_OFFSET     0x0030  /* RCC PLL3 configuration register */
#define STM32_RCC_PLL1DIVR_OFFSET     0x0034  /* RCC PLL1 dividers register */
#define STM32_RCC_PLL1FRACR_OFFSET    0x0038  /* RCC PLL1 fractional divider register */
#define STM32_RCC_PLL2DIVR_OFFSET     0x003c  /* RCC PLL2 dividers register */
#define STM32_RCC_PLL2FRACR_OFFSET    0x0040  /* RCC PLL2 fractional divider register */
#define STM32_RCC_PLL3DIVR_OFFSET     0x0044  /* RCC PLL3 dividers register */
#define STM32_RCC_PLL3FRACR_OFFSET    0x0048  /* RCC PLL3 fractional divider register */
#define STM32_RCC_CIER_OFFSET         0x0050  /* RCC clock interrupt enable register */
#define STM32_RCC_CIFR_OFFSET         0x0054  /* RCC clock interrupt flag register */
#define STM32_RCC_CICR_OFFSET         0x0058  /* RCC clock interrupt clear register */
#define STM32_RCC_AHB1RSTR_OFFSET     0x0060  /* RCC AHB1 peripheral reset register */
#define STM32_RCC_AHB2RSTR_OFFSET     0x0064  /* RCC AHB2 peripheral reset register 1 */
#define STM32_RCC_AHB4RSTR_OFFSET     0x006c  /* RCC AHB4 peripheral reset register*/
#define STM32_RCC_APB1LRSTR_OFFSET    0x0074  /* RCC APB1 peripheral reset register 1 */
#define STM32_RCC_APB1HRSTR_OFFSET    0x0078  /* RCC APB1 peripheral reset register 2 */
#define STM32_RCC_APB2RSTR_OFFSET     0x007c  /* RCC APB2 peripheral reset register */
#define STM32_RCC_APB3RSTR_OFFSET     0x0080  /* RCC APB3 peripheral reset register */
#define STM32_RCC_AHB1ENR_OFFSET      0x0088  /* RCC AHB1 peripheral clock enable register */
#define STM32_RCC_AHB2ENR_OFFSET      0x008c  /* RCC AHB2 peripheral clock enable register */
#define STM32_RCC_AHB4ENR_OFFSET      0x0094  /* RCC AHB4 peripheral clock enable register */
#define STM32_RCC_APB1LENR_OFFSET     0x009c  /* RCC APB1 peripheral clock enable register 1 */
#define STM32_RCC_APB1HENR_OFFSET     0x00a0  /* RCC APB1 peripheral clock enable register 2 */
#define STM32_RCC_APB2ENR_OFFSET      0x00a4  /* RCC APB2 peripheral clock enable register */
#define STM32_RCC_APB3ENR_OFFSET      0x00a8  /* RCC APB3 peripheral clock enable register */
#define STM32_RCC_AHB1LPENR_OFFSET    0x00b0  /* RCC AHB1 peripheral clocks enable in Sleep and Stop modes register */
#define STM32_RCC_AHB2LPENR_OFFSET    0x00b4  /* RCC AHB2 peripheral clocks enable in Sleep and Stop modes register */
#define STM32_RCC_AHB4LPENR_OFFSET    0x00bc  /* RCC AHB3 peripheral clocks enable in Sleep and Stop modes register */
#define STM32_RCC_APB1LPENR1_OFFSET   0x00c4  /* RCC APB1 peripheral clocks enable in Sleep and Stop modes register 1 */
#define STM32_RCC_APB1LPENR2_OFFSET   0x00c8  /* RCC APB1 peripheral clocks enable in Sleep and Stop modes register 2 */
#define STM32_RCC_APB2LPENR_OFFSET    0x00cc  /* RCC APB2 peripheral clocks enable in Sleep and Stop modes register */
#define STM32_RCC_APB3LPENR_OFFSET    0x00d0  /* RCC APB3 peripheral clocks enable in Sleep and Stop modes register */
#define STM32_RCC_CCIPR1_OFFSET       0x00d8  /* RCC peripherals independent clock configuration register 1 */
#define STM32_RCC_CCIPR2_OFFSET       0x00dc  /* RCC peripherals independent clock configuration register 2 */
#define STM32_RCC_CCIPR3_OFFSET       0x00e0  /* RCC peripherals independent clock configuration register 3 */
#define STM32_RCC_CCIPR4_OFFSET       0x00e4  /* RCC peripherals independent clock configuration register 5 */
#define STM32_RCC_CCIPR5_OFFSET       0x00e8  /* RCC peripherals independent clock configuration register 5 */
#define STM32_RCC_BDCR_OFFSET         0x00f0  /* RCC Backup domain control register */
#define STM32_RCC_RSR_OFFSET          0x00f4  /* RCC control/status register */ /* TODO: CSR in U5 */
#define STM32_RCC_SECCFGR_OFFSET      0x0110  /* RCC secure configuration register */
#define STM32_RCC_PRIVCFGR_OFFSET     0x0114  /* RCC privilege configuration register */

/* Register Addresses *******************************************************/

#define STM32_RCC_CR           (STM32_RCC_BASE +  STM32_RCC_CR_OFFSET)
#define STM32_RCC_HSICFGR      (STM32_RCC_BASE + STM32_RCC_HSICFGR_OFFSET)
#define STM32_RCC_CRRCR        (STM32_RCC_BASE + STM32_RCC_CRRCR_OFFSET)
#define STM32_RCC_CFGR1        (STM32_RCC_BASE + STM32_RCC_CFGR1_OFFSET)
#define STM32_RCC_CFGR2        (STM32_RCC_BASE + STM32_RCC_CFGR2_OFFSET)
#define STM32_RCC_PLL1CFGR     (STM32_RCC_BASE + STM32_RCC_PLL1CFGR_OFFSET)
#define STM32_RCC_PLL2CFGR     (STM32_RCC_BASE + STM32_RCC_PLL2CFGR_OFFSET)
#define STM32_RCC_PLL3CFGR     (STM32_RCC_BASE + STM32_RCC_PLL3CFGR_OFFSET)
#define STM32_RCC_PLL1DIVR     (STM32_RCC_BASE + STM32_RCC_PLL1DIVR_OFFSET)
#define STM32_RCC_PLL1FRACR    (STM32_RCC_BASE + STM32_RCC_PLL1FRACR_OFFSET)
#define STM32_RCC_PLL2DIVR     (STM32_RCC_BASE + STM32_RCC_PLL2DIVR_OFFSET)
#define STM32_RCC_PLL2FRACR    (STM32_RCC_BASE + STM32_RCC_PLL2FRACR_OFFSET)
#define STM32_RCC_PLL3DIVR     (STM32_RCC_BASE + STM32_RCC_PLL3DIVR_OFFSET)
#define STM32_RCC_PLL3FRACR    (STM32_RCC_BASE + STM32_RCC_PLL3FRACR_OFFSET)
#define STM32_RCC_CIER         (STM32_RCC_BASE + STM32_RCC_CIER_OFFSET)
#define STM32_RCC_CIFR         (STM32_RCC_BASE + STM32_RCC_CIFR_OFFSET)
#define STM32_RCC_CICR         (STM32_RCC_BASE + STM32_RCC_CICR_OFFSET)
#define STM32_RCC_AHB1RSTR     (STM32_RCC_BASE + STM32_RCC_AHB1RSTR_OFFSET)
#define STM32_RCC_AHB2RSTR     (STM32_RCC_BASE + STM32_RCC_AHB2RSTR_OFFSET)
#define STM32_RCC_AHB4RSTR     (STM32_RCC_BASE + STM32_RCC_AHB4RSTR_OFFSET)
#define STM32_RCC_APB1LRSTR    (STM32_RCC_BASE + STM32_RCC_APB1LRSTR_OFFSET)
#define STM32_RCC_APB1HRSTR    (STM32_RCC_BASE + STM32_RCC_APB1HRSTR_OFFSET)
#define STM32_RCC_APB2RSTR     (STM32_RCC_BASE + STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_APB3RSTR     (STM32_RCC_BASE + STM32_RCC_APB3RSTR_OFFSET)
#define STM32_RCC_AHB1ENR      (STM32_RCC_BASE + STM32_RCC_AHB1ENR_OFFSET)
#define STM32_RCC_AHB2ENR      (STM32_RCC_BASE + STM32_RCC_AHB2ENR_OFFSET)
#define STM32_RCC_AHB4ENR      (STM32_RCC_BASE + STM32_RCC_AHB4ENR_OFFSET)
#define STM32_RCC_APB1LENR     (STM32_RCC_BASE + STM32_RCC_APB1LENR_OFFSET)
#define STM32_RCC_APB1HENR     (STM32_RCC_BASE + STM32_RCC_APB1HENR_OFFSET)
#define STM32_RCC_APB2ENR      (STM32_RCC_BASE + STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_APB3ENR      (STM32_RCC_BASE + STM32_RCC_APB3ENR_OFFSET)
#define STM32_RCC_AHB1LPENR    (STM32_RCC_BASE + STM32_RCC_AHB1LPENR_OFFSET)
#define STM32_RCC_AHB2LPENR    (STM32_RCC_BASE + STM32_RCC_AHB2LPENR_OFFSET)
#define STM32_RCC_AHB4LPENR    (STM32_RCC_BASE + STM32_RCC_AHB4LPENR_OFFSET)
#define STM32_RCC_APB1LPENR    (STM32_RCC_BASE + STM32_RCC_APB1LPENR1_OFFSET)
#define STM32_RCC_APB1HPENR    (STM32_RCC_BASE + STM32_RCC_APB1LPENR2_OFFSET)
#define STM32_RCC_APB2LPENR    (STM32_RCC_BASE + STM32_RCC_APB2LPENR_OFFSET)
#define STM32_RCC_APB3LPENR    (STM32_RCC_BASE + STM32_RCC_APB3LPENR_OFFSET)
#define STM32_RCC_CCIPR1       (STM32_RCC_BASE + STM32_RCC_CCIPR1_OFFSET)
#define STM32_RCC_CCIPR2       (STM32_RCC_BASE + STM32_RCC_CCIPR2_OFFSET)
#define STM32_RCC_CCIPR3       (STM32_RCC_BASE + STM32_RCC_CCIPR3_OFFSET)
#define STM32_RCC_CCIPR4       (STM32_RCC_BASE + STM32_RCC_CCIPR4_OFFSET)
#define STM32_RCC_CCIPR5       (STM32_RCC_BASE + STM32_RCC_CCIPR5_OFFSET)
#define STM32_RCC_BDCR         (STM32_RCC_BASE + STM32_RCC_BDCR_OFFSET)
#define STM32_RCC_RSR          (STM32_RCC_BASE + STM32_RCC_RSR_OFFSET)
#define STM32_RCC_SECCFGR      (STM32_RCC_BASE + STM32_RCC_SECCFGR_OFFSET)
#define STM32_RCC_PRIVCFGR     (STM32_RCC_BASE + STM32_RCC_PRIVCFGR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Clock control register */

#define RCC_CR_HSION                     (1 << 0)  /* Bit    0: Internal High Speed clock enable */
#define RCC_CR_HSIRDY                    (1 << 1)  /* Bit    1: Internal High Speed clock ready flag */
#define RCC_CR_HSIKERON                  (1 << 2)  /* Bit    2: HSI clock enable in Stop mode */
#define RCC_CR_HSIDIV_SHIFT              (3)       /* Bit [4:3] HSI Divider */
#define RCC_CR_HSIDIV_MASK               (0x3 << RCC_CR_HSIDIV_SHIFT)
#  define RCC_CR_HSIDIV(n)               (((n) << RCC_CR_HSIDIV_SHIFT & RCC_CR_HSIDIV_MASK))
#define RCC_CR_HSIDIVF                   (1 << 5)  /* Bit    5: HSI divider flag */

#define RCC_CR_CSION                     (1 << 8)  /* Bit    8:  CSI clock enable */
#define RCC_CR_CSIRDY                    (1 << 9)  /* Bit    9:  CSI clock ready flag */
#define RCC_CR_CSIKERON                  (1 << 10) /* Bit    10: CSI clock enable in Stop mode */

#define RCC_CR_HSI48ON                   (1 << 12) /* Bit    12:  HSI48 clock enable */
#define RCC_CR_HSI48RDY                  (1 << 13) /* Bit    13:  HSI48 clock ready flag */

#define RCC_CR_HSEON                     (1 << 16) /* Bit    16:  HSE clock enable */
#define RCC_CR_HSERDY                    (1 << 17) /* Bit    17:  HSE clock ready flag */
#define RCC_CR_HSEBYP                    (1 << 18) /* Bit    18:  HSE clock bypass */
#define RCC_CR_HSECSSON                  (1 << 19) /* Bit    19:  HSE clock security system enable */
#define RCC_CR_HSEEXT                    (1 << 20) /* Bit    20:  HSE external high speed clock
                                                    *             type in bypass mode */
#define RCC_CR_PLL1ON                    (1 << 24) /* Bit    24: PLL1 enable */
#define RCC_CR_PLL1RDY                   (1 << 25) /* Bit    25: PLL1 clock ready flag */
#define RCC_CR_PLL2ON                    (1 << 26) /* Bit    26: PLL2 enable */
#define RCC_CR_PLL2RDY                   (1 << 27) /* Bit    27: PLL2 clock ready flag */
#define RCC_CR_PLL3ON                    (1 << 28) /* Bit    28: PLL3 enable */
#define RCC_CR_PLL3RDY                   (1 << 29) /* Bit    29: PLL3 clock ready flag */

/* HSI Calibration register */

#define RCC_HSICFGR_HSITRIM_SHIFT        (16)      /* Bits 22-16: Internal High Speed clock trimming */
#define RCC_HSICFGR_HSITRIM_MASK         (0x7f << RCC_HSICFGR_HSITRIM_SHIFT)
#define RCC_HSICFGR_HSICAL_SHIFT         (0)      /* Bits 11-0: Internal High Speed clock Calibration */
#define RCC_HSICFGR_HSICAL_MASK          (0xfff << RCC_HSICFGR_HSICAL_SHIFT)

/* Clock Recovery RC register */

#define RCC_CRRCR_HSI48CAL_SHIFT         (0)      /* Bits 9-0: Internal RC 48MHz Clock Calibration */
#define RCC_CRRCR_HSI48CAL_MASK          (0x3ff << RCC_CRRCR_HSI48CAL_SHIFT)

/* CSI Calibration register */

#define RCC_CSICFGR_CSITRIM_SHIFT        (16)      /* Bits 21-16: Internal High Speed clock trimming */
#define RCC_CSICFGR_CSITRIM_MASK         (0x3f << RCC_CSICFGR_CSITRIM_SHIFT)
#define RCC_CSICFGR_CSICAL_SHIFT         (0)      /* Bits 11-0: Internal High Speed clock Calibration */
#define RCC_CSICFGR_CSICAL_MASK          (0xff << RCC_CSICFGR_CSICAL_SHIFT)

/* Clock configuration register 1 */

#define RCC_CFGR1_SW_SHIFT                (0)       /* Bits 0-1: System clock Switch */
#define RCC_CFGR1_SW_MASK                 (3 << RCC_CFGR1_SW_SHIFT)
#  define RCC_CFGR1_SW_MSI                (0 << RCC_CFGR1_SW_SHIFT) /* 00: MSI selected as system clock */
#  define RCC_CFGR1_SW_HSI16              (1 << RCC_CFGR1_SW_SHIFT) /* 00: HSI16 selected as system clock */
#  define RCC_CFGR1_SW_HSE                (2 << RCC_CFGR1_SW_SHIFT) /* 01: HSE selected as system clock */
#  define RCC_CFGR1_SW_PLL                (3 << RCC_CFGR1_SW_SHIFT) /* 10: PLL selected as system clock */

#define RCC_CFGR1_SWS_SHIFT               (3)       /* Bits 3-4: System Clock Switch Status */
#define RCC_CFGR1_SWS_MASK                (3 << RCC_CFGR1_SWS_SHIFT)
#  define RCC_CFGR1_SWS_MSI               (0 << RCC_CFGR1_SWS_SHIFT) /* 00: MSI oscillator used as system clock */
#  define RCC_CFGR1_SWS_HSI16             (1 << RCC_CFGR1_SWS_SHIFT) /* 00: HSI16 oscillator used as system clock */
#  define RCC_CFGR1_SWS_HSE               (2 << RCC_CFGR1_SWS_SHIFT) /* 01: HSE oscillator used as system clock */
#  define RCC_CFGR1_SWS_PLL               (3 << RCC_CFGR1_SWS_SHIFT) /* 10: PLL used as system clock */

#define RCC_CFGR1_STOPWUCK               (1 << 6) /* System clock selection after a wakeup from system stop */
#define RCC_CFGR1_STOPKERWUCK            (1 << 7) /* Kernel clock selection after a wakeup from system stop */

#define RCC_CFGR1_RTCPRE_SHIFT           (8)                            /* Bits 13-8: RTC prescaler */
#define RCC_CFGR1_RTCPRE_MASK            (0x3f << RCC_CFGR1_RTCPRE_SHIFT)
#  define RCC_CFGR1_RTCPRE_HSE           (0 << RCC_CFGR1_RTCPRE_SHIFT)  /* 0xxx: HSE not divided */
#  define RCC_CFGR1_RTCPRE_HSEd2         (2 << RCC_CFGR1_RTCPRE_SHIFT)  /* 10: HSE divided by 2 */
#  define RCC_CFGR1_RTCPRE_HSEd4         (4 << RCC_CFGR1_RTCPRE_SHIFT)  /* 100: HSE divided by 4 */
#  define RCC_CFGR1_RTCPRE_HSEd8         (8 << RCC_CFGR1_RTCPRE_SHIFT)  /* 1000: HSE divided by 8 */
#  define RCC_CFGR1_RTCPRE_HSEd16        (16 << RCC_CFGR1_RTCPRE_SHIFT) /* 10000: HSE divided by 16 */
#  define RCC_CFGR1_RTCPRE_HSEd32        (32 << RCC_CFGR1_RTCPRE_SHIFT) /* 100000: HSE divided by 32 */
#  define RCC_CFGR1_RTCPRE_HSEd63        (63 << RCC_CFGR1_RTCPRE_SHIFT) /* 111111: HSE divided by 63 */

#define RCC_CFGR1_TIMPRE                 (1 << 15) /* timers clocks prescaler selection */

#define RCC_CFGR1_MCO1PRE_SHIFT             (18) /* Bits 21-18: MCO1 Prescaler */
#define RCC_CFGR1_MCO1PRE_MASK              (0xf << RCC_CFGR1_MCO1PRE_SHIFT)
#  define RCC_CFGR1_MCO1PRE_MCO1            (0 << RCC_CFGR1_MCO1PRE_SHIFT)  /* 0xx: MCO1 not divided */
#  define RCC_CFGR1_MCO1PRE_MCO1d2          (2 << RCC_CFGR1_MCO1PRE_SHIFT)  /* 10: MCO1 divided by 2 */
#  define RCC_CFGR1_MCO1PRE_MCO1d4          (4 << RCC_CFGR1_MCO1PRE_SHIFT)  /* 100: MCO1 divided by 4 */
#  define RCC_CFGR1_MCO1PRE_MCO1d8          (8 << RCC_CFGR1_MCO1PRE_SHIFT)  /* 1000: MCO1 divided by 8 */
#  define RCC_CFGR1_MCO1PRE_MCO1d15         (15 << RCC_CFGR1_MCO1PRE_SHIFT) /* 1111: MCO1 divided by 15 */

#define RCC_CFGR1_MCO1SEL_SHIFT            (22) /* Bits 24-22: Microcontroller Clock Output1 */
#define RCC_CFGR1_MCO1SEL_MASK             (0x7 << RCC_CFGR1_MCO1SEL_SHIFT)
#  define RCC_CFGR1_MCO1SEL_HSI            (0 << RCC_CFGR1_MCO1SEL_SHIFT) /* 0000: HSI clock selected */
#  define RCC_CFGR1_MCO1SEL_LSE            (1 << RCC_CFGR1_MCO1SEL_SHIFT) /* 0001: LSE clock selected */
#  define RCC_CFGR1_MCO1SEL_HSE            (2 << RCC_CFGR1_MCO1SEL_SHIFT) /* 0010: HSE clock selected */
#  define RCC_CFGR1_MCO1SEL_PLL1           (3 << RCC_CFGR1_MCO1SEL_SHIFT) /* 0011: Main PLL1 selected  */
#  define RCC_CFGR1_MCO1SEL_HSI48          (4 << RCC_CFGR1_MCO1SEL_SHIFT) /* 0100: HSI48 clock selected */

#define RCC_CFGR1_MCO2PRE_SHIFT             (25) /* Bits 28-25: MCO2 Prescaler */
#define RCC_CFGR1_MCO2PRE_MASK              (0xf << RCC_CFGR1_MCO2PRE_SHIFT)
#  define RCC_CFGR1_MCO2PRE_MCO2            (0 << RCC_CFGR1_MCO2PRE_SHIFT)  /* 0xx: MCO2 not divided */
#  define RCC_CFGR1_MCO2PRE_MCO2d2          (2 << RCC_CFGR1_MCO2PRE_SHIFT)  /* 10: MCO2 divided by 2 */
#  define RCC_CFGR1_MCO2PRE_MCO2d4          (4 << RCC_CFGR1_MCO2PRE_SHIFT)  /* 100: MCO2 divided by 4 */
#  define RCC_CFGR1_MCO2PRE_MCO2d8          (8 << RCC_CFGR1_MCO2PRE_SHIFT)  /* 1000: MCO2 divided by 8 */
#  define RCC_CFGR1_MCO2PRE_MCO2d15         (15 << RCC_CFGR1_MCO2PRE_SHIFT) /* 1111: MCO2 divided by 15 */

#define RCC_CFGR1_MCO2SEL_SHIFT            (29) /* Bits 31-29: Microcontroller Clock Output1 */
#define RCC_CFGR1_MCO2SEL_MASK             (0x7 << RCC_CFGR1_MCO2SEL_SHIFT)
#  define RCC_CFGR1_MCO2SEL_SYSCLLK        (0 << RCC_CFGR1_MCO2SEL_SHIFT) /* 0000: Main SYSCLLK selected  */
#  define RCC_CFGR1_MCO2SEL_PLL2           (1 << RCC_CFGR1_MCO2SEL_SHIFT) /* 0001: Main PLL2 selected  */
#  define RCC_CFGR1_MCO2SEL_HSE            (2 << RCC_CFGR1_MCO2SEL_SHIFT) /* 0010: HSE clock selected */
#  define RCC_CFGR1_MCO2SEL_PLL1           (3 << RCC_CFGR1_MCO2SEL_SHIFT) /* 0011: Main PLL1 selected  */
#  define RCC_CFGR1_MCO2SEL_CSI            (4 << RCC_CFGR1_MCO2SEL_SHIFT) /* 0100: CSI clock selected */
#  define RCC_CFGR1_MCO2SEL_LSI            (5 << RCC_CFGR1_MCO2SEL_SHIFT) /* 0101: LSI clock selected */

/* Clock configuration register 2 */

#define RCC_CFGR2_HPRE_SHIFT             (0) /* Bits 3-0: HPRE Prescaler */
#define RCC_CFGR2_HPRE_MASK              (0xf << RCC_CFGR2_HPRE_SHIFT)
#  define RCC_CFGR2_HPRE_SYSCLK          (0 << RCC_CFGR2_HPRE_SHIFT)  /* 0xx: SYSCLK not divided */
#  define RCC_CFGR2_HPRE_SYSCLKd2        (8 << RCC_CFGR2_HPRE_SHIFT)  /* 1000: SYSCLK divided by 2 */
#  define RCC_CFGR2_HPRE_SYSCLKd4        (9 << RCC_CFGR2_HPRE_SHIFT)  /* 1001: SYSCLK divided by 4 */
#  define RCC_CFGR2_HPRE_SYSCLKd8        (10 << RCC_CFGR2_HPRE_SHIFT) /* 1010: SYSCLK divided by 8 */
#  define RCC_CFGR2_HPRE_SYSCLKd16       (11 << RCC_CFGR2_HPRE_SHIFT) /* 1011: SYSCLK divided by 16 */
#  define RCC_CFGR2_HPRE_SYSCLKd64       (12 << RCC_CFGR2_HPRE_SHIFT) /* 1100: SYSCLK divided by 64 */
#  define RCC_CFGR2_HPRE_SYSCLKd128      (13 << RCC_CFGR2_HPRE_SHIFT) /* 1101: SYSCLK divided by 128 */
#  define RCC_CFGR2_HPRE_SYSCLKd256      (14 << RCC_CFGR2_HPRE_SHIFT) /* 1110: SYSCLK divided by 256 */
#  define RCC_CFGR2_HPRE_SYSCLKd512      (15 << RCC_CFGR2_HPRE_SHIFT) /* 1111: SYSCLK divided by 512 */

#define RCC_CFGR2_PPRE1_SHIFT            (0) /* Bits 6-4: PPRE1 Prescaler */
#define RCC_CFGR2_PPRE1_MASK             (0x7 << RCC_CFGR2_PPRE1_SHIFT)
#  define RCC_CFGR2_PPRE1_HCLK1          (0 << RCC_CFGR2_PPRE1_SHIFT)  /* 0xx: HCLK1 not divided */
#  define RCC_CFGR2_PPRE1_HCLK1d2        (4 << RCC_CFGR2_PPRE1_SHIFT)  /* 1000: HCLK1 divided by 2 */
#  define RCC_CFGR2_PPRE1_HCLK1d4        (5 << RCC_CFGR2_PPRE1_SHIFT)  /* 1001: HCLK1 divided by 4 */
#  define RCC_CFGR2_PPRE1_HCLK1d8        (6 << RCC_CFGR2_PPRE1_SHIFT)  /* 1010: HCLK1 divided by 8 */
#  define RCC_CFGR2_PPRE1_HCLK1d16       (7 << RCC_CFGR2_PPRE1_SHIFT)  /* 1011: HCLK1 divided by 16 */

#define RCC_CFGR2_PPRE2_SHIFT            (8) /* Bits 10-8: PPRE2 Prescaler */
#define RCC_CFGR2_PPRE2_MASK             (0x7 << RCC_CFGR2_PPRE2_SHIFT)
#  define RCC_CFGR2_PPRE2_HCLK1          (0 << RCC_CFGR2_PPRE2_SHIFT)  /* 0xx: HCLK1 not divided */
#  define RCC_CFGR2_PPRE2_HCLK1d2        (4 << RCC_CFGR2_PPRE2_SHIFT)  /* 1000: HCLK1 divided by 2 */
#  define RCC_CFGR2_PPRE2_HCLK1d4        (5 << RCC_CFGR2_PPRE2_SHIFT)  /* 1001: HCLK1 divided by 4 */
#  define RCC_CFGR2_PPRE2_HCLK1d8        (6 << RCC_CFGR2_PPRE2_SHIFT)  /* 1010: HCLK1 divided by 8 */
#  define RCC_CFGR2_PPRE2_HCLK1d16       (7 << RCC_CFGR2_PPRE2_SHIFT)  /* 1011: HCLK1 divided by 16 */

#define RCC_CFGR2_PPRE3_SHIFT            (12) /* Bits 14-12: PPRE3 Prescaler */
#define RCC_CFGR2_PPRE3_MASK             (0x7 << RCC_CFGR2_PPRE3_SHIFT)
#  define RCC_CFGR2_PPRE3_HCLK1          (0 << RCC_CFGR2_PPRE3_SHIFT)  /* 0xx: HCLK1 not divided */
#  define RCC_CFGR2_PPRE3_HCLK1d2        (4 << RCC_CFGR2_PPRE3_SHIFT)  /* 1000: HCLK1 divided by 2 */
#  define RCC_CFGR2_PPRE3_HCLK1d4        (5 << RCC_CFGR2_PPRE3_SHIFT)  /* 1001: HCLK1 divided by 4 */
#  define RCC_CFGR2_PPRE3_HCLK1d8        (6 << RCC_CFGR2_PPRE3_SHIFT)  /* 1010: HCLK1 divided by 8 */
#  define RCC_CFGR2_PPRE3_HCLK1d16       (7 << RCC_CFGR2_PPRE3_SHIFT)  /* 1011: HCLK1 divided by 16 */

#define RCC_CFGR2_AHB1DIS                (1 << 16) /* AHB1 clock disable */
#define RCC_CFGR2_AHB2DIS                (1 << 17) /* AHB2 clock disable */
#define RCC_CFGR2_AHB4DIS                (1 << 19) /* AHB4 clock disable */
#define RCC_CFGR2_APB1DIS                (1 << 20) /* APB1 clock disable */
#define RCC_CFGR2_APB2DIS                (1 << 21) /* APB2 clock disable */
#define RCC_CFGR2_APB3DIS                (1 << 22) /* APB3 clock disable */

/* PLL1 configuration register */

#define RCC_PLL1CFGR_PLL1SRC_SHIFT          (0) /* Bits 1-0: Main PLL(PLL) and audio PLLs (PLLSAIx)
                                               * entry clock source */
#define RCC_PLL1CFGR_PLL1SRC_MASK           (3 << RCC_PLL1CFGR_PLL1SRC_SHIFT)
#  define RCC_PLL1CFGR_PLL1SRC_NONE         (0 << RCC_PLL1CFGR_PLL1SRC_SHIFT) /* 000: No clock sent to PLLs */
#  define RCC_PLL1CFGR_PLL1SRC_HSI          (1 << RCC_PLL1CFGR_PLL1SRC_SHIFT) /* 001: HSI selected as PLL source */
#  define RCC_PLL1CFGR_PLL1SRC_CSI          (2 << RCC_PLL1CFGR_PLL1SRC_SHIFT) /* 010: CSI selected as PLL source */
#  define RCC_PLL1CFGR_PLL1SRC_HSE          (3 << RCC_PLL1CFGR_PLL1SRC_SHIFT) /* 011: HSE selected as PLL source */

#define RCC_PLL1CFGR_PLL1RGE_SHIFT          (2) /* Bit 3-2:  */
#define RCC_PLL1CFGR_PLL1RGE_MASK           (3 << RCC_PLL1CFGR_PLL1RGE_SHIFT)
#  define RCC_PLL1CFGR_PLL1RGE_1_2M         (0 << RCC_PLL1CFGR_PLL1RGE_SHIFT) /* 000: Input Clock Range Frequency 1-2 MHz */
#  define RCC_PLL1CFGR_PLL1RGE_2_4M         (1 << RCC_PLL1CFGR_PLL1RGE_SHIFT) /* 001: Input Clock Range Frequency 2-4 MHz */
#  define RCC_PLL1CFGR_PLL1RGE_4_8M         (2 << RCC_PLL1CFGR_PLL1RGE_SHIFT) /* 010: Input Clock Range Frequency 4-8 MHz **/
#  define RCC_PLL1CFGR_PLL1RGE_8_16M        (3 << RCC_PLL1CFGR_PLL1RGE_SHIFT) /* 011: Input Clock Range Frequency 8-16 MHz **/

#define RCC_PLL1CFGR_PLL1FRACEN             (1 << 4) /* PLL1 Fractional Latch Enable */
#define RCC_PLL1CFGR_PLL1VCOSEL             (1 << 5) /* PLL1 VCO Selection */

#define RCC_PLL1CFGR_PLL1M_SHIFT            (8) /* Bit 13-8:  */
#define RCC_PLL1CFGR_PLL1M_MASK             (0x3f << RCC_PLL1CFGR_PLL1M_SHIFT)
#  define RCC_PLL1CFGR_PLL1M(n)              ((n) << RCC_PLL1CFGR_PLL1M_SHIFT) /* m = 1..63 */

#define RCC_PLL1CFGR_PLL1PEN                (1 << 16) /* PLL1 DIVP divder output enable */
#define RCC_PLL1CFGR_PLL1QEN                (1 << 17) /* PLL1 DIVQ divder output enable */
#define RCC_PLL1CFGR_PLL1REN                (1 << 18) /* PLL1 DIVR divder output enable */

/* PLL2 configuration register */

#define RCC_PLL2CFGR_PLL2SRC_SHIFT          (0) /* Bits 1-0: Main PLL(PLL) and audio PLLs (PLLSAIx)
                                               * entry clock source */
#define RCC_PLL2CFGR_PLL2SRC_MASK           (3 << RCC_PLL2CFGR_PLL2SRC_SHIFT)
#  define RCC_PLL2CFGR_PLL2SRC_NONE         (0 << RCC_PLL2CFGR_PLL2SRC_SHIFT) /* 000: No clock sent to PLLs */
#  define RCC_PLL2CFGR_PLL2SRC_HSI          (1 << RCC_PLL2CFGR_PLL2SRC_SHIFT) /* 001: HSI selected as PLL source */
#  define RCC_PLL2CFGR_PLL2SRC_CSI          (2 << RCC_PLL2CFGR_PLL2SRC_SHIFT) /* 010: CSI selected as PLL source */
#  define RCC_PLL2CFGR_PLL2SRC_HSE          (3 << RCC_PLL2CFGR_PLL2SRC_SHIFT) /* 011: HSE selected as PLL source */

#define RCC_PLL2CFGR_PLL2RGE_SHIFT          (2) /* Bit 3-2:  */
#define RCC_PLL2CFGR_PLL2RGE_MASK           (3 << RCC_PLL2CFGR_PLL2RGE_SHIFT)
#  define RCC_PLL2CFGR_PLL2RGE_1_2M         (0 << RCC_PLL2CFGR_PLL2RGE_SHIFT) /* 000: Input Clock Range Frequency 1-2 MHz  */
#  define RCC_PLL2CFGR_PLL2RGE_2_4M         (1 << RCC_PLL2CFGR_PLL2RGE_SHIFT) /* 001: Input Clock Range Frequency 2-4 MHz  */
#  define RCC_PLL2CFGR_PLL2RGE_4_8M         (2 << RCC_PLL2CFGR_PLL2RGE_SHIFT) /* 010: Input Clock Range Frequency 4-8 MHz  */
#  define RCC_PLL2CFGR_PLL2RGE_8_16M        (3 << RCC_PLL2CFGR_PLL2RGE_SHIFT) /* 011: Input Clock Range Frequency 8-16 MHz */

#define RCC_PLL2CFGR_PLL2FRACEN             (1 << 4) /* PLL2 Fractional Latch Enable */
#define RCC_PLL2CFGR_PLL2VCOSEL             (1 << 5) /* PLL2 VCO Selection */

#define RCC_PLL2CFGR_PLL2M_SHIFT            (8) /* Bit 13-8:  */
#define RCC_PLL2CFGR_PLL2M_MASK             (0x3f << RCC_PLL2CFGR_PLL2M_SHIFT)
#  define RCC_PLL2CFGR_PLL2M(n)              ((n) << RCC_PLL2CFGR_PLL2M_SHIFT) /* m = 1..63 */

#define RCC_PLL2CFGR_PLL2PEN                (1 << 16) /* PLL2 DIVP divder output enable */
#define RCC_PLL2CFGR_PLL2QEN                (1 << 17) /* PLL2 DIVQ divder output enable */
#define RCC_PLL2CFGR_PLL2REN                (1 << 18) /* PLL2 DIVR divder output enable */

/* PLL3 configuration register */

#define RCC_PLL3CFGR_PLL3SRC_SHIFT          (0) /* Bits 1-0: Main PLL(PLL) and audio PLLs (PLLSAIx)
                                               * entry clock source */
#define RCC_PLL3CFGR_PLL3SRC_MASK           (3 << RCC_PLL3CFGR_PLL3SRC_SHIFT)
#  define RCC_PLL3CFGR_PLL3SRC_NONE         (0 << RCC_PLL3CFGR_PLL3SRC_SHIFT) /* 000: No clock sent to PLLs */
#  define RCC_PLL3CFGR_PLL3SRC_HSI          (1 << RCC_PLL3CFGR_PLL3SRC_SHIFT) /* 001: HSI selected as PLL source */
#  define RCC_PLL3CFGR_PLL3SRC_CSI          (2 << RCC_PLL3CFGR_PLL3SRC_SHIFT) /* 010: CSI selected as PLL source */
#  define RCC_PLL3CFGR_PLL3SRC_HSE          (3 << RCC_PLL3CFGR_PLL3SRC_SHIFT) /* 011: HSE selected as PLL source */

#define RCC_PLL3CFGR_PLL3RGE_SHIFT          (2) /* Bit 3-2:  */
#define RCC_PLL3CFGR_PLL3RGE_MASK           (3 << RCC_PLL3CFGR_PLL3RGE_SHIFT)
#  define RCC_PLL3CFGR_PLL3RGE_1_2M         (0 << RCC_PLL3CFGR_PLL3RGE_SHIFT) /* 000: Input Clock Range Frequency 1-2 MHz */
#  define RCC_PLL3CFGR_PLL3RGE_2_4M         (1 << RCC_PLL3CFGR_PLL3RGE_SHIFT) /* 001: Input Clock Range Frequency 2-4 MHz */
#  define RCC_PLL3CFGR_PLL3RGE_4_8M         (2 << RCC_PLL3CFGR_PLL3RGE_SHIFT) /* 010: Input Clock Range Frequency 4-8 MHz **/
#  define RCC_PLL3CFGR_PLL3RGE_8_16M        (3 << RCC_PLL3CFGR_PLL3RGE_SHIFT) /* 011: Input Clock Range Frequency 8-16 MHz **/

#define RCC_PLL3CFGR_PLL3FRACEN             (1 << 4) /* PLL3 Fractional Latch Enable */
#define RCC_PLL3CFGR_PLL3VCOSEL             (1 << 5) /* PLL3 VCO Selection */

#define RCC_PLL3CFGR_PLL3M_SHIFT            (8) /* Bit 13-8:  */
#define RCC_PLL3CFGR_PLL3M_MASK             (0x3f << RCC_PLL3CFGR_PLL3M_SHIFT)
#  define RCC_PLL3CFGR_PLL3M(n)              ((n) << RCC_PLL3CFGR_PLL3M_SHIFT) /* m = 1..63 */

#define RCC_PLL3CFGR_PLL3PEN                (1 << 16) /* PLL3 DIVP divder output enable */
#define RCC_PLL3CFGR_PLL3QEN                (1 << 17) /* PLL3 DIVQ divder output enable */
#define RCC_PLL3CFGR_PLL3REN                (1 << 18) /* PLL3 DIVR divder output enable */

/* PLL1 dividers register */

#define RCC_PLL1DIVR_PLL1N_SHIFT            (0) /* Bits 8-0:  */
#define RCC_PLL1DIVR_PLL1N_MASK             (0x1ff << RCC_PLL1DIVR_PLL1N_SHIFT)
#  define RCC_PLL1DIVR_PLL1N(n)             ((n-1) << RCC_PLL1DIVR_PLL1N_SHIFT) /* m = 4..512 */

#define RCC_PLL1DIVR_PLL1P_SHIFT            (9) /* Bits 15-9:  */
#define RCC_PLL1DIVR_PLL1P_MASK             (0x7f << RCC_PLL1DIVR_PLL1P_SHIFT)
#  define RCC_PLL1DIVR_PLL1P(n)             ((n-1) << RCC_PLL1DIVR_PLL1P_SHIFT) /* m = 2..128 evens */

#define RCC_PLL1DIVR_PLL1Q_SHIFT            (16) /* Bits 22-16:  */
#define RCC_PLL1DIVR_PLL1Q_MASK             (0x7f << RCC_PLL1DIVR_PLL1Q_SHIFT)
#  define RCC_PLL1DIVR_PLL1Q(n)             ((n-1) << RCC_PLL1DIVR_PLL1Q_SHIFT) /* m = 1..128 */

#define RCC_PLL1DIVR_PLL1R_SHIFT            (24) /* Bits 30-24:  */
#define RCC_PLL1DIVR_PLL1R_MASK             (0x7f << RCC_PLL1DIVR_PLL1R_SHIFT)
#  define RCC_PLL1DIVR_PLL1R(n)             ((n-1) << RCC_PLL1DIVR_PLL1R_SHIFT) /* m = 1..128 */

/* PLL1 fractional divider register */

#define RCC_PLL1FRACR_PLL1FRACN_SHIFT       (3)
#define RCC_PLL1FRACR_PLL1FRACN_MASK        (0x1fff << RCC_PLL1FRACR_PLL1FRACN_SHIFT)
#define RCC_PLL1FRACR_PLL1FRACN(n)          (n << RCC_PLL1FRACR_PLL1FRACN_SHIFT) /* m = 0..8192 */

/* PLL2 dividers register */

#define RCC_PLL2DIVR_PLL2N_SHIFT            (0) /* Bits 8-0:  */
#define RCC_PLL2DIVR_PLL2N_MASK             (0x1ff << RCC_PLL2DIVR_PLL2N_SHIFT)
#  define RCC_PLL2DIVR_PLL2N(n)             ((n-1) << RCC_PLL2DIVR_PLL2N_SHIFT) /* m = 4..512 */

#define RCC_PLL2DIVR_PLL2P_SHIFT            (9) /* Bits 15-9:  */
#define RCC_PLL2DIVR_PLL2P_MASK             (0x7f << RCC_PLL2DIVR_PLL2P_SHIFT)
#  define RCC_PLL2DIVR_PLL2P(n)             ((n-1) << RCC_PLL2DIVR_PLL2P_SHIFT) /* m = 2..128 evens */

#define RCC_PLL2DIVR_PLL2Q_SHIFT            (16) /* Bits 22-16:  */
#define RCC_PLL2DIVR_PLL2Q_MASK             (0x7f << RCC_PLL2DIVR_PLL2Q_SHIFT)
#  define RCC_PLL2DIVR_PLL2Q(n)             ((n-1) << RCC_PLL2DIVR_PLL2Q_SHIFT) /* m = 1..128 */

#define RCC_PLL2DIVR_PLL2R_SHIFT            (24) /* Bits 30-24:  */
#define RCC_PLL2DIVR_PLL2R_MASK             (0x7f << RCC_PLL2DIVR_PLL2R_SHIFT)
#  define RCC_PLL2DIVR_PLL2R(n)             ((n-1) << RCC_PLL2DIVR_PLL2R_SHIFT) /* m = 1..128 */

/* PLL2 fractional divider register */

#define RCC_PLL2FRACR_PLL2FRACN_SHIFT       (3)
#define RCC_PLL2FRACR_PLL2FRACN_MASK        (0x1fff << RCC_PLL2FRACR_PLL2FRACN_SHIFT)
#define RCC_PLL2FRACR_PLL2FRACN(n)          (n << RCC_PLL2FRACR_PLL2FRACN_SHIFT) /* m = 0..8192 */

/* PLL3 dividers register */

#define RCC_PLL3DIVR_PLL3N_SHIFT            (0) /* Bits 8-0:  */
#define RCC_PLL3DIVR_PLL3N_MASK             (0x1ff << RCC_PLL3DIVR_PLL3N_SHIFT)
#  define RCC_PLL3DIVR_PLL3N(n)             ((n-1) << RCC_PLL3DIVR_PLL3N_SHIFT) /* m = 4..512 */

#define RCC_PLL3DIVR_PLL3P_SHIFT            (9) /* Bits 15-9:  */
#define RCC_PLL3DIVR_PLL3P_MASK             (0x7f << RCC_PLL3DIVR_PLL3P_SHIFT)
#  define RCC_PLL3DIVR_PLL3P(n)             ((n-1) << RCC_PLL3DIVR_PLL3P_SHIFT) /* m = 2..128 evens */

#define RCC_PLL3DIVR_PLL3Q_SHIFT            (16) /* Bits 22-16:  */
#define RCC_PLL3DIVR_PLL3Q_MASK             (0x7f << RCC_PLL3DIVR_PLL3Q_SHIFT)
#  define RCC_PLL3DIVR_PLL3Q(n)             ((n-1) << RCC_PLL3DIVR_PLL3Q_SHIFT) /* m = 1..128 */

#define RCC_PLL3DIVR_PLL3R_SHIFT            (24) /* Bits 30-24:  */
#define RCC_PLL3DIVR_PLL3R_MASK             (0x7f << RCC_PLL3DIVR_PLL3R_SHIFT)
#  define RCC_PLL3DIVR_PLL3R(n)             ((n-1) << RCC_PLL3DIVR_PLL3R_SHIFT) /* m = 1..128 */

/* PLL3 fractional divider register */

#define RCC_PLL3FRACR_PLL3FRACN_SHIFT       (3)
#define RCC_PLL3FRACR_PLL3FRACN_MASK        (0x1fff << RCC_PLL3FRACR_PLL3FRACN_SHIFT)
#define RCC_PLL3FRACR_PLL3FRACN(n)          (n << RCC_PLL3FRACR_PLL3FRACN_SHIFT) /* m = 0..8192 */

/* Clock interrupt enable register */

#define RCC_CIER_LSIRDYIE                (1 << 0)  /* Bit 0: LSI Ready Interrupt Enable */
#define RCC_CIER_LSERDYIE                (1 << 1)  /* Bit 1: LSE Ready Interrupt Enable */
#define RCC_CIER_CSIRDYIE                (1 << 2)  /* Bit 2: CSI Ready Interrupt Enable */
#define RCC_CIER_HSIRDYIE                (1 << 3)  /* Bit 3: HSI Ready Interrupt Enable */
#define RCC_CIER_HSERDYIE                (1 << 4)  /* Bit 4: HSE Ready Interrupt Enable */
#define RCC_CIER_HSI48RDYIE              (1 << 5)  /* Bit 5: HSI48 Ready Interrupt Enable */
#define RCC_CIER_PLL1RDYIE               (1 << 6)  /* Bit 6: PLL1 Ready Interrupt Enable */
#define RCC_CIER_PLL2RDYIE               (1 << 7)  /* Bit 7: PLL2 Ready Interrupt Enable */
#define RCC_CIER_PLL3RDYIE               (1 << 8)  /* Bit 8: PLL3 Ready Interrupt Enable */

/* Clock interrupt flag register */

#define RCC_CIFR_LSIRDYF                (1 << 0)  /* Bit 0: LSI Ready Interrupt Flag */
#define RCC_CIFR_LSERDYF                (1 << 1)  /* Bit 1: LSE Ready Interrupt Flag */
#define RCC_CIFR_CSIRDYF                (1 << 2)  /* Bit 2: CSI Ready Interrupt Flag */
#define RCC_CIFR_HSIRDYF                (1 << 3)  /* Bit 3: HSI Ready Interrupt Flag */
#define RCC_CIFR_HSERDYF                (1 << 4)  /* Bit 4: HSE Ready Interrupt Flag */
#define RCC_CIFR_HSI48RDYF              (1 << 5)  /* Bit 5: HSI48 Ready Interrupt Flag */
#define RCC_CIFR_PLL1RDYF               (1 << 6)  /* Bit 6: PLL1 Ready Interrupt Flag */
#define RCC_CIFR_PLL2RDYF               (1 << 7)  /* Bit 7: PLL2 Ready Interrupt Flag */
#define RCC_CIFR_PLL3RDYF               (1 << 8)  /* Bit 8: PLL3 Ready Interrupt Flag */
#define RCC_CIFR_HSECSSF                (1 << 10) /* Bit 10: HSE Clock Security System Interrupt Flag */

/* Clock interrupt clear register */

#define RCC_CICR_LSIRDYC                (1 << 0)  /* Bit 0: LSI Ready Interrupt Flag */
#define RCC_CICR_LSERDYC                (1 << 1)  /* Bit 1: LSE Ready Interrupt Flag */
#define RCC_CICR_CSIRDYC                (1 << 2)  /* Bit 2: CSI Ready Interrupt Flag */
#define RCC_CICR_HSIRDYC                (1 << 3)  /* Bit 3: HSI Ready Interrupt Flag */
#define RCC_CICR_HSERDYC                (1 << 4)  /* Bit 4: HSE Ready Interrupt Flag */
#define RCC_CICR_HSI48RDYC              (1 << 5)  /* Bit 5: HSI48 Ready Interrupt Flag */
#define RCC_CICR_PLL1RDYC               (1 << 6)  /* Bit 6: PLL1 Ready Interrupt Flag */
#define RCC_CICR_PLL2RDYC               (1 << 7)  /* Bit 7: PLL2 Ready Interrupt Flag */
#define RCC_CICR_PLL3RDYC               (1 << 8)  /* Bit 8: PLL3 Ready Interrupt Flag */
#define RCC_CICR_HSECSSC                (1 << 10) /* Bit 10: HSE Clock Security System Interrupt Flag */

/* AHB1 peripheral reset register */

#define RCC_AHB1RSTR_GPDMA1RST             (1 << 0)  /* Bit 0:  GPDMA1 reset */
#define RCC_AHB1RSTR_GPDMA2RST             (1 << 1)  /* Bit 1:  GPDMA2 reset */
#define RCC_AHB1RSTR_CRCRST                (1 << 12) /* Bit 12:  CRC reset */
#define RCC_AHB1RSTR_CORDICRST             (1 << 14) /* Bit 14:  CORDIC reset */
#define RCC_AHB1RSTR_FMACRST               (1 << 15) /* Bit 15:  FMAC reset */
#define RCC_AHB1RSTR_RAMCFGRST             (1 << 17) /* Bit 17:  RAMCFG reset */
#define RCC_AHB1RSTR_ETHRST                (1 << 19) /* Bit 19:  ETH reset */

/* AHB2 peripheral reset register */

#define RCC_AHB2RSTR_GPIORST(n)          (1 << (n))
#define RCC_AHB2RSTR_GPIOARST            (1 << 0)  /* Bit 0:  IO port A reset */
#define RCC_AHB2RSTR_GPIOBRST            (1 << 1)  /* Bit 1:  IO port B reset */
#define RCC_AHB2RSTR_GPIOCRST            (1 << 2)  /* Bit 2:  IO port C reset */
#define RCC_AHB2RSTR_GPIODRST            (1 << 3)  /* Bit 3:  IO port D reset */
#define RCC_AHB2RSTR_GPIOERST            (1 << 4)  /* Bit 4:  IO port E reset */
#define RCC_AHB2RSTR_GPIOFRST            (1 << 5)  /* Bit 5:  IO port F reset */
#define RCC_AHB2RSTR_GPIOGRST            (1 << 6)  /* Bit 6:  IO port G reset */
#define RCC_AHB2RSTR_GPIOHRST            (1 << 7)  /* Bit 7:  IO port H reset */
#define RCC_AHB2RSTR_GPIOIRST            (1 << 8)  /* Bit 8:  IO port I reset */
#define RCC_AHB2RSTR_ADCRST              (1 << 10) /* Bit 10: ADC interface reset (common to all ADCs) */
#define RCC_AHB2RSTR_DACRST              (1 << 11) /* Bit 11: DAC Block reset */
#define RCC_AHB2RSTR_DCMI_PSSIRST        (1 << 12) /* Bit 12: Digital Camera Interface block reset */
#define RCC_AHB2RSTR_AESRST              (1 << 16) /* Bit 16: AES Cryptographic module reset */
#define RCC_AHB2RSTR_HASHRST             (1 << 17) /* Bit 17: HASH block reset */
#define RCC_AHB2RSTR_RNGRST              (1 << 18) /* Bit 18: Random number generator module reset */
#define RCC_AHB2RSTR_PKARST              (1 << 19) /* Bit 19: Public Key Accelerator module reset */
#define RCC_AHB2RSTR_SAESRST             (1 << 20) /* Bit 20: SAES block reset */

/* AHB4 peripheral reset register */

#define RCC_AHB4RSTR_OTFDEC1RST          (1 << 7)  /* Bit 0:  OTFDEC1 block reset */
#define RCC_AHB4RSTR_SDMMC1RST           (1 << 11) /* Bit 11: SDMMC1RST blocks reset  */
#define RCC_AHB4RSTR_SDMMC2RST           (1 << 12) /* Bit 12: SDMMC2RST blocks reset  */
#define RCC_AHB4RSTR_FMCRST              (1 << 16) /* Bit 16: FMC block reset */
#define RCC_AHB4RSTR_OSPI1RST            (1 << 20) /* Bit 20: OCTOSPI1 block reset */

/* APB1 Peripheral reset register 1 */

#define RCC_APB1LRSTR_TIM2RST            (1 << 0)  /* Bit 0:  TIM2 reset */
#define RCC_APB1LRSTR_TIM3RST            (1 << 1)  /* Bit 1:  TIM3 reset */
#define RCC_APB1LRSTR_TIM4RST            (1 << 2)  /* Bit 2:  TIM4 reset */
#define RCC_APB1LRSTR_TIM5RST            (1 << 3)  /* Bit 3:  TIM5 reset */
#define RCC_APB1LRSTR_TIM6RST            (1 << 4)  /* Bit 4:  TIM6 reset */
#define RCC_APB1LRSTR_TIM7RST            (1 << 5)  /* Bit 5:  TIM7 reset */
#define RCC_APB1LRSTR_TIM12RST           (1 << 6)  /* Bit 5:  TIM12 reset */
#define RCC_APB1LRSTR_TIM13RST           (1 << 7)  /* Bit 5:  TIM13 reset */
#define RCC_APB1LRSTR_TIM14RST           (1 << 8)  /* Bit 5:  TIM14 reset */
#define RCC_APB1LRSTR_SPI2RST            (1 << 14) /* Bit 14: SPI2 reset */
#define RCC_APB1LRSTR_SPI3RST            (1 << 15) /* Bit 15: SPI3 reset */
#define RCC_APB1LRSTR_USART2RST          (1 << 17) /* Bit 17: USART2 reset */
#define RCC_APB1LRSTR_USART3RST          (1 << 18) /* Bit 18: USART3 reset */
#define RCC_APB1LRSTR_UART4RST           (1 << 19) /* Bit 19: UART4 reset */
#define RCC_APB1LRSTR_UART5RST           (1 << 20) /* Bit 20: UART5 reset */
#define RCC_APB1LRSTR_I2C1RST            (1 << 21) /* Bit 21: I2C1 reset */
#define RCC_APB1LRSTR_I2C2RST            (1 << 22) /* Bit 22: I2C2 reset */
#define RCC_APB1LRSTR_I3C1RST            (1 << 23) /* Bit 23: I3C1 reset */
#define RCC_APB1LRSTR_CRSRST             (1 << 24) /* Bit 24: CRS reset */
#define RCC_APB1LRSTR_USART6RST          (1 << 25) /* Bit 25: USART6 reset */
#define RCC_APB1LRSTR_USART10RST         (1 << 26) /* Bit 26: USART10 reset */
#define RCC_APB1LRSTR_USART11RST         (1 << 27) /* Bit 27: USART11 reset */
#define RCC_APB1LRSTR_CECRST             (1 << 28) /* Bit 28: CEC reset */
#define RCC_APB1LRSTR_UART7RST           (1 << 30) /* Bit 30: UART7 reset */
#define RCC_APB1LRSTR_UART8RST           (1 << 31) /* Bit 31: UART8 reset */

/* APB1 Peripheral reset register 2 */

#define RCC_APB1HRSTR_UART9RST           (1 << 0)  /* Bit 0:  UART9 reset */
#define RCC_APB1HRSTR_UART12RST          (1 << 1)  /* Bit 1:  UART12 reset */
#define RCC_APB1HRSTR_DTSRST             (1 << 3)  /* Bit 3:  DTS  reset */
#define RCC_APB1HRSTR_LPTIM2RST          (1 << 5)  /* Bit 5:  Low-power Timer 2 reset */
#define RCC_APB1HRSTR_FDCANRST           (1 << 9)  /* Bit 9:  FDCAN reset */
#define RCC_APB1HRSTR_UCPD1RST           (1 << 23) /* Bit 23: UCPD1 reset */

/* APB2 Peripheral reset register */

#define RCC_APB2RSTR_TIM1RST             (1 << 11) /* Bit 11: TIM1 reset */
#define RCC_APB2RSTR_SPI1RST             (1 << 12) /* Bit 12: SPI1 reset */
#define RCC_APB2RSTR_TIM8RST             (1 << 13) /* Bit 13: TIM8 reset */
#define RCC_APB2RSTR_USART1RST           (1 << 14) /* Bit 14: USART1 reset */
#define RCC_APB2RSTR_TIM15RST            (1 << 16) /* Bit 16: TIM15 reset */
#define RCC_APB2RSTR_TIM16RST            (1 << 17) /* Bit 17: TIM16 reset */
#define RCC_APB2RSTR_TIM17RST            (1 << 18) /* Bit 18: TIM17 reset */
#define RCC_APB2RSTR_SPI4RST             (1 << 19) /* Bit 19: SPI4 reset */
#define RCC_APB2RSTR_SPI67RST            (1 << 20) /* Bit 20: SPI6 reset */
#define RCC_APB2RSTR_SAI1RST             (1 << 21) /* Bit 21: SAI1 reset */
#define RCC_APB2RSTR_SAI2RST             (1 << 22) /* Bit 22: SAI2 reset */
#define RCC_APB2RSTR_USBRST              (1 << 24) /* Bit 24: USB reset */

/* APB3 Peripheral reset register */

#define RCC_APB3RSTR_SPI5RST             (1 << 5)  /* Bit 5: SPI5 reset */
#define RCC_APB3RSTR_LPUART1RST          (1 << 6)  /* Bit 6: LPUART1 reset */
#define RCC_APB3RSTR_I2C3RST             (1 << 7)  /* Bit 7: I2C3 reset */
#define RCC_APB3RSTR_I2C4RST             (1 << 8)  /* Bit 8: I2C4 reset */
#define RCC_APB3RSTR_I3C2RST             (1 << 9)  /* Bit 9: I3C2 reset */
#define RCC_APB3RSTR_LPTIM1RST           (1 << 11) /* Bit 11: LPTIM1 reset */
#define RCC_APB3RSTR_LPTIM3RST           (1 << 12) /* Bit 12: LPTIM3 reset */
#define RCC_APB3RSTR_LPTIM4RST           (1 << 13) /* Bit 13: LPTIM4 reset */
#define RCC_APB3RSTR_LPTIM5RST           (1 << 14) /* Bit 14: LPTIM5 reset */
#define RCC_APB3RSTR_LPTIM6RST           (1 << 15) /* Bit 15: LPTIM6 reset */
#define RCC_APB3RSTR_VREFRST             (1 << 20) /* Bit 20: VREF reset */

/* AHB1 Peripheral Clock enable register */

#define RCC_AHB1ENR_GPDMA1EN             (1 << 0)  /* Bit 0:  GPDMA1 clock enable */
#define RCC_AHB1ENR_GPDMA2EN             (1 << 1)  /* Bit 1:  GPDMA2 clock enable */
#define RCC_AHB1ENR_FLASHEN              (1 << 8)  /* Bit 8:  Flash Interace clock enable */
#define RCC_AHB1ENR_CRCEN                (1 << 12) /* Bit 12:  CRC clock enable */
#define RCC_AHB1ENR_CORDICEN             (1 << 14) /* Bit 14:  CORDIC clock enable */
#define RCC_AHB1ENR_FMACEN               (1 << 15) /* Bit 15:  FMAC clock enable */
#define RCC_AHB1ENR_RAMCFGEN             (1 << 17) /* Bit 17:  RAMCFG clock enable */
#define RCC_AHB1ENR_ETHEN                (1 << 19) /* Bit 19:  ETH clock enable */
#define RCC_AHB1ENR_ETHTXEN              (1 << 20) /* Bit 20:  ETH TX clock enable */
#define RCC_AHB1ENR_ETHRXEN              (1 << 21) /* Bit 21:  ETH RX clock enable */
#define RCC_AHB1ENR_TZSC1EN              (1 << 24) /* Bit 24:  TZSC1 clock enable */
#define RCC_AHB1ENR_BKPRAMEN             (1 << 28) /* Bit 28:  BKPRAM clock enable */
#define RCC_AHB1ENR_DCACHEEN             (1 << 30) /* Bit 25:  DCACHE clock enable */
#define RCC_AHB1ENR_SRAM1EN              (1 << 31) /* Bit 25:  SRAM1 clock enable */

/* AHB2 Peripheral Clock enable register */

#define RCC_AHB2ENR_GPIOEN(n)            (1 << (n))
#define RCC_AHB2ENR_GPIOAEN              (1 << 0)  /* Bit 0:  IO port A enable */
#define RCC_AHB2ENR_GPIOBEN              (1 << 1)  /* Bit 1:  IO port B enable */
#define RCC_AHB2ENR_GPIOCEN              (1 << 2)  /* Bit 2:  IO port C enable */
#define RCC_AHB2ENR_GPIODEN              (1 << 3)  /* Bit 3:  IO port D enable */
#define RCC_AHB2ENR_GPIOEEN              (1 << 4)  /* Bit 4:  IO port E enable */
#define RCC_AHB2ENR_GPIOFEN              (1 << 5)  /* Bit 5:  IO port F enable */
#define RCC_AHB2ENR_GPIOGEN              (1 << 6)  /* Bit 6:  IO port G enable */
#define RCC_AHB2ENR_GPIOHEN              (1 << 7)  /* Bit 7:  IO port H enable */
#define RCC_AHB2ENR_GPIOIEN              (1 << 8)  /* Bit 8:  IO port I enable */
#define RCC_AHB2ENR_ADCEN                (1 << 10) /* Bit 10: ADC interface enable (common to all ADCs) */
#define RCC_AHB2ENR_DAC1EN               (1 << 11) /* Bit 11: DAC clock enable */
#define RCC_AHB2ENR_DCMI_PSSIEN          (1 << 12) /* Bit 12: Digital Camera Interface Clock Enable */
#define RCC_AHB2ENR_AESEN                (1 << 16) /* Bit 16: AES Cryptographic module enable */
#define RCC_AHB2ENR_HASHEN               (1 << 17) /* Bit 17: HASH module enable */
#define RCC_AHB2ENR_RNGEN                (1 << 18) /* Bit 18: Random number generator module enable */
#define RCC_AHB2ENR_PKAEN                (1 << 19) /* Bit 19: PKA clock enable */
#define RCC_AHB2ENR_SAESEN               (1 << 20) /* Bit 20: SAES clock enable */
#define RCC_AHB2ENR_SRAM2EN              (1 << 30) /* Bit 30: SRAM2 clock enable */
#define RCC_AHB2ENR_SRAM3EN              (1 << 31) /* Bit 30: SRAM2 clock enable */

/* AHB4 Peripheral Clock enable register */

#define RCC_AHB4ENR_OTFDEC1EN            (1 << 7)   /* Bit  7: On-the-fly decryption module clock enable */
#define RCC_AHB4ENR_SDMMC1EN             (1 << 11)  /* Bit 11: SDMMC1 clock enable */
#define RCC_AHB4ENR_SDMMC2EN             (1 << 12)  /* Bit 12: SDMMC2 clock enable */
#define RCC_AHB4ENR_FMCEN                (1 << 16)  /* Bit 16: Flexible memory controller module enable */
#define RCC_AHB4ENR_OSPI1EN              (1 << 20)  /* Bit 20: OCTOSPI1 module enable */

/* APB1 Peripheral clock enable register 1 */

#define RCC_APB1LENR_TIM2EN            (1 << 0)  /* Bit 0:  TIM2 clock enable */
#define RCC_APB1LENR_TIM3EN            (1 << 1)  /* Bit 1:  TIM3 clock enable */
#define RCC_APB1LENR_TIM4EN            (1 << 2)  /* Bit 2:  TIM4 clock enable */
#define RCC_APB1LENR_TIM5EN            (1 << 3)  /* Bit 3:  TIM5 clock enable */
#define RCC_APB1LENR_TIM6EN            (1 << 4)  /* Bit 4:  TIM6 clock enable */
#define RCC_APB1LENR_TIM7EN            (1 << 5)  /* Bit 5:  TIM7 clock enable */
#define RCC_APB1LENR_TIM12EN           (1 << 6)  /* Bit 5:  TIM12 clock enable */
#define RCC_APB1LENR_TIM13EN           (1 << 7)  /* Bit 5:  TIM13 clock enable */
#define RCC_APB1LENR_TIM14EN           (1 << 8)  /* Bit 5:  TIM14 clock enable */
#define RCC_APB1LENR_SPI2EN            (1 << 14) /* Bit 14: SPI2 clock enable */
#define RCC_APB1LENR_SPI3EN            (1 << 15) /* Bit 15: SPI3 clock enable */
#define RCC_APB1LENR_USART2EN          (1 << 17) /* Bit 17: USART2 clock enable */
#define RCC_APB1LENR_USART3EN          (1 << 18) /* Bit 18: USART3 clock enable */
#define RCC_APB1LENR_UART4EN           (1 << 19) /* Bit 19: UART4 clock enable */
#define RCC_APB1LENR_UART5EN           (1 << 20) /* Bit 20: UART5 clock enable */
#define RCC_APB1LENR_I2C1EN            (1 << 21) /* Bit 21: I2C1 clock enable */
#define RCC_APB1LENR_I2C2EN            (1 << 22) /* Bit 22: I2C2 clock enable */
#define RCC_APB1LENR_I3C1EN            (1 << 23) /* Bit 23: I3C1 clock enable */
#define RCC_APB1LENR_CRSEN             (1 << 24) /* Bit 24: CRS clock enable */
#define RCC_APB1LENR_USART6EN          (1 << 25) /* Bit 25: USART6 clock enable */
#define RCC_APB1LENR_USART10EN         (1 << 26) /* Bit 26: USART10 clock enable */
#define RCC_APB1LENR_USART11EN         (1 << 27) /* Bit 27: USART11 clock enable */
#define RCC_APB1LENR_CECEN             (1 << 28) /* Bit 28: CEC clock enable */
#define RCC_APB1LENR_UART7EN           (1 << 30) /* Bit 30: UART7 clock enable */
#define RCC_APB1LENR_UART8EN           (1 << 31) /* Bit 31: UART8 clock enable */

/* APB1 Peripheral clock enable register 2 */

#define RCC_APB1HENR_UART9EN           (1 << 0)  /* Bit 0:  UART9 clock enable */
#define RCC_APB1HENR_UART12EN          (1 << 1)  /* Bit 1:  UART12 clock enable */
#define RCC_APB1HENR_DTSEN             (1 << 3)  /* Bit 3:  DTS  clock enable */
#define RCC_APB1HENR_LPTIM2EN          (1 << 5)  /* Bit 5:  Low-power Timer 2 clock enable */
#define RCC_APB1HENR_FDCANEN           (1 << 9)  /* Bit 9:  FDCAN clock enable */
#define RCC_APB1HENR_UCPD1EN           (1 << 23) /* Bit 23: UCPD1 clock enable */

/* APB2 Peripheral clock enable register */

#define RCC_APB2ENR_TIM1EN             (1 << 11) /* Bit 11: TIM1 clock enable */
#define RCC_APB2ENR_SPI1EN             (1 << 12) /* Bit 12: SPI1 clock enable */
#define RCC_APB2ENR_TIM8EN             (1 << 13) /* Bit 13: TIM8 clock enable */
#define RCC_APB2ENR_USART1EN           (1 << 14) /* Bit 14: USART1 clock enable */
#define RCC_APB2ENR_TIM15EN            (1 << 16) /* Bit 16: TIM15 clock enable */
#define RCC_APB2ENR_TIM16EN            (1 << 17) /* Bit 17: TIM16 clock enable */
#define RCC_APB2ENR_TIM17EN            (1 << 18) /* Bit 18: TIM17 clock enable */
#define RCC_APB2ENR_SPI4EN             (1 << 19) /* Bit 19: SPI4 clock enable */
#define RCC_APB2ENR_SPI67EN            (1 << 20) /* Bit 20: SPI6 clock enable */
#define RCC_APB2ENR_SAI1EN             (1 << 21) /* Bit 21: SAI1 clock enable */
#define RCC_APB2ENR_SAI2EN             (1 << 22) /* Bit 22: SAI2 clock enable */
#define RCC_APB2ENR_USBEN              (1 << 24) /* Bit 24: USB clock enable */

/* APB3 Peripheral clock enable register */

#define RCC_APB3ENR_SBSEN              (1 << 1)  /* Bit 1: SBS clock enable */
#define RCC_APB3ENR_SPI5EN             (1 << 5)  /* Bit 5: SPI5 clock enable */
#define RCC_APB3ENR_LPUART1EN          (1 << 6)  /* Bit 6: LPUART1 clock enable */
#define RCC_APB3ENR_I2C3EN             (1 << 7)  /* Bit 7: I2C3 clock enable */
#define RCC_APB3ENR_I2C4EN             (1 << 8)  /* Bit 8: I2C4 clock enable */
#define RCC_APB3ENR_I3C2EN             (1 << 9)  /* Bit 9: I3C2 clock enable */
#define RCC_APB3ENR_LPTIM1EN           (1 << 11) /* Bit 11: LPTIM1 clock enable */
#define RCC_APB3ENR_LPTIM3EN           (1 << 12) /* Bit 12: LPTIM3 clock enable */
#define RCC_APB3ENR_LPTIM4EN           (1 << 13) /* Bit 13: LPTIM4 clock enable */
#define RCC_APB3ENR_LPTIM5EN           (1 << 14) /* Bit 14: LPTIM5 clock enable */
#define RCC_APB3ENR_LPTIM6EN           (1 << 15) /* Bit 15: LPTIM6 clock enable */
#define RCC_APB3ENR_VREFEN             (1 << 20) /* Bit 20: VREF clock enable */
#define RCC_APB3ENR_RTCAPBEN           (1 << 21) /* Bit 21: RTCABP clock enable */

/* RCC AHB1 Sleep and Stop modes peripheral clock enable register */

#define RCC_AHB1LPENR_GPDMA1LPEN             (1 << 0)  /* Bit 0:  GPDMA1 clock enable during sleep mode */
#define RCC_AHB1LPENR_GPDMA2LPEN             (1 << 1)  /* Bit 1:  GPDMA2 clock enable during sleep mode */
#define RCC_AHB1LPENR_FLITFLPEN              (1 << 8)  /* Bit 8:  Flash Interace clock enable during sleep mode */
#define RCC_AHB1LPENR_CRCLPEN                (1 << 12) /* Bit 12:  CRC clock enable during sleep mode */
#define RCC_AHB1LPENR_CORDICLPEN             (1 << 14) /* Bit 14:  CORDIC clock enable during sleep mode */
#define RCC_AHB1LPENR_FMACLPEN               (1 << 15) /* Bit 15:  FMAC clock enable during sleep mode */
#define RCC_AHB1LPENR_RAMCFGLPEN             (1 << 17) /* Bit 17:  RAMCFG clock enable during sleep mode */
#define RCC_AHB1LPENR_ETHLPEN                (1 << 19) /* Bit 19:  ETH clock enable during sleep mode */
#define RCC_AHB1LPENR_ETHTXLPEN              (1 << 20) /* Bit 20:  ETH TX clock enable during sleep mode */
#define RCC_AHB1LPENR_ETHRXLPEN              (1 << 21) /* Bit 21:  ETH RX clock enable during sleep mode */
#define RCC_AHB1LPENR_TZSC1LPEN              (1 << 24) /* Bit 24:  TZSC1 clock enable during sleep mode */
#define RCC_AHB1LPENR_BKPRAMLPEN             (1 << 28) /* Bit 28:  BKPRAM clock enable during sleep mode */
#define RCC_AHB1LPENR_DCACHELPEN             (1 << 30) /* Bit 25:  DCACHE clock enable during sleep mode */
#define RCC_AHB1LPENR_SRAM1LPEN              (1 << 31) /* Bit 25:  SRAM1 clock enable during sleep mode */

/* RCC AHB2 low power mode peripheral clock enable register */

#define RCC_AHB2LPENR_GPIOALPEN              (1 << 0)  /* Bit 0:  IO port A clock enable during sleep mode */
#define RCC_AHB2LPENR_GPIOBLPEN              (1 << 1)  /* Bit 1:  IO port B clock enable during sleep mode */
#define RCC_AHB2LPENR_GPIOCLPEN              (1 << 2)  /* Bit 2:  IO port C clock enable during sleep mode */
#define RCC_AHB2LPENR_GPIODLPEN              (1 << 3)  /* Bit 3:  IO port D clock enable during sleep mode */
#define RCC_AHB2LPENR_GPIOELPEN              (1 << 4)  /* Bit 4:  IO port E clock enable during sleep mode */
#define RCC_AHB2LPENR_GPIOFLPEN              (1 << 5)  /* Bit 5:  IO port F clock enable during sleep mode */
#define RCC_AHB2LPENR_GPIOGLPEN              (1 << 6)  /* Bit 6:  IO port G clock enable during sleep mode */
#define RCC_AHB2LPENR_GPIOHLPEN              (1 << 7)  /* Bit 7:  IO port H clock enable during sleep mode */
#define RCC_AHB2LPENR_GPIOILPEN              (1 << 8)  /* Bit 8:  IO port I clock enable during sleep mode */
#define RCC_AHB2LPENR_ADCLPEN                (1 << 10) /* Bit 10: ADC interface enable (common to all ADCs) */
#define RCC_AHB2LPENR_DAC1LPEN               (1 << 11) /* Bit 11: DAC clock enable during sleep mode */
#define RCC_AHB2LPENR_DCMI_PSSILPEN          (1 << 12) /* Bit 12: Digital Camera Interface Clock Enable */
#define RCC_AHB2LPENR_AESLPEN                (1 << 16) /* Bit 16: AES Cryptographic module enable */
#define RCC_AHB2LPENR_HASHLPEN               (1 << 17) /* Bit 17: HASH module enable */
#define RCC_AHB2LPENR_RNGLPEN                (1 << 18) /* Bit 18: Random number generator module enable */
#define RCC_AHB2LPENR_PKALPEN                (1 << 19) /* Bit 19: PKA clock enable during sleep mode */
#define RCC_AHB2LPENR_SAESLPEN               (1 << 20) /* Bit 20: SAES clock enable during sleep mode */
#define RCC_AHB2LPENR_SRAM2LPEN              (1 << 30) /* Bit 30: SRAM2 clock enable during sleep mode */
#define RCC_AHB2LPENR_SRAM3LPEN              (1 << 31) /* Bit 30: SRAM2 clock enable during sleep mode */

/* RCC AHB4 low power mode peripheral clock enable register */

#define RCC_AHB4LPENR_OTFDEC1LPEN            (1 << 7)   /* Bit  7: OTFDEC1 clock enable during sleep mode */
#define RCC_AHB4LPENR_SDMMC1LPEN             (1 << 11)  /* Bit 11: SDMMC1 clock enable during sleep mode */
#define RCC_AHB4LPENR_SDMMC2LPEN             (1 << 12)  /* Bit 12: SDMMC2 clock enable during sleep mode */
#define RCC_AHB4LPENR_FMCLPEN                (1 << 16)  /* Bit 16: Flexible memory controller module enable */
#define RCC_AHB4LPENR_OSPI1LPEN              (1 << 20)  /* Bit 20: OCTOSPI1 module clock enable during sleep mode */

/* APB1 Peripheral clock enable register 1 */

#define RCC_APB1LLPENR_TIM2LPEN            (1 << 0)  /* Bit 0:  TIM2 clock enable during sleep mode */
#define RCC_APB1LLPENR_TIM3LPEN            (1 << 1)  /* Bit 1:  TIM3 clock enable during sleep mode */
#define RCC_APB1LLPENR_TIM4LPEN            (1 << 2)  /* Bit 2:  TIM4 clock enable during sleep mode */
#define RCC_APB1LLPENR_TIM5LPEN            (1 << 3)  /* Bit 3:  TIM5 clock enable during sleep mode */
#define RCC_APB1LLPENR_TIM6LPEN            (1 << 4)  /* Bit 4:  TIM6 clock enable during sleep mode */
#define RCC_APB1LLPENR_TIM7LPEN            (1 << 5)  /* Bit 5:  TIM7 clock enable during sleep mode */
#define RCC_APB1LLPENR_TIM12LPEN           (1 << 6)  /* Bit 6:  TIM12 clock enable during sleep mode */
#define RCC_APB1LLPENR_TIM13LPEN           (1 << 7)  /* Bit 7:  TIM13 clock enable during sleep mode */
#define RCC_APB1LLPENR_TIM14LPEN           (1 << 8)  /* Bit 8:  TIM14 clock enable during sleep mode */
#define RCC_APB1LLPENR_WWDGLPEN            (1 << 11) /* Bit 11: WWDG clock enable during sleep mode */
#define RCC_APB1LLPENR_SPI2LPEN            (1 << 14) /* Bit 14: SPI2 clock enable during sleep mode */
#define RCC_APB1LLPENR_SPI3LPEN            (1 << 15) /* Bit 15: SPI3 clock enable during sleep mode */
#define RCC_APB1LLPENR_USART2LPEN          (1 << 17) /* Bit 17: USART2 clock enable during sleep mode */
#define RCC_APB1LLPENR_USART3LPEN          (1 << 18) /* Bit 18: USART3 clock enable during sleep mode */
#define RCC_APB1LLPENR_UART4LPEN           (1 << 19) /* Bit 19: UART4 clock enable during sleep mode */
#define RCC_APB1LLPENR_UART5LPEN           (1 << 20) /* Bit 20: UART5 clock enable during sleep mode */
#define RCC_APB1LLPENR_I2C1LPEN            (1 << 21) /* Bit 21: I2C1 clock enable during sleep mode */
#define RCC_APB1LLPENR_I2C2LPEN            (1 << 22) /* Bit 22: I2C2 clock enable during sleep mode */
#define RCC_APB1LLPENR_I3C1LPEN            (1 << 23) /* Bit 23: I3C1 clock enable during sleep mode */
#define RCC_APB1LLPENR_CRSLPEN             (1 << 24) /* Bit 24: CRS clock enable during sleep mode */
#define RCC_APB1LLPENR_USART6LPEN          (1 << 25) /* Bit 25: USART6 clock enable during sleep mode */
#define RCC_APB1LLPENR_USART10LPEN         (1 << 26) /* Bit 26: USART10 clock enable during sleep mode */
#define RCC_APB1LLPENR_USART11LPEN         (1 << 27) /* Bit 27: USART11 clock enable during sleep mode */
#define RCC_APB1LLPENR_CECLPEN             (1 << 28) /* Bit 28: CEC clock enable during sleep mode */
#define RCC_APB1LLPENR_UART7LPEN           (1 << 30) /* Bit 30: UART7 clock enable during sleep mode */
#define RCC_APB1LLPENR_UART8LPEN           (1 << 31) /* Bit 31: UART8 clock enable during sleep mode */

/* APB1 Peripheral clock enable register 2 */

#define RCC_APB1HLPENR_UART9LPEN           (1 << 0)  /* Bit 0:  UART9 clock enable during sleep mode */
#define RCC_APB1HLPENR_UART12LPEN          (1 << 1)  /* Bit 1:  UART12 clock enable during sleep mode */
#define RCC_APB1HLPENR_DTSLPEN             (1 << 3)  /* Bit 3:  DTS  clock enable during sleep mode */
#define RCC_APB1HLPENR_LPTIM2LPEN          (1 << 5)  /* Bit 5:  Low-power Timer 2 clock enable during sleep mode */
#define RCC_APB1HLPENR_FDCANLPEN           (1 << 9)  /* Bit 9:  FDCAN clock enable during sleep mode */
#define RCC_APB1HLPENR_UCPD1LPEN           (1 << 23) /* Bit 23: UCPD1 clock enable during sleep mode */

/* APB2 Peripheral clock enable register */

#define RCC_APB2LPENR_TIM1LPEN             (1 << 11) /* Bit 11: TIM1 clock enable during sleep mode */
#define RCC_APB2LPENR_SPI1LPEN             (1 << 12) /* Bit 12: SPI1 clock enable during sleep mode */
#define RCC_APB2LPENR_TIM8LPEN             (1 << 13) /* Bit 13: TIM8 clock enable during sleep mode */
#define RCC_APB2LPENR_USART1LPEN           (1 << 14) /* Bit 14: USART1 clock enable during sleep mode */
#define RCC_APB2LPENR_TIM15LPEN            (1 << 16) /* Bit 16: TIM15 clock enable during sleep mode */
#define RCC_APB2LPENR_TIM16LPEN            (1 << 17) /* Bit 17: TIM16 clock enable during sleep mode */
#define RCC_APB2LPENR_TIM17LPEN            (1 << 18) /* Bit 18: TIM17 clock enable during sleep mode */
#define RCC_APB2LPENR_SPI4LPEN             (1 << 19) /* Bit 19: SPI4 clock enable during sleep mode */
#define RCC_APB2LPENR_SPI67LPEN            (1 << 20) /* Bit 20: SPI6 clock enable during sleep mode */
#define RCC_APB2LPENR_SAI1LPEN             (1 << 21) /* Bit 21: SAI1 clock enable during sleep mode */
#define RCC_APB2LPENR_SAI2LPEN             (1 << 22) /* Bit 22: SAI2 clock enable during sleep mode */
#define RCC_APB2LPENR_USBLPEN              (1 << 24) /* Bit 24: USB clock enable during sleep mode */

/* APB3 Peripheral clock enable register */

#define RCC_APB3LPENR_SBSLPEN              (1 << 1)  /* Bit 1: SBS clock enable during sleep mode */
#define RCC_APB3LPENR_SPI5LPEN             (1 << 5)  /* Bit 5: SPI5 clock enable during sleep mode */
#define RCC_APB3LPENR_LPUART1LPEN          (1 << 6)  /* Bit 6: LPUART1 clock enable during sleep mode */
#define RCC_APB3LPENR_I2C3LPEN             (1 << 7)  /* Bit 7: I2C3 clock enable during sleep mode */
#define RCC_APB3LPENR_I2C4LPEN             (1 << 8)  /* Bit 8: I2C4 clock enable during sleep mode */
#define RCC_APB3LPENR_I3C2LPEN             (1 << 9)  /* Bit 9: I3C2 clock enable during sleep mode */
#define RCC_APB3LPENR_LPTIM1LPEN           (1 << 11) /* Bit 11: LPTIM1 clock enable during sleep mode */
#define RCC_APB3LPENR_LPTIM3LPEN           (1 << 12) /* Bit 12: LPTIM3 clock enable during sleep mode */
#define RCC_APB3LPENR_LPTIM4LPEN           (1 << 13) /* Bit 13: LPTIM4 clock enable during sleep mode */
#define RCC_APB3LPENR_LPTIM5LPEN           (1 << 14) /* Bit 14: LPTIM5 clock enable during sleep mode */
#define RCC_APB3LPENR_LPTIM6LPEN           (1 << 15) /* Bit 15: LPTIM6 clock enable during sleep mode */
#define RCC_APB3LPENR_VREFLPEN             (1 << 20) /* Bit 20: VREF clock enable during sleep mode */
#define RCC_APB3LPENR_RTCAPBLPEN           (1 << 21) /* Bit 21: RTCABP clock enable during sleep mode */

/* Kernel Clock Configuration register 1 */

#define RCC_CCIPR1_USART1SEL_SHIFT        (0)
#define RCC_CCIPR1_USART1SEL_MASK         (7 << RCC_CCIPR1_USART1SEL_SHIFT)
#  define RCC_CCIPR1_USART1SEL_RCCPCLK1   (0 << RCC_CCIPR1_USART1SEL_SHIFT)
#  define RCC_CCIPR1_USART1SEL_PLL2QCK    (1 << RCC_CCIPR1_USART1SEL_SHIFT)
#  define RCC_CCIPR1_USART1SEL_PLL3QCK    (2 << RCC_CCIPR1_USART1SEL_SHIFT)
#  define RCC_CCIPR1_USART1SEL_HSIKERCK   (3 << RCC_CCIPR1_USART1SEL_SHIFT)
#  define RCC_CCIPR1_USART1SEL_CSIKERCK   (4 << RCC_CCIPR1_USART1SEL_SHIFT)
#  define RCC_CCIPR1_USART1SEL_LSECK      (5 << RCC_CCIPR1_USART1SEL_SHIFT)

#define RCC_CCIPR1_USART2SEL_SHIFT        (3)
#define RCC_CCIPR1_USART2SEL_MASK         (7 << RCC_CCIPR1_USART2SEL_SHIFT)
#  define RCC_CCIPR1_USART2SEL_RCCPCLK1   (0 << RCC_CCIPR1_USART2SEL_SHIFT)
#  define RCC_CCIPR1_USART2SEL_PLL2QCK    (1 << RCC_CCIPR1_USART2SEL_SHIFT)
#  define RCC_CCIPR1_USART2SEL_PLL3QCK    (2 << RCC_CCIPR1_USART2SEL_SHIFT)
#  define RCC_CCIPR1_USART2SEL_HSIKERCK   (3 << RCC_CCIPR1_USART2SEL_SHIFT)
#  define RCC_CCIPR1_USART2SEL_CSIKERCK   (4 << RCC_CCIPR1_USART2SEL_SHIFT)
#  define RCC_CCIPR1_USART2SEL_LSECK      (5 << RCC_CCIPR1_USART2SEL_SHIFT)

#define RCC_CCIPR1_USART3SEL_SHIFT        (6)
#define RCC_CCIPR1_USART3SEL_MASK         (7 << RCC_CCIPR1_USART3SEL_SHIFT)
#  define RCC_CCIPR1_USART3SEL_RCCPCLK1   (0 << RCC_CCIPR1_USART3SEL_SHIFT)
#  define RCC_CCIPR1_USART3SEL_PLL2QCK    (1 << RCC_CCIPR1_USART3SEL_SHIFT)
#  define RCC_CCIPR1_USART3SEL_PLL3QCK    (2 << RCC_CCIPR1_USART3SEL_SHIFT)
#  define RCC_CCIPR1_USART3SEL_HSIKERCK   (3 << RCC_CCIPR1_USART3SEL_SHIFT)
#  define RCC_CCIPR1_USART3SEL_CSIKERCK   (4 << RCC_CCIPR1_USART3SEL_SHIFT)
#  define RCC_CCIPR1_USART3SEL_LSECK      (5 << RCC_CCIPR1_USART3SEL_SHIFT)

#define RCC_CCIPR1_UART4SEL_SHIFT        (9)
#define RCC_CCIPR1_UART4SEL_MASK         (7 << RCC_CCIPR1_UART4SEL_SHIFT)
#  define RCC_CCIPR1_UART4SEL_RCCPCLK1   (0 << RCC_CCIPR1_UART4SEL_SHIFT)
#  define RCC_CCIPR1_UART4SEL_PLL2QCK    (1 << RCC_CCIPR1_UART4SEL_SHIFT)
#  define RCC_CCIPR1_UART4SEL_PLL3QCK    (2 << RCC_CCIPR1_UART4SEL_SHIFT)
#  define RCC_CCIPR1_UART4SEL_HSIKERCK   (3 << RCC_CCIPR1_UART4SEL_SHIFT)
#  define RCC_CCIPR1_UART4SEL_CSIKERCK   (4 << RCC_CCIPR1_UART4SEL_SHIFT)
#  define RCC_CCIPR1_UART4SEL_LSECK      (5 << RCC_CCIPR1_UART4SEL_SHIFT)

#define RCC_CCIPR1_UART5SEL_SHIFT        (12)
#define RCC_CCIPR1_UART5SEL_MASK         (7 << RCC_CCIPR1_UART5SEL_SHIFT)
#  define RCC_CCIPR1_UART5SEL_RCCPCLK1   (0 << RCC_CCIPR1_UART5SEL_SHIFT)
#  define RCC_CCIPR1_UART5SEL_PLL2QCK    (1 << RCC_CCIPR1_UART5SEL_SHIFT)
#  define RCC_CCIPR1_UART5SEL_PLL3QCK    (2 << RCC_CCIPR1_UART5SEL_SHIFT)
#  define RCC_CCIPR1_UART5SEL_HSIKERCK   (3 << RCC_CCIPR1_UART5SEL_SHIFT)
#  define RCC_CCIPR1_UART5SEL_CSIKERCK   (4 << RCC_CCIPR1_UART5SEL_SHIFT)
#  define RCC_CCIPR1_UART5SEL_LSECK      (5 << RCC_CCIPR1_UART5SEL_SHIFT)

#define RCC_CCIPR1_UART6SEL_SHIFT        (15)
#define RCC_CCIPR1_UART6SEL_MASK         (7 << RCC_CCIPR1_UART6SEL_SHIFT)
#  define RCC_CCIPR1_UART6SEL_RCCPCLK1   (0 << RCC_CCIPR1_UART6SEL_SHIFT)
#  define RCC_CCIPR1_UART6SEL_PLL2QCK    (1 << RCC_CCIPR1_UART6SEL_SHIFT)
#  define RCC_CCIPR1_UART6SEL_PLL3QCK    (2 << RCC_CCIPR1_UART6SEL_SHIFT)
#  define RCC_CCIPR1_UART6SEL_HSIKERCK   (3 << RCC_CCIPR1_UART6SEL_SHIFT)
#  define RCC_CCIPR1_UART6SEL_CSIKERCK   (4 << RCC_CCIPR1_UART6SEL_SHIFT)
#  define RCC_CCIPR1_UART6SEL_LSECK      (5 << RCC_CCIPR1_UART6SEL_SHIFT)

#define RCC_CCIPR1_UART7SEL_SHIFT        (18)
#define RCC_CCIPR1_UART7SEL_MASK         (7 << RCC_CCIPR1_UART7SEL_SHIFT)
#  define RCC_CCIPR1_UART7SEL_RCCPCLK1   (0 << RCC_CCIPR1_UART7SEL_SHIFT)
#  define RCC_CCIPR1_UART7SEL_PLL2QCK    (1 << RCC_CCIPR1_UART7SEL_SHIFT)
#  define RCC_CCIPR1_UART7SEL_PLL3QCK    (2 << RCC_CCIPR1_UART7SEL_SHIFT)
#  define RCC_CCIPR1_UART7SEL_HSIKERCK   (3 << RCC_CCIPR1_UART7SEL_SHIFT)
#  define RCC_CCIPR1_UART7SEL_CSIKERCK   (4 << RCC_CCIPR1_UART7SEL_SHIFT)
#  define RCC_CCIPR1_UART7SEL_LSECK      (5 << RCC_CCIPR1_UART7SEL_SHIFT)

#define RCC_CCIPR1_UART8SEL_SHIFT        (21)
#define RCC_CCIPR1_UART8SEL_MASK         (7 << RCC_CCIPR1_UART8SEL_SHIFT)
#  define RCC_CCIPR1_UART8SEL_RCCPCLK1   (0 << RCC_CCIPR1_UART8SEL_SHIFT)
#  define RCC_CCIPR1_UART8SEL_PLL2QCK    (1 << RCC_CCIPR1_UART8SEL_SHIFT)
#  define RCC_CCIPR1_UART8SEL_PLL3QCK    (2 << RCC_CCIPR1_UART8SEL_SHIFT)
#  define RCC_CCIPR1_UART8SEL_HSIKERCK   (3 << RCC_CCIPR1_UART8SEL_SHIFT)
#  define RCC_CCIPR1_UART8SEL_CSIKERCK   (4 << RCC_CCIPR1_UART8SEL_SHIFT)
#  define RCC_CCIPR1_UART8SEL_LSECK      (5 << RCC_CCIPR1_UART8SEL_SHIFT)

#define RCC_CCIPR1_UART9SEL_SHIFT        (24)
#define RCC_CCIPR1_UART9SEL_MASK         (7 << RCC_CCIPR1_UART9SEL_SHIFT)
#  define RCC_CCIPR1_UART9SEL_RCCPCLK1   (0 << RCC_CCIPR1_UART9SEL_SHIFT)
#  define RCC_CCIPR1_UART9SEL_PLL2QCK    (1 << RCC_CCIPR1_UART9SEL_SHIFT)
#  define RCC_CCIPR1_UART9SEL_PLL3QCK    (2 << RCC_CCIPR1_UART9SEL_SHIFT)
#  define RCC_CCIPR1_UART9SEL_HSIKERCK   (3 << RCC_CCIPR1_UART9SEL_SHIFT)
#  define RCC_CCIPR1_UART9SEL_CSIKERCK   (4 << RCC_CCIPR1_UART9SEL_SHIFT)
#  define RCC_CCIPR1_UART9SEL_LSECK      (5 << RCC_CCIPR1_UART9SEL_SHIFT)

#define RCC_CCIPR1_USART10SEL_SHIFT       (27)
#define RCC_CCIPR1_USART10SEL_MASK        (7 << RCC_CCIPR1_USART10SEL_SHIFT)
#  define RCC_CCIPR1_USART10SEL_RCCPCLK1  (0 << RCC_CCIPR1_USART10SEL_SHIFT)
#  define RCC_CCIPR1_USART10SEL_PLL2QCK   (1 << RCC_CCIPR1_USART10SEL_SHIFT)
#  define RCC_CCIPR1_USART10SEL_PLL3QCK   (2 << RCC_CCIPR1_USART10SEL_SHIFT)
#  define RCC_CCIPR1_USART10SEL_HSIKERCK  (3 << RCC_CCIPR1_USART10SEL_SHIFT)
#  define RCC_CCIPR1_USART10SEL_CSIKERCK  (4 << RCC_CCIPR1_USART10SEL_SHIFT)
#  define RCC_CCIPR1_USART10SEL_LSECK     (5 << RCC_CCIPR1_USART10SEL_SHIFT)

#define RCC_CCIPR1_TIMICSEL               (1 << 31)

/* Kernel Clock Configuration register 2 */

#define RCC_CCIPR2_USART11SEL_SHIFT        (0)
#define RCC_CCIPR2_USART11SEL_MASK         (7 << RCC_CCIPR2_USART11SEL_SHIFT)
#  define RCC_CCIPR2_USART11SEL_RCCPCLK1   (0 << RCC_CCIPR2_USART11SEL_SHIFT)
#  define RCC_CCIPR2_USART11SEL_PLL2QCK    (1 << RCC_CCIPR2_USART11SEL_SHIFT)
#  define RCC_CCIPR2_USART11SEL_PLL3QCK    (2 << RCC_CCIPR2_USART11SEL_SHIFT)
#  define RCC_CCIPR2_USART11SEL_HSIKERCK   (3 << RCC_CCIPR2_USART11SEL_SHIFT)
#  define RCC_CCIPR2_USART11SEL_CSIKERCK   (4 << RCC_CCIPR2_USART11SEL_SHIFT)
#  define RCC_CCIPR2_USART11SEL_LSECK      (5 << RCC_CCIPR2_USART11SEL_SHIFT)

#define RCC_CCIPR2_USART12SEL_SHIFT        (4)
#define RCC_CCIPR2_USART12SEL_MASK         (7 << RCC_CCIPR2_USART12SEL_SHIFT)
#  define RCC_CCIPR2_USART12SEL_RCCPCLK1   (0 << RCC_CCIPR2_USART12SEL_SHIFT)
#  define RCC_CCIPR2_USART12SEL_PLL2QCK    (1 << RCC_CCIPR2_USART12SEL_SHIFT)
#  define RCC_CCIPR2_USART12SEL_PLL3QCK    (2 << RCC_CCIPR2_USART12SEL_SHIFT)
#  define RCC_CCIPR2_USART12SEL_HSIKERCK   (3 << RCC_CCIPR2_USART12SEL_SHIFT)
#  define RCC_CCIPR2_USART12SEL_CSIKERCK   (4 << RCC_CCIPR2_USART12SEL_SHIFT)
#  define RCC_CCIPR2_USART12SEL_LSECK      (5 << RCC_CCIPR2_USART12SEL_SHIFT)

#define RCC_CCIPR2_LPTIM1SEL_SHIFT         (8)
#define RCC_CCIPR2_LPTIM1SEL_MASK          (7 << RCC_CCIPR2_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM1SEL_RCCPCLK3    (0 << RCC_CCIPR2_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM1SEL_PLL2PCK     (1 << RCC_CCIPR2_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM1SEL_PLL3RCK     (2 << RCC_CCIPR2_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM1SEL_HSIKERCK    (3 << RCC_CCIPR2_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM1SEL_CSIKERCK    (4 << RCC_CCIPR2_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM1SEL_LSECK       (5 << RCC_CCIPR2_LPTIM1SEL_SHIFT)

#define RCC_CCIPR2_LPTIM2SEL_SHIFT         (12)
#define RCC_CCIPR2_LPTIM2SEL_MASK          (7 << RCC_CCIPR2_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM2SEL_RCCPCLK1    (0 << RCC_CCIPR2_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM2SEL_PLL2PCK     (1 << RCC_CCIPR2_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM2SEL_PLL3RCK     (2 << RCC_CCIPR2_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM2SEL_HSIKERCK    (3 << RCC_CCIPR2_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM2SEL_CSIKERCK    (4 << RCC_CCIPR2_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM2SEL_LSECK       (5 << RCC_CCIPR2_LPTIM2SEL_SHIFT)

#define RCC_CCIPR2_LPTIM3SEL_SHIFT         (16)
#define RCC_CCIPR2_LPTIM3SEL_MASK          (7 << RCC_CCIPR2_LPTIM3SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM3SEL_RCCPCLK3    (0 << RCC_CCIPR2_LPTIM3SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM3SEL_PLL2PCK     (1 << RCC_CCIPR2_LPTIM3SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM3SEL_PLL3RCK     (2 << RCC_CCIPR2_LPTIM3SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM3SEL_HSIKERCK    (3 << RCC_CCIPR2_LPTIM3SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM3SEL_CSIKERCK    (4 << RCC_CCIPR2_LPTIM3SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM3SEL_LSECK       (5 << RCC_CCIPR2_LPTIM3SEL_SHIFT)

#define RCC_CCIPR2_LPTIM4SEL_SHIFT         (20)
#define RCC_CCIPR2_LPTIM4SEL_MASK          (7 << RCC_CCIPR2_LPTIM4SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM4SEL_RCCPCLK3    (0 << RCC_CCIPR2_LPTIM4SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM4SEL_PLL2PCK     (1 << RCC_CCIPR2_LPTIM4SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM4SEL_PLL3RCK     (2 << RCC_CCIPR2_LPTIM4SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM4SEL_HSIKERCK    (3 << RCC_CCIPR2_LPTIM4SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM4SEL_CSIKERCK    (4 << RCC_CCIPR2_LPTIM4SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM4SEL_LSECK       (5 << RCC_CCIPR2_LPTIM4SEL_SHIFT)

#define RCC_CCIPR2_LPTIM5SEL_SHIFT         (24)
#define RCC_CCIPR2_LPTIM5SEL_MASK          (7 << RCC_CCIPR2_LPTIM5SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM5SEL_RCCPCLK3    (0 << RCC_CCIPR2_LPTIM5SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM5SEL_PLL2PCK     (1 << RCC_CCIPR2_LPTIM5SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM5SEL_PLL3RCK     (2 << RCC_CCIPR2_LPTIM5SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM5SEL_HSIKERCK    (3 << RCC_CCIPR2_LPTIM5SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM5SEL_CSIKERCK    (4 << RCC_CCIPR2_LPTIM5SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM5SEL_LSECK       (5 << RCC_CCIPR2_LPTIM5SEL_SHIFT)

#define RCC_CCIPR2_LPTIM6SEL_SHIFT         (28)
#define RCC_CCIPR2_LPTIM6SEL_MASK          (7 << RCC_CCIPR2_LPTIM6SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM6SEL_RCCPCLK3    (0 << RCC_CCIPR2_LPTIM6SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM6SEL_PLL2PCK     (1 << RCC_CCIPR2_LPTIM6SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM6SEL_PLL3RCK     (2 << RCC_CCIPR2_LPTIM6SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM6SEL_HSIKERCK    (3 << RCC_CCIPR2_LPTIM6SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM6SEL_CSIKERCK    (4 << RCC_CCIPR2_LPTIM6SEL_SHIFT)
#  define RCC_CCIPR2_LPTIM6SEL_LSECK       (5 << RCC_CCIPR2_LPTIM6SEL_SHIFT)

/* Kernel Clock Configuration register 3 */

#define RCC_CCIPR3_SPI1SEL_SHIFT         (0)
#define RCC_CCIPR3_SPI1SEL_MASK          (7 << RCC_CCIPR3_SPI1SEL_SHIFT)
#  define RCC_CCIPR3_SPI1SEL_PLL1QCK     (0 << RCC_CCIPR3_SPI1SEL_SHIFT)
#  define RCC_CCIPR3_SPI1SEL_PLL2PCK     (1 << RCC_CCIPR3_SPI1SEL_SHIFT)
#  define RCC_CCIPR3_SPI1SEL_PLL3PCK     (2 << RCC_CCIPR3_SPI1SEL_SHIFT)
#  define RCC_CCIPR3_SPI1SEL_AUDIOCK     (3 << RCC_CCIPR3_SPI1SEL_SHIFT)
#  define RCC_CCIPR3_SPI1SEL_PERCK       (4 << RCC_CCIPR3_SPI1SEL_SHIFT)

#define RCC_CCIPR3_SPI2SEL_SHIFT         (3)
#define RCC_CCIPR3_SPI2SEL_MASK          (7 << RCC_CCIPR3_SPI2SEL_SHIFT)
#  define RCC_CCIPR3_SPI2SEL_PLL1QCK     (0 << RCC_CCIPR3_SPI2SEL_SHIFT)
#  define RCC_CCIPR3_SPI2SEL_PLL2PCK     (1 << RCC_CCIPR3_SPI2SEL_SHIFT)
#  define RCC_CCIPR3_SPI2SEL_PLL3PCK     (2 << RCC_CCIPR3_SPI2SEL_SHIFT)
#  define RCC_CCIPR3_SPI2SEL_AUDIOCK     (3 << RCC_CCIPR3_SPI2SEL_SHIFT)
#  define RCC_CCIPR3_SPI2SEL_PERCK       (4 << RCC_CCIPR3_SPI2SEL_SHIFT)

#define RCC_CCIPR3_SPI3SEL_SHIFT         (6)
#define RCC_CCIPR3_SPI3SEL_MASK          (7 << RCC_CCIPR3_SPI3SEL_SHIFT)
#  define RCC_CCIPR3_SPI3SEL_PLL1QCK     (0 << RCC_CCIPR3_SPI3SEL_SHIFT)
#  define RCC_CCIPR3_SPI3SEL_PLL2PCK     (1 << RCC_CCIPR3_SPI3SEL_SHIFT)
#  define RCC_CCIPR3_SPI3SEL_PLL3PCK     (2 << RCC_CCIPR3_SPI3SEL_SHIFT)
#  define RCC_CCIPR3_SPI3SEL_AUDIOCK     (3 << RCC_CCIPR3_SPI3SEL_SHIFT)
#  define RCC_CCIPR3_SPI3SEL_PERCK       (4 << RCC_CCIPR3_SPI3SEL_SHIFT)

#define RCC_CCIPR3_SPI4SEL_SHIFT         (9)
#define RCC_CCIPR3_SPI4SEL_MASK          (7 << RCC_CCIPR3_SPI4SEL_SHIFT)
#  define RCC_CCIPR3_SPI4SEL_RCCPCLK2    (0 << RCC_CCIPR3_SPI4SEL_SHIFT)
#  define RCC_CCIPR3_SPI4SEL_PLL2QCK     (1 << RCC_CCIPR3_SPI4SEL_SHIFT)
#  define RCC_CCIPR3_SPI4SEL_PLL3QCK     (2 << RCC_CCIPR3_SPI4SEL_SHIFT)
#  define RCC_CCIPR3_SPI4SEL_HSIKERCK    (3 << RCC_CCIPR3_SPI4SEL_SHIFT)
#  define RCC_CCIPR3_SPI4SEL_CSIKERCK    (4 << RCC_CCIPR3_SPI4SEL_SHIFT)
#  define RCC_CCIPR3_SPI4SEL_HSECK       (5 << RCC_CCIPR3_SPI4SEL_SHIFT)

#define RCC_CCIPR3_SPI5SEL_SHIFT         (12)
#define RCC_CCIPR3_SPI5SEL_MASK          (7 << RCC_CCIPR3_SPI5SEL_SHIFT)
#  define RCC_CCIPR3_SPI5SEL_RCCPCLK3    (0 << RCC_CCIPR3_SPI5SEL_SHIFT)
#  define RCC_CCIPR3_SPI5SEL_PLL2QCK     (1 << RCC_CCIPR3_SPI5SEL_SHIFT)
#  define RCC_CCIPR3_SPI5SEL_PLL3QCK     (2 << RCC_CCIPR3_SPI5SEL_SHIFT)
#  define RCC_CCIPR3_SPI5SEL_HSIKERCK    (3 << RCC_CCIPR3_SPI5SEL_SHIFT)
#  define RCC_CCIPR3_SPI5SEL_CSIKERCK    (4 << RCC_CCIPR3_SPI5SEL_SHIFT)
#  define RCC_CCIPR3_SPI5SEL_HSECK       (5 << RCC_CCIPR3_SPI5SEL_SHIFT)

#define RCC_CCIPR3_SPI6SEL_SHIFT         (15)
#define RCC_CCIPR3_SPI6SEL_MASK          (7 << RCC_CCIPR3_SPI6SEL_SHIFT)
#  define RCC_CCIPR3_SPI6SEL_RCCPCLK2    (0 << RCC_CCIPR3_SPI6SEL_SHIFT)
#  define RCC_CCIPR3_SPI6SEL_PLL2QCK     (1 << RCC_CCIPR3_SPI6SEL_SHIFT)
#  define RCC_CCIPR3_SPI6SEL_PLL3QCK     (2 << RCC_CCIPR3_SPI6SEL_SHIFT)
#  define RCC_CCIPR3_SPI6SEL_HSIKERCK    (3 << RCC_CCIPR3_SPI6SEL_SHIFT)
#  define RCC_CCIPR3_SPI6SEL_CSIKERCK    (4 << RCC_CCIPR3_SPI6SEL_SHIFT)
#  define RCC_CCIPR3_SPI6SEL_HSECK       (5 << RCC_CCIPR3_SPI6SEL_SHIFT)

#define RCC_CCIPR3_LPUART1SEL_SHIFT         (24)
#define RCC_CCIPR3_LPUART1SEL_MASK          (7 << RCC_CCIPR3_LPUART1SEL_SHIFT)
#  define RCC_CCIPR3_LPUART1SEL_RCCPCLK3    (0 << RCC_CCIPR3_LPUART1SEL_SHIFT)
#  define RCC_CCIPR3_LPUART1SEL_PLL2QCK     (1 << RCC_CCIPR3_LPUART1SEL_SHIFT)
#  define RCC_CCIPR3_LPUART1SEL_PLL3QCK     (2 << RCC_CCIPR3_LPUART1SEL_SHIFT)
#  define RCC_CCIPR3_LPUART1SEL_HSIKERCK    (3 << RCC_CCIPR3_LPUART1SEL_SHIFT)
#  define RCC_CCIPR3_LPUART1SEL_CSIKERCK    (4 << RCC_CCIPR3_LPUART1SEL_SHIFT)
#  define RCC_CCIPR3_LPUART1SEL_LSECK       (5 << RCC_CCIPR3_LPUART1SEL_SHIFT)

/* Kernel Clock Configuration register 4 */

#define RCC_CCIPR4_OCTOSPI1SEL_SHIFT        (0)
#define RCC_CCIPR4_OCTOSPI1SEL_MASK         (3 << RCC_CCIPR4_OCTOSPI1SEL_SHIFT)
#  define RCC_CCIPR4_OCTOSPI1SEL_RCCHCLK4   (0 << RCC_CCIPR4_OCTOSPI1SEL_SHIFT)
#  define RCC_CCIPR4_OCTOSPI1SEL_PLL1QCK    (1 << RCC_CCIPR4_OCTOSPI1SEL_SHIFT)
#  define RCC_CCIPR4_OCTOSPI1SEL_PLL2RCK    (2 << RCC_CCIPR4_OCTOSPI1SEL_SHIFT)
#  define RCC_CCIPR4_OCTOSPI1SEL_PERCK   (3 << RCC_CCIPR4_OCTOSPI1SEL_SHIFT)

#define RCC_CCIPR4_SYSTICKSEL_SHIFT        (2)
#define RCC_CCIPR4_SYSTICKSEL_MASK         (3 << RCC_CCIPR4_SYSTICKSEL_SHIFT)
#  define RCC_CCIPR4_SYSTICKSEL_RCCHCLKd8  (0 << RCC_CCIPR4_SYSTICKSEL_SHIFT)
#  define RCC_CCIPR4_SYSTICKSEL_LSIKERCK1  (1 << RCC_CCIPR4_SYSTICKSEL_SHIFT)
#  define RCC_CCIPR4_SYSTICKSEL_LSICK1     (2 << RCC_CCIPR4_SYSTICKSEL_SHIFT)

#define RCC_CCIPR4_USBSEL_SHIFT        (4)
#define RCC_CCIPR4_USBSEL_MASK         (3 << RCC_CCIPR4_USBSEL_SHIFT)
#  define RCC_CCIPR4_USBSEL_NOCK       (0 << RCC_CCIPR4_USBSEL_SHIFT)
#  define RCC_CCIPR4_USBSEL_PLL1QCK    (1 << RCC_CCIPR4_USBSEL_SHIFT)
#  define RCC_CCIPR4_USBSEL_PLL3QCK    (2 << RCC_CCIPR4_USBSEL_SHIFT)
#  define RCC_CCIPR4_USBSEL_HSI48KERCK (3 << RCC_CCIPR4_USBSEL_SHIFT)

#define RCC_CCIPR4_SDMMC1SEL           (1 << 6)
#define RCC_CCIPR4_SDMMC2SEL           (1 << 7)

#define RCC_CCIPR4_I2C1SEL_SHIFT        (16)
#define RCC_CCIPR4_I2C1SEL_MASK         (3 << RCC_CCIPR4_I2C1SEL_SHIFT)
#  define RCC_CCIPR4_I2C1SEL_RCCPCLK1   (0 << RCC_CCIPR4_I2C1SEL_SHIFT)
#  define RCC_CCIPR4_I2C1SEL_PLL3RCK    (1 << RCC_CCIPR4_I2C1SEL_SHIFT)
#  define RCC_CCIPR4_I2C1SEL_HSIKERCK   (2 << RCC_CCIPR4_I2C1SEL_SHIFT)
#  define RCC_CCIPR4_I2C1SEL_CSIKERCK   (3 << RCC_CCIPR4_I2C1SEL_SHIFT)

#define RCC_CCIPR4_I2C2SEL_SHIFT        (18)
#define RCC_CCIPR4_I2C2SEL_MASK         (3 << RCC_CCIPR4_I2C2SEL_SHIFT)
#  define RCC_CCIPR4_I2C2SEL_RCCPCLK1   (0 << RCC_CCIPR4_I2C2SEL_SHIFT)
#  define RCC_CCIPR4_I2C2SEL_PLL3RCK    (1 << RCC_CCIPR4_I2C2SEL_SHIFT)
#  define RCC_CCIPR4_I2C2SEL_HSIKERCK   (2 << RCC_CCIPR4_I2C2SEL_SHIFT)
#  define RCC_CCIPR4_I2C2SEL_CSIKERCK   (3 << RCC_CCIPR4_I2C2SEL_SHIFT)

#define RCC_CCIPR4_I2C3SEL_SHIFT        (20)
#define RCC_CCIPR4_I2C3SEL_MASK         (3 << RCC_CCIPR4_I2C3SEL_SHIFT)
#  define RCC_CCIPR4_I2C3SEL_RCCPCLK3   (0 << RCC_CCIPR4_I2C3SEL_SHIFT)
#  define RCC_CCIPR4_I2C3SEL_PLL3RCK    (1 << RCC_CCIPR4_I2C3SEL_SHIFT)
#  define RCC_CCIPR4_I2C3SEL_HSIKERCK   (2 << RCC_CCIPR4_I2C3SEL_SHIFT)
#  define RCC_CCIPR4_I2C3SEL_CSIKERCK   (3 << RCC_CCIPR4_I2C3SEL_SHIFT)

#define RCC_CCIPR4_I2C4SEL_SHIFT        (22)
#define RCC_CCIPR4_I2C4SEL_MASK         (3 << RCC_CCIPR4_I2C4SEL_SHIFT)
#  define RCC_CCIPR4_I2C4SEL_RCCPCLK3   (0 << RCC_CCIPR4_I2C4SEL_SHIFT)
#  define RCC_CCIPR4_I2C4SEL_PLL3RCK    (1 << RCC_CCIPR4_I2C4SEL_SHIFT)
#  define RCC_CCIPR4_I2C4SEL_HSIKERCK   (2 << RCC_CCIPR4_I2C4SEL_SHIFT)
#  define RCC_CCIPR4_I2C4SEL_CSIKERCK   (3 << RCC_CCIPR4_I2C4SEL_SHIFT)

#define RCC_CCIPR4_I3C1SEL_SHIFT        (24)
#define RCC_CCIPR4_I3C1SEL_MASK         (3 << RCC_CCIPR4_I3C1SEL_SHIFT)
#  define RCC_CCIPR4_I3C1SEL_RCCPCLK1   (0 << RCC_CCIPR4_I3C1SEL_SHIFT)
#  define RCC_CCIPR4_I3C1SEL_PLL3RCK    (1 << RCC_CCIPR4_I3C1SEL_SHIFT)
#  define RCC_CCIPR4_I3C1SEL_HSIKERCK   (2 << RCC_CCIPR4_I3C1SEL_SHIFT)
#  define RCC_CCIPR4_I3C1SEL_NOCK       (3 << RCC_CCIPR4_I3C1SEL_SHIFT)

#define RCC_CCIPR4_I3C2SEL_SHIFT        (24)
#define RCC_CCIPR4_I3C2SEL_MASK         (3 << RCC_CCIPR4_I3C2SEL_SHIFT)
#  define RCC_CCIPR4_I3C2SEL_RCCPCLK1   (0 << RCC_CCIPR4_I3C2SEL_SHIFT)
#  define RCC_CCIPR4_I3C2SEL_PLL3RCK    (1 << RCC_CCIPR4_I3C2SEL_SHIFT)
#  define RCC_CCIPR4_I3C2SEL_HSIKERCK   (2 << RCC_CCIPR4_I3C2SEL_SHIFT)
#  define RCC_CCIPR4_I3C2SEL_NOCK       (3 << RCC_CCIPR4_I3C2SEL_SHIFT)

/* Kernel Clock Configuration register 5 */

#define RCC_CCIPR5_ADCDACSEL_SHIFT        (0)
#define RCC_CCIPR5_ADCDACSEL_MASK         (7 << RCC_CCIPR5_ADCDACSEL_SHIFT)
#  define RCC_CCIPR5_ADCDACSEL_RCCHCLK    (0 << RCC_CCIPR5_ADCDACSEL_SHIFT)
#  define RCC_CCIPR5_ADCDACSEL_SYSCK      (1 << RCC_CCIPR5_ADCDACSEL_SHIFT)
#  define RCC_CCIPR5_ADCDACSEL_PLL2RCK    (2 << RCC_CCIPR5_ADCDACSEL_SHIFT)
#  define RCC_CCIPR5_ADCDACSEL_HSECK      (3 << RCC_CCIPR5_ADCDACSEL_SHIFT)
#  define RCC_CCIPR5_ADCDACSEL_HSEKERCK   (4 << RCC_CCIPR5_ADCDACSEL_SHIFT)
#  define RCC_CCIPR5_ADCDACSEL_CSIKERCK   (5 << RCC_CCIPR5_ADCDACSEL_SHIFT)

#define RCC_CCIPR5_DACSEL                 (1 << 3)

#define RCC_CCIPR5_RNGSEL_SHIFT        (4)
#define RCC_CCIPR5_RNGSEL_MASK         (3 << RCC_CCIPR5_RNGSEL_SHIFT)
#  define RCC_CCIPR5_RNGSEL_HSI48KERCK (0 << RCC_CCIPR5_RNGSEL_SHIFT)
#  define RCC_CCIPR5_RNGSEL_PLL1QCK    (1 << RCC_CCIPR5_RNGSEL_SHIFT)
#  define RCC_CCIPR5_RNGSEL_LSECK      (2 << RCC_CCIPR5_RNGSEL_SHIFT)
#  define RCC_CCIPR5_RNGSEL_LSIKERCK   (3 << RCC_CCIPR5_RNGSEL_SHIFT)

#define RCC_CCIPR5_CECSEL_SHIFT          (6)
#define RCC_CCIPR5_CECSEL_MASK           (3 << RCC_CCIPR5_CECSEL_SHIFT)
#  define RCC_CCIPR5_CECSEL_LSECK        (0 << RCC_CCIPR5_CECSEL_SHIFT)
#  define RCC_CCIPR5_CECSEL_LSIKERCK     (1 << RCC_CCIPR5_CECSEL_SHIFT)
#  define RCC_CCIPR5_CECSEL_CSIKERCKd122 (2 << RCC_CCIPR5_CECSEL_SHIFT)

#define RCC_CCIPR5_FDCANSEL_SHIFT      (8)
#define RCC_CCIPR5_FDCANSEL_MASK       (3 << RCC_CCIPR5_FDCANSEL_SHIFT)
#  define RCC_CCIPR5_FDCANSEL_HSECK    (0 << RCC_CCIPR5_FDCANSEL_SHIFT)
#  define RCC_CCIPR5_FDCANSEL_PLL1QCK  (1 << RCC_CCIPR5_FDCANSEL_SHIFT)
#  define RCC_CCIPR5_FDCANSEL_PLL2QCK  (2 << RCC_CCIPR5_FDCANSEL_SHIFT)

#define RCC_CCIPR5_SAI1SEL_SHIFT        (16)
#define RCC_CCIPR5_SAI1SEL_MASK         (7 << RCC_CCIPR5_SAI1SEL_SHIFT)
#  define RCC_CCIPR5_SAI1SEL_PLL1QCK    (0 << RCC_CCIPR5_SAI1SEL_SHIFT)
#  define RCC_CCIPR5_SAI1SEL_PLL2PCK    (1 << RCC_CCIPR5_SAI1SEL_SHIFT)
#  define RCC_CCIPR5_SAI1SEL_PLL3PCK    (2 << RCC_CCIPR5_SAI1SEL_SHIFT)
#  define RCC_CCIPR5_SAI1SEL_AUDIOCLK   (3 << RCC_CCIPR5_SAI1SEL_SHIFT)
#  define RCC_CCIPR5_SAI1SEL_PERCK      (4 << RCC_CCIPR5_SAI1SEL_SHIFT)

#define RCC_CCIPR5_SAI2SEL_SHIFT        (19)
#define RCC_CCIPR5_SAI2SEL_MASK         (7 << RCC_CCIPR5_SAI2SEL_SHIFT)
#  define RCC_CCIPR5_SAI2SEL_PLL1QCK    (0 << RCC_CCIPR5_SAI2SEL_SHIFT)
#  define RCC_CCIPR5_SAI2SEL_PLL2PCK    (1 << RCC_CCIPR5_SAI2SEL_SHIFT)
#  define RCC_CCIPR5_SAI2SEL_PLL3PCK    (2 << RCC_CCIPR5_SAI2SEL_SHIFT)
#  define RCC_CCIPR5_SAI2SEL_AUDIOCLK   (3 << RCC_CCIPR5_SAI2SEL_SHIFT)
#  define RCC_CCIPR5_SAI2SEL_PERCK      (4 << RCC_CCIPR5_SAI2SEL_SHIFT)

#define RCC_CCIPR5_CKPERSEL_SHIFT       (30)
#define RCC_CCIPR5_CKPERSEL_MASK        (3 << RCC_CCIPR5_CKPERSEL_SHIFT)
#  define RCC_CCIPR5_CKPERSEL_HSIKERCK  (0 << RCC_CCIPR5_CKPERSEL_SHIFT)
#  define RCC_CCIPR5_CKPERSEL_CSIKERCK  (1 << RCC_CCIPR5_CKPERSEL_SHIFT)
#  define RCC_CCIPR5_CKPERSEL_HSECK     (2 << RCC_CCIPR5_CKPERSEL_SHIFT)

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
#define RCC_BDCR_LSEEXT                  (1 << 7) /* Bit 7: LSE external clock type in bypass mode */

#define RCC_BDCR_RTCSEL_SHIFT            (8)       /* Bits 9:8: RTC clock source selection */
#define RCC_BDCR_RTCSEL_MASK             (3 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_NOCLK          (0 << RCC_BDCR_RTCSEL_SHIFT) /* 00: No clock */
#  define RCC_BDCR_RTCSEL_LSE            (1 << RCC_BDCR_RTCSEL_SHIFT) /* 01: LSE oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_LSI            (2 << RCC_BDCR_RTCSEL_SHIFT) /* 10: LSI oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_HSE            (3 << RCC_BDCR_RTCSEL_SHIFT) /* 11: HSE oscillator clock divided by 32 used as RTC clock */

#define RCC_BDCR_RTCEN                   (1 << 15)         /* Bit 15: RTC clock enable */
#define RCC_BDCR_VSWRST                  (1 << 16)         /* Bit 16: VSwitch domain software reset */
#define RCC_BDCR_LSCOEN                  (1 << 24)         /* Bit 24: Low speed clock output enable */
#define RCC_BDCR_LSCOSEL                 (1 << 25)         /* Bit 25: Low speed clock output selection */
#  define RCC_BCDR_LSCOSEL_LSI           0                 /* LSI selected */
#  define RCC_BDCR_LSCOSEL_LSE           RCC_BDCR_LSCOSEL  /* LSE selected */

#define RCC_BDCR_LSION                   (1 << 26)         /* Bit 26: LSI Oscillator Enable */
#define RCC_BDCR_LSIRDY                  (1 << 27)         /* Bit 27: LSI Oscillator Ready */

/* Reset status register */

#define RCC_RSR_RMVF                     (1 << 23) /* Bit 23: Remove reset flag */
#define RCC_RSR_PINRSTF                  (1 << 26) /* Bit 26: PIN reset flag */
#define RCC_RSR_BORRSTF                  (1 << 27) /* Bit 27: BOR reset flag */
#define RCC_RSR_SFTRSTF                  (1 << 28) /* Bit 28: Software Reset flag */
#define RCC_RSR_IWDGRSTF                 (1 << 29) /* Bit 29: Independent Watchdog reset flag */
#define RCC_RSR_WWDGRSTF                 (1 << 30) /* Bit 30: Window watchdog reset flag */
#define RCC_RSR_LPWRRSTF                 (1 << 31) /* Bit 31: Low-Power reset flag */

/* Secure Configuration Register */

#define RCC_SECCFGR_HSISEC               (1 << 0)  /* HSI clock configuration and status bits security */
#define RCC_SECCFGR_HSESEC               (1 << 1)  /* HSE clock configuration and status bits security */
#define RCC_SECCFGR_CSISEC               (1 << 2)  /* CSI clock configuration and status bits security */
#define RCC_SECCFGR_LSISEC               (1 << 3)  /* LSI clock configuration and status bits security */
#define RCC_SECCFGR_LSESEC               (1 << 4)  /* LSE clock configuration and status bits security */
#define RCC_SECCFGR_SYSCLKSEC            (1 << 5)  /* SYSCLK configuration and status bits security */
#define RCC_SECCFGR_PRESCSEC             (1 << 6)  /* PRESC configuration and status bits security */
#define RCC_SECCFGR_PLL1SEC              (1 << 7)  /* PLL1 configuration and status bits security */
#define RCC_SECCFGR_PLL2SEC              (1 << 8)  /* PLL2 configuration and status bits security */
#define RCC_SECCFGR_PLL3SEC              (1 << 9)  /* PLL3 configuration and status bits security */
#define RCC_SECCFGR_HSI48SEC             (1 << 11) /* HSI48 configuration and status bits security */
#define RCC_SECCFGR_RMVRST               (1 << 12) /* Remove Reset flag security */
#define RCC_SECCFGR_CKPERSELSEC          (1 << 13) /* PER_CK selection security */

/* Privilege Configuration Register */

#define RCC_PRIVCFGR_SPRIV               (1 << 0) /* Secure functions privilege configuration */
#define RCC_PRIVCFGR_NSPRIV              (1 << 1) /* Non-secure functions privilege configuration */

#endif /* CONFIG_STM32H5_STM32H562XX */
#endif /* __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32H5XXX_RCC_H */
