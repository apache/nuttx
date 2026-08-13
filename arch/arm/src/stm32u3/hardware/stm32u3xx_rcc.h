/****************************************************************************
 * arch/arm/src/stm32u3/hardware/stm32u3xx_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32U3XX_RCC_H
#define __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32U3XX_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_RCC_CR_OFFSET          0x0000
#define STM32_RCC_ICSCR1_OFFSET      0x0008
#define STM32_RCC_ICSCR2_OFFSET      0x000c
#define STM32_RCC_ICSCR3_OFFSET      0x0010
#define STM32_RCC_CRRCR_OFFSET       0x0014
#define STM32_RCC_CFGR1_OFFSET       0x001c
#define STM32_RCC_CFGR2_OFFSET       0x0020
#define STM32_RCC_CFGR3_OFFSET       0x0024
#define STM32_RCC_CFGR4_OFFSET       0x0028
#define STM32_RCC_CIER_OFFSET        0x0050
#define STM32_RCC_CIFR_OFFSET        0x0054
#define STM32_RCC_CICR_OFFSET        0x0058
#define STM32_RCC_AHB1RSTR1_OFFSET   0x0060
#define STM32_RCC_AHB2RSTR1_OFFSET   0x0064
#define STM32_RCC_AHB2RSTR2_OFFSET   0x0068
#define STM32_RCC_APB1RSTR1_OFFSET   0x0074
#define STM32_RCC_APB1RSTR2_OFFSET   0x0078
#define STM32_RCC_APB2RSTR_OFFSET    0x007c
#define STM32_RCC_APB3RSTR_OFFSET    0x0080
#define STM32_RCC_AHB1ENR1_OFFSET    0x0088
#define STM32_RCC_AHB2ENR1_OFFSET    0x008c
#define STM32_RCC_AHB2ENR2_OFFSET    0x0090
#define STM32_RCC_AHB1ENR2_OFFSET    0x0094
#define STM32_RCC_APB1ENR1_OFFSET    0x009c
#define STM32_RCC_APB1ENR2_OFFSET    0x00a0
#define STM32_RCC_APB2ENR_OFFSET     0x00a4
#define STM32_RCC_APB3ENR_OFFSET     0x00a8
#define STM32_RCC_AHB1SLPENR1_OFFSET 0x00b0
#define STM32_RCC_AHB2SLPENR1_OFFSET 0x00b4
#define STM32_RCC_AHB2SLPENR2_OFFSET 0x00b8
#define STM32_RCC_AHB1SLPENR2_OFFSET 0x00bc
#define STM32_RCC_APB1SLPENR1_OFFSET 0x00c4
#define STM32_RCC_APB1SLPENR2_OFFSET 0x00c8
#define STM32_RCC_APB2SLPENR_OFFSET  0x00cc
#define STM32_RCC_APB3SLPENR_OFFSET  0x00d0
#define STM32_RCC_AHB1STPENR1_OFFSET 0x00d8
#define STM32_RCC_AHB2STPENR1_OFFSET 0x00dc
#define STM32_RCC_APB1STPENR1_OFFSET 0x00ec
#define STM32_RCC_APB1STPENR2_OFFSET 0x00f0
#define STM32_RCC_APB2STPENR_OFFSET  0x00f4
#define STM32_RCC_APB3STPENR_OFFSET  0x00f8
#define STM32_RCC_CCIPR1_OFFSET      0x0100
#define STM32_RCC_CCIPR2_OFFSET      0x0104
#define STM32_RCC_CCIPR3_OFFSET      0x0108
#define STM32_RCC_BDCR_OFFSET        0x0110
#define STM32_RCC_CSR_OFFSET         0x0114
#define STM32_RCC_SECCFGR_OFFSET     0x0130
#define STM32_RCC_PRIVCFGR_OFFSET    0x0134

/* Register Addresses *******************************************************/

#define STM32_RCC_CR          (STM32_RCC_BASE + STM32_RCC_CR_OFFSET)
#define STM32_RCC_ICSCR1      (STM32_RCC_BASE + STM32_RCC_ICSCR1_OFFSET)
#define STM32_RCC_ICSCR2      (STM32_RCC_BASE + STM32_RCC_ICSCR2_OFFSET)
#define STM32_RCC_ICSCR3      (STM32_RCC_BASE + STM32_RCC_ICSCR3_OFFSET)
#define STM32_RCC_CRRCR       (STM32_RCC_BASE + STM32_RCC_CRRCR_OFFSET)
#define STM32_RCC_CFGR1       (STM32_RCC_BASE + STM32_RCC_CFGR1_OFFSET)
#define STM32_RCC_CFGR2       (STM32_RCC_BASE + STM32_RCC_CFGR2_OFFSET)
#define STM32_RCC_CFGR3       (STM32_RCC_BASE + STM32_RCC_CFGR3_OFFSET)
#define STM32_RCC_CFGR4       (STM32_RCC_BASE + STM32_RCC_CFGR4_OFFSET)
#define STM32_RCC_CIER        (STM32_RCC_BASE + STM32_RCC_CIER_OFFSET)
#define STM32_RCC_CIFR        (STM32_RCC_BASE + STM32_RCC_CIFR_OFFSET)
#define STM32_RCC_CICR        (STM32_RCC_BASE + STM32_RCC_CICR_OFFSET)
#define STM32_RCC_AHB1RSTR1   (STM32_RCC_BASE + STM32_RCC_AHB1RSTR1_OFFSET)
#define STM32_RCC_AHB2RSTR1   (STM32_RCC_BASE + STM32_RCC_AHB2RSTR1_OFFSET)
#define STM32_RCC_AHB2RSTR2   (STM32_RCC_BASE + STM32_RCC_AHB2RSTR2_OFFSET)
#define STM32_RCC_APB1RSTR1   (STM32_RCC_BASE + STM32_RCC_APB1RSTR1_OFFSET)
#define STM32_RCC_APB1RSTR2   (STM32_RCC_BASE + STM32_RCC_APB1RSTR2_OFFSET)
#define STM32_RCC_APB2RSTR    (STM32_RCC_BASE + STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_APB3RSTR    (STM32_RCC_BASE + STM32_RCC_APB3RSTR_OFFSET)
#define STM32_RCC_AHB1ENR1    (STM32_RCC_BASE + STM32_RCC_AHB1ENR1_OFFSET)
#define STM32_RCC_AHB2ENR1    (STM32_RCC_BASE + STM32_RCC_AHB2ENR1_OFFSET)
#define STM32_RCC_AHB2ENR2    (STM32_RCC_BASE + STM32_RCC_AHB2ENR2_OFFSET)
#define STM32_RCC_AHB1ENR2    (STM32_RCC_BASE + STM32_RCC_AHB1ENR2_OFFSET)
#define STM32_RCC_APB1ENR1    (STM32_RCC_BASE + STM32_RCC_APB1ENR1_OFFSET)
#define STM32_RCC_APB1ENR2    (STM32_RCC_BASE + STM32_RCC_APB1ENR2_OFFSET)
#define STM32_RCC_APB2ENR     (STM32_RCC_BASE + STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_APB3ENR     (STM32_RCC_BASE + STM32_RCC_APB3ENR_OFFSET)
#define STM32_RCC_AHB1SLPENR1 (STM32_RCC_BASE + STM32_RCC_AHB1SLPENR1_OFFSET)
#define STM32_RCC_AHB2SLPENR1 (STM32_RCC_BASE + STM32_RCC_AHB2SLPENR1_OFFSET)
#define STM32_RCC_AHB2SLPENR2 (STM32_RCC_BASE + STM32_RCC_AHB2SLPENR2_OFFSET)
#define STM32_RCC_AHB1SLPENR2 (STM32_RCC_BASE + STM32_RCC_AHB1SLPENR2_OFFSET)
#define STM32_RCC_APB1SLPENR1 (STM32_RCC_BASE + STM32_RCC_APB1SLPENR1_OFFSET)
#define STM32_RCC_APB1SLPENR2 (STM32_RCC_BASE + STM32_RCC_APB1SLPENR2_OFFSET)
#define STM32_RCC_APB2SLPENR  (STM32_RCC_BASE + STM32_RCC_APB2SLPENR_OFFSET)
#define STM32_RCC_APB3SLPENR  (STM32_RCC_BASE + STM32_RCC_APB3SLPENR_OFFSET)
#define STM32_RCC_AHB1STPENR1 (STM32_RCC_BASE + STM32_RCC_AHB1STPENR1_OFFSET)
#define STM32_RCC_AHB2STPENR1 (STM32_RCC_BASE + STM32_RCC_AHB2STPENR1_OFFSET)
#define STM32_RCC_APB1STPENR1 (STM32_RCC_BASE + STM32_RCC_APB1STPENR1_OFFSET)
#define STM32_RCC_APB1STPENR2 (STM32_RCC_BASE + STM32_RCC_APB1STPENR2_OFFSET)
#define STM32_RCC_APB2STPENR  (STM32_RCC_BASE + STM32_RCC_APB2STPENR_OFFSET)
#define STM32_RCC_APB3STPENR  (STM32_RCC_BASE + STM32_RCC_APB3STPENR_OFFSET)
#define STM32_RCC_CCIPR1      (STM32_RCC_BASE + STM32_RCC_CCIPR1_OFFSET)
#define STM32_RCC_CCIPR2      (STM32_RCC_BASE + STM32_RCC_CCIPR2_OFFSET)
#define STM32_RCC_CCIPR3      (STM32_RCC_BASE + STM32_RCC_CCIPR3_OFFSET)
#define STM32_RCC_BDCR        (STM32_RCC_BASE + STM32_RCC_BDCR_OFFSET)
#define STM32_RCC_CSR         (STM32_RCC_BASE + STM32_RCC_CSR_OFFSET)
#define STM32_RCC_SECCFGR     (STM32_RCC_BASE + STM32_RCC_SECCFGR_OFFSET)
#define STM32_RCC_PRIVCFGR    (STM32_RCC_BASE + STM32_RCC_PRIVCFGR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* RCC clock control register */

#define RCC_CR_RESET                  0x0000001d
#define RCC_CR_MSISON                 (1 << 0)
#define RCC_CR_MSIKERON               (1 << 1)
#define RCC_CR_MSISRDY                (1 << 2)
#define RCC_CR_MSIKON                 (1 << 3)
#define RCC_CR_MSIKRDY                (1 << 4)
#define RCC_CR_MSIPLL1EN              (1 << 5)
#define RCC_CR_MSIPLL0EN              (1 << 6)
#define RCC_CR_MSIPLL1FAST            (1 << 7)
#define RCC_CR_MSIPLL0FAST            (1 << 8)
#define RCC_CR_MSIPLL1RDY             (1 << 9)
#define RCC_CR_MSIPLL0RDY             (1 << 10)
#define RCC_CR_HSION                  (1 << 11)
#define RCC_CR_HSIKERON               (1 << 12)
#define RCC_CR_HSIRDY                 (1 << 13)
#define RCC_CR_HSI48ON                (1 << 14)
#define RCC_CR_HSI48RDY               (1 << 15)
#define RCC_CR_HSEON                  (1 << 16)
#define RCC_CR_HSERDY                 (1 << 17)
#define RCC_CR_HSEBYP                 (1 << 18)
#define RCC_CR_HSECSSON               (1 << 19)
#define RCC_CR_HSEEXT                 (1 << 20)

/* RCC internal clock source calibration register 1 */

#define RCC_ICSCR1_RESET              0xb4000000
#define RCC_ICSCR1_MSICAL1_SHIFT      (0)
#define RCC_ICSCR1_MSICAL1_MASK       (0x3f << RCC_ICSCR1_MSICAL1_SHIFT)
#define RCC_ICSCR1_MSICAL1(n)         ((n) << RCC_ICSCR1_MSICAL1_SHIFT)
#define RCC_ICSCR1_MSICAL0_SHIFT      (6)
#define RCC_ICSCR1_MSICAL0_MASK       (0x3f << RCC_ICSCR1_MSICAL0_SHIFT)
#define RCC_ICSCR1_MSICAL0(n)         ((n) << RCC_ICSCR1_MSICAL0_SHIFT)
#define RCC_ICSCR1_MSIHSINDIV         (1 << 19)
#define RCC_ICSCR1_MSIPLL1SEL         (1 << 20)
#define RCC_ICSCR1_MSIPLL0SEL         (1 << 21)
#define RCC_ICSCR1_MSIBIAS            (1 << 22)
#define RCC_ICSCR1_MSIRGSEL           (1 << 23)
#define RCC_ICSCR1_MSIPLL1N_SHIFT     (24)
#define RCC_ICSCR1_MSIPLL1N_MASK      (0x3 << RCC_ICSCR1_MSIPLL1N_SHIFT)
#define RCC_ICSCR1_MSIPLL1N(n)        ((n) << RCC_ICSCR1_MSIPLL1N_SHIFT)
#define RCC_ICSCR1_MSIKDIV_SHIFT      (26)
#define RCC_ICSCR1_MSIKDIV_MASK       (0x3 << RCC_ICSCR1_MSIKDIV_SHIFT)
#define RCC_ICSCR1_MSIKDIV(n)         ((n) << RCC_ICSCR1_MSIKDIV_SHIFT)
#  define RCC_ICSCR1_MSIKDIV_DIV1     RCC_ICSCR1_MSIKDIV(0)
#  define RCC_ICSCR1_MSIKDIV_DIV2     RCC_ICSCR1_MSIKDIV(1)
#  define RCC_ICSCR1_MSIKDIV_DIV4     RCC_ICSCR1_MSIKDIV(2)
#  define RCC_ICSCR1_MSIKDIV_DIV8     RCC_ICSCR1_MSIKDIV(3)
#define RCC_ICSCR1_MSIKSEL            (1 << 28)
#  define RCC_ICSCR1_MSIKSEL_MSIRC0   (0)
#  define RCC_ICSCR1_MSIKSEL_MSIRC1   RCC_ICSCR1_MSIKSEL
#define RCC_ICSCR1_MSISDIV_SHIFT      (29)
#define RCC_ICSCR1_MSISDIV_MASK       (0x3 << RCC_ICSCR1_MSISDIV_SHIFT)
#define RCC_ICSCR1_MSISDIV(n)         ((n) << RCC_ICSCR1_MSISDIV_SHIFT)
#  define RCC_ICSCR1_MSISDIV_DIV1     RCC_ICSCR1_MSISDIV(0)
#  define RCC_ICSCR1_MSISDIV_DIV2     RCC_ICSCR1_MSISDIV(1)
#  define RCC_ICSCR1_MSISDIV_DIV4     RCC_ICSCR1_MSISDIV(2)
#  define RCC_ICSCR1_MSISDIV_DIV8     RCC_ICSCR1_MSISDIV(3)
#define RCC_ICSCR1_MSISSEL            (1 << 31)
#  define RCC_ICSCR1_MSISSEL_MSIRC0   (0)
#  define RCC_ICSCR1_MSISSEL_MSIRC1   RCC_ICSCR1_MSISSEL

/* RCC internal clock source calibration register 2 */

#define RCC_ICSCR2_RESET              0x00000820
#define RCC_ICSCR2_MSITRIM1_SHIFT     (0)
#define RCC_ICSCR2_MSITRIM1_MASK      (0x3f << RCC_ICSCR2_MSITRIM1_SHIFT)
#define RCC_ICSCR2_MSITRIM1(n)        ((n) << RCC_ICSCR2_MSITRIM1_SHIFT)
#define RCC_ICSCR2_MSITRIM0_SHIFT     (6)
#define RCC_ICSCR2_MSITRIM0_MASK      (0x3f << RCC_ICSCR2_MSITRIM0_SHIFT)
#define RCC_ICSCR2_MSITRIM0(n)        ((n) << RCC_ICSCR2_MSITRIM0_SHIFT)

/* RCC internal clock source calibration register 3 */

#define RCC_ICSCR3_RESET              0x00100000
#define RCC_ICSCR3_HSICAL_SHIFT       (0)
#define RCC_ICSCR3_HSICAL_MASK        (0xfff << RCC_ICSCR3_HSICAL_SHIFT)
#define RCC_ICSCR3_HSICAL(n)          ((n) << RCC_ICSCR3_HSICAL_SHIFT)
#define RCC_ICSCR3_HSITRIM_SHIFT      (16)
#define RCC_ICSCR3_HSITRIM_MASK       (0x1f << RCC_ICSCR3_HSITRIM_SHIFT)
#define RCC_ICSCR3_HSITRIM(n)         ((n) << RCC_ICSCR3_HSITRIM_SHIFT)

/* RCC clock recovery RC register */

#define RCC_CRRCR_RESET               0x00000000
#define RCC_CRRCR_HSI48CAL_SHIFT      (0)
#define RCC_CRRCR_HSI48CAL_MASK       (0x1ff << RCC_CRRCR_HSI48CAL_SHIFT)
#define RCC_CRRCR_HSI48CAL(n)         ((n) << RCC_CRRCR_HSI48CAL_SHIFT)

/* RCC clock configuration register 1 */

#define RCC_CFGR1_RESET               0x00000000
#define RCC_CFGR1_SW_SHIFT            (0)
#define RCC_CFGR1_SW_MASK             (0x3 << RCC_CFGR1_SW_SHIFT)
#define RCC_CFGR1_SW(n)               ((n) << RCC_CFGR1_SW_SHIFT)
#  define RCC_CFGR1_SW_MSIS           RCC_CFGR1_SW(0)
#  define RCC_CFGR1_SW_HSI            RCC_CFGR1_SW(1)
#  define RCC_CFGR1_SW_HSE            RCC_CFGR1_SW(2)
#define RCC_CFGR1_SWS_SHIFT           (2)
#define RCC_CFGR1_SWS_MASK            (0x3 << RCC_CFGR1_SWS_SHIFT)
#define RCC_CFGR1_SWS(n)              ((n) << RCC_CFGR1_SWS_SHIFT)
#  define RCC_CFGR1_SWS_MSIS          RCC_CFGR1_SWS(0)
#  define RCC_CFGR1_SWS_HSI           RCC_CFGR1_SWS(1)
#  define RCC_CFGR1_SWS_HSE           RCC_CFGR1_SWS(2)
#define RCC_CFGR1_STOPWUCK            (1 << 4)
#  define RCC_CFGR1_STOPWUCK_MSIS     (0)
#  define RCC_CFGR1_STOPWUCK_HSI      RCC_CFGR1_STOPWUCK
#define RCC_CFGR1_STOPKERWUCK         (1 << 5)
#  define RCC_CFGR1_STOPKERWUCK_MSIK  (0)
#  define RCC_CFGR1_STOPKERWUCK_HSI   RCC_CFGR1_STOPKERWUCK
#define RCC_CFGR1_MCO2SEL_SHIFT       (16)
#define RCC_CFGR1_MCO2SEL_MASK        (0xf << RCC_CFGR1_MCO2SEL_SHIFT)
#define RCC_CFGR1_MCO2SEL(n)          ((n) << RCC_CFGR1_MCO2SEL_SHIFT)
#  define RCC_CFGR1_MCO2SEL_NONE      RCC_CFGR1_MCO2SEL(0)
#  define RCC_CFGR1_MCO2SEL_SYSCLK    RCC_CFGR1_MCO2SEL(1)
#  define RCC_CFGR1_MCO2SEL_MSIS      RCC_CFGR1_MCO2SEL(2)
#  define RCC_CFGR1_MCO2SEL_HSI       RCC_CFGR1_MCO2SEL(3)
#  define RCC_CFGR1_MCO2SEL_HSE       RCC_CFGR1_MCO2SEL(4)
#  define RCC_CFGR1_MCO2SEL_LSI       RCC_CFGR1_MCO2SEL(5)
#  define RCC_CFGR1_MCO2SEL_LSE       RCC_CFGR1_MCO2SEL(6)
#  define RCC_CFGR1_MCO2SEL_HSI48     RCC_CFGR1_MCO2SEL(7)
#  define RCC_CFGR1_MCO2SEL_MSIK      RCC_CFGR1_MCO2SEL(8)
#define RCC_CFGR1_MCO2PRE_SHIFT       (20)
#define RCC_CFGR1_MCO2PRE_MASK        (0x7 << RCC_CFGR1_MCO2PRE_SHIFT)
#define RCC_CFGR1_MCO2PRE(n)          ((n) << RCC_CFGR1_MCO2PRE_SHIFT)
#  define RCC_CFGR1_MCO2PRE_DIV1      RCC_CFGR1_MCO2PRE(0)
#  define RCC_CFGR1_MCO2PRE_DIV2      RCC_CFGR1_MCO2PRE(1)
#  define RCC_CFGR1_MCO2PRE_DIV4      RCC_CFGR1_MCO2PRE(2)
#  define RCC_CFGR1_MCO2PRE_DIV8      RCC_CFGR1_MCO2PRE(3)
#  define RCC_CFGR1_MCO2PRE_DIV16     RCC_CFGR1_MCO2PRE(4)
#  define RCC_CFGR1_MCO2PRE_DIV32     RCC_CFGR1_MCO2PRE(5)
#  define RCC_CFGR1_MCO2PRE_DIV64     RCC_CFGR1_MCO2PRE(6)
#  define RCC_CFGR1_MCO2PRE_DIV128    RCC_CFGR1_MCO2PRE(7)
#define RCC_CFGR1_MCOSEL_SHIFT        (24)
#define RCC_CFGR1_MCOSEL_MASK         (0xf << RCC_CFGR1_MCOSEL_SHIFT)
#define RCC_CFGR1_MCOSEL(n)           ((n) << RCC_CFGR1_MCOSEL_SHIFT)
#  define RCC_CFGR1_MCOSEL_NONE       RCC_CFGR1_MCOSEL(0)
#  define RCC_CFGR1_MCOSEL_SYSCLK     RCC_CFGR1_MCOSEL(1)
#  define RCC_CFGR1_MCOSEL_MSIS       RCC_CFGR1_MCOSEL(2)
#  define RCC_CFGR1_MCOSEL_HSI        RCC_CFGR1_MCOSEL(3)
#  define RCC_CFGR1_MCOSEL_HSE        RCC_CFGR1_MCOSEL(4)
#  define RCC_CFGR1_MCOSEL_LSI        RCC_CFGR1_MCOSEL(5)
#  define RCC_CFGR1_MCOSEL_LSE        RCC_CFGR1_MCOSEL(6)
#  define RCC_CFGR1_MCOSEL_HSI48      RCC_CFGR1_MCOSEL(7)
#  define RCC_CFGR1_MCOSEL_MSIK       RCC_CFGR1_MCOSEL(8)
#define RCC_CFGR1_MCOPRE_SHIFT        (28)
#define RCC_CFGR1_MCOPRE_MASK         (0x7 << RCC_CFGR1_MCOPRE_SHIFT)
#define RCC_CFGR1_MCOPRE(n)           ((n) << RCC_CFGR1_MCOPRE_SHIFT)
#  define RCC_CFGR1_MCOPRE_DIV1       RCC_CFGR1_MCOPRE(0)
#  define RCC_CFGR1_MCOPRE_DIV2       RCC_CFGR1_MCOPRE(1)
#  define RCC_CFGR1_MCOPRE_DIV4       RCC_CFGR1_MCOPRE(2)
#  define RCC_CFGR1_MCOPRE_DIV8       RCC_CFGR1_MCOPRE(3)
#  define RCC_CFGR1_MCOPRE_DIV16      RCC_CFGR1_MCOPRE(4)
#  define RCC_CFGR1_MCOPRE_DIV32      RCC_CFGR1_MCOPRE(5)
#  define RCC_CFGR1_MCOPRE_DIV64      RCC_CFGR1_MCOPRE(6)
#  define RCC_CFGR1_MCOPRE_DIV128     RCC_CFGR1_MCOPRE(7)

/* RCC clock configuration register 2 */

#define RCC_CFGR2_RESET               0x00000000
#define RCC_CFGR2_HPRE_SHIFT          (0)
#define RCC_CFGR2_HPRE_MASK           (0xf << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE(n)             ((n) << RCC_CFGR2_HPRE_SHIFT)
#  define RCC_CFGR2_HPRE_SYSCLK       RCC_CFGR2_HPRE(0)
#  define RCC_CFGR2_HPRE_DIV2         RCC_CFGR2_HPRE(8)
#  define RCC_CFGR2_HPRE_DIV4         RCC_CFGR2_HPRE(9)
#  define RCC_CFGR2_HPRE_DIV8         RCC_CFGR2_HPRE(10)
#  define RCC_CFGR2_HPRE_DIV16        RCC_CFGR2_HPRE(11)
#  define RCC_CFGR2_HPRE_DIV64        RCC_CFGR2_HPRE(12)
#  define RCC_CFGR2_HPRE_DIV128       RCC_CFGR2_HPRE(13)
#  define RCC_CFGR2_HPRE_DIV256       RCC_CFGR2_HPRE(14)
#  define RCC_CFGR2_HPRE_DIV512       RCC_CFGR2_HPRE(15)
#define RCC_CFGR2_PPRE1_SHIFT         (4)
#define RCC_CFGR2_PPRE1_MASK          (0x7 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE1(n)            ((n) << RCC_CFGR2_PPRE1_SHIFT)
#  define RCC_CFGR2_PPRE1_HCLK        RCC_CFGR2_PPRE1(0)
#  define RCC_CFGR2_PPRE1_DIV2        RCC_CFGR2_PPRE1(4)
#  define RCC_CFGR2_PPRE1_DIV4        RCC_CFGR2_PPRE1(5)
#  define RCC_CFGR2_PPRE1_DIV8        RCC_CFGR2_PPRE1(6)
#  define RCC_CFGR2_PPRE1_DIV16       RCC_CFGR2_PPRE1(7)
#define RCC_CFGR2_PPRE2_SHIFT         (8)
#define RCC_CFGR2_PPRE2_MASK          (0x7 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE2(n)            ((n) << RCC_CFGR2_PPRE2_SHIFT)
#  define RCC_CFGR2_PPRE2_HCLK        RCC_CFGR2_PPRE2(0)
#  define RCC_CFGR2_PPRE2_DIV2        RCC_CFGR2_PPRE2(4)
#  define RCC_CFGR2_PPRE2_DIV4        RCC_CFGR2_PPRE2(5)
#  define RCC_CFGR2_PPRE2_DIV8        RCC_CFGR2_PPRE2(6)
#  define RCC_CFGR2_PPRE2_DIV16       RCC_CFGR2_PPRE2(7)

/* RCC clock configuration register 3 */

#define RCC_CFGR3_RESET               0x00000000
#define RCC_CFGR3_PPRE3_SHIFT         (4)
#define RCC_CFGR3_PPRE3_MASK          (0x7 << RCC_CFGR3_PPRE3_SHIFT)
#define RCC_CFGR3_PPRE3(n)            ((n) << RCC_CFGR3_PPRE3_SHIFT)
#  define RCC_CFGR3_PPRE3_HCLK        RCC_CFGR3_PPRE3(0)
#  define RCC_CFGR3_PPRE3_DIV2        RCC_CFGR3_PPRE3(4)
#  define RCC_CFGR3_PPRE3_DIV4        RCC_CFGR3_PPRE3(5)
#  define RCC_CFGR3_PPRE3_DIV8        RCC_CFGR3_PPRE3(6)
#  define RCC_CFGR3_PPRE3_DIV16       RCC_CFGR3_PPRE3(7)

/* RCC clock configuration register 4 */

#define RCC_CFGR4_RESET               0x00000000
#define RCC_CFGR4_BOOSTSEL_SHIFT      (0)
#define RCC_CFGR4_BOOSTSEL_MASK       (0x3 << RCC_CFGR4_BOOSTSEL_SHIFT)
#define RCC_CFGR4_BOOSTSEL(n)         ((n) << RCC_CFGR4_BOOSTSEL_SHIFT)
#  define RCC_CFGR4_BOOSTSEL_NONE     RCC_CFGR4_BOOSTSEL(0)
#  define RCC_CFGR4_BOOSTSEL_MSIS     RCC_CFGR4_BOOSTSEL(1)
#  define RCC_CFGR4_BOOSTSEL_HSI      RCC_CFGR4_BOOSTSEL(2)
#  define RCC_CFGR4_BOOSTSEL_HSE      RCC_CFGR4_BOOSTSEL(3)
#define RCC_CFGR4_BOOSTDIV_SHIFT      (12)
#define RCC_CFGR4_BOOSTDIV_MASK       (0xf << RCC_CFGR4_BOOSTDIV_SHIFT)
#define RCC_CFGR4_BOOSTDIV(n)         ((n) << RCC_CFGR4_BOOSTDIV_SHIFT)
#  define RCC_CFGR4_BOOSTDIV_DIV1     RCC_CFGR4_BOOSTDIV(0)
#  define RCC_CFGR4_BOOSTDIV_DIV2     RCC_CFGR4_BOOSTDIV(1)
#  define RCC_CFGR4_BOOSTDIV_DIV4     RCC_CFGR4_BOOSTDIV(2)
#  define RCC_CFGR4_BOOSTDIV_DIV6     RCC_CFGR4_BOOSTDIV(3)
#  define RCC_CFGR4_BOOSTDIV_DIV8     RCC_CFGR4_BOOSTDIV(4)
#  define RCC_CFGR4_BOOSTDIV_DIV10    RCC_CFGR4_BOOSTDIV(5)
#  define RCC_CFGR4_BOOSTDIV_DIV12    RCC_CFGR4_BOOSTDIV(6)
#  define RCC_CFGR4_BOOSTDIV_DIV14    RCC_CFGR4_BOOSTDIV(7)
#  define RCC_CFGR4_BOOSTDIV_DIV16    RCC_CFGR4_BOOSTDIV(8)

/* RCC clock interrupt enable register */

#define RCC_CIER_RESET                0x00000000
#define RCC_CIER_LSIRDYIE             (1 << 0)
#define RCC_CIER_LSERDYIE             (1 << 1)
#define RCC_CIER_MSISRDYIE            (1 << 2)
#define RCC_CIER_HSIRDYIE             (1 << 3)
#define RCC_CIER_HSERDYIE             (1 << 4)
#define RCC_CIER_HSI48RDYIE           (1 << 5)
#define RCC_CIER_MSIPLL1RDYIE         (1 << 6)
#define RCC_CIER_MSIPLL0RDYIE         (1 << 7)
#define RCC_CIER_MSIPLLUIE            (1 << 8)
#define RCC_CIER_MSIPLLHSUIE          (1 << 9)
#define RCC_CIER_MSIKRDYIE            (1 << 11)
#define RCC_CIER_LSECSSIE             (1 << 12)

/* RCC clock interrupt flag register */

#define RCC_CIFR_RESET                0x00000000
#define RCC_CIFR_LSIRDYF              (1 << 0)
#define RCC_CIFR_LSERDYF              (1 << 1)
#define RCC_CIFR_MSISRDYF             (1 << 2)
#define RCC_CIFR_HSIRDYF              (1 << 3)
#define RCC_CIFR_HSERDYF              (1 << 4)
#define RCC_CIFR_HSI48RDYF            (1 << 5)
#define RCC_CIFR_MSIPLL1RDYF          (1 << 6)
#define RCC_CIFR_MSIPLL0RDYF          (1 << 7)
#define RCC_CIFR_MSIPLLUF             (1 << 8)
#define RCC_CIFR_MSIPLLHSUF           (1 << 9)
#define RCC_CIFR_CSSF                 (1 << 10)
#define RCC_CIFR_MSIKRDYF             (1 << 11)
#define RCC_CIFR_LSECSSF              (1 << 12)

/* RCC clock interrupt clear register */

#define RCC_CICR_RESET                0x00000000
#define RCC_CICR_LSIRDYC              (1 << 0)
#define RCC_CICR_LSERDYC              (1 << 1)
#define RCC_CICR_MSISRDYC             (1 << 2)
#define RCC_CICR_HSIRDYC              (1 << 3)
#define RCC_CICR_HSERDYC              (1 << 4)
#define RCC_CICR_HSI48RDYC            (1 << 5)
#define RCC_CICR_MSIPLL1RDYC          (1 << 6)
#define RCC_CICR_MSIPLL0RDYC          (1 << 7)
#define RCC_CICR_MSIPLLUC             (1 << 8)
#define RCC_CICR_MSIPLLHSUC           (1 << 9)
#define RCC_CICR_CSSC                 (1 << 10)
#define RCC_CICR_MSIKRDYC             (1 << 11)
#define RCC_CICR_LSECSSC              (1 << 12)

/* RCC AHB1 peripheral reset register 1 */

#define RCC_AHB1RSTR1_RESET           0x00000000
#define RCC_AHB1RSTR1_GPDMA1RST       (1 << 0)
#define RCC_AHB1RSTR1_ADF1RST         (1 << 3)
#define RCC_AHB1RSTR1_HSP1RST         (1 << 4)
#define RCC_AHB1RSTR1_CRCRST          (1 << 12)
#define RCC_AHB1RSTR1_TSCRST          (1 << 16)
#define RCC_AHB1RSTR1_RAMCFGRST       (1 << 17)

/* RCC AHB2 peripheral reset register 1 */

#define RCC_AHB2RSTR1_RESET           0x00000000
#define RCC_AHB2RSTR1_GPIOARST        (1 << 0)
#define RCC_AHB2RSTR1_GPIOBRST        (1 << 1)
#define RCC_AHB2RSTR1_GPIOCRST        (1 << 2)
#define RCC_AHB2RSTR1_GPIODRST        (1 << 3)
#define RCC_AHB2RSTR1_GPIOERST        (1 << 4)
#define RCC_AHB2RSTR1_GPIOFRST        (1 << 5)
#define RCC_AHB2RSTR1_GPIOGRST        (1 << 6)
#define RCC_AHB2RSTR1_GPIOHRST        (1 << 7)
#define RCC_AHB2RSTR1_ADC12RST        (1 << 10)
#define RCC_AHB2RSTR1_DAC1RST         (1 << 11)
#define RCC_AHB2RSTR1_AESRST          (1 << 16)
#define RCC_AHB2RSTR1_HASHRST         (1 << 17)
#define RCC_AHB2RSTR1_RNGRST          (1 << 18)
#define RCC_AHB2RSTR1_PKARST          (1 << 19)
#define RCC_AHB2RSTR1_SAESRST         (1 << 20)
#define RCC_AHB2RSTR1_CCBRST          (1 << 21)
#define RCC_AHB2RSTR1_SDMMC1RST       (1 << 27)

/* RCC AHB2 peripheral reset register 2 */

#define RCC_AHB2RSTR2_RESET           0x00000000
#define RCC_AHB2RSTR2_OCTOSPI1RST     (1 << 4)

/* RCC APB1 peripheral reset register 1 */

#define RCC_APB1RSTR1_RESET           0x00000000
#define RCC_APB1RSTR1_TIM2RST         (1 << 0)
#define RCC_APB1RSTR1_TIM3RST         (1 << 1)
#define RCC_APB1RSTR1_TIM4RST         (1 << 2)
#define RCC_APB1RSTR1_TIM6RST         (1 << 4)
#define RCC_APB1RSTR1_TIM7RST         (1 << 5)
#define RCC_APB1RSTR1_SPI3RST         (1 << 8)
#define RCC_APB1RSTR1_SPI4RST         (1 << 9)
#define RCC_APB1RSTR1_SPI2RST         (1 << 14)
#define RCC_APB1RSTR1_USART2RST       (1 << 17)
#define RCC_APB1RSTR1_USART3RST       (1 << 18)
#define RCC_APB1RSTR1_UART4RST        (1 << 19)
#define RCC_APB1RSTR1_UART5RST        (1 << 20)
#define RCC_APB1RSTR1_I2C1RST         (1 << 21)
#define RCC_APB1RSTR1_I2C2RST         (1 << 22)
#define RCC_APB1RSTR1_I3C1RST         (1 << 23)
#define RCC_APB1RSTR1_CRSRST          (1 << 24)
#define RCC_APB1RSTR1_OPAMPRST        (1 << 28)
#define RCC_APB1RSTR1_VREFRST         (1 << 29)

/* RCC APB1 peripheral reset register 2 */

#define RCC_APB1RSTR2_RESET           0x00000000
#define RCC_APB1RSTR2_I2C4RST         (1 << 1)
#define RCC_APB1RSTR2_LPTIM2RST       (1 << 5)
#define RCC_APB1RSTR2_FDCANRST        (1 << 9)

/* RCC APB2 peripheral reset register */

#define RCC_APB2RSTR_RESET            0x00000000
#define RCC_APB2RSTR_TIM1RST          (1 << 11)
#define RCC_APB2RSTR_SPI1RST          (1 << 12)
#define RCC_APB2RSTR_TIM8RST          (1 << 13)
#define RCC_APB2RSTR_USART1RST        (1 << 14)
#define RCC_APB2RSTR_TIM12RST         (1 << 15)
#define RCC_APB2RSTR_TIM15RST         (1 << 16)
#define RCC_APB2RSTR_TIM16RST         (1 << 17)
#define RCC_APB2RSTR_TIM17RST         (1 << 18)
#define RCC_APB2RSTR_SAI1RST          (1 << 21)
#define RCC_APB2RSTR_USB1RST          (1 << 24)
#define RCC_APB2RSTR_I3C2RST          (1 << 27)

/* RCC APB3 peripheral reset register */

#define RCC_APB3RSTR_RESET            0x00000000
#define RCC_APB3RSTR_SYSCFGRST        (1 << 1)
#define RCC_APB3RSTR_LPUART1RST       (1 << 6)
#define RCC_APB3RSTR_I2C3RST          (1 << 7)
#define RCC_APB3RSTR_LPTIM1RST        (1 << 11)
#define RCC_APB3RSTR_LPTIM3RST        (1 << 12)
#define RCC_APB3RSTR_LPTIM4RST        (1 << 13)
#define RCC_APB3RSTR_COMPRST          (1 << 15)

/* RCC AHB1 peripheral clock enable register 1 */

#define RCC_AHB1ENR1_RESET            0xc0000100
#define RCC_AHB1ENR1_GPDMA1EN         (1 << 0)
#define RCC_AHB1ENR1_ADF1EN           (1 << 3)
#define RCC_AHB1ENR1_HSP1EN           (1 << 4)
#define RCC_AHB1ENR1_FLASHEN          (1 << 8)
#define RCC_AHB1ENR1_CRCEN            (1 << 12)
#define RCC_AHB1ENR1_TSCEN            (1 << 16)
#define RCC_AHB1ENR1_RAMCFGEN         (1 << 17)
#define RCC_AHB1ENR1_GTZC1EN          (1 << 24)
#define RCC_AHB1ENR1_SRAM4EN          (1 << 30)
#define RCC_AHB1ENR1_SRAM1EN          (1 << 31)

/* RCC AHB2 peripheral clock enable register 1 */

#define RCC_AHB2ENR1_RESET            0xc0000000
#define RCC_AHB2ENR1_GPIOAEN          (1 << 0)
#define RCC_AHB2ENR1_GPIOBEN          (1 << 1)
#define RCC_AHB2ENR1_GPIOCEN          (1 << 2)
#define RCC_AHB2ENR1_GPIODEN          (1 << 3)
#define RCC_AHB2ENR1_GPIOEEN          (1 << 4)
#define RCC_AHB2ENR1_GPIOFEN          (1 << 5)
#define RCC_AHB2ENR1_GPIOGEN          (1 << 6)
#define RCC_AHB2ENR1_GPIOHEN          (1 << 7)
#define RCC_AHB2ENR1_ADC12EN          (1 << 10)
#define RCC_AHB2ENR1_DAC1EN           (1 << 11)
#define RCC_AHB2ENR1_AESEN            (1 << 16)
#define RCC_AHB2ENR1_HASHEN           (1 << 17)
#define RCC_AHB2ENR1_RNGEN            (1 << 18)
#define RCC_AHB2ENR1_PKAEN            (1 << 19)
#define RCC_AHB2ENR1_SAESEN           (1 << 20)
#define RCC_AHB2ENR1_CCBEN            (1 << 21)
#define RCC_AHB2ENR1_SDMMC1EN         (1 << 27)
#define RCC_AHB2ENR1_SRAM2EN          (1 << 30)
#define RCC_AHB2ENR1_SRAM3EN          (1 << 31)

/* RCC AHB2 peripheral clock enable register 2 */

#define RCC_AHB2ENR2_RESET            0x00000000
#define RCC_AHB2ENR2_OCTOSPI1EN       (1 << 4)

/* RCC AHB1 peripheral clock enable register 2 */

#define RCC_AHB1ENR2_RESET            0x00000000
#define RCC_AHB1ENR2_PWREN            (1 << 2)

/* RCC APB1 peripheral clock enable register 1 */

#define RCC_APB1ENR1_RESET            0x00000000
#define RCC_APB1ENR1_TIM2EN           (1 << 0)
#define RCC_APB1ENR1_TIM3EN           (1 << 1)
#define RCC_APB1ENR1_TIM4EN           (1 << 2)
#define RCC_APB1ENR1_TIM6EN           (1 << 4)
#define RCC_APB1ENR1_TIM7EN           (1 << 5)
#define RCC_APB1ENR1_SPI3EN           (1 << 8)
#define RCC_APB1ENR1_SPI4EN           (1 << 9)
#define RCC_APB1ENR1_WWDGEN           (1 << 11)
#define RCC_APB1ENR1_SPI2EN           (1 << 14)
#define RCC_APB1ENR1_USART2EN         (1 << 17)
#define RCC_APB1ENR1_USART3EN         (1 << 18)
#define RCC_APB1ENR1_UART4EN          (1 << 19)
#define RCC_APB1ENR1_UART5EN          (1 << 20)
#define RCC_APB1ENR1_I2C1EN           (1 << 21)
#define RCC_APB1ENR1_I2C2EN           (1 << 22)
#define RCC_APB1ENR1_I3C1EN           (1 << 23)
#define RCC_APB1ENR1_CRSEN            (1 << 24)
#define RCC_APB1ENR1_OPAMPEN          (1 << 28)
#define RCC_APB1ENR1_VREFEN           (1 << 29)
#define RCC_APB1ENR1_RTCAPBEN         (1 << 30)

/* RCC APB1 peripheral clock enable register 2 */

#define RCC_APB1ENR2_RESET            0x00000000
#define RCC_APB1ENR2_I2C4EN           (1 << 1)
#define RCC_APB1ENR2_LPTIM2EN         (1 << 5)
#define RCC_APB1ENR2_FDCANEN          (1 << 9)

/* RCC APB2 peripheral clock enable register */

#define RCC_APB2ENR_RESET             0x00000000
#define RCC_APB2ENR_TIM1EN            (1 << 11)
#define RCC_APB2ENR_SPI1EN            (1 << 12)
#define RCC_APB2ENR_TIM8EN            (1 << 13)
#define RCC_APB2ENR_USART1EN          (1 << 14)
#define RCC_APB2ENR_TIM12EN           (1 << 15)
#define RCC_APB2ENR_TIM15EN           (1 << 16)
#define RCC_APB2ENR_TIM16EN           (1 << 17)
#define RCC_APB2ENR_TIM17EN           (1 << 18)
#define RCC_APB2ENR_SAI1EN            (1 << 21)
#define RCC_APB2ENR_USB1EN            (1 << 24)
#define RCC_APB2ENR_I3C2EN            (1 << 27)

/* RCC APB3 peripheral clock enable register */

#define RCC_APB3ENR_RESET             0x00000000
#define RCC_APB3ENR_SYSCFGEN          (1 << 1)
#define RCC_APB3ENR_LPUART1EN         (1 << 6)
#define RCC_APB3ENR_I2C3EN            (1 << 7)
#define RCC_APB3ENR_LPTIM1EN          (1 << 11)
#define RCC_APB3ENR_LPTIM3EN          (1 << 12)
#define RCC_APB3ENR_LPTIM4EN          (1 << 13)
#define RCC_APB3ENR_COMPEN            (1 << 15)

/* RCC AHB1 peripheral clock enable in Sleep mode register */

#define RCC_AHB1SLPENR1_RESET         0xffffffff
#define RCC_AHB1SLPENR1_GPDMA1SLPEN   (1 << 0)
#define RCC_AHB1SLPENR1_ADF1SLPEN     (1 << 3)
#define RCC_AHB1SLPENR1_HSP1SLPEN     (1 << 4)
#define RCC_AHB1SLPENR1_FLASHSLPEN    (1 << 8)
#define RCC_AHB1SLPENR1_CRCSLPEN      (1 << 12)
#define RCC_AHB1SLPENR1_TSCSLPEN      (1 << 16)
#define RCC_AHB1SLPENR1_RAMCFGSLPEN   (1 << 17)
#define RCC_AHB1SLPENR1_GTZC1SLPEN    (1 << 24)
#define RCC_AHB1SLPENR1_ICACHESLPEN   (1 << 29)
#define RCC_AHB1SLPENR1_SRAM4SLPEN    (1 << 30)
#define RCC_AHB1SLPENR1_SRAM1SLPEN    (1 << 31)

/* RCC AHB2 peripheral clock enable in Sleep mode register 1 */

#define RCC_AHB2SLPENR1_RESET         0xffffffff
#define RCC_AHB2SLPENR1_GPIOASLPEN    (1 << 0)
#define RCC_AHB2SLPENR1_GPIOBSLPEN    (1 << 1)
#define RCC_AHB2SLPENR1_GPIOCSLPEN    (1 << 2)
#define RCC_AHB2SLPENR1_GPIODSLPEN    (1 << 3)
#define RCC_AHB2SLPENR1_GPIOESLPEN    (1 << 4)
#define RCC_AHB2SLPENR1_GPIOFSLPEN    (1 << 5)
#define RCC_AHB2SLPENR1_GPIOGSLPEN    (1 << 6)
#define RCC_AHB2SLPENR1_GPIOHSLPEN    (1 << 7)
#define RCC_AHB2SLPENR1_ADC12SLPEN    (1 << 10)
#define RCC_AHB2SLPENR1_DAC1SLPEN     (1 << 11)
#define RCC_AHB2SLPENR1_AESSLPEN      (1 << 16)
#define RCC_AHB2SLPENR1_HASHSLPEN     (1 << 17)
#define RCC_AHB2SLPENR1_RNGSLPEN      (1 << 18)
#define RCC_AHB2SLPENR1_PKASLPEN      (1 << 19)
#define RCC_AHB2SLPENR1_SAESSLPEN     (1 << 20)
#define RCC_AHB2SLPENR1_CCBSLPEN      (1 << 21)
#define RCC_AHB2SLPENR1_SDMMC1SLPEN   (1 << 27)
#define RCC_AHB2SLPENR1_SRAM2SLPEN    (1 << 30)
#define RCC_AHB2SLPENR1_SRAM3SLPEN    (1 << 31)

/* RCC AHB2 peripheral clock enable in Sleep mode register 2 */

#define RCC_AHB2SLPENR2_RESET         0xffffffff
#define RCC_AHB2SLPENR2_OCTOSPI1SLPEN (1 << 4)

/* RCC AHB1 peripheral clock enable in Sleep mode register 2 */

#define RCC_AHB1SLPENR2_RESET         0xffffffff
#define RCC_AHB1SLPENR2_PWRSLPEN      (1 << 2)

/* RCC APB1 peripheral clock enable in Sleep mode register 1 */

#define RCC_APB1SLPENR1_RESET         0xffffffff
#define RCC_APB1SLPENR1_TIM2SLPEN     (1 << 0)
#define RCC_APB1SLPENR1_TIM3SLPEN     (1 << 1)
#define RCC_APB1SLPENR1_TIM4SLPEN     (1 << 2)
#define RCC_APB1SLPENR1_TIM6SLPEN     (1 << 4)
#define RCC_APB1SLPENR1_TIM7SLPEN     (1 << 5)
#define RCC_APB1SLPENR1_SPI3SLPEN     (1 << 8)
#define RCC_APB1SLPENR1_SPI4SLPEN     (1 << 9)
#define RCC_APB1SLPENR1_WWDGSLPEN     (1 << 11)
#define RCC_APB1SLPENR1_SPI2SLPEN     (1 << 14)
#define RCC_APB1SLPENR1_USART2SLPEN   (1 << 17)
#define RCC_APB1SLPENR1_USART3SLPEN   (1 << 18)
#define RCC_APB1SLPENR1_UART4SLPEN    (1 << 19)
#define RCC_APB1SLPENR1_UART5SLPEN    (1 << 20)
#define RCC_APB1SLPENR1_I2C1SLPEN     (1 << 21)
#define RCC_APB1SLPENR1_I2C2SLPEN     (1 << 22)
#define RCC_APB1SLPENR1_I3C1SLPEN     (1 << 23)
#define RCC_APB1SLPENR1_CRSSLPEN      (1 << 24)
#define RCC_APB1SLPENR1_OPAMPSLPEN    (1 << 28)
#define RCC_APB1SLPENR1_VREFSLPEN     (1 << 29)
#define RCC_APB1SLPENR1_RTCAPBSLPEN   (1 << 30)

/* RCC APB1 peripheral clock enable in Sleep mode register 2 */

#define RCC_APB1SLPENR2_RESET         0xffffffff
#define RCC_APB1SLPENR2_I2C4SLPEN     (1 << 1)
#define RCC_APB1SLPENR2_LPTIM2SLPEN   (1 << 5)
#define RCC_APB1SLPENR2_FDCANSLPEN    (1 << 9)

/* RCC APB2 peripheral clock enable in Sleep mode register */

#define RCC_APB2SLPENR_RESET          0xffffffff
#define RCC_APB2SLPENR_TIM1SLPEN      (1 << 11)
#define RCC_APB2SLPENR_SPI1SLPEN      (1 << 12)
#define RCC_APB2SLPENR_TIM8SLPEN      (1 << 13)
#define RCC_APB2SLPENR_USART1SLPEN    (1 << 14)
#define RCC_APB2SLPENR_TIM12SLPEN     (1 << 15)
#define RCC_APB2SLPENR_TIM15SLPEN     (1 << 16)
#define RCC_APB2SLPENR_TIM16SLPEN     (1 << 17)
#define RCC_APB2SLPENR_TIM17SLPEN     (1 << 18)
#define RCC_APB2SLPENR_SAI1SLPEN      (1 << 21)
#define RCC_APB2SLPENR_USB1SLPEN      (1 << 24)
#define RCC_APB2SLPENR_I3C2SLPEN      (1 << 27)

/* RCC APB3 peripheral clock enable in Sleep mode register */

#define RCC_APB3SLPENR_RESET          0xffffffff
#define RCC_APB3SLPENR_SYSCFGSLPEN    (1 << 1)
#define RCC_APB3SLPENR_LPUART1SLPEN   (1 << 6)
#define RCC_APB3SLPENR_I2C3SLPEN      (1 << 7)
#define RCC_APB3SLPENR_LPTIM1SLPEN    (1 << 11)
#define RCC_APB3SLPENR_LPTIM3SLPEN    (1 << 12)
#define RCC_APB3SLPENR_LPTIM4SLPEN    (1 << 13)
#define RCC_APB3SLPENR_COMPSLPEN      (1 << 15)

/* RCC AHB1 peripheral clock enable in Stop mode register */

#define RCC_AHB1STPENR1_RESET         0xffffffff
#define RCC_AHB1STPENR1_GPDMA1STPEN   (1 << 0)
#define RCC_AHB1STPENR1_ADF1STPEN     (1 << 3)
#define RCC_AHB1STPENR1_FLASHSTPEN    (1 << 8)
#define RCC_AHB1STPENR1_RAMCFGSTPEN   (1 << 17)
#define RCC_AHB1STPENR1_GTZC1STPEN    (1 << 24)
#define RCC_AHB1STPENR1_SRAM4STPEN    (1 << 30)
#define RCC_AHB1STPENR1_SRAM1STPEN    (1 << 31)

/* RCC AHB2 peripheral clock enable in Stop mode register 1 */

#define RCC_AHB2STPENR1_RESET         0xffffffff
#define RCC_AHB2STPENR1_GPIOASTPEN    (1 << 0)
#define RCC_AHB2STPENR1_GPIOBSTPEN    (1 << 1)
#define RCC_AHB2STPENR1_GPIOCSTPEN    (1 << 2)
#define RCC_AHB2STPENR1_GPIODSTPEN    (1 << 3)
#define RCC_AHB2STPENR1_GPIOESTPEN    (1 << 4)
#define RCC_AHB2STPENR1_GPIOFSTPEN    (1 << 5)
#define RCC_AHB2STPENR1_GPIOGSTPEN    (1 << 6)
#define RCC_AHB2STPENR1_GPIOHSTPEN    (1 << 7)
#define RCC_AHB2STPENR1_DAC1STPEN     (1 << 11)
#define RCC_AHB2STPENR1_SRAM2STPEN    (1 << 30)
#define RCC_AHB2STPENR1_SRAM3STPEN    (1 << 31)

/* RCC APB1 peripheral clock enable in Stop mode register 1 */

#define RCC_APB1STPENR1_RESET         0xffffffff
#define RCC_APB1STPENR1_SPI3STPEN     (1 << 8)
#define RCC_APB1STPENR1_SPI4STPEN     (1 << 9)
#define RCC_APB1STPENR1_SPI2STPEN     (1 << 14)
#define RCC_APB1STPENR1_USART2STPEN   (1 << 17)
#define RCC_APB1STPENR1_USART3STPEN   (1 << 18)
#define RCC_APB1STPENR1_UART4STPEN    (1 << 19)
#define RCC_APB1STPENR1_UART5STPEN    (1 << 20)
#define RCC_APB1STPENR1_I2C1STPEN     (1 << 21)
#define RCC_APB1STPENR1_I2C2STPEN     (1 << 22)
#define RCC_APB1STPENR1_I3C1STPEN     (1 << 23)
#define RCC_APB1STPENR1_OPAMPSTPEN    (1 << 28)
#define RCC_APB1STPENR1_VREFSTPEN     (1 << 29)
#define RCC_APB1STPENR1_RTCAPBSTPEN   (1 << 30)

/* RCC APB1 peripheral clock enable in Stop mode register 2 */

#define RCC_APB1STPENR2_RESET         0xffffffff
#define RCC_APB1STPENR2_I2C4STPEN     (1 << 1)
#define RCC_APB1STPENR2_LPTIM2STPEN   (1 << 5)

/* RCC APB2 peripheral clock enable in Stop mode register */

#define RCC_APB2STPENR_RESET          0xffffffff
#define RCC_APB2STPENR_SPI1STPEN      (1 << 12)
#define RCC_APB2STPENR_USART1STPEN    (1 << 14)
#define RCC_APB2STPENR_USB1STPEN      (1 << 24)
#define RCC_APB2STPENR_I3C2STPEN      (1 << 27)

/* RCC APB3 peripheral clock enable in Stop mode register */

#define RCC_APB3STPENR_RESET          0xffffffff
#define RCC_APB3STPENR_LPUART1STPEN   (1 << 6)
#define RCC_APB3STPENR_I2C3STPEN      (1 << 7)
#define RCC_APB3STPENR_LPTIM1STPEN    (1 << 11)
#define RCC_APB3STPENR_LPTIM3STPEN    (1 << 12)
#define RCC_APB3STPENR_LPTIM4STPEN    (1 << 13)
#define RCC_APB3STPENR_COMPSTPEN      (1 << 15)

/* RCC peripheral independent clock configuration register 1 */

#define RCC_CCIPR1_RESET              0x00000000
#define RCC_CCIPR1_USART1SEL          (1 << 0)
#  define RCC_CCIPR1_USART1SEL_PCLK2  (0)
#  define RCC_CCIPR1_USART1SEL_HSI    RCC_CCIPR1_USART1SEL
#define RCC_CCIPR1_USART3SEL          (1 << 2)
#  define RCC_CCIPR1_USART3SEL_PCLK1  (0)
#  define RCC_CCIPR1_USART3SEL_HSI    RCC_CCIPR1_USART3SEL
#define RCC_CCIPR1_UART4SEL           (1 << 4)
#  define RCC_CCIPR1_UART4SEL_PCLK1   (0)
#  define RCC_CCIPR1_UART4SEL_HSI     RCC_CCIPR1_UART4SEL
#define RCC_CCIPR1_UART5SEL           (1 << 6)
#  define RCC_CCIPR1_UART5SEL_PCLK1   (0)
#  define RCC_CCIPR1_UART5SEL_HSI     RCC_CCIPR1_UART5SEL
#define RCC_CCIPR1_I3C1SEL            (1 << 8)
#  define RCC_CCIPR1_I3C1SEL_PCLK1    (0)
#  define RCC_CCIPR1_I3C1SEL_MSIK     RCC_CCIPR1_I3C1SEL
#define RCC_CCIPR1_I2C1SEL            (1 << 10)
#  define RCC_CCIPR1_I2C1SEL_PCLK1    (0)
#  define RCC_CCIPR1_I2C1SEL_MSIK     RCC_CCIPR1_I2C1SEL
#define RCC_CCIPR1_I2C2SEL            (1 << 12)
#  define RCC_CCIPR1_I2C2SEL_PCLK1    (0)
#  define RCC_CCIPR1_I2C2SEL_MSIK     RCC_CCIPR1_I2C2SEL
#define RCC_CCIPR1_I3C2SEL            (1 << 14)
#  define RCC_CCIPR1_I3C2SEL_PCLK2    (0)
#  define RCC_CCIPR1_I3C2SEL_MSIK     RCC_CCIPR1_I3C2SEL
#define RCC_CCIPR1_SPI2SEL            (1 << 16)
#  define RCC_CCIPR1_SPI2SEL_PCLK1    (0)
#  define RCC_CCIPR1_SPI2SEL_MSIK     RCC_CCIPR1_SPI2SEL
#define RCC_CCIPR1_LPTIM2SEL_SHIFT    (18)
#define RCC_CCIPR1_LPTIM2SEL_MASK     (0x3 << RCC_CCIPR1_LPTIM2SEL_SHIFT)
#define RCC_CCIPR1_LPTIM2SEL(n)       ((n) << RCC_CCIPR1_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR1_LPTIM2SEL_PCLK1  RCC_CCIPR1_LPTIM2SEL(0)
#  define RCC_CCIPR1_LPTIM2SEL_LSI    RCC_CCIPR1_LPTIM2SEL(1)
#  define RCC_CCIPR1_LPTIM2SEL_HSI    RCC_CCIPR1_LPTIM2SEL(2)
#  define RCC_CCIPR1_LPTIM2SEL_LSE    RCC_CCIPR1_LPTIM2SEL(3)
#define RCC_CCIPR1_SPI1SEL            (1 << 20)
#  define RCC_CCIPR1_SPI1SEL_PCLK2    (0)
#  define RCC_CCIPR1_SPI1SEL_MSIK     RCC_CCIPR1_SPI1SEL
#define RCC_CCIPR1_SYSTICKSEL_SHIFT   (22)
#define RCC_CCIPR1_SYSTICKSEL_MASK    (0x3 << RCC_CCIPR1_SYSTICKSEL_SHIFT)
#define RCC_CCIPR1_SYSTICKSEL(n)      ((n) << RCC_CCIPR1_SYSTICKSEL_SHIFT)
#  define RCC_CCIPR1_SYSTICKSEL_HCLK8 RCC_CCIPR1_SYSTICKSEL(0)
#  define RCC_CCIPR1_SYSTICKSEL_LSI   RCC_CCIPR1_SYSTICKSEL(1)
#  define RCC_CCIPR1_SYSTICKSEL_LSE   RCC_CCIPR1_SYSTICKSEL(2)
#define RCC_CCIPR1_FDCANSEL           (1 << 24)
#  define RCC_CCIPR1_FDCANSEL_SYSCLK  (0)
#  define RCC_CCIPR1_FDCANSEL_MSIK    RCC_CCIPR1_FDCANSEL
#define RCC_CCIPR1_ICLKSEL_SHIFT      (26)
#define RCC_CCIPR1_ICLKSEL_MASK       (0x3 << RCC_CCIPR1_ICLKSEL_SHIFT)
#define RCC_CCIPR1_ICLKSEL(n)         ((n) << RCC_CCIPR1_ICLKSEL_SHIFT)
#  define RCC_CCIPR1_ICLKSEL_HSI48    RCC_CCIPR1_ICLKSEL(0)
#  define RCC_CCIPR1_ICLKSEL_MSIK     RCC_CCIPR1_ICLKSEL(1)
#  define RCC_CCIPR1_ICLKSEL_HSE      RCC_CCIPR1_ICLKSEL(2)
#  define RCC_CCIPR1_ICLKSEL_SYSCLK   RCC_CCIPR1_ICLKSEL(3)
#define RCC_CCIPR1_USB1SEL            (1 << 28)
#  define RCC_CCIPR1_USB1SEL_ICLK     (0)
#  define RCC_CCIPR1_USB1SEL_ICLK_DIV2 RCC_CCIPR1_USB1SEL
#define RCC_CCIPR1_TIMICSEL_SHIFT     (29)
#define RCC_CCIPR1_TIMICSEL_MASK      (0x7 << RCC_CCIPR1_TIMICSEL_SHIFT)
#define RCC_CCIPR1_TIMICSEL(n)        ((n) << RCC_CCIPR1_TIMICSEL_SHIFT)
#  define RCC_CCIPR1_TIMICSEL_DISABLE RCC_CCIPR1_TIMICSEL(0)
#  define RCC_CCIPR1_TIMICSEL_HSI256_S1024_S4 \
    RCC_CCIPR1_TIMICSEL(4)
#  define RCC_CCIPR1_TIMICSEL_HSI256_S1024_K4 \
    RCC_CCIPR1_TIMICSEL(5)
#  define RCC_CCIPR1_TIMICSEL_HSI256_K1024_S4 \
    RCC_CCIPR1_TIMICSEL(6)
#  define RCC_CCIPR1_TIMICSEL_HSI256_K1024_K4 \
    RCC_CCIPR1_TIMICSEL(7)

/* RCC peripheral independent clock configuration register 2 */

#define RCC_CCIPR2_RESET              0x00000000
#define RCC_CCIPR2_ADF1SEL_SHIFT      (0)
#define RCC_CCIPR2_ADF1SEL_MASK       (0x3 << RCC_CCIPR2_ADF1SEL_SHIFT)
#define RCC_CCIPR2_ADF1SEL(n)         ((n) << RCC_CCIPR2_ADF1SEL_SHIFT)
#  define RCC_CCIPR2_ADF1SEL_HCLK     RCC_CCIPR2_ADF1SEL(0)
#  define RCC_CCIPR2_ADF1SEL_AUDIOCLK RCC_CCIPR2_ADF1SEL(1)
#  define RCC_CCIPR2_ADF1SEL_MSIK     RCC_CCIPR2_ADF1SEL(2)
#  define RCC_CCIPR2_ADF1SEL_SAI1     RCC_CCIPR2_ADF1SEL(3)
#define RCC_CCIPR2_SPI3SEL            (1 << 3)
#  define RCC_CCIPR2_SPI3SEL_PCLK1    (0)
#  define RCC_CCIPR2_SPI3SEL_MSIK     RCC_CCIPR2_SPI3SEL
#define RCC_CCIPR2_SAI1SEL_SHIFT      (5)
#define RCC_CCIPR2_SAI1SEL_MASK       (0x3 << RCC_CCIPR2_SAI1SEL_SHIFT)
#define RCC_CCIPR2_SAI1SEL(n)         ((n) << RCC_CCIPR2_SAI1SEL_SHIFT)
#  define RCC_CCIPR2_SAI1SEL_MSIK     RCC_CCIPR2_SAI1SEL(0)
#  define RCC_CCIPR2_SAI1SEL_AUDIOCLK RCC_CCIPR2_SAI1SEL(1)
#  define RCC_CCIPR2_SAI1SEL_HSE      RCC_CCIPR2_SAI1SEL(2)
#define RCC_CCIPR2_SPI4SEL            (1 << 7)
#  define RCC_CCIPR2_SPI4SEL_PCLK1    (0)
#  define RCC_CCIPR2_SPI4SEL_MSIK     RCC_CCIPR2_SPI4SEL
#define RCC_CCIPR2_I2C4SEL            (1 << 9)
#  define RCC_CCIPR2_I2C4SEL_PCLK1    (0)
#  define RCC_CCIPR2_I2C4SEL_MSIK     RCC_CCIPR2_I2C4SEL
#define RCC_CCIPR2_RNGSEL             (1 << 11)
#  define RCC_CCIPR2_RNGSEL_HSI48     (0)
#  define RCC_CCIPR2_RNGSEL_MSIK      RCC_CCIPR2_RNGSEL
#define RCC_CCIPR2_ADCDACPRE_SHIFT    (12)
#define RCC_CCIPR2_ADCDACPRE_MASK     (0xf << RCC_CCIPR2_ADCDACPRE_SHIFT)
#define RCC_CCIPR2_ADCDACPRE(n)       ((n) << RCC_CCIPR2_ADCDACPRE_SHIFT)
#  define RCC_CCIPR2_ADCDACPRE_DIV1   RCC_CCIPR2_ADCDACPRE(0)
#  define RCC_CCIPR2_ADCDACPRE_DIV2   RCC_CCIPR2_ADCDACPRE(1)
#  define RCC_CCIPR2_ADCDACPRE_DIV4   RCC_CCIPR2_ADCDACPRE(8)
#  define RCC_CCIPR2_ADCDACPRE_DIV8   RCC_CCIPR2_ADCDACPRE(9)
#  define RCC_CCIPR2_ADCDACPRE_DIV16  RCC_CCIPR2_ADCDACPRE(10)
#  define RCC_CCIPR2_ADCDACPRE_DIV32  RCC_CCIPR2_ADCDACPRE(11)
#  define RCC_CCIPR2_ADCDACPRE_DIV64  RCC_CCIPR2_ADCDACPRE(12)
#  define RCC_CCIPR2_ADCDACPRE_DIV128 RCC_CCIPR2_ADCDACPRE(13)
#  define RCC_CCIPR2_ADCDACPRE_DIV256 RCC_CCIPR2_ADCDACPRE(14)
#  define RCC_CCIPR2_ADCDACPRE_DIV512 RCC_CCIPR2_ADCDACPRE(15)
#define RCC_CCIPR2_ADCDACSEL_SHIFT    (16)
#define RCC_CCIPR2_ADCDACSEL_MASK     (0x3 << RCC_CCIPR2_ADCDACSEL_SHIFT)
#define RCC_CCIPR2_ADCDACSEL(n)       ((n) << RCC_CCIPR2_ADCDACSEL_SHIFT)
#  define RCC_CCIPR2_ADCDACSEL_HCLK   RCC_CCIPR2_ADCDACSEL(0)
#  define RCC_CCIPR2_ADCDACSEL_HSE    RCC_CCIPR2_ADCDACSEL(1)
#  define RCC_CCIPR2_ADCDACSEL_MSIK   RCC_CCIPR2_ADCDACSEL(2)
#define RCC_CCIPR2_DAC1SHSEL          (1 << 19)
#  define RCC_CCIPR2_DAC1SHSEL_LSE    (0)
#  define RCC_CCIPR2_DAC1SHSEL_LSI    RCC_CCIPR2_DAC1SHSEL
#define RCC_CCIPR2_OCTOSPISEL         (1 << 20)
#  define RCC_CCIPR2_OCTOSPISEL_SYSCLK (0)
#  define RCC_CCIPR2_OCTOSPISEL_MSIK  RCC_CCIPR2_OCTOSPISEL
#define RCC_CCIPR2_USART2SEL          (1 << 22)
#  define RCC_CCIPR2_USART2SEL_PCLK1  (0)
#  define RCC_CCIPR2_USART2SEL_HSI    RCC_CCIPR2_USART2SEL

/* RCC peripheral independent clock configuration register 3 */

#define RCC_CCIPR3_RESET              0x00000000
#define RCC_CCIPR3_LPUART1SEL_SHIFT   (0)
#define RCC_CCIPR3_LPUART1SEL_MASK    (0x3 << RCC_CCIPR3_LPUART1SEL_SHIFT)
#define RCC_CCIPR3_LPUART1SEL(n)      ((n) << RCC_CCIPR3_LPUART1SEL_SHIFT)
#  define RCC_CCIPR3_LPUART1SEL_PCLK3 RCC_CCIPR3_LPUART1SEL(0)
#  define RCC_CCIPR3_LPUART1SEL_HSI   RCC_CCIPR3_LPUART1SEL(1)
#  define RCC_CCIPR3_LPUART1SEL_LSE   RCC_CCIPR3_LPUART1SEL(2)
#  define RCC_CCIPR3_LPUART1SEL_MSIK  RCC_CCIPR3_LPUART1SEL(3)
#define RCC_CCIPR3_I2C3SEL            (1 << 6)
#  define RCC_CCIPR3_I2C3SEL_PCLK3    (0)
#  define RCC_CCIPR3_I2C3SEL_MSIK     RCC_CCIPR3_I2C3SEL
#define RCC_CCIPR3_LPTIM34SEL_SHIFT   (8)
#define RCC_CCIPR3_LPTIM34SEL_MASK    (0x3 << RCC_CCIPR3_LPTIM34SEL_SHIFT)
#define RCC_CCIPR3_LPTIM34SEL(n)      ((n) << RCC_CCIPR3_LPTIM34SEL_SHIFT)
#  define RCC_CCIPR3_LPTIM34SEL_MSIK  RCC_CCIPR3_LPTIM34SEL(0)
#  define RCC_CCIPR3_LPTIM34SEL_LSI   RCC_CCIPR3_LPTIM34SEL(1)
#  define RCC_CCIPR3_LPTIM34SEL_HSI   RCC_CCIPR3_LPTIM34SEL(2)
#  define RCC_CCIPR3_LPTIM34SEL_LSE   RCC_CCIPR3_LPTIM34SEL(3)
#define RCC_CCIPR3_LPTIM1SEL_SHIFT    (10)
#define RCC_CCIPR3_LPTIM1SEL_MASK     (0x3 << RCC_CCIPR3_LPTIM1SEL_SHIFT)
#define RCC_CCIPR3_LPTIM1SEL(n)       ((n) << RCC_CCIPR3_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR3_LPTIM1SEL_MSIK   RCC_CCIPR3_LPTIM1SEL(0)
#  define RCC_CCIPR3_LPTIM1SEL_LSI    RCC_CCIPR3_LPTIM1SEL(1)
#  define RCC_CCIPR3_LPTIM1SEL_HSI    RCC_CCIPR3_LPTIM1SEL(2)
#  define RCC_CCIPR3_LPTIM1SEL_LSE    RCC_CCIPR3_LPTIM1SEL(3)

/* RCC backup domain control register */

#define RCC_BDCR_RESET                0x00000000
#define RCC_BDCR_LSEON                (1 << 0)
#define RCC_BDCR_LSERDY               (1 << 1)
#define RCC_BDCR_LSEBYP               (1 << 2)
#define RCC_BDCR_LSEDRV_SHIFT         (3)
#define RCC_BDCR_LSEDRV_MASK          (0x3 << RCC_BDCR_LSEDRV_SHIFT)
#define RCC_BDCR_LSEDRV(n)            ((n) << RCC_BDCR_LSEDRV_SHIFT)
#  define RCC_BDCR_LSEDRV_LOW         RCC_BDCR_LSEDRV(0)
#  define RCC_BDCR_LSEDRV_MEDLOW      RCC_BDCR_LSEDRV(1)
#  define RCC_BDCR_LSEDRV_MEDHIGH     RCC_BDCR_LSEDRV(2)
#  define RCC_BDCR_LSEDRV_HIGH        RCC_BDCR_LSEDRV(3)
#define RCC_BDCR_LSECSSON             (1 << 5)
#define RCC_BDCR_LSECSSD              (1 << 6)
#define RCC_BDCR_LSESYSEN             (1 << 7)
#define RCC_BDCR_RTCSEL_SHIFT         (8)
#define RCC_BDCR_RTCSEL_MASK          (0x3 << RCC_BDCR_RTCSEL_SHIFT)
#define RCC_BDCR_RTCSEL(n)            ((n) << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_NONE        RCC_BDCR_RTCSEL(0)
#  define RCC_BDCR_RTCSEL_LSE         RCC_BDCR_RTCSEL(1)
#  define RCC_BDCR_RTCSEL_LSI         RCC_BDCR_RTCSEL(2)
#  define RCC_BDCR_RTCSEL_HSE_DIV32   RCC_BDCR_RTCSEL(3)
#define RCC_BDCR_LSESYSRDY            (1 << 11)
#define RCC_BDCR_LSEGFON              (1 << 12)
#define RCC_BDCR_RTCEN                (1 << 15)
#define RCC_BDCR_BDRST                (1 << 16)
#define RCC_BDCR_LSCOEN               (1 << 24)
#define RCC_BDCR_LSCOSEL              (1 << 25)
#  define RCC_BDCR_LSCOSEL_LSI        (0)
#  define RCC_BDCR_LSCOSEL_LSE        RCC_BDCR_LSCOSEL

/* RCC control/status register */

#define RCC_CSR_RESET                 0x0c005500
#define RCC_CSR_LSION                 (1 << 0)
#define RCC_CSR_LSIRDY                (1 << 1)
#define RCC_CSR_LSIPREDIV             (1 << 2)
#define RCC_CSR_MSIKDIVS_SHIFT        (8)
#define RCC_CSR_MSIKDIVS_MASK         (0x3 << RCC_CSR_MSIKDIVS_SHIFT)
#define RCC_CSR_MSIKDIVS(n)           ((n) << RCC_CSR_MSIKDIVS_SHIFT)
#  define RCC_CSR_MSIKDIVS_12MHZ      RCC_CSR_MSIKDIVS(1)
#  define RCC_CSR_MSIKDIVS_6MHZ       RCC_CSR_MSIKDIVS(2)
#  define RCC_CSR_MSIKDIVS_3MHZ       RCC_CSR_MSIKDIVS(3)
#define RCC_CSR_MSISDIVS_SHIFT        (12)
#define RCC_CSR_MSISDIVS_MASK         (0x3 << RCC_CSR_MSISDIVS_SHIFT)
#define RCC_CSR_MSISDIVS(n)           ((n) << RCC_CSR_MSISDIVS_SHIFT)
#  define RCC_CSR_MSISDIVS_12MHZ      RCC_CSR_MSISDIVS(1)
#  define RCC_CSR_MSISDIVS_6MHZ       RCC_CSR_MSISDIVS(2)
#  define RCC_CSR_MSISDIVS_3MHZ       RCC_CSR_MSISDIVS(3)
#define RCC_CSR_RMVF                  (1 << 23)
#define RCC_CSR_OBLRSTF               (1 << 25)
#define RCC_CSR_PINRSTF               (1 << 26)
#define RCC_CSR_BORRSTF               (1 << 27)
#define RCC_CSR_SFTRSTF               (1 << 28)
#define RCC_CSR_IWDGRSTF              (1 << 29)
#define RCC_CSR_WWDGRSTF              (1 << 30)
#define RCC_CSR_LPWRRSTF              (1 << 31)

/* RCC secure configuration register */

#define RCC_SECCFGR_RESET             0x00000000
#define RCC_SECCFGR_HSISEC            (1 << 0)
#define RCC_SECCFGR_HSESEC            (1 << 1)
#define RCC_SECCFGR_MSISEC            (1 << 2)
#define RCC_SECCFGR_LSISEC            (1 << 3)
#define RCC_SECCFGR_LSESEC            (1 << 4)
#define RCC_SECCFGR_SYSCLKSEC         (1 << 5)
#define RCC_SECCFGR_PRESCSEC          (1 << 6)
#define RCC_SECCFGR_BOOSTSEC          (1 << 7)
#define RCC_SECCFGR_ICLKSEC           (1 << 10)
#define RCC_SECCFGR_HSI48SEC          (1 << 11)
#define RCC_SECCFGR_RMVFSEC           (1 << 12)

/* RCC privilege configuration register */

#define RCC_PRIVCFGR_RESET            0x00000000
#define RCC_PRIVCFGR_SPRIV            (1 << 0)
#define RCC_PRIVCFGR_NSPRIV           (1 << 1)

#endif /* __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32U3XX_RCC_H */
