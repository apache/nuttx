/****************************************************************************
 * arch/arm/src/stm32u5/hardware/stm32u585xx_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32U585xx_RCC_H
#define __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32U585xx_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_STM32U5_STM32U585XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_RCC_CR_OFFSET         0x0000  /* RCC clock control register */
#define STM32_RCC_ICSCR1_OFFSET     0x0008  /* RCC internal clock sources calibration register 1 */
#define STM32_RCC_ICSCR2_OFFSET     0x000c  /* RCC internal clock sources calibration register 2 */
#define STM32_RCC_ICSCR3_OFFSET     0x0010  /* RCC internal clock sources calibration register 3 */
#define STM32_RCC_CRRCR_OFFSET      0x0014  /* RCC clock recovery RC register */
#define STM32_RCC_CFGR1_OFFSET      0x001c  /* RCC clock configuration register 1 */
#define STM32_RCC_CFGR2_OFFSET      0x0020  /* RCC clock configuration register 2 */
#define STM32_RCC_CFGR3_OFFSET      0x0024  /* RCC clock configuration register 3 */
#define STM32_RCC_PLL1CFGR_OFFSET   0x0028  /* RCC PLL1 configuration register */
#define STM32_RCC_PLL2CFGR_OFFSET   0x002c  /* RCC PLL2 configuration register */
#define STM32_RCC_PLL3CFGR_OFFSET   0x0030  /* RCC PLL3 configuration register */
#define STM32_RCC_PLL1DIVR_OFFSET   0x0034  /* RCC PLL1 dividers register */
#define STM32_RCC_PLL1FRACR_OFFSET  0x0038  /* RCC PLL1 fractional divider register */
#define STM32_RCC_PLL2DIVR_OFFSET   0x003c  /* RCC PLL2 dividers register */
#define STM32_RCC_PLL2FRACR_OFFSET  0x0040  /* RCC PLL2 fractional divider register */
#define STM32_RCC_PLL3DIVR_OFFSET   0x0044  /* RCC PLL3 dividers register */
#define STM32_RCC_PLL3FRACR_OFFSET  0x0048  /* RCC PLL3 fractional divider register */
#define STM32_RCC_CIER_OFFSET       0x0050  /* RCC clock interrupt enable register */
#define STM32_RCC_CIFR_OFFSET       0x0054  /* RCC clock interrupt flag register */
#define STM32_RCC_CICR_OFFSET       0x0058  /* RCC clock interrupt clear register */
#define STM32_RCC_AHB1RSTR_OFFSET   0x0060  /* RCC AHB1 peripheral reset register */
#define STM32_RCC_AHB2RSTR1_OFFSET  0x0064  /* RCC AHB2 peripheral reset register 1 */
#define STM32_RCC_AHB2RSTR2_OFFSET  0x0068  /* RCC AHB2 peripheral reset register 2 */
#define STM32_RCC_AHB3RSTR_OFFSET   0x006c  /* RCC AHB3 peripheral reset register */
#define STM32_RCC_APB1RSTR1_OFFSET  0x0074  /* RCC APB1 peripheral reset register 1 */
#define STM32_RCC_APB1RSTR2_OFFSET  0x0078  /* RCC APB1 peripheral reset register 2 */
#define STM32_RCC_APB2RSTR_OFFSET   0x007c  /* RCC APB2 peripheral reset register */
#define STM32_RCC_APB3RSTR_OFFSET   0x0080  /* RCC APB3 peripheral reset register */
#define STM32_RCC_AHB1ENR_OFFSET    0x0088  /* RCC AHB1 peripheral clock enable register */
#define STM32_RCC_AHB2ENR1_OFFSET   0x008c  /* RCC AHB2 peripheral clock enable register 1 */
#define STM32_RCC_AHB2ENR2_OFFSET   0x0090  /* RCC AHB2 peripheral clock enable register 2 */
#define STM32_RCC_AHB3ENR_OFFSET    0x0094  /* RCC AHB3 peripheral clock enable register */
#define STM32_RCC_APB1ENR1_OFFSET   0x009c  /* RCC APB1 peripheral clock enable register 1 */
#define STM32_RCC_APB1ENR2_OFFSET   0x00a0  /* RCC APB1 peripheral clock enable register 2 */
#define STM32_RCC_APB2ENR_OFFSET    0x00a4  /* RCC APB2 peripheral clock enable register */
#define STM32_RCC_APB3ENR_OFFSET    0x00a8  /* RCC APB3 peripheral clock enable register */
#define STM32_RCC_AHB1SMENR_OFFSET  0x00b0  /* RCC AHB1 peripheral clocks enable in Sleep and Stop modes register */
#define STM32_RCC_AHB2SMENR1_OFFSET 0x00b4  /* RCC AHB2 peripheral clocks enable in Sleep and Stop modes register 1 */
#define STM32_RCC_AHB2SMENR2_OFFSET 0x00b8  /* RCC AHB2 peripheral clocks enable in Sleep and Stop modes register 2 */
#define STM32_RCC_AHB3SMENR_OFFSET  0x00bc  /* RCC AHB3 peripheral clocks enable in Sleep and Stop modes register */
#define STM32_RCC_APB1SMENR1_OFFSET 0x00c4  /* RCC APB1 peripheral clocks enable in Sleep and Stop modes register 1 */
#define STM32_RCC_APB1SMENR2_OFFSET 0x00c8  /* RCC APB1 peripheral clocks enable in Sleep and Stop modes register 2 */
#define STM32_RCC_APB2SMENR_OFFSET  0x00cc  /* RCC APB2 peripheral clocks enable in Sleep and Stop modes register */
#define STM32_RCC_APB3SMENR_OFFSET  0x00d0  /* RCC APB3 peripheral clocks enable in Sleep and Stop modes register */
#define STM32_RCC_SRDAMR_OFFSET     0x00d8  /* RCC SmartRun domain peripheral autonomous mode register */
#define STM32_RCC_CCIPR1_OFFSET     0x00e0  /* RCC peripherals independent clock configuration register 1 */
#define STM32_RCC_CCIPR2_OFFSET     0x00e4  /* RCC peripherals independent clock configuration register 2 */
#define STM32_RCC_CCIPR3_OFFSET     0x00e8  /* RCC peripherals independent clock configuration register 3 */
#define STM32_RCC_BDCR_OFFSET       0x00f0  /* RCC Backup domain control register */
#define STM32_RCC_CSR_OFFSET        0x00f4  /* RCC control/status register */
#define STM32_RCC_SECCFGR_OFFSET    0x0110  /* RCC secure configuration register */
#define STM32_RCC_PRIVCFGR_OFFSET   0x0114  /* RCC privilege configuration register */

/* Register Addresses *******************************************************/

#define STM32_RCC_CR         (STM32_RCC_BASE + STM32_RCC_CR_OFFSET)
#define STM32_RCC_ICSCR1     (STM32_RCC_BASE + STM32_RCC_ICSCR1_OFFSET)
#define STM32_RCC_ICSCR2     (STM32_RCC_BASE + STM32_RCC_ICSCR2_OFFSET)
#define STM32_RCC_ICSCR3     (STM32_RCC_BASE + STM32_RCC_ICSCR3_OFFSET)
#define STM32_RCC_CRRCR      (STM32_RCC_BASE + STM32_RCC_CRRCR_OFFSET)
#define STM32_RCC_CFGR1      (STM32_RCC_BASE + STM32_RCC_CFGR1_OFFSET)
#define STM32_RCC_CFGR2      (STM32_RCC_BASE + STM32_RCC_CFGR2_OFFSET)
#define STM32_RCC_CFGR3      (STM32_RCC_BASE + STM32_RCC_CFGR3_OFFSET)
#define STM32_RCC_PLL1CFGR   (STM32_RCC_BASE + STM32_RCC_PLL1CFGR_OFFSET)
#define STM32_RCC_PLL2CFGR   (STM32_RCC_BASE + STM32_RCC_PLL2CFGR_OFFSET)
#define STM32_RCC_PLL3CFGR   (STM32_RCC_BASE + STM32_RCC_PLL3CFGR_OFFSET)
#define STM32_RCC_PLL1DIVR   (STM32_RCC_BASE + STM32_RCC_PLL1DIVR_OFFSET)
#define STM32_RCC_PLL1FRACR  (STM32_RCC_BASE + STM32_RCC_PLL1FRACR_OFFSET)
#define STM32_RCC_PLL2DIVR   (STM32_RCC_BASE + STM32_RCC_PLL2DIVR_OFFSET)
#define STM32_RCC_PLL2FRACR  (STM32_RCC_BASE + STM32_RCC_PLL2FRACR_OFFSET)
#define STM32_RCC_PLL3DIVR   (STM32_RCC_BASE + STM32_RCC_PLL3DIVR_OFFSET)
#define STM32_RCC_PLL3FRACR  (STM32_RCC_BASE + STM32_RCC_PLL3FRACR_OFFSET)
#define STM32_RCC_CIER       (STM32_RCC_BASE + STM32_RCC_CIER_OFFSET)
#define STM32_RCC_CIFR       (STM32_RCC_BASE + STM32_RCC_CIFR_OFFSET)
#define STM32_RCC_CICR       (STM32_RCC_BASE + STM32_RCC_CICR_OFFSET)
#define STM32_RCC_AHB1RSTR   (STM32_RCC_BASE + STM32_RCC_AHB1RSTR_OFFSET)
#define STM32_RCC_AHB2RSTR1  (STM32_RCC_BASE + STM32_RCC_AHB2RSTR1_OFFSET)
#define STM32_RCC_AHB2RSTR2  (STM32_RCC_BASE + STM32_RCC_AHB2RSTR2_OFFSET)
#define STM32_RCC_AHB3RSTR   (STM32_RCC_BASE + STM32_RCC_AHB3RSTR_OFFSET)
#define STM32_RCC_APB1RSTR1  (STM32_RCC_BASE + STM32_RCC_APB1RSTR1_OFFSET)
#define STM32_RCC_APB1RSTR2  (STM32_RCC_BASE + STM32_RCC_APB1RSTR2_OFFSET)
#define STM32_RCC_APB2RSTR   (STM32_RCC_BASE + STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_APB3RSTR   (STM32_RCC_BASE + STM32_RCC_APB3RSTR_OFFSET)
#define STM32_RCC_AHB1ENR    (STM32_RCC_BASE + STM32_RCC_AHB1ENR_OFFSET)
#define STM32_RCC_AHB2ENR1   (STM32_RCC_BASE + STM32_RCC_AHB2ENR1_OFFSET)
#define STM32_RCC_AHB2ENR2   (STM32_RCC_BASE + STM32_RCC_AHB2ENR2_OFFSET)
#define STM32_RCC_AHB3ENR    (STM32_RCC_BASE + STM32_RCC_AHB3ENR_OFFSET)
#define STM32_RCC_APB1ENR1   (STM32_RCC_BASE + STM32_RCC_APB1ENR1_OFFSET)
#define STM32_RCC_APB1ENR2   (STM32_RCC_BASE + STM32_RCC_APB1ENR2_OFFSET)
#define STM32_RCC_APB2ENR    (STM32_RCC_BASE + STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_APB3ENR    (STM32_RCC_BASE + STM32_RCC_APB3ENR_OFFSET)
#define STM32_RCC_AHB1SMENR  (STM32_RCC_BASE + STM32_RCC_AHB1SMENR_OFFSET)
#define STM32_RCC_AHB2SMENR1 (STM32_RCC_BASE + STM32_RCC_AHB2SMENR1_OFFSET)
#define STM32_RCC_AHB2SMENR2 (STM32_RCC_BASE + STM32_RCC_AHB2SMENR2_OFFSET)
#define STM32_RCC_AHB3SMENR  (STM32_RCC_BASE + STM32_RCC_AHB3SMENR_OFFSET)
#define STM32_RCC_APB1SMENR1 (STM32_RCC_BASE + STM32_RCC_APB1SMENR1_OFFSET)
#define STM32_RCC_APB1SMENR2 (STM32_RCC_BASE + STM32_RCC_APB1SMENR2_OFFSET)
#define STM32_RCC_APB2SMENR  (STM32_RCC_BASE + STM32_RCC_APB2SMENR_OFFSET)
#define STM32_RCC_APB3SMENR  (STM32_RCC_BASE + STM32_RCC_APB3SMENR_OFFSET)
#define STM32_RCC_SRDAMR     (STM32_RCC_BASE + STM32_RCC_SRDAMR_OFFSET)
#define STM32_RCC_CCIPR1     (STM32_RCC_BASE + STM32_RCC_CCIPR1_OFFSET)
#define STM32_RCC_CCIPR2     (STM32_RCC_BASE + STM32_RCC_CCIPR2_OFFSET)
#define STM32_RCC_CCIPR3     (STM32_RCC_BASE + STM32_RCC_CCIPR3_OFFSET)
#define STM32_RCC_BDCR       (STM32_RCC_BASE + STM32_RCC_BDCR_OFFSET)
#define STM32_RCC_CSR        (STM32_RCC_BASE + STM32_RCC_CSR_OFFSET)
#define STM32_RCC_SECCFGR    (STM32_RCC_BASE + STM32_RCC_SECCFGR_OFFSET)
#define STM32_RCC_PRIVCFGR   (STM32_RCC_BASE + STM32_RCC_PRIVCFGR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* RCC clock control register */

#define RCC_CR_MSISON                     (1 << 0)  /* Bit 0: MSIS clock enable */
#define RCC_CR_MSIKERON                   (1 << 1)  /* Bit 1: MSI enable for some peripheral kernels */
#define RCC_CR_MSISRDY                    (1 << 2)  /* Bit 2: MSIS clock ready flag */
#define RCC_CR_MSIPLLEN                   (1 << 3)  /* Bit 3: MSI clock PLL enable */
#define RCC_CR_MSIKON                     (1 << 4)  /* Bit 4: MSIK clock enable */
#define RCC_CR_MSIKRDY                    (1 << 5)  /* Bit 4: MSIK clock ready flag */
#define RCC_CR_MSIPLLSEL                  (1 << 6)  /* Bit 6: MSI clock with PLL mode selection */
#  define RCC_CR_MSIPLLSEL_MSIK           (0)       /* 0: PLL mode applied to MSIK clock output */
#  define RCC_CR_MSIPLLSEL_MSIS           (1 << 6)  /* 1: PLL mode applied to MSIS clock output */
#define RCC_CR_MSIPLLFAST                 (1 << 7)  /* Bit 7: MSI PLL fast start-up */
#define RCC_CR_HSION                      (1 << 8)  /* Bit 8: Internal High Speed clock enable */
#define RCC_CR_HSIKERON                   (1 << 9)  /* Bit 9: HSI16 always enable for peripheral kernels */
#define RCC_CR_HSIRDY                     (1 << 10) /* Bit 10: Internal High Speed clock ready flag */
#define RCC_CR_HSI48ON                    (1 << 12) /* Bit 12: HSI48 clock enable */
#define RCC_CR_HSI48RDY                   (1 << 13) /* Bit 13: HSI48 clock ready flag */
#define RCC_CR_SHSION                     (1 << 14) /* Bit 14: SHSI clock enable */
#define RCC_CR_SHSIRDY                    (1 << 15) /* Bit 15: SHSI clock ready flag */
#define RCC_CR_HSEON                      (1 << 16) /* Bit 16: External High Speed clock enable */
#define RCC_CR_HSERDY                     (1 << 17) /* Bit 17: External High Speed clock ready flag */
#define RCC_CR_HSEBYP                     (1 << 18) /* Bit 18: External High Speed clock Bypass */
#define RCC_CR_CSSON                      (1 << 19) /* Bit 19: Clock Security System enable */
#define RCC_HSEEXT                        (1 << 20) /* Bit 20: HSE external clock bypass mode */
#define RCC_CR_PLL1ON                     (1 << 24) /* Bit 24: PLL1 enable */
#define RCC_CR_PLL1RDY                    (1 << 25) /* Bit 25: PLL1 clock ready flag */
#define RCC_CR_PLL2ON                     (1 << 26) /* Bit 26: PLL2 enable */
#define RCC_CR_PLL2RDY                    (1 << 27) /* Bit 27: PLL2 clock ready flag */
#define RCC_CR_PLL3ON                     (1 << 28) /* Bit 28: PLL3 enable */
#define RCC_CR_PLL3RDY                    (1 << 29) /* Bit 29: PLL3 clock ready flag */

/* RCC internal clock sources calibration register 1 */

#define RCC_ICSCR1_MSICAL3_SHIFT          (0) /* Bits 0-4: MSIRC3 clock calibration for MSI ranges 12 to 15 */
#define RCC_ICSCR1_MSICAL3_MASK           (0x1f << RCC_ICSCR1_MSICAL3_SHIFT)
#define RCC_ICSCR1_MSICAL3(n)             ((n) << RCC_ICSCR1_MSICAL3_SHIFT)
#define RCC_ICSCR1_MSICAL2_SHIFT          (5) /* Bits 5-9: MSIRC2 clock calibration for MSI ranges 8 to 11 */
#define RCC_ICSCR1_MSICAL2_MASK           (0x1f << RCC_ICSCR1_MSICAL2_SHIFT)
#define RCC_ICSCR1_MSICAL2(n)             ((n) << RCC_ICSCR1_MSICAL2_SHIFT)
#define RCC_ICSCR1_MSICAL1_SHIFT          (0) /* Bits 10-14: MSIRC1 clock calibration for MSI ranges 4 to 7 */
#define RCC_ICSCR1_MSICAL1_MASK           (0x1f << RCC_ICSCR1_MSICAL1_SHIFT)
#define RCC_ICSCR1_MSICAL1(n)             ((n) << RCC_ICSCR1_MSICAL1_SHIFT)
#define RCC_ICSCR1_MSICAL0_SHIFT          (0) /* Bits 15-19: MSIRC0 clock calibration for MSI ranges 0 to 3 */
#define RCC_ICSCR1_MSICAL0_MASK           (0x1f << RCC_ICSCR1_MSICAL0_SHIFT)
#define RCC_ICSCR1_MSICAL0(n)             ((n) << RCC_ICSCR1_MSICAL0_SHIFT)
#define RCC_ICSCR1_MSIBIAS                (1 << 22) /* Bit 22: MSI bias mode selection */
#define RCC_ICSCR1_MSIRGSEL_MASK          (1 << 23) /* Bit 23: MSI clock range selection */
#define RCC_ICSCR1_MSIRGSEL_CSR           0
#define RCC_ICSCR1_MSIRGSEL_ICSCR1        RCC_ICSCR1_MSIRGSEL_MASK
#define RCC_ICSCR1_MSIKRANGE_SHIFT        (24) /* Bits 24-27: MSIK clock ranges */
#define RCC_ICSCR1_MSIKRANGE_MASK         (0xf << RCC_ICSCR1_MSIKRANGE_SHIFT)
#define RCC_ICSCR1_MSIKRANGE(n)           ((n) << RCC_ICSCR1_MSIKRANGE_SHIFT)
#define RCC_ICSCR1_MSIKRANGE_48MHZ        RCC_ICSCR1_MSIKRANGE(0x0)
#define RCC_ICSCR1_MSIKRANGE_24MHZ        RCC_ICSCR1_MSIKRANGE(0x1)
#define RCC_ICSCR1_MSIKRANGE_16MHZ        RCC_ICSCR1_MSIKRANGE(0x2)
#define RCC_ICSCR1_MSIKRANGE_12MHZ        RCC_ICSCR1_MSIKRANGE(0x3)
#define RCC_ICSCR1_MSIKRANGE_4MHZ         RCC_ICSCR1_MSIKRANGE(0x4)
#define RCC_ICSCR1_MSIKRANGE_2MHZ         RCC_ICSCR1_MSIKRANGE(0x5)
#define RCC_ICSCR1_MSIKRANGE_1330KHZ      RCC_ICSCR1_MSIKRANGE(0x6)
#define RCC_ICSCR1_MSIKRANGE_1MHZ         RCC_ICSCR1_MSIKRANGE(0x7)
#define RCC_ICSCR1_MSIKRANGE_3072KHZ      RCC_ICSCR1_MSIKRANGE(0x8)
#define RCC_ICSCR1_MSIKRANGE_1536KHZ      RCC_ICSCR1_MSIKRANGE(0x9)
#define RCC_ICSCR1_MSIKRANGE_1024KHZ      RCC_ICSCR1_MSIKRANGE(0xa)
#define RCC_ICSCR1_MSIKRANGE_768KHZ       RCC_ICSCR1_MSIKRANGE(0xb)
#define RCC_ICSCR1_MSIKRANGE_400KHZ       RCC_ICSCR1_MSIKRANGE(0xc)
#define RCC_ICSCR1_MSIKRANGE_200KHZ       RCC_ICSCR1_MSIKRANGE(0xd)
#define RCC_ICSCR1_MSIKRANGE_133KHZ       RCC_ICSCR1_MSIKRANGE(0xe)
#define RCC_ICSCR1_MSIKRANGE_100KHZ       RCC_ICSCR1_MSIKRANGE(0xf)
#define RCC_ICSCR1_MSISRANGE_SHIFT        (28) /* Bits 28-31: MSIS clock ranges */
#define RCC_ICSCR1_MSISRANGE_MASK         (0xf << RCC_ICSCR1_MSISRANGE_SHIFT)
#define RCC_ICSCR1_MSISRANGE(n)           ((n) << RCC_ICSCR1_MSISRANGE_SHIFT)
#define RCC_ICSCR1_MSISRANGE_48MHZ        RCC_ICSCR1_MSISRANGE(0x0)
#define RCC_ICSCR1_MSISRANGE_24MHZ        RCC_ICSCR1_MSISRANGE(0x1)
#define RCC_ICSCR1_MSISRANGE_16MHZ        RCC_ICSCR1_MSISRANGE(0x2)
#define RCC_ICSCR1_MSISRANGE_12MHZ        RCC_ICSCR1_MSISRANGE(0x3)
#define RCC_ICSCR1_MSISRANGE_4MHZ         RCC_ICSCR1_MSISRANGE(0x4)
#define RCC_ICSCR1_MSISRANGE_2MHZ         RCC_ICSCR1_MSISRANGE(0x5)
#define RCC_ICSCR1_MSISRANGE_1330KHZ      RCC_ICSCR1_MSISRANGE(0x6)
#define RCC_ICSCR1_MSISRANGE_1MHZ         RCC_ICSCR1_MSISRANGE(0x7)
#define RCC_ICSCR1_MSISRANGE_3072KHZ      RCC_ICSCR1_MSISRANGE(0x8)
#define RCC_ICSCR1_MSISRANGE_1536KHZ      RCC_ICSCR1_MSISRANGE(0x9)
#define RCC_ICSCR1_MSISRANGE_1024KHZ      RCC_ICSCR1_MSISRANGE(0xa)
#define RCC_ICSCR1_MSISRANGE_768KHZ       RCC_ICSCR1_MSISRANGE(0xb)
#define RCC_ICSCR1_MSISRANGE_400KHZ       RCC_ICSCR1_MSISRANGE(0xc)
#define RCC_ICSCR1_MSISRANGE_200KHZ       RCC_ICSCR1_MSISRANGE(0xd)
#define RCC_ICSCR1_MSISRANGE_133KHZ       RCC_ICSCR1_MSISRANGE(0xe)
#define RCC_ICSCR1_MSISRANGE_100KHZ       RCC_ICSCR1_MSISRANGE(0xf)

/* RCC clock configuration register 1 */

#define RCC_CFGR1_SW_SHIFT                (0)       /* Bits 0-1: System clock Switch */
#define RCC_CFGR1_SW_MASK                 (3 << RCC_CFGR1_SW_SHIFT)
#  define RCC_CFGR1_SW_MSIS               (0 << RCC_CFGR1_SW_SHIFT) /* 00: MSIS selected as system clock */
#  define RCC_CFGR1_SW_HSI16              (1 << RCC_CFGR1_SW_SHIFT) /* 00: HSI16 selected as system clock */
#  define RCC_CFGR1_SW_HSE                (2 << RCC_CFGR1_SW_SHIFT) /* 01: HSE selected as system clock */
#  define RCC_CFGR1_SW_PLL                (3 << RCC_CFGR1_SW_SHIFT) /* 10: PLL pll1_r_ck selected as system clock */

#define RCC_CFGR1_SWS_SHIFT               (2)       /* Bits 2-3: System Clock Switch Status */
#define RCC_CFGR1_SWS_MASK                (3 << RCC_CFGR1_SWS_SHIFT)
#  define RCC_CFGR1_SWS_MSIS              (0 << RCC_CFGR1_SWS_SHIFT) /* 00: MSIS oscillator used as system clock */
#  define RCC_CFGR1_SWS_HSI16             (1 << RCC_CFGR1_SWS_SHIFT) /* 00: HSI16 oscillator used as system clock */
#  define RCC_CFGR1_SWS_HSE               (2 << RCC_CFGR1_SWS_SHIFT) /* 01: HSE oscillator used as system clock */
#  define RCC_CFGR1_SWS_PLL               (3 << RCC_CFGR1_SWS_SHIFT) /* 10: PLL used as system clock */

#define RCC_CFGR1_STOPWUCK                (1 << 4) /* Bit 4: Wakeup from Stop and CSS backup clock selection */
#  define RCC_CFGR1_STOPWUCK_MSIS         (0 << 4) /* 0: MSIS */
#  define RCC_CFGR1_STOPWUCK_HSI16        (1 << 4) /* 0: HSI16 */

#define RCC_CFGR_STOPKERWUCK              (1 << 5) /* Bit 5: Wakeup from Stop kernel clock automatic enable selection */
#  define RCC_CFGR1_STOPKERWUCK_MSIK      (0 << 5) /* 0: MSIK */
#  define RCC_CFGR1_STOPKERWUCK_HSI16     (1 << 5) /* 0: HSI16 */

#define RCC_CFGR1_MCOSEL_SHIFT            (24)      /* Bits 24-27: Microcontroller Clock Output */
#define RCC_CFGR1_MCOSEL_MASK             (0x0f << RCC_CFGR1_MCOSEL_SHIFT)
#  define RCC_CFGR1_MCOSEL_NONE           (0 << RCC_CFGR1_MCOSEL_SHIFT) /* 0000: Disabled */
#  define RCC_CFGR1_MCOSEL_SYSCLK         (1 << RCC_CFGR1_MCOSEL_SHIFT) /* 0001: SYSCLK system clock selected */
#  define RCC_CFGR1_MCOSEL_MSIS           (2 << RCC_CFGR1_MCOSEL_SHIFT) /* 0010: MSIS clock selected */
#  define RCC_CFGR1_MCOSEL_HSI16          (3 << RCC_CFGR1_MCOSEL_SHIFT) /* 0011: HSI16 clock selected */
#  define RCC_CFGR1_MCOSEL_HSE            (4 << RCC_CFGR1_MCOSEL_SHIFT) /* 0100: HSE clock selected */
#  define RCC_CFGR1_MCOSEL_PLL            (5 << RCC_CFGR1_MCOSEL_SHIFT) /* 0101: Main PLL clock pll1_r_ck selected  */
#  define RCC_CFGR1_MCOSEL_LSI            (6 << RCC_CFGR1_MCOSEL_SHIFT) /* 0110: LSI clock selected */
#  define RCC_CFGR1_MCOSEL_LSE            (7 << RCC_CFGR1_MCOSEL_SHIFT) /* 0111: LSE clock selected */
#  define RCC_CFGR1_MCOSEL_HSI48          (8 << RCC_CFGR1_MCOSEL_SHIFT) /* 1000: HSI48 clock selected */
#  define RCC_CFGR1_MCOSEL_MSIK           (9 << RCC_CFGR1_MCOSEL_SHIFT) /* 1001: MSIK clock selected */

#define RCC_CFGR1_MCOPRE_SHIFT            (28)      /* Bits 28-30: MCO prescaler */
#define RCC_CFGR1_MCOPRE_MASK             (7 << RCC_CFGR1_MCOPRE_SHIFT)
#  define RCC_CFGR1_MCOPRE_NONE           (0 << RCC_CFGR1_MCOPRE_SHIFT) /* 000: no division */
#  define RCC_CFGR1_MCOPRE_DIV2           (1 << RCC_CFGR1_MCOPRE_SHIFT) /* 001: division by 2 */
#  define RCC_CFGR1_MCOPRE_DIV4           (2 << RCC_CFGR1_MCOPRE_SHIFT) /* 010: division by 4 */
#  define RCC_CFGR1_MCOPRE_DIV8           (3 << RCC_CFGR1_MCOPRE_SHIFT) /* 011: division by 8 */
#  define RCC_CFGR1_MCOPRE_DIV16          (4 << RCC_CFGR1_MCOPRE_SHIFT) /* 100: division by 16 */

/* RCC clock configuration register 2 */

#define RCC_CFGR2_HPRE_SHIFT              (0)       /* Bits 0-3: AHB prescaler */
#define RCC_CFGR2_HPRE_MASK               (0xf << RCC_CFGR2_HPRE_SHIFT)
#  define RCC_CFGR2_HPRE_SYSCLK           ( 0 << RCC_CFGR2_HPRE_SHIFT) /* 0xxx: SYSCLK not divided */
#  define RCC_CFGR2_HPRE_SYSCLK_DIV2      ( 8 << RCC_CFGR2_HPRE_SHIFT) /* 1000: SYSCLK divided by 2 */
#  define RCC_CFGR2_HPRE_SYSCLK_DIV4      ( 9 << RCC_CFGR2_HPRE_SHIFT) /* 1001: SYSCLK divided by 4 */
#  define RCC_CFGR2_HPRE_SYSCLK_DIV8      (10 << RCC_CFGR2_HPRE_SHIFT) /* 1010: SYSCLK divided by 8 */
#  define RCC_CFGR2_HPRE_SYSCLK_DIV16     (11 << RCC_CFGR2_HPRE_SHIFT) /* 1011: SYSCLK divided by 16 */
#  define RCC_CFGR2_HPRE_SYSCLK_DIV64     (12 << RCC_CFGR2_HPRE_SHIFT) /* 1100: SYSCLK divided by 64 */
#  define RCC_CFGR2_HPRE_SYSCLK_DIV128    (13 << RCC_CFGR2_HPRE_SHIFT) /* 1101: SYSCLK divided by 128 */
#  define RCC_CFGR2_HPRE_SYSCLK_DIV256    (14 << RCC_CFGR2_HPRE_SHIFT) /* 1110: SYSCLK divided by 256 */
#  define RCC_CFGR2_HPRE_SYSCLK_DIV512    (15 << RCC_CFGR2_HPRE_SHIFT) /* 1111: SYSCLK divided by 512 */

#define RCC_CFGR2_PPRE1_SHIFT             (4)       /* Bits 4-6: APB1 prescaler */
#define RCC_CFGR2_PPRE1_MASK              (0x7 << RCC_CFGR2_HPRE_SHIFT)
#  define RCC_CFGR2_PPRE1_HCLK            (0 << RCC_CFGR2_HPRE_SHIFT) /* 0xxx: HCLK not divided */
#  define RCC_CFGR2_PPRE1_HCLK_DIV2       (4 << RCC_CFGR2_HPRE_SHIFT) /* 1000: HCLK divided by 2 */
#  define RCC_CFGR2_PPRE1_HCLK_DIV4       (5 << RCC_CFGR2_HPRE_SHIFT) /* 1001: HCLK divided by 4 */
#  define RCC_CFGR2_PPRE1_HCLK_DIV8       (6 << RCC_CFGR2_HPRE_SHIFT) /* 1010: HCLK divided by 8 */
#  define RCC_CFGR2_PPRE1_HCLK_DIV16      (7 << RCC_CFGR2_HPRE_SHIFT) /* 1011: HCLK divided by 16 */

#define RCC_CFGR2_PPRE2_SHIFT             (8)       /* Bits 8-10: APB2 prescaler */
#define RCC_CFGR2_PPRE2_MASK              (0x7 << RCC_CFGR2_PPRE2_SHIFT)
#  define RCC_CFGR2_PPRE2_HCLK            (0 << RCC_CFGR2_PPRE2_SHIFT) /* 0xxx: HCLK not divided */
#  define RCC_CFGR2_PPRE2_HCLK_DIV2       (4 << RCC_CFGR2_PPRE2_SHIFT) /* 1000: HCLK divided by 2 */
#  define RCC_CFGR2_PPRE2_HCLK_DIV4       (5 << RCC_CFGR2_PPRE2_SHIFT) /* 1001: HCLK divided by 4 */
#  define RCC_CFGR2_PPRE2_HCLK_DIV8       (6 << RCC_CFGR2_PPRE2_SHIFT) /* 1010: HCLK divided by 8 */
#  define RCC_CFGR2_PPRE2_HCLK_DIV16      (7 << RCC_CFGR2_PPRE2_SHIFT) /* 1011: HCLK divided by 16 */

#define RCC_CFGR2_AHB1DIS                 (1 << 16) /* Bit 16: AHB1 clock disable */
#define RCC_CFGR2_AHB2DIS1                (1 << 17) /* Bit 17: AHB2_1 clock disable */
#define RCC_CFGR2_AHB2DIS2                (1 << 18) /* Bit 18: AHB2_2 clock disable */
#define RCC_CFGR2_APB1DIS                 (1 << 19) /* Bit 19: APB1 clock disable */
#define RCC_CFGR2_APB2DIS                 (1 << 20) /* Bit 20: APB2 clock disable */

/* RCC clock configuration register 3 */

#define RCC_CFGR3_PPRE3_SHIFT             (4)       /* Bits 4-6: APB3 prescaler */
#define RCC_CFGR3_PPRE3_MASK              (0x7 << RCC_CFGR3_PPRE3_SHIFT)
#  define RCC_CFGR3_PPRE3_HCLK            (0 << RCC_CFGR3_PPRE3_SHIFT) /* 0xxx: HCLK not divided */
#  define RCC_CFGR3_PPRE3_HCLK_DIV2       (4 << RCC_CFGR3_PPRE3_SHIFT) /* 1000: HCLK divided by 2 */
#  define RCC_CFGR3_PPRE3_HCLK_DIV4       (5 << RCC_CFGR3_PPRE3_SHIFT) /* 1001: HCLK divided by 4 */
#  define RCC_CFGR3_PPRE3_HCLK_DIV8       (6 << RCC_CFGR3_PPRE3_SHIFT) /* 1010: HCLK divided by 8 */
#  define RCC_CFGR3_PPRE3_HCLK_DIV16      (7 << RCC_CFGR3_PPRE3_SHIFT) /* 1011: HCLK divided by 16 */

#define RCC_CFGR3_AHB3DIS                 (1 << 16) /* Bit 16: AHB3 clock disable */
#define RCC_CFGR3_APB3DIS                 (1 << 17) /* Bit 17: APB3 clock disable */

/* RCC PLL1 configuration register */

#define RCC_PLL1CFGR_PLL1SRC_SHIFT        (0)                               /* Bits 0-1: PLL1 entry clock source */
#define RCC_PLL1CFGR_PLL1SRC_MASK         (3 << RCC_PLL1CFGR_PLL1SRC_SHIFT)
#define RCC_PLL1CFGR_PLL1SRC_NONE         (0 << RCC_PLL1CFGR_PLL1SRC_SHIFT) /* 00: No clock send to PLL1 */
#define RCC_PLL1CFGR_PLL1SRC_MSIS         (1 << RCC_PLL1CFGR_PLL1SRC_SHIFT) /* 01: MSIS clock selected as PLL1 clock entry */
#define RCC_PLL1CFGR_PLL1SRC_HSI16        (2 << RCC_PLL1CFGR_PLL1SRC_SHIFT) /* 10: HSI16 clock selected as PLL1 clock entry */
#define RCC_PLL1CFGR_PLL1SRC_HSE          (3 << RCC_PLL1CFGR_PLL1SRC_SHIFT) /* 11: HSE clock selected as PLL1 clock entry */
#define RCC_PLL1CFGR_PLL1RGE_SHIFT        (2)                               /* Bits 2-3: PLL1 input frequency range */
#define RCC_PLL1CFGR_PLL1RGE_MASK         (3 << RCC_PLL1CFGR_PLL1RGE_SHIFT)
#define RCC_PLL1CFGR_PLL1RGE_4_TO_8MHZ    (0 << RCC_PLL1CFGR_PLL1RGE_SHIFT) /* 00-01-10: PLL1 input (ref1_ck) clock range frequency between 4 and 8 MHz */
#define RCC_PLL1CFGR_PLL1RGE_8_TO_16MHZ   (3 << RCC_PLL1CFGR_PLL1RGE_SHIFT) /* 11: PLL1 input (ref1_ck) clock range frequeny between 8 and 16 MHz */
#define RCC_PLL1CFGR_PLL1FRACEN           (1 << 4)                          /* Bit 4: PLL1 fractional latch enable */
#define RCC_PLL1CFGR_PLL1M_SHIFT          (8)                               /* Bits 8-11: Prescaler for PLL1 */
#define RCC_PLL1CFGR_PLL1M_MASK           (0xf << RCC_PLL1CFGR_PLL1M_SHIFT)
#define RCC_PLL1CFGR_PLL1M(n)             (((n) - 1) << RCC_PLL1CFGR_PLL1M_SHIFT)
#define RCC_PLL1CFGR_PLL1MBOOST_SHIFT     (12)                              /* Bits 12-15: Prescaler for EPOD booster input clock */
#define RCC_PLL1CFGR_PLL1MBOOST_MASK      (0xf << RCC_PLL1CFGR_PLL1MBOOST_SHIFT)
#define RCC_PLL1CFGR_PLL1MBOOST_DIV_1     (0 << RCC_PLL1CFGR_PLL1MBOOST_SHIFT) /* 0000: division by 1 (bypass) */
#define RCC_PLL1CFGR_PLL1MBOOST_DIV_2     (1 << RCC_PLL1CFGR_PLL1MBOOST_SHIFT) /* 0001: division by 2 */
#define RCC_PLL1CFGR_PLL1MBOOST_DIV_4     (2 << RCC_PLL1CFGR_PLL1MBOOST_SHIFT) /* 0010: division by 4 */
#define RCC_PLL1CFGR_PLL1MBOOST_DIV_6     (3 << RCC_PLL1CFGR_PLL1MBOOST_SHIFT) /* 0011: division by 6 */
#define RCC_PLL1CFGR_PLL1MBOOST_DIV_8     (4 << RCC_PLL1CFGR_PLL1MBOOST_SHIFT) /* 0100: division by 8 */
#define RCC_PLL1CFGR_PLL1MBOOST_DIV_10    (5 << RCC_PLL1CFGR_PLL1MBOOST_SHIFT) /* 0101: division by 10 */
#define RCC_PLL1CFGR_PLL1MBOOST_DIV_12    (6 << RCC_PLL1CFGR_PLL1MBOOST_SHIFT) /* 0110: division by 12 */
#define RCC_PLL1CFGR_PLL1MBOOST_DIV_14    (7 << RCC_PLL1CFGR_PLL1MBOOST_SHIFT) /* 0111: division by 14 */
#define RCC_PLL1CFGR_PLL1MBOOST_DIV_16    (8 << RCC_PLL1CFGR_PLL1MBOOST_SHIFT) /* 1000: division by 16 */
#define RCC_PLL1CFGR_PLL1PEN              (1 << 16)                            /* Bit 16: PLL1 DIVP divider output enable */
#define RCC_PLL1CFGR_PLL1QEN              (1 << 17)                            /* Bit 17: PLL1 DIVQ divider output enable */
#define RCC_PLL1CFGR_PLL1REN              (1 << 18)                            /* Bit 18: PLL1 DIVR divider output enable */

/* RCC PLL1 dividers register */

#define RCC_PLL1DIVR_PLL1N_SHIFT          (0)       /* Bits 0-8: Multiplication factor for PLL1 VCO */
#define RCC_PLL1DIVR_PLL1N_MASK           (0x1ff << RCC_PLL1DIVR_PLL1N_SHIFT)
#define RCC_PLL1DIVR_PLL1N(n)             (((n) - 1) << RCC_PLL1DIVR_PLL1N_SHIFT)
#define RCC_PLL1DIVR_PLL1P_SHIFT          (9)       /* Bits 9-15: PLL1 DIVP division factor */
#define RCC_PLL1DIVR_PLL1P_MASK           (0x7f << RCC_PLL1DIVR_PLL1P_SHIFT)
#define RCC_PLL1DIVR_PLL1P(n)             (((n) - 1) << RCC_PLL1DIVR_PLL1P_SHIFT)
#define RCC_PLL1DIVR_PLL1Q_SHIFT          (16)      /* Bits 16-22: PLL1 DIVQ division factor */
#define RCC_PLL1DIVR_PLL1Q_MASK           (0x7f << RCC_PLL1DIVR_PLL1Q_SHIFT)
#define RCC_PLL1DIVR_PLL1Q(n)             (((n) - 1) << RCC_PLL1DIVR_PLL1Q_SHIFT)
#define RCC_PLL1DIVR_PLL1R_SHIFT          (24)      /* Bits 24-30: PLL1 DIVR division factor */
#define RCC_PLL1DIVR_PLL1R_MASK           (0x7f << RCC_PLL1DIVR_PLL1R_SHIFT)
#define RCC_PLL1DIVR_PLL1R(n)             (((n) - 1) << RCC_PLL1DIVR_PLL1R_SHIFT)

/* APB2 peripheral reset register */

#define RCC_APB2RSTR_TIM1RST              (1 << 11) /* Bit 11: TIM1 reset */
#define RCC_APB2RSTR_SPI1RST              (1 << 12) /* Bit 12: SPI1 reset */
#define RCC_APB2RSTR_TIM8RST              (1 << 13) /* Bit 13: TIM8 reset */
#define RCC_APB2RSTR_USART1RST            (1 << 14) /* Bit 14: USART1 reset */
#define RCC_APB2RSTR_TIM15RST             (1 << 16) /* Bit 16: TIM15 reset */
#define RCC_APB2RSTR_TIM16RST             (1 << 17) /* Bit 17: TIM16 reset */
#define RCC_APB2RSTR_TIM17RST             (1 << 18) /* Bit 18: TIM17 reset */
#define RCC_APB2RSTR_SAI1RST              (1 << 21) /* Bit 21: SAI1 reset */
#define RCC_APB2RSTR_SAI2RST              (1 << 22) /* Bit 22: SAI2 reset */

/* AHB1 Peripheral Clock enable register */

#define RCC_AHB1ENR_GPDMA1EN              (1 << 0)  /* Bit 0:  GPDMA1 clock enable */
#define RCC_AHB1ENR_CORDICEN              (1 << 1)  /* Bit 1:  CORDIC clock enable */
#define RCC_AHB1ENR_FMACEN                (1 << 2)  /* Bit 2:  FMAC clock enable */
#define RCC_AHB1ENR_MDF1EN                (1 << 2)  /* Bit 2:  MDF1 clock enable */
#define RCC_AHB1ENR_FLASHEN               (1 << 8)  /* Bit 8:  Flash memory interface clock enable */
#define RCC_AHB1ENR_CRCEN                 (1 << 12) /* Bit 12: CRC clock enable */
#define RCC_AHB1ENR_TSCEN                 (1 << 16) /* Bit 16: Touch Sensing Controller enable */
#define RCC_AHB1ENR_RAMCFGCEN             (1 << 17) /* Bit 17: RAMCFG clock enable */
#define RCC_AHB1ENR_DMA2DEN               (1 << 18) /* Bit 18: DMA2D clock enable */
#define RCC_AHB1ENR_GTZC1EN               (1 << 24) /* Bit 24: GTZC1 clock enable */
#define RCC_AHB1ENR_BKPSRAMEN             (1 << 31) /* Bit 31: BKPUPSRAM clock enable */
#define RCC_AHB1ENR_DCACHE1EN             (1 << 30) /* Bit 30: DCACHE1 clock enable */
#define RCC_AHB1ENR_SRAM1EN               (1 << 31) /* Bit 31: SRAM1 clock enable */

/* AHB2 Peripheral Clock enable register 1 */

#define RCC_AHB2ENR1_GPIOEN(n)            (1 << (n))
#define RCC_AHB2ENR1_GPIOAEN              (1 << 0)  /* Bit 0:  IO port A enable */
#define RCC_AHB2ENR1_GPIOBEN              (1 << 1)  /* Bit 1:  IO port B enable */
#define RCC_AHB2ENR1_GPIOCEN              (1 << 2)  /* Bit 2:  IO port C enable */
#define RCC_AHB2ENR1_GPIODEN              (1 << 3)  /* Bit 3:  IO port D enable */
#define RCC_AHB2ENR1_GPIOEEN              (1 << 4)  /* Bit 4:  IO port E enable */
#define RCC_AHB2ENR1_GPIOFEN              (1 << 5)  /* Bit 5:  IO port F enable */
#define RCC_AHB2ENR1_GPIOGEN              (1 << 6)  /* Bit 6:  IO port G enable */
#define RCC_AHB2ENR1_GPIOHEN              (1 << 7)  /* Bit 7:  IO port H enable */
#define RCC_AHB2ENR1_GPIOIEN              (1 << 8)  /* Bit 8:  IO port I enable */
#define RCC_AHB2ENR1_ADC1EN               (1 << 10) /* Bit 10: ADC1 interface enable */
#define RCC_AHB2ENR1_DCMI_PCSSI_EN        (1 << 12) /* Bit 12: DCMI and PSSI enable */
#define RCC_AHB2ENR1_OTGEN                (1 << 14) /* Bit 14: OTG_FS module enable */
#define RCC_AHB2ENR1_AESEN                (1 << 16) /* Bit 16: AES Cryptographic module enable */
#define RCC_AHB2ENR1_HASHEN               (1 << 17) /* Bit 17: HASH module enable */
#define RCC_AHB2ENR1_RNGEN                (1 << 18) /* Bit 18: Random number generator module enable */
#define RCC_AHB2ENR1_PKAEN                (1 << 19) /* Bit 19: PKA clock enable */
#define RCC_AHB2ENR1_SAESEN               (1 << 20) /* Bit 20: SAES clock enable */
#define RCC_AHB2ENR1_OCTOSPIMEN           (1 << 21) /* Bit 21: OCTOSPIM clock enable */
#define RCC_AHB2ENR1_OTFDEC1EN            (1 << 23) /* Bit 21: OTFDEC1 clock enable */
#define RCC_AHB2ENR1_OTFDEC2EN            (1 << 24) /* Bit 21: OTFDEC2 clock enable */
#define RCC_AHB2ENR1_SDMMC1EN             (1 << 27) /* Bit 27: SDMMC1 clock enable */
#define RCC_AHB2ENR1_SDMMC2EN             (1 << 28) /* Bit 28: SDMMC2 clock enable */
#define RCC_AHB2ENR1_SRAM2EN              (1 << 30) /* Bit 30: SRAM2 clock enable */
#define RCC_AHB2ENR1_SRAM3EN              (1 << 31) /* Bit 31: SRAM3 clock enable */

/* AHB2 Peripheral Clock enable register 2 */

#define RCC_AHB2ENR2_FSMCEN               (1 << 0)  /* Bit 0: FSMC clock enable */
#define RCC_AHB2ENR2_OCTOSPI1EN           (1 << 4)  /* Bit 4: OCTOSPI1 clock enable */
#define RCC_AHB2ENR2_OCTOSPI2EN           (1 << 8)  /* Bit 8: OCTOSPI2 clock enable */

/* RCC AHB3 peripheral clock enable register */

#define RCC_AHB3ENR_LPGPIO1EN             (1 << 0)  /* Bit 0: LPGPIO1 clock enable */
#define RCC_AHB3ENR_PWREN                 (1 << 2)  /* Bit 2: PWR clock enable */
#define RCC_AHB3ENR_ADC4EN                (1 << 5)  /* Bit 5: ADC4 clock enable */
#define RCC_AHB3ENR_DAC1EN                (1 << 6)  /* Bit 6: DAC1 clock enable */
#define RCC_AHB3ENR_LPDMA1EN              (1 << 9)  /* Bit 9: LPDMA1 clock enable */
#define RCC_AHB3ENR_ADF1EN                (1 << 10) /* Bit 10: ADF1 clock enable */
#define RCC_AHB3ENR_GTZC2EN               (1 << 12) /* Bit 12: GTZC2 clock enable */
#define RCC_AHB3ENR_SRAM4EN               (1 << 31) /* Bit 31: SRAM4 clock enable */

/* APB1 Peripheral Clock enable register 1 */

#define RCC_APB1ENR1_TIM2EN               (1 << 0)  /* Bit 0: TIM2 clock enable */
#define RCC_APB1ENR1_TIM3EN               (1 << 1)  /* Bit 1: TIM3 clock enable */
#define RCC_APB1ENR1_TIM4EN               (1 << 2)  /* Bit 2: TIM4 clock enable */
#define RCC_APB1ENR1_TIM5EN               (1 << 3)  /* Bit 3: TIM5 clock enable */
#define RCC_APB1ENR1_TIM6EN               (1 << 4)  /* Bit 4: TIM6 clock enable */
#define RCC_APB1ENR1_TIM7EN               (1 << 5)  /* Bit 5: TIM7 clock enable */
#define RCC_APB1ENR1_WWDGEN               (1 << 11) /* Bit 11: WWDG clock enable */
#define RCC_APB1ENR1_SPI2EN               (1 << 14) /* Bit 14: SPI2 clock enable */
#define RCC_APB1ENR1_USART2EN             (1 << 17) /* Bit 17: USART2 clock enable */
#define RCC_APB1ENR1_USART3EN             (1 << 18) /* Bit 18: USART3 clock enable */
#define RCC_APB1ENR1_USART4EN             (1 << 19) /* Bit 19: USART4 clock enable */
#define RCC_APB1ENR1_USART5EN             (1 << 20) /* Bit 20: USART5 clock enable */
#define RCC_APB1ENR1_I2C1EN               (1 << 21) /* Bit 21: I2C1 clock enable */
#define RCC_APB1ENR1_I2C2EN               (1 << 22) /* Bit 22: I2C2 clock enable */
#define RCC_APB1ENR1_CRSEN                (1 << 24) /* Bit 24: CRSEN clock enable */

/* APB1 Peripheral Clock enable register 2 */

#define RCC_APB1ENR2_I2C4EN               (1 << 1)  /* Bit 1: I2C4 clock enable */
#define RCC_APB1ENR2_LPTIM2EN             (1 << 5)  /* Bit 5: LPTIM2 clock enable */
#define RCC_APB1ENR2_FDCAN1EN             (1 << 9)  /* Bit 9: FDCAN1EN clock enable */
#define RCC_APB1ENR2_UCPD1EN              (1 << 23) /* Bit 23: UCPD1EN clock enable */

/* APB2 Peripheral Clock enable register */

#define RCC_APB2ENR_TIM1EN               (1 << 11) /* Bit 11: TIM1 clock enable */
#define RCC_APB2ENR_SPI1EN               (1 << 12) /* Bit 12: SPI1 clock enable */
#define RCC_APB2ENR_TIM8EN               (1 << 13) /* Bit 13: TIM8 clock enable */
#define RCC_APB2ENR_USART1EN             (1 << 14) /* Bit 14: USART1 clock enable */
#define RCC_APB2ENR_TIM15EN              (1 << 16) /* Bit 16: TIM15 clock enable */
#define RCC_APB2ENR_TIM16EN              (1 << 17) /* Bit 17: TIM16 clock enable */
#define RCC_APB2ENR_TIM17EN              (1 << 18) /* Bit 18: TIM17 clock enable */
#define RCC_APB2ENR_SAI1EN               (1 << 21) /* Bit 21: SAI1 clock enable */
#define RCC_APB2ENR_SAI2EN               (1 << 22) /* Bit 22: SAI2 clock enable */

/* APB3 Peripheral Clock enable register */

#define RCC_APB3ENR_SYSCFGEN             (1 << 1)  /* Bit 1: SYSCFG clock enable */
#define RCC_APB3ENR_SPI3EN               (1 << 5)  /* Bit 5: SPI3 clock enable */
#define RCC_APB3ENR_LPUART1EN            (1 << 6)  /* Bit 6: LPUART1 clock enable */
#define RCC_APB3ENR_I2C3EN               (1 << 7)  /* Bit 7: I2C3 clock enable */
#define RCC_APB3ENR_LPTIM1EN             (1 << 11) /* Bit 11: LPTIM1 clock enable */
#define RCC_APB3ENR_LPTIM3EN             (1 << 12) /* Bit 12: LPTIM3 clock enable */
#define RCC_APB3ENR_LPTIM4EN             (1 << 13) /* Bit 13: LPTIM4 clock enable */
#define RCC_APB3ENR_OPAMPEN              (1 << 14) /* Bit 14: OPAMP clock enable */
#define RCC_APB3ENR_COMPEN               (1 << 15) /* Bit 15: COMP clock enable */
#define RCC_APB3ENR_VREFEN               (1 << 20) /* Bit 20: VREF clock enable */
#define RCC_APB3ENR_RTCAPBEN             (1 << 21) /* Bit 21: RTC and TAMP APB clock enable */

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
#define RCC_BDCR_LSEGFON                 (1 << 12) /* Bit 12: LSE clock glitch filter enable */

#define RCC_BDCR_RTCEN                   (1 << 15)         /* Bit 15: RTC clock enable */
#define RCC_BDCR_BDRST                   (1 << 16)         /* Bit 16: Backup domain software reset */
#define RCC_BDCR_LSCOEN                  (1 << 24)         /* Bit 24: Low speed clock output enable */
#define RCC_BDCR_LSCOSEL                 (1 << 25)         /* Bit 25: Low speed clock output selection */
#  define RCC_BCDR_LSCOSEL_LSI           0                 /* LSI selected */
#  define RCC_BDCR_LSCOSEL_LSE           RCC_BDCR_LSCOSEL  /* LSE selected */

#define RCC_BDCR_LSION                   (1 << 26)  /* Bit 26: Low Speed Internal oscillator enable */
#define RCC_BDCR_LSIRDY                  (1 << 27)  /* Bit 27: Low Speed Internal oscillator Ready */
#define RCC_BDCR_LSIPREDIV               (1 << 28)  /* Bit 28: Low-speed clock divider configuration */
#  define RCC_BCDR_LSIPREDIV_NONE        0          /* LSI not divided */
#  define RCC_BCDR_LSIPREDIV_128         1          /* LSI divided by 128 */

#if 0
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
#define RCC_CR_PRIV                      (1 << 31)                     /* Bit 21: RCC privilege */

/* Internal Clock Sources Calibration */

#define RCC_CR_HSITRIM_SHIFT             (24)      /* Bits 30-24: Internal High Speed clock trimming */
#define RCC_CR_HSITRIM_MASK              (0x7f << RCC_CR_HSITRIM_SHIFT)
#define RCC_CR_HSICAL_SHIFT              (16)      /* Bits 23-16: Internal High Speed clock Calibration */
#define RCC_CR_HSICAL_MASK               (0xff << RCC_CR_HSICAL_SHIFT)
#define RCC_CR_MSITRIM_SHIFT             (8)       /* Bits 15-8: Internal Multi Speed clock trimming */
#define RCC_CR_MSITRIM_MASK              (0xff << RCC_CR_MSITRIM_SHIFT)
#define RCC_CR_MSICAL_SHIFT              (0)       /* Bits 7-0: Internal Multi Speed clock Calibration */
#define RCC_CR_MSICAL_MASK               (0xff << RCC_CR_MSICAL_SHIFT)

/* RCC clock configuration register 1 */

#define RCC_CFGR1_SW_SHIFT                (0)       /* Bits 0-1: System clock Switch */
#define RCC_CFGR1_SW_MASK                 (3 << RCC_CFGR1_SW_SHIFT)
#  define RCC_CFGR1_SW_MSIS               (0 << RCC_CFGR1_SW_SHIFT) /* 00: MSIS selected as system clock */
#  define RCC_CFGR1_SW_HSI16              (1 << RCC_CFGR1_SW_SHIFT) /* 00: HSI16 selected as system clock */
#  define RCC_CFGR1_SW_HSE                (2 << RCC_CFGR1_SW_SHIFT) /* 01: HSE selected as system clock */
#  define RCC_CFGR1_SW_PLL                (3 << RCC_CFGR1_SW_SHIFT) /* 10: PLL pll1_r_ck selected as system clock */

#define RCC_CFGR1_SWS_SHIFT               (2)       /* Bits 2-3: System Clock Switch Status */
#define RCC_CFGR1_SWS_MASK                (3 << RCC_CFGR1_SWS_SHIFT)
#  define RCC_CFGR1_SWS_MSIS              (0 << RCC_CFGR1_SWS_SHIFT) /* 00: MSIS oscillator used as system clock */
#  define RCC_CFGR1_SWS_HSI16             (1 << RCC_CFGR1_SWS_SHIFT) /* 00: HSI16 oscillator used as system clock */
#  define RCC_CFGR1_SWS_HSE               (2 << RCC_CFGR1_SWS_SHIFT) /* 01: HSE oscillator used as system clock */
#  define RCC_CFGR1_SWS_PLL               (3 << RCC_CFGR1_SWS_SHIFT) /* 10: PLL used as system clock */

#define RCC_CFGR1_STOPWUCK                (1 << 4) /* Bit 4: Wakeup from Stop and CSS backup clock selection */
#  define RCC_CFGR1_STOPWUCK_MSIS         (0 << 4) /* 0: MSIS */
#  define RCC_CFGR1_STOPWUCK_HSI16        (1 << 4) /* 0: HSI16 */

#define RCC_CFGR_STOPKERWUCK              (1 << 5) /* Bit 5: Wakeup from Stop kernel clock automatic enable selection */
#  define RCC_CFGR1_STOPKERWUCK_MSIK      (0 << 5) /* 0: MSIK */
#  define RCC_CFGR1_STOPKERWUCK_HSI16     (1 << 5) /* 0: HSI16 */

#define RCC_CFGR1_MCOSEL_SHIFT            (24)      /* Bits 24-27: Microcontroller Clock Output */
#define RCC_CFGR1_MCOSEL_MASK             (0x0f << RCC_CFGR1_MCOSEL_SHIFT)
#  define RCC_CFGR1_MCOSEL_NONE           (0 << RCC_CFGR1_MCOSEL_SHIFT) /* 0000: Disabled */
#  define RCC_CFGR1_MCOSEL_SYSCLK         (1 << RCC_CFGR1_MCOSEL_SHIFT) /* 0001: SYSCLK system clock selected */
#  define RCC_CFGR1_MCOSEL_MSIS           (2 << RCC_CFGR1_MCOSEL_SHIFT) /* 0010: MSIS clock selected */
#  define RCC_CFGR1_MCOSEL_HSI16          (3 << RCC_CFGR1_MCOSEL_SHIFT) /* 0011: HSI16 clock selected */
#  define RCC_CFGR1_MCOSEL_HSE            (4 << RCC_CFGR1_MCOSEL_SHIFT) /* 0100: HSE clock selected */
#  define RCC_CFGR1_MCOSEL_PLL            (5 << RCC_CFGR1_MCOSEL_SHIFT) /* 0101: Main PLL clock pll1_r_ck selected  */
#  define RCC_CFGR1_MCOSEL_LSI            (6 << RCC_CFGR1_MCOSEL_SHIFT) /* 0110: LSI clock selected */
#  define RCC_CFGR1_MCOSEL_LSE            (7 << RCC_CFGR1_MCOSEL_SHIFT) /* 0111: LSE clock selected */
#  define RCC_CFGR1_MCOSEL_HSI48          (8 << RCC_CFGR1_MCOSEL_SHIFT) /* 1000: HSI48 clock selected */
#  define RCC_CFGR1_MCOSEL_MSIK           (9 << RCC_CFGR1_MCOSEL_SHIFT) /* 1001: MSIK clock selected */

#define RCC_CFGR1_MCOPRE_SHIFT            (28)      /* Bits 28-30: MCO prescaler */
#define RCC_CFGR1_MCOPRE_MASK             (7 << RCC_CFGR1_MCOPRE_SHIFT)
#  define RCC_CFGR1_MCOPRE_NONE           (0 << RCC_CFGR1_MCOPRE_SHIFT) /* 000: no division */
#  define RCC_CFGR1_MCOPRE_DIV2           (1 << RCC_CFGR1_MCOPRE_SHIFT) /* 001: division by 2 */
#  define RCC_CFGR1_MCOPRE_DIV4           (2 << RCC_CFGR1_MCOPRE_SHIFT) /* 010: division by 4 */
#  define RCC_CFGR1_MCOPRE_DIV8           (3 << RCC_CFGR1_MCOPRE_SHIFT) /* 011: division by 8 */
#  define RCC_CFGR1_MCOPRE_DIV16          (4 << RCC_CFGR1_MCOPRE_SHIFT) /* 100: division by 16 */

/* RCC clock configuration register 2 */

#define RCC_CFGR_HPRE_SHIFT              (4)       /* Bits 4-7: AHB prescaler */
#define RCC_CFGR_HPRE_MASK               (0x0f << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLK           ( 0 << RCC_CFGR_HPRE_SHIFT) /* 0xxx: SYSCLK not divided */
#  define RCC_CFGR_HPRE_SYSCLKd2         ( 8 << RCC_CFGR_HPRE_SHIFT) /* 1000: SYSCLK divided by 2 */
#  define RCC_CFGR_HPRE_SYSCLKd4         ( 9 << RCC_CFGR_HPRE_SHIFT) /* 1001: SYSCLK divided by 4 */
#  define RCC_CFGR_HPRE_SYSCLKd8         (10 << RCC_CFGR_HPRE_SHIFT) /* 1010: SYSCLK divided by 8 */
#  define RCC_CFGR_HPRE_SYSCLKd16        (11 << RCC_CFGR_HPRE_SHIFT) /* 1011: SYSCLK divided by 16 */
#  define RCC_CFGR_HPRE_SYSCLKd64        (12 << RCC_CFGR_HPRE_SHIFT) /* 1100: SYSCLK divided by 64 */
#  define RCC_CFGR_HPRE_SYSCLKd128       (13 << RCC_CFGR_HPRE_SHIFT) /* 1101: SYSCLK divided by 128 */
#  define RCC_CFGR_HPRE_SYSCLKd256       (14 << RCC_CFGR_HPRE_SHIFT) /* 1110: SYSCLK divided by 256 */
#  define RCC_CFGR_HPRE_SYSCLKd512       (15 << RCC_CFGR_HPRE_SHIFT) /* 1111: SYSCLK divided by 512 */

#define RCC_CFGR_PPRE1_SHIFT             (8)       /* Bits 8-10: APB Low speed prescaler (APB1) */
#define RCC_CFGR_PPRE1_MASK              (7 << RCC_CFGR_PPRE1_SHIFT)
#  define RCC_CFGR_PPRE1_HCLK            (0 << RCC_CFGR_PPRE1_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE1_HCLKd2          (4 << RCC_CFGR_PPRE1_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE1_HCLKd4          (5 << RCC_CFGR_PPRE1_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE1_HCLKd8          (6 << RCC_CFGR_PPRE1_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE1_HCLKd16         (7 << RCC_CFGR_PPRE1_SHIFT) /* 111: HCLK divided by 16 */

#define RCC_CFGR_PPRE2_SHIFT             (11)      /* Bits 11-13: APB High speed prescaler (APB2) */
#define RCC_CFGR_PPRE2_MASK              (7 << RCC_CFGR_PPRE2_SHIFT)
#  define RCC_CFGR_PPRE2_HCLK            (0 << RCC_CFGR_PPRE2_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE2_HCLKd2          (4 << RCC_CFGR_PPRE2_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE2_HCLKd4          (5 << RCC_CFGR_PPRE2_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE2_HCLKd8          (6 << RCC_CFGR_PPRE2_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE2_HCLKd16         (7 << RCC_CFGR_PPRE2_SHIFT) /* 111: HCLK divided by 16 */

/* PLL configuration register */

#define RCC_PLLCFG_PLLSRC_SHIFT          (0) /* Bit 0-1: Main PLL(PLL) and audio PLLs (PLLSAIx)
                                               * entry clock source */
#define RCC_PLLCFG_PLLSRC_MASK           (3 << RCC_PLLCFG_PLLSRC_SHIFT)
#  define RCC_PLLCFG_PLLSRC_NONE         (0 << RCC_PLLCFG_PLLSRC_SHIFT) /* 000: No clock sent to PLLs */
#  define RCC_PLLCFG_PLLSRC_MSI          (1 << RCC_PLLCFG_PLLSRC_SHIFT) /* 001: MSI selected as PLL source */
#  define RCC_PLLCFG_PLLSRC_HSI16        (2 << RCC_PLLCFG_PLLSRC_SHIFT) /* 010: HSI16 selected as PLL source */
#  define RCC_PLLCFG_PLLSRC_HSE          (3 << RCC_PLLCFG_PLLSRC_SHIFT) /* 011: HSE selected as PLL source */

#define RCC_PLLCFG_PLLM_SHIFT            (4)      /* Bits 4-7: Main PLL (PLL) input clock divider */
#define RCC_PLLCFG_PLLM_MASK             (0x0f << RCC_PLLCFG_PLLM_SHIFT)
#  define RCC_PLLCFG_PLLM(n)             ((n-1) << RCC_PLLCFG_PLLM_SHIFT) /* m = 1..16 */

#define RCC_PLLCFG_PLLN_SHIFT            (8)      /* Bits 8-14: Main PLL (PLL) VCO multiplier */
#define RCC_PLLCFG_PLLN_MASK             (0x7f << RCC_PLLCFG_PLLN_SHIFT)
#  define RCC_PLLCFG_PLLN(n)             ((n) << RCC_PLLCFG_PLLN_SHIFT) /* n = 8..86 */

#define RCC_PLLCFG_PLLPEN                (1 << 16) /* Bit 16: Main PLL PLLSAI3CLK output enable */

#define RCC_PLLCFG_PLLP                  (1 << 17)       /* Bit 17: Main PLL div factor for PLLSAI3CLK */
#  define RCC_PLLCFG_PLLP_7              0               /* 0: PLLP = 7 */
#  define RCC_PLLCFG_PLLP_17             RCC_PLLCFG_PLLP /* 1: PLLP = 17 */

#define RCC_PLLCFG_PLLQEN                (1 << 20) /* Bit 20: Main PLL PLL48M1CLK output enable */

#define RCC_PLLCFG_PLLQ_SHIFT            (21)      /* Bits 21-22: Main PLL division factor for PLL48M1CLK (48 MHz clock) */
#define RCC_PLLCFG_PLLQ_MASK             (3 << RCC_PLLCFG_PLLQ_SHIFT)
#  define RCC_PLLCFG_PLLQ(n)             ((((n)>>1)-1)<< RCC_PLLCFG_PLLQ_SHIFT) /* n=2,4,6,8 */
#  define RCC_PLLCFG_PLLQ_2              (0 << RCC_PLLCFG_PLLQ_SHIFT)           /* 00: PLLQ = 2 */
#  define RCC_PLLCFG_PLLQ_4              (1 << RCC_PLLCFG_PLLQ_SHIFT)           /* 01: PLLQ = 4 */
#  define RCC_PLLCFG_PLLQ_6              (2 << RCC_PLLCFG_PLLQ_SHIFT)           /* 10: PLLQ = 6 */
#  define RCC_PLLCFG_PLLQ_8              (3 << RCC_PLLCFG_PLLQ_SHIFT)           /* 11: PLLQ = 8 */

#define RCC_PLLCFG_PLLREN                (1 << 24) /* Bit 24: Main PLL PLLCLK output enable */

#define RCC_PLLCFG_PLLR_SHIFT            (25)      /* Bits 25-26: Main PLL division factor for PLLCLK (system clock) */
#define RCC_PLLCFG_PLLR_MASK             (3 << RCC_PLLCFG_PLLR_SHIFT)
#  define RCC_PLLCFG_PLLR(n)             ((((n)>>1)-1)<< RCC_PLLCFG_PLLR_SHIFT) /* n=2,4,6,8 */
#  define RCC_PLLCFG_PLLR_2              (0 << RCC_PLLCFG_PLLR_SHIFT)           /* 00: PLLR = 2 */
#  define RCC_PLLCFG_PLLR_4              (1 << RCC_PLLCFG_PLLR_SHIFT)           /* 01: PLLR = 4 */
#  define RCC_PLLCFG_PLLR_6              (2 << RCC_PLLCFG_PLLR_SHIFT)           /* 10: PLLR = 6 */
#  define RCC_PLLCFG_PLLR_8              (3 << RCC_PLLCFG_PLLR_SHIFT)           /* 11: PLLR = 8 */

#define RCC_PLLCFG_PLLPDIV_SHIFT         (27)      /* Bits 31-27: Main PLL division factor for PLLSAI3CLK */
#define RCC_PLLCFG_PLLPDIV_MASK          (0x1f << RCC_PLLCFG_PLLPDIV_SHIFT)
#  define RCC_PLLCFG_PLLPDIV(n)          ((n) << RCC_PLLCFG_PLLDIV_SHIFT) /* n = 2..31 for VCO / 2 .. VCO / 31 */
#  define RCC_PLLCFG_PLLP_CNTRLD         RCC_PLLCFG_PLLPDIV(0)

#define RCC_PLLCFG_RESET                 (0x00001000) /* PLLCFG reset value */

/* PLLSAI1 Configuration register */

#define RCC_PLLSAI1CFG_PLLSRC_SHIFT      (0) /* Bit 0-1: Main PLSAI1 entry clock source */
#define RCC_PLLSAI1CFG_PLLSRC_MASK       (3 << RCC_PLLSAI1CFG_PLLSRC_SHIFT)
#  define RCC_PLLSAI1CFG_PLLSRC_NONE     (0 << RCC_PLLSAI1CFG_PLLSRC_SHIFT) /* 000: No clock sent to PLLs */
#  define RCC_PLLSAI1CFG_PLLSRC_MSI      (1 << RCC_PLLSAI1CFG_PLLSRC_SHIFT) /* 001: MSI selected as PLL source */
#  define RCC_PLLSAI1CFG_PLLSRC_HSI16    (2 << RCC_PLLSAI1CFG_PLLSRC_SHIFT) /* 010: HSI16 selected as PLL source */
#  define RCC_PLLSAI1CFG_PLLSRC_HSE      (3 << RCC_PLLSAI1CFG_PLLSRC_SHIFT) /* 011: HSE selected as PLL source */

#define RCC_PLLSAI1CFG_PLLM_SHIFT        (4)      /* Bits 4-7: Division factor for PLLSAI1 input clock */
#define RCC_PLLSAI1CFG_PLLM_MASK         (0x0f << RCC_PLLSAI1CFG_PLLM_SHIFT)
#  define RCC_PLLSAI1CFG_PLLM(n)         ((n-1) << RCC_PLLSAI1CFG_PLLM_SHIFT) /* m = 1..16 */

#define RCC_PLLSAI1CFG_PLLN_SHIFT        (8)      /* Bits 8-14: SAI1 PLL (PLLSAI1) VCO multiplier */
#define RCC_PLLSAI1CFG_PLLN_MASK         (0x7f << RCC_PLLSAI1CFG_PLLN_SHIFT)
#  define RCC_PLLSAI1CFG_PLLN(n)         ((n) << RCC_PLLSAI1CFG_PLLN_SHIFT) /* n = 8..86 */

#define RCC_PLLSAI1CFG_PLLPEN            (1 << 16) /* Bit 16: SAI1 PLL PLLSAI1CLK output enable */

#define RCC_PLLSAI1CFG_PLLP              (1 << 17)           /* Bit 17: Main PLL div factor for PLLSAI1CLK */
#  define RCC_PLLSAI1CFG_PLLP_7          0                   /* 0: PLLP = 7 */
#  define RCC_PLLSAI1CFG_PLLP_17         RCC_PLLSAI1CFG_PLLP /* 1: PLLP = 17 */

#define RCC_PLLSAI1CFG_PLLQEN            (1 << 20) /* Bit 20: Main PLL PLL48M2CLK output enable */

#define RCC_PLLSAI1CFG_PLLQ_SHIFT        (21)
#define RCC_PLLSAI1CFG_PLLQ_MASK         (3 << RCC_PLLSAI1CFG_PLLQ_SHIFT)
#  define RCC_PLLSAI1CFG_PLLQ(n)         ((((n)>>1)-1)<< RCC_PLLSAI1CFG_PLLQ_SHIFT) /* n=2,4,6,8 */
#  define RCC_PLLSAI1CFG_PLLQ_2          (0 << RCC_PLLSAI1CFG_PLLQ_SHIFT)           /* 00: PLLQ = 2 */
#  define RCC_PLLSAI1CFG_PLLQ_4          (1 << RCC_PLLSAI1CFG_PLLQ_SHIFT)           /* 01: PLLQ = 4 */
#  define RCC_PLLSAI1CFG_PLLQ_6          (2 << RCC_PLLSAI1CFG_PLLQ_SHIFT)           /* 10: PLLQ = 6 */
#  define RCC_PLLSAI1CFG_PLLQ_8          (3 << RCC_PLLSAI1CFG_PLLQ_SHIFT)           /* 11: PLLQ = 8 */

#define RCC_PLLSAI1CFG_PLLREN            (1 << 24) /* Bit 24: SAI1 PLL PLLADC1CLK output enable */

#define RCC_PLLSAI1CFG_PLLR_SHIFT        (25)
#define RCC_PLLSAI1CFG_PLLR_MASK         (3 << RCC_PLLSAI1CFG_PLLR_SHIFT)
#  define RCC_PLLSAI1CFG_PLLR(n)         ((((n)>>1)-1)<< RCC_PLLSAI1CFG_PLLR_SHIFT) /* n=2,4,6,8 */
#  define RCC_PLLSAI1CFG_PLLR_2          (0 << RCC_PLLSAI1CFG_PLLR_SHIFT)           /* 00: PLLR = 2 */
#  define RCC_PLLSAI1CFG_PLLR_4          (1 << RCC_PLLSAI1CFG_PLLR_SHIFT)           /* 01: PLLR = 4 */
#  define RCC_PLLSAI1CFG_PLLR_6          (2 << RCC_PLLSAI1CFG_PLLR_SHIFT)           /* 10: PLLR = 6 */
#  define RCC_PLLSAI1CFG_PLLR_8          (3 << RCC_PLLSAI1CFG_PLLR_SHIFT)           /* 11: PLLR = 8 */

#define RCC_PLLSAI1CFG_PLLPDIV_SHIFT     (27)      /* Bits 31-27: PLLSAI1 division factor for PLLSAI1CLK */
#define RCC_PLLSAI1CFG_PLLPDIV_MASK      (0x1f << RCC_PLLSAI1CFG_PLLPDIV_SHIFT)
#  define RCC_PLLSAI1CFG_PLLPDIV(n)      ((n) << RCC_PLLSAI1CFG_PLLDIV_SHIFT) /* n = 2..31 for VCO / 2 .. VCO / 31 */
#  define RCC_PLLSAI1CFG_PLLP_CNTRLD     RCC_PLLSAI1CFG_PLLPDIV(0)

/* PLLSAI2 Configuration register */

#define RCC_PLLSAI2CFG_PLLSRC_SHIFT      (0) /* Bit 0-1: Main PLSAI2 entry clock source */
#define RCC_PLLSAI2CFG_PLLSRC_MASK       (3 << RCC_PLLSAI2CFG_PLLSRC_SHIFT)
#  define RCC_PLLSAI2CFG_PLLSRC_NONE     (0 << RCC_PLLSAI2CFG_PLLSRC_SHIFT) /* 000: No clock sent to PLLs */
#  define RCC_PLLSAI2CFG_PLLSRC_MSI      (1 << RCC_PLLSAI2CFG_PLLSRC_SHIFT) /* 001: MSI selected as PLL source */
#  define RCC_PLLSAI2CFG_PLLSRC_HSI16    (2 << RCC_PLLSAI2CFG_PLLSRC_SHIFT) /* 010: HSI16 selected as PLL source */
#  define RCC_PLLSAI2CFG_PLLSRC_HSE      (3 << RCC_PLLSAI2CFG_PLLSRC_SHIFT) /* 011: HSE selected as PLL source */

#define RCC_PLLSAI2CFG_PLLM_SHIFT        (4)      /* Bits 4-7: Division factor for PLLSAI1 input clock */
#define RCC_PLLSAI2CFG_PLLM_MASK         (0x0f << RCC_PLLSAI1CFG_PLLM_SHIFT)
#  define RCC_PLLSA21CFG_PLLM(n)         ((n-1) << RCC_PLLSAI1CFG_PLLM_SHIFT) /* m = 1..16 */

#define RCC_PLLSAI2CFG_PLLN_SHIFT        (8)      /* Bits 8-14: SAI2 PLL (PLLSAI2) VCO multiplier */
#define RCC_PLLSAI2CFG_PLLN_MASK         (0x7f << RCC_PLLSAI2CFG_PLLN_SHIFT)
#  define RCC_PLLSAI2CFG_PLLN(n)         ((n) << RCC_PLLSAI2CFG_PLLN_SHIFT) /* n = 8..86 */

#define RCC_PLLSAI2CFG_PLLPEN            (1 << 16) /* Bit 16: SAI1 PLL PLLSAI2CLK output enable */

#define RCC_PLLSAI2CFG_PLLP              (1 << 17)           /* Bit 17: Main PLL div factor for PLLSAI2CLK */
#  define RCC_PLLSAI2CFG_PLLP_7          0                   /* 0: PLLP = 7 */
#  define RCC_PLLSAI2CFG_PLLP_17         RCC_PLLSAI2CFG_PLLP /* 1: PLLP = 17 */

#define RCC_PLLSAI2CFG_PLLPDIV_SHIFT     (27)      /* Bits 31-27: PLLSAI2 division factor for PLLSAI2CLK */
#define RCC_PLLSAI2CFG_PLLPDIV_MASK      (0x1f << RCC_PLLSAI2CFG_PLLPDIV_SHIFT)
#  define RCC_PLLSAI2CFG_PLLPDIV(n)      ((n) << RCC_PLLSAI2CFG_PLLDIV_SHIFT) /* n = 2..31 for VCO / 2 .. VCO / 31 */
#  define RCC_PLLSAI2CFG_PLLP_CNTRLD     RCC_PLLSAI2CFG_PLLPDIV(0)

/* Clock interrupt enable register */

#define RCC_CIER_LSIRDYIE                (1 << 0)  /* Bit 0: LSI Ready Interrupt Enable */
#define RCC_CIER_LSERDYIE                (1 << 1)  /* Bit 1: LSE Ready Interrupt Enable */
#define RCC_CIER_MSIRDYIE                (1 << 2)  /* Bit 2: MSI Ready Interrupt Enable */
#define RCC_CIER_HSIRDYIE                (1 << 3)  /* Bit 3: HSI Ready Interrupt Enable */
#define RCC_CIER_HSERDYIE                (1 << 4)  /* Bit 4: HSE Ready Interrupt Enable */
#define RCC_CIER_PLLRDYIE                (1 << 5)  /* Bit 5: PLL Ready Interrupt Enable */
#define RCC_CIER_PLLSAI1RDYIE            (1 << 6)  /* Bit 6: PLLSAI1 Ready Interrupt enable */
#define RCC_CIER_PLLSAI2RDYIE            (1 << 7)  /* Bit 7: PLLSAI2 Ready Interrupt enable */
#define RCC_CIER_HSI48RDYIE              (1 << 10) /* Bit 10: HSI48 Ready Interrupt Enable */

/* Clock interrupt flag register */

#define RCC_CIFR_LSIRDYIF                (1 << 0)  /* Bit 0: LSI Ready Interrupt Flag */
#define RCC_CIFR_LSERDYIF                (1 << 1)  /* Bit 1: LSE Ready Interrupt Flag */
#define RCC_CIFR_MSIRDYIF                (1 << 2)  /* Bit 2: MSI Ready Interrupt Flag */
#define RCC_CIFR_HSIRDYIF                (1 << 3)  /* Bit 3: HSI Ready Interrupt Flag */
#define RCC_CIFR_HSERDYIF                (1 << 4)  /* Bit 4: HSE Ready Interrupt Flag */
#define RCC_CIFR_PLLRDYIF                (1 << 5)  /* Bit 5: PLL Ready Interrupt Flag */
#define RCC_CIFR_PLLSAI1RDYIF            (1 << 6)  /* Bit 6: PLLSAI1 Ready Interrupt Flag */
#define RCC_CIFR_PLLSAI2RDYIF            (1 << 7)  /* Bit 7: PLLSAI2 Ready Interrupt Flag */
#define RCC_CIFR_CSSF                    (1 << 8)  /* Bit 8: Clock Security System Interrupt Flag */
#define RCC_CIFR_HSI48RDYIF              (1 << 10) /* Bit 10: HSI48 Ready Interrupt Flag */

/* Clock interrupt clear register */

#define RCC_CICR_LSIRDYIC                (1 << 0)  /* Bit 0: LSI Ready Interrupt Clear */
#define RCC_CICR_LSERDYIC                (1 << 1)  /* Bit 1: LSE Ready Interrupt Clear */
#define RCC_CICR_MSIRDYIC                (1 << 2)  /* Bit 2: MSI Ready Interrupt Clear */
#define RCC_CICR_HSIRDYIC                (1 << 3)  /* Bit 3: HSI Ready Interrupt Clear */
#define RCC_CICR_HSERDYIC                (1 << 4)  /* Bit 4: HSE Ready Interrupt Clear */
#define RCC_CICR_PLLRDYIC                (1 << 5)  /* Bit 5: PLL Ready Interrupt Clear */
#define RCC_CICR_PLLSAI1RDYIC            (1 << 6)  /* Bit 6: PLLSAI1 Ready Interrupt Clear */
#define RCC_CICR_PLLSAI2RDYIC            (1 << 7)  /* Bit 7: PLLSAI2 Ready Interrupt Clear */
#define RCC_CICR_CSSC                    (1 << 8)  /* Bit 8: Clock Security System Interrupt Clear */
#define RCC_CICR_HSI48RDYIC              (1 << 10) /* Bit 10: HSI48 Oscillator Ready Interrupt Clear */

/* AHB1 peripheral reset register */

#define RCC_AHB1RSTR_DMA1RST             (1 << 0)  /* Bit 0:  DMA1 reset */
#define RCC_AHB1RSTR_DMA2RST             (1 << 1)  /* Bit 1:  DMA2 reset */
#define RCC_AHB1RSTR_DMAMUX1RST          (1 << 2)  /* Bit 2:  DMAMUX1 reset */
#define RCC_AHB1RSTR_FLASHRST            (1 << 8)  /* Bit 8:  Flash memory interface reset */
#define RCC_AHB1RSTR_CRCRST              (1 << 12) /* Bit 12: CRC reset */
#define RCC_AHB1RSTR_TSCRST              (1 << 16) /* Bit 16: Touch Sensing Controller reset */

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
#define RCC_AHB2RSTR_ADCRST              (1 << 13) /* Bit 13: ADC interface reset (common to all ADCs) */
#define RCC_AHB2RSTR_AESRST              (1 << 16) /* Bit 16: AES Cryptographic module reset */
#define RCC_AHB2RSTR_HASHRST             (1 << 17) /* Bit 17: HASH module reset */
#define RCC_AHB2RSTR_RNGRST              (1 << 18) /* Bit 18: Random number generator module reset */
#define RCC_AHB2RSTR_PKARST              (1 << 19) /* Bit 19: Public Key Accelerator module reset */
#define RCC_AHB2RSTR_OTFDEC1RST          (1 << 21) /* Bit 21: On-the-fly decryption module reset */
#define RCC_AHB2RSTR_SDMMC1RST           (1 << 31) /* Bit 22: SDMMC1 module reset */

/* AHB3 peripheral reset register */

#define RCC_AHB3RSTR_FMCRST              (1 << 0)  /* Bit 0: Flexible memory controller module reset */
#define RCC_AHB3RSTR_OSPI1RST            (1 << 8)  /* Bit 8: Octo SPI1 module reset */

/* APB1 Peripheral reset register 1 */

#define RCC_APB1RSTR1_TIM2RST            (1 << 0)  /* Bit 0:  TIM2 reset */
#define RCC_APB1RSTR1_TIM3RST            (1 << 1)  /* Bit 1:  TIM3 reset */
#define RCC_APB1RSTR1_TIM4RST            (1 << 2)  /* Bit 2:  TIM4 reset */
#define RCC_APB1RSTR1_TIM5RST            (1 << 3)  /* Bit 3:  TIM5 reset */
#define RCC_APB1RSTR1_TIM6RST            (1 << 4)  /* Bit 4:  TIM6 reset */
#define RCC_APB1RSTR1_TIM7RST            (1 << 5)  /* Bit 5:  TIM7 reset */
#define RCC_APB1RSTR1_SPI2RST            (1 << 14) /* Bit 14: SPI2 reset */
#define RCC_APB1RSTR1_SPI3RST            (1 << 15) /* Bit 15: SPI3 reset */
#define RCC_APB1RSTR1_USART2RST          (1 << 17) /* Bit 17: USART2 reset */
#define RCC_APB1RSTR1_USART3RST          (1 << 18) /* Bit 18: USART3 reset */
#define RCC_APB1RSTR1_UART4RST           (1 << 19) /* Bit 19: UART4 reset */
#define RCC_APB1RSTR1_UART5RST           (1 << 20) /* Bit 20: UART5 reset */
#define RCC_APB1RSTR1_I2C1RST            (1 << 21) /* Bit 21: I2C1 reset */
#define RCC_APB1RSTR1_I2C2RST            (1 << 22) /* Bit 22: I2C2 reset */
#define RCC_APB1RSTR1_I2C3RST            (1 << 23) /* Bit 23: I2C3 reset */
#define RCC_APB1RSTR1_CRSRST             (1 << 24) /* Bit 24: CRS reset */
#define RCC_APB1RSTR1_PWRRST             (1 << 28) /* Bit 28: Power interface reset */
#define RCC_APB1RSTR1_DAC1RST            (1 << 29) /* Bit 29: DAC1 reset */
#define RCC_APB1RSTR1_OPAMPRST           (1 << 30) /* Bit 30: OPAMP reset */
#define RCC_APB1RSTR1_LPTIM1RST          (1 << 31) /* Bit 31: Low-power Timer 1 reset */

/* APB1 Peripheral reset register 2 */

#define RCC_APB1RSTR2_LPUART1RST         (1 << 0)  /* Bit 0:  Low-power UART 1 reset */
#define RCC_APB1RSTR2_I2C4RST            (1 << 1)  /* Bit 1:  I2C4 reset */
#define RCC_APB1RSTR2_LPTIM2RST          (1 << 5)  /* Bit 5:  Low-power Timer 2 reset */
#define RCC_APB1RSTR2_LPTIM3RST          (1 << 6)  /* Bit 6:  Low-power Timer 3 reset */
#define RCC_APB1RSTR2_FDCAN1RST          (1 << 9)  /* Bit 9:  FDCAN1 reset */
#define RCC_APB1RSTR2_USBFSRST           (1 << 21) /* Bit 21: USB FS reset */
#define RCC_APB1RSTR2_UCPD1RST           (1 << 23) /* Bit 21: UCPD1 reset */

/* APB2 Peripheral reset register */

#define RCC_APB2RSTR_SYSCFGRST           (1 << 0)  /* Bit 0:  System configuration controller reset */
#define RCC_APB2RSTR_TIM1RST             (1 << 11) /* Bit 11: TIM1 reset */
#define RCC_APB2RSTR_SPI1RST             (1 << 12) /* Bit 12: SPI1 reset */
#define RCC_APB2RSTR_TIM8RST             (1 << 13) /* Bit 13: TIM8 reset */
#define RCC_APB2RSTR_USART1RST           (1 << 14) /* Bit 14: USART1 reset */
#define RCC_APB2RSTR_TIM15RST            (1 << 16) /* Bit 16: TIM15 reset */
#define RCC_APB2RSTR_TIM16RST            (1 << 17) /* Bit 17: TIM16 reset */
#define RCC_APB2RSTR_TIM17RST            (1 << 18) /* Bit 18: TIM17 reset */
#define RCC_APB2RSTR_SAI1RST             (1 << 21) /* Bit 21: SAI1 reset */
#define RCC_APB2RSTR_SAI2RST             (1 << 22) /* Bit 22: SAI2 reset */
#define RCC_APB2RSTR_DFSDMRST            (1 << 24) /* Bit 24: DFSDM reset */

/* AHB1 Peripheral Clock enable register */

#define RCC_AHB1ENR_DMA1EN               (1 << 0)  /* Bit 0:  DMA1 enable */
#define RCC_AHB1ENR_DMA2EN               (1 << 1)  /* Bit 1:  DMA2 enable */
#define RCC_AHB1ENR_DMAMUX1EN            (1 << 2)  /* Bit 2:  DMAMUX1 enable */
#define RCC_AHB1ENR_FLASHEN              (1 << 8)  /* Bit 8:  Flash memory interface enable */
#define RCC_AHB1ENR_CRCEN                (1 << 12) /* Bit 12: CRC enable */
#define RCC_AHB1ENR_TSCEN                (1 << 16) /* Bit 16: Touch Sensing Controller enable */
#define RCC_AHB1ENR_GTZCEN               (1 << 22) /* Bit 22: GTZC clock enable */

/* AHB3 Peripheral Clock enable register */

#define RCC_AHB3ENR_FMCEN                (1 << 0)  /* Bit 0: Flexible memory controller module enable */
#define RCC_AHB3ENR_OSPI1EN              (1 << 8)  /* Bit 8: OCTOSPI1 module enable */

/* APB1 Peripheral Clock enable register 1 */

#define RCC_APB1ENR1_TIM2EN              (1 << 0)  /* Bit 0:  TIM2 enable */
#define RCC_APB1ENR1_TIM3EN              (1 << 1)  /* Bit 1:  TIM3 enable */
#define RCC_APB1ENR1_TIM4EN              (1 << 2)  /* Bit 2:  TIM4 enable */
#define RCC_APB1ENR1_TIM5EN              (1 << 3)  /* Bit 3:  TIM5 enable */
#define RCC_APB1ENR1_TIM6EN              (1 << 4)  /* Bit 4:  TIM6 enable */
#define RCC_APB1ENR1_TIM7EN              (1 << 5)  /* Bit 5:  TIM7 enable */
#define RCC_APB1ENR1_RTCAPBEN            (1 << 10) /* Bit 10: RTC APB clock enable */
#define RCC_APB1ENR1_WWDGEN              (1 << 11) /* Bit 11: Windowed Watchdog enable */
#define RCC_APB1ENR1_SPI2EN              (1 << 14) /* Bit 14: SPI2 enable */
#define RCC_APB1ENR1_SPI3EN              (1 << 15) /* Bit 15: SPI3 enable */
#define RCC_APB1ENR1_USART2EN            (1 << 17) /* Bit 17: USART2 enable */
#define RCC_APB1ENR1_USART3EN            (1 << 18) /* Bit 18: USART3 enable */
#define RCC_APB1ENR1_UART4EN             (1 << 19) /* Bit 19: USART4 enable */
#define RCC_APB1ENR1_UART5EN             (1 << 20) /* Bit 20: USART5 enable */
#define RCC_APB1ENR1_I2C1EN              (1 << 21) /* Bit 21: I2C1 enable */
#define RCC_APB1ENR1_I2C2EN              (1 << 22) /* Bit 22: I2C2 enable */
#define RCC_APB1ENR1_I2C3EN              (1 << 23) /* Bit 23: I2C3 enable */
#define RCC_APB1ENR1_CRSEN               (1 << 24) /* Bit 24: CRSEN enable */
#define RCC_APB1ENR1_PWREN               (1 << 28) /* Bit 28: Power interface enable */
#define RCC_APB1ENR1_DAC1EN              (1 << 29) /* Bit 29: DAC1 enable */
#define RCC_APB1ENR1_OPAMPEN             (1 << 30) /* Bit 30: OPAMP enable */
#define RCC_APB1ENR1_LPTIM1EN            (1 << 31) /* Bit 31: Low-power Timer 1 enable */

/* APB1 Peripheral Clock enable register 2 */

#define RCC_APB1ENR2_LPUART1EN           (1 << 0)  /* Bit 0:  Low-power UART 1 enable */
#define RCC_APB1ENR2_I2C4EN              (1 << 1)  /* Bit 1:  I2C4 enable */
#define RCC_APB1ENR2_LPTIM2EN            (1 << 5)  /* Bit 5:  Low-power Timer 2 enable */
#define RCC_APB1ENR2_LPTIM3EN            (1 << 6)  /* Bit 6:  Low-power Timer 3 enable */
#define RCC_APB1ENR2_FDCAN1EN            (1 << 9)  /* Bit 9:  FDCAN1 enable */
#define RCC_APB1ENR2_USBFSEN             (1 << 21) /* Bit 21: USB FS enable */
#define RCC_APB1ENR2_UCPD1EN             (1 << 23) /* Bit 23: UCPD1 enable */

/* APB2 Peripheral Clock enable register */

#define RCC_APB2ENR_SYSCFGEN             (1 << 0)  /* Bit 0:  SYSCFG + COMP + VREFBUF enable */
#define RCC_APB2ENR_TIM1EN               (1 << 11) /* Bit 11: TIM1 enable */
#define RCC_APB2ENR_SPI1EN               (1 << 12) /* Bit 12: SPI1 enable */
#define RCC_APB2ENR_TIM8EN               (1 << 13) /* Bit 13: TIM8 enable */
#define RCC_APB2ENR_USART1EN             (1 << 14) /* Bit 14: USART1 enable */
#define RCC_APB2ENR_TIM15EN              (1 << 16) /* Bit 16: TIM15 enable */
#define RCC_APB2ENR_TIM16EN              (1 << 17) /* Bit 17: TIM16 enable */
#define RCC_APB2ENR_TIM17EN              (1 << 18) /* Bit 18: TIM17 enable */
#define RCC_APB2ENR_SAI1EN               (1 << 21) /* Bit 21: SAI1 enable */
#define RCC_APB2ENR_SAI2EN               (1 << 22) /* Bit 22: SAI2 enable */
#define RCC_APB2ENR_DFSDM1EN             (1 << 24) /* Bit 24: DFSDM1 enable */

/* RCC AHB1 Sleep and Stop modes peripheral clock enable register */

#define RCC_AHB1SMENR_DMA1SMEN           (1 << 0)  /* Bit 0:  DMA1 enable during Sleep mode */
#define RCC_AHB1SMENR_DMA2SMEN           (1 << 1)  /* Bit 1:  DMA2 enable during Sleep mode */
#define RCC_AHB1SMENR_DMAMUX1SMEN        (1 << 2)  /* Bit 2:  DMAMUX1 enable during Sleep mode */
#define RCC_AHB1SMENR_FLASHSMEN          (1 << 8)  /* Bit 8:  Flash memory interface enable during Sleep mode */
#define RCC_AHB1SMENR_SRAM1SMEN          (1 << 9)  /* Bit 9:  SRAM1 enable during Sleep mode */
#define RCC_AHB1SMENR_CRCSMEN            (1 << 12) /* Bit 12: CRC enable during Sleep mode */
#define RCC_AHB1SMENR_TSCSMEN            (1 << 16) /* Bit 16: Touch sensing controller enable during Sleep mode */
#define RCC_AHB1SMENR_GTZCSMEN           (1 << 22) /* Bit 22: GTZC enable during Sleep mode */
#define RCC_AHB1SMENR_ICACHESMEN         (1 << 23) /* Bit 23: Instruction cache enable during Sleep mode */

/* RCC AHB2 low power mode peripheral clock enable register */

#define RCC_AHB2SMENR_GPIOASMEN          (1 << 0)  /* Bit 0:  IO port A enable during Sleep mode */
#define RCC_AHB2SMENR_GPIOBSMEN          (1 << 1)  /* Bit 1:  IO port B enable during Sleep mode */
#define RCC_AHB2SMENR_GPIOCSMEN          (1 << 2)  /* Bit 2:  IO port C enable during Sleep mode */
#define RCC_AHB2SMENR_GPIODSMEN          (1 << 3)  /* Bit 3:  IO port D enable during Sleep mode */
#define RCC_AHB2SMENR_GPIOESMEN          (1 << 4)  /* Bit 4:  IO port E enable during Sleep mode */
#define RCC_AHB2SMENR_GPIOFSMEN          (1 << 5)  /* Bit 5:  IO port F enable during Sleep mode */
#define RCC_AHB2SMENR_GPIOGSMEN          (1 << 6)  /* Bit 6:  IO port G enable during Sleep mode */
#define RCC_AHB2SMENR_GPIOHSMEN          (1 << 7)  /* Bit 7:  IO port H enable during Sleep mode */
#define RCC_AHB2SMENR_SRAM2SMEN          (1 << 9)  /* Bit 9:  SRAM2 enable during Sleep mode */
#define RCC_AHB2SMENR_ADCSMEN            (1 << 13) /* Bit 13: ADC interface enable during Sleep mode (common to all ADCs) */
#define RCC_AHB2SMENR_AESSMEN            (1 << 16) /* Bit 16: AES Cryptographic module enable during Sleep mode */
#define RCC_AHB2SMENR_HASHSMEN           (1 << 17) /* Bit 17: HASH module enable during Sleep mode */
#define RCC_AHB2SMENR_RNGSMEN            (1 << 18) /* Bit 18: Random number generator module enable during Sleep mode */
#define RCC_AHB2SMENR_PKASMEN            (1 << 19) /* Bit 19: PKA module enable during Sleep mode */
#define RCC_AHB2SMENR_OTFDEC1SMEN        (1 << 21) /* Bit 21: OTFDEC1 module enable during Sleep mode */
#define RCC_AHB2SMENR_SDMMC1SMEN         (1 << 22) /* Bit 22: SDMMC1 module enable during Sleep mode */

/* RCC AHB3 low power mode peripheral clock enable register */

#define RCC_AHB3SMENR_FMCSMEN            (1 << 0)  /* Bit 0: Flexible memory controller module enable during Sleep mode */
#define RCC_AHB3SMENR_OSPISMEN           (1 << 8)  /* Bit 8: OCTOSPI1 module enable during Sleep mode */

/* RCC APB1 low power mode peripheral clock enable register 1 */

#define RCC_APB1SMENR1_TIM2SMEN          (1 << 0)  /* Bit 0:  TIM2 enable during Sleep mode */
#define RCC_APB1SMENR1_TIM3SMEN          (1 << 1)  /* Bit 1:  TIM3 enable during Sleep mode */
#define RCC_APB1SMENR1_TIM4SMEN          (1 << 2)  /* Bit 2:  TIM4 enable during Sleep mode */
#define RCC_APB1SMENR1_TIM5SMEN          (1 << 3)  /* Bit 3:  TIM5 enable during Sleep mode */
#define RCC_APB1SMENR1_TIM6SMEN          (1 << 4)  /* Bit 4:  TIM6 enable during Sleep mode */
#define RCC_APB1SMENR1_TIM7SMEN          (1 << 5)  /* Bit 5:  TIM7 enable during Sleep mode */
#define RCC_APB1SMENR1_RTCAPBSMEN        (1 << 10) /* Bit 10: RTC APB clock enable during Sleep mode */
#define RCC_APB1SMENR1_WWDGSMEN          (1 << 11) /* Bit 11: Windowed Watchdog enable during Sleep mode */
#define RCC_APB1SMENR1_SPI2SMEN          (1 << 14) /* Bit 14: SPI2 enable during Sleep mode */
#define RCC_APB1SMENR1_SPI3SMEN          (1 << 15) /* Bit 15: SPI3 enable during Sleep mode */
#define RCC_APB1SMENR1_USART2SMEN        (1 << 17) /* Bit 17: USART2 enable during Sleep mode */
#define RCC_APB1SMENR1_USART3SMEN        (1 << 18) /* Bit 18: USART3 enable during Sleep mode */
#define RCC_APB1SMENR1_UART4SMEN         (1 << 19) /* Bit 19: USART4 enable during Sleep mode */
#define RCC_APB1SMENR1_UART5SMEN         (1 << 20) /* Bit 20: USART5 enable during Sleep mode */
#define RCC_APB1SMENR1_I2C1SMEN          (1 << 21) /* Bit 21: I2C1 enable during Sleep mode */
#define RCC_APB1SMENR1_I2C2SMEN          (1 << 22) /* Bit 22: I2C2 enable during Sleep mode */
#define RCC_APB1SMENR1_I2C3SMEN          (1 << 23) /* Bit 23: I2C3 enable during Sleep mode */
#define RCC_APB1SMENR1_CRSSMEN           (1 << 24) /* Bit 24: CRS enable during Sleep mode */
#define RCC_APB1SMENR1_PWRSMEN           (1 << 28) /* Bit 28: Power interface enable during Sleep mode */
#define RCC_APB1SMENR1_DAC1SMEN          (1 << 29) /* Bit 29: DAC1 enable during Sleep mode */
#define RCC_APB1SMENR1_OPAMPSMEN         (1 << 30) /* Bit 30: OPAMP enable during Sleep mode */
#define RCC_APB1SMENR1_LPTIM1SMEN        (1 << 31) /* Bit 31: Low-power Timer 1 enable during Sleep mode */

/* RCC APB1 low power modeperipheral clock enable register 2 */

#define RCC_APB1SMENR2_LPUART1SMEN       (1 << 0)  /* Bit 0:  Low-power UART 1 enable during Sleep mode */
#define RCC_APB1SMENR2_I2C4SMEN          (1 << 1)  /* Bit 1:  I2C4 enable during Sleep mode */
#define RCC_APB1SMENR2_LPTIM2SMEN        (1 << 5)  /* Bit 5:  Low-power Timer 2 enable during Sleep mode */
#define RCC_APB1SMENR2_LPTIM3SMEN        (1 << 6)  /* Bit 6:  Low-power Timer 3 enable during Sleep mode */
#define RCC_APB1SMENR2_FDCAN1SMEN        (1 << 9)  /* Bit 9:  FDCAN1 enable during Sleep mode */
#define RCC_APB1SMENR2_USBFSSMEN         (1 << 21) /* Bit 21: USB FS enable during Sleep mode */
#define RCC_APB1SMENR2_UCPD1SMEN         (1 << 23) /* Bit 23: UCPDS1 enable during Sleep mode */

/* RCC APB2 low power mode peripheral clock enable register */

#define RCC_APB2SMENR_SYSCFGSMEN         (1 << 0)  /* Bit 0:  System configuration controller enable during Sleep mode */
#define RCC_APB2SMENR_TIM1SMEN           (1 << 11) /* Bit 11: TIM1 enable during Sleep mode */
#define RCC_APB2SMENR_SPI1SMEN           (1 << 12) /* Bit 12: SPI1 enable during Sleep mode */
#define RCC_APB2SMENR_TIM8SMEN           (1 << 13) /* Bit 13: TIM8 enable during Sleep mode */
#define RCC_APB2SMENR_USART1SMEN         (1 << 14) /* Bit 14: USART1 enable during Sleep mode */
#define RCC_APB2SMENR_TIM15SMEN          (1 << 16) /* Bit 16: TIM15 enable during Sleep mode */
#define RCC_APB2SMENR_TIM16SMEN          (1 << 17) /* Bit 17: TIM16 enable during Sleep mode */
#define RCC_APB2SMENR_TIM17SMEN          (1 << 18) /* Bit 18: TIM17 enable during Sleep mode */
#define RCC_APB2SMENR_SAI1SMEN           (1 << 21) /* Bit 21: SAI1 enable during Sleep mode */
#define RCC_APB2SMENR_SAI2SMEN           (1 << 22) /* Bit 22: SAI2 enable during Sleep mode */
#define RCC_APB2SMENR_DFSDM1SMEN         (1 << 24) /* Bit 24: DFSDM1 enable during Sleep mode */

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

#define RCC_CCIPR_USART3SEL_SHIFT        (4)
#define RCC_CCIPR_USART3SEL_MASK         (3 << RCC_CCIPR_USART3SEL_SHIFT)
#  define RCC_CCIPR_USART3SEL_PCLK1      (0 << RCC_CCIPR_USART3SEL_SHIFT)
#  define RCC_CCIPR_USART3SEL_SYSCLK     (1 << RCC_CCIPR_USART3SEL_SHIFT)
#  define RCC_CCIPR_USART3SEL_HSI16      (2 << RCC_CCIPR_USART3SEL_SHIFT)
#  define RCC_CCIPR_USART3SEL_LSE        (3 << RCC_CCIPR_USART3SEL_SHIFT)

#define RCC_CCIPR_UART4SEL_SHIFT         (6)
#define RCC_CCIPR_UART4SEL_MASK          (3 << RCC_CCIPR_UART4SEL_SHIFT)
#  define RCC_CCIPR_UART4SEL_PCLK1       (0 << RCC_CCIPR_UART4SEL_SHIFT)
#  define RCC_CCIPR_UART4SEL_SYSCLK      (1 << RCC_CCIPR_UART4SEL_SHIFT)
#  define RCC_CCIPR_UART4SEL_HSI16       (2 << RCC_CCIPR_UART4SEL_SHIFT)
#  define RCC_CCIPR_UART4SEL_LSE         (3 << RCC_CCIPR_UART4SEL_SHIFT)

#define RCC_CCIPR_UART5SEL_SHIFT         (8)
#define RCC_CCIPR_UART5SEL_MASK          (3 << RCC_CCIPR_UART5SEL_SHIFT)
#  define RCC_CCIPR_UART5SEL_PCLK1       (0 << RCC_CCIPR_UART5SEL_SHIFT)
#  define RCC_CCIPR_UART5SEL_SYSCLK      (1 << RCC_CCIPR_UART5SEL_SHIFT)
#  define RCC_CCIPR_UART5SEL_HSI16       (2 << RCC_CCIPR_UART5SEL_SHIFT)
#  define RCC_CCIPR_UART5SEL_LSE         (3 << RCC_CCIPR_UART5SEL_SHIFT)

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

#define RCC_CCIPR_FDCANSEL_SHIFT         (24)
#define RCC_CCIPR_FDCANSEL_MASK          (3 << RCC_CCIPR_SAI2SEL_SHIFT)
#  define RCC_CCIPR_FDCANSEL_HSE         (0 << RCC_CCIPR_SAI2SEL_SHIFT)
#  define RCC_CCIPR_FDCANSEL_PLL48M1CLK  (1 << RCC_CCIPR_SAI2SEL_SHIFT)
#  define RCC_CCIPR_FDCANSEL_PLLSAI1CLK  (2 << RCC_CCIPR_SAI2SEL_SHIFT)

#define RCC_CCIPR_CLK48MSEL_SHIFT        (26)
#define RCC_CCIPR_CLK48MSEL_MASK         (3 << RCC_CCIPR_CLK48SEL_SHIFT)
#  define RCC_CCIPR_CLK48MSEL_HSI48      (0 << RCC_CCIPR_CLK48SEL_SHIFT)
#  define RCC_CCIPR_CLK48MSEL_PLL48M2CLK (1 << RCC_CCIPR_CLK48SEL_SHIFT)
#  define RCC_CCIPR_CLK48MSEL_PLL48M1CLK (2 << RCC_CCIPR_CLK48SEL_SHIFT)
#  define RCC_CCIPR_CLK48MSEL_MSI        (3 << RCC_CCIPR_CLK48SEL_SHIFT)

#define RCC_CCIPR_ADCSEL_SHIFT           (28)
#define RCC_CCIPR_ADCSEL_MASK            (3 << RCC_CCIPR_ADCSEL_SHIFT)
#  define RCC_CCIPR_ADCSEL_NONE          (0 << RCC_CCIPR_ADCSEL_SHIFT)
#  define RCC_CCIPR_ADCSEL_PLLADC1CLK    (1 << RCC_CCIPR_ADCSEL_SHIFT)
#  define RCC_CCIPR_ADCSEL_SYSCLK        (3 << RCC_CCIPR_ADCSEL_SHIFT)

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

#define RCC_CSR_RMVF                     (1 << 23) /* Bit 23: Remove reset flag */
#define RCC_CSR_OBLRSTF                  (1 << 25) /* Bit 25: Option byte loader reset flag */
#define RCC_CSR_PINRSTF                  (1 << 26) /* Bit 26: PIN reset flag */
#define RCC_CSR_BORRSTF                  (1 << 27) /* Bit 27: BOR reset flag */
#define RCC_CSR_SFTRSTF                  (1 << 28) /* Bit 28: Software Reset flag */
#define RCC_CSR_IWDGRSTF                 (1 << 29) /* Bit 29: Independent Watchdog reset flag */
#define RCC_CSR_WWDGRSTF                 (1 << 30) /* Bit 30: Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF                 (1 << 31) /* Bit 31: Low-Power reset flag */

/* Clock recovery RC register */

#define RCC_CRRCR_HSI48CAL_SHIFT         7
#  define RCC_CRRCR_HSI48CAL_MASK        (0x01ff << RCC_CRRCR_HSI48CAL_SHIFT) /* HSI48 clock calibration */

#define RCC_CRRCR_HSI48ON                (1 << 0)  /* Bit 0: HSI48 clock enable */
#define RCC_CRRCR_HSI48RDY               (1 << 1)  /* Bit 1: HSI48 clock ready flag */

/* Peripheral Independent Clock Configuration 2 register */

#define RCC_CCIPR2_I2C4SEL_SHIFT         (0)    /* Bits 0-1: I2C4 clock source selection */
#define RCC_CCIPR2_I2C4SEL_MASK          (3 << RCC_CCIPR2_I2C4SEL_SHIFT)
#  define RCC_CCIPR2_I2C4SEL_PCLK        (0 << RCC_CCIPR2_I2C4SEL_SHIFT)
#  define RCC_CCIPR2_I2C4SEL_SYSCLK      (1 << RCC_CCIPR2_I2C4SEL_SHIFT)
#  define RCC_CCIPR2_I2C4SEL_HSI         (2 << RCC_CCIPR2_I2C4SEL_SHIFT)

#define RCC_CCIPR2_DFSDMSEL_SHIFT        (2)    /* Bit 2: DFSDMSEL kernel clock source selection */
#define RCC_CCIPR2_DFSDMSEL_MASK         (1 << RCC_CCIPR2_DFSDMSEL_SHIFT)
#  define RCC_CCIPR2_DFSDMSEL_PCLK2      (0 << RCC_CCIPR2_DFSDMSEL_SHIFT)
#  define RCC_CCIPR2_FDSDMSEL_SYSCLK     (1 << RCC_CCIPR2_FDSDMSEL_SHIFT)

#define RCC_CCIPR2_ADFSDMSEL_SHIFT       (3)    /* Bit 3-4: DFSDMSEL audio clock source selection */
#define RCC_CCIPR2_ADFSDMSEL_MASK        (3 << RCC_CCIPR2_ADFSDMSEL_SHIFT)
#  define RCC_CCIPR2_ADFSDMSEL_SAI1      (0 << RCC_CCIPR2_ADFSDMSEL_SHIFT)
#  define RCC_CCIPR2_AFDSDMSEL_SYSCLK    (1 << RCC_CCIPR2_ADFSDMSEL_SHIFT)
#  define RCC_CCIPR2_ADFSDMSEL_HSI16     (2 << RCC_CCIPR2_ADFSDMSEL_SHIFT)

#define RCC_CCIPR2_SAI1SEL_SHIFT         (5)    /* Bit 5-7: SAI1 clock source selection */
#define RCC_CCIPR2_SAI1SEL_MASK          (7 << RCC_CCIPR2_SAI1SEL_SHIFT)
#  define RCC_CCIPR2_SAI1SEL_PLLSAI1CLK  (0 << RCC_CCIPR2_SAI1SEL_SHIFT)
#  define RCC_CCIPR2_SAI1SEL_PLLSAI2CLK  (1 << RCC_CCIPR2_SAI1SEL_SHIFT)
#  define RCC_CCIPR2_SAI1SEL_PLLSAI3CLK  (2 << RCC_CCIPR2_SAI1SEL_SHIFT)
#  define RCC_CCIPR2_SAI1SEL_EXTCLK      (3 << RCC_CCIPR2_SAI1SEL_SHIFT)
#  define RCC_CCIPR2_SAI1SEL_HSI         (4 << RCC_CCIPR2_SAI1SEL_SHIFT)

#define RCC_CCIPR2_SAI2SEL_SHIFT         (8)    /* Bit 8-10: SAI2 clock source selection */
#define RCC_CCIPR2_SAI2SEL_MASK          (7 << RCC_CCIPR2_SAI2SEL_SHIFT)
#  define RCC_CCIPR2_SAI2SEL_PLLSAI1CLK  (0 << RCC_CCIPR2_SAI2SEL_SHIFT)
#  define RCC_CCIPR2_SAI2SEL_PLLSAI2CLK  (1 << RCC_CCIPR2_SAI2SEL_SHIFT)
#  define RCC_CCIPR2_SAI2SEL_PLLSAI3CLK  (2 << RCC_CCIPR2_SAI2SEL_SHIFT)
#  define RCC_CCIPR2_SAI2SEL_EXTCLK      (3 << RCC_CCIPR2_SAI2SEL_SHIFT)
#  define RCC_CCIPR2_SAI2SEL_HSI         (4 << RCC_CCIPR2_SAI2SEL_SHIFT)

#define RCC_CCIPR2_SDMMCSEL_SHIFT        (14)    /* Bit 14: SDMMC clock source selection */
#define RCC_CCIPR2_SDMMCSEL_MASK         (1 << RCC_CCIPR2_SDMMCSEL_SHIFT)
#  define RCC_CCIPR2_SDMMCSEL_PCLK2      (0 << RCC_CCIPR2_SDMMCSEL_SHIFT)
#  define RCC_CCIPR2_SDMMCSEL_SYSCLK     (1 << RCC_CCIPR2_SDMMCSEL_SHIFT)

#define RCC_CCIPR2_OSPISEL_SHIFT         (20)    /* Bit 21-20: OCTOSPI clock source selection */
#define RCC_CCIPR2_OSPISEL_MASK          (3 << RCC_CCIPR2_OSPISEL_SHIFT)
#  define RCC_CCIPR2_OSPISEL_SYSCLK      (0 << RCC_CCIPR2_OSPISEL_SHIFT)
#  define RCC_CCIPR2_OSPISEL_MSI         (1 << RCC_CCIPR2_OSPISEL_SHIFT)
#  define RCC_CCIPR2_OSPISEL_PLL48M1CLK  (2 << RCC_CCIPR2_OSPISEL_SHIFT)
#endif

#endif /* CONFIG_STM32U5_STM32U585XX */
#endif /* __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32U585XX_RCC_H */
