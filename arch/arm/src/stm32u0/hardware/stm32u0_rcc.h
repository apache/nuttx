/****************************************************************************
 * arch/arm/src/stm32u0/hardware/stm32u0_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32U0_HARDWARE_STM32U0_RCC_H
#define __ARCH_ARM_SRC_STM32U0_HARDWARE_STM32U0_RCC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_RCC_CR_OFFSET           0x0000 /* Clock control register */
#define STM32_RCC_ICSCR_OFFSET        0x0004 /* Internal clock sources calibration register */
#define STM32_RCC_CFGR_OFFSET         0x0008 /* Clock configuration register */
#define STM32_RCC_PLLCFG_OFFSET       0x000c /* PLL clock configuration register */
#define STM32_RCC_CIER_OFFSET         0x0018 /* Clock interrupt enable register */
#define STM32_RCC_CIFR_OFFSET         0x001c /* Clock interrupt flag register */
#define STM32_RCC_CICR_OFFSET         0x0020 /* Clock interrupt clear register */
#define STM32_RCC_AHBRSTR_OFFSET      0x0028 /* AHB peripheral reset register */
#define STM32_RCC_IOPRSTR_OFFSET      0x002c /* GPIO reset register */
#define STM32_RCC_APB1RSTR_OFFSET     0x0038 /* APB1 peripheral reset register (APBRSTR1) */
#define STM32_RCC_APB2RSTR_OFFSET     0x0040 /* APB2 peripheral reset register (APBRSTR2) */
#define STM32_RCC_AHBENR_OFFSET       0x0048 /* AHB peripheral clock enable register */
#define STM32_RCC_IOPENR_OFFSET       0x004c /* GPIO clock enable register */
#define STM32_RCC_DBGCFGR_OFFSET      0x0050 /* Debug configuration register */
#define STM32_RCC_APB1ENR_OFFSET      0x0058 /* APB1 peripheral clock enable register (APBENR1) */
#define STM32_RCC_APB2ENR_OFFSET      0x0060 /* APB2 peripheral clock enable register (APBENR2) */
#define STM32_RCC_AHBSMENR_OFFSET     0x0068 /* AHB peripheral clock enable in Sleep mode register */
#define STM32_RCC_IOPSMENR_OFFSET     0x006c /* GPIO clock enable in Sleep mode register */
#define STM32_RCC_APB1SMENR_OFFSET    0x0078 /* APB1 peripheral clock enable in Sleep mode register */
#define STM32_RCC_APB2SMENR_OFFSET    0x0080 /* APB2 peripheral clock enable in Sleep mode register */
#define STM32_RCC_CCIPR_OFFSET        0x0088 /* Peripherals independent clock configuration register */
#define STM32_RCC_BDCR_OFFSET         0x0090 /* RTC domain control register */
#define STM32_RCC_CSR_OFFSET          0x0094 /* Control/status register */
#define STM32_RCC_CRRCR_OFFSET        0x0098 /* Clock recovery RC register */

/* Register Addresses *******************************************************/

#define STM32_RCC_CR                  (STM32_RCC_BASE+STM32_RCC_CR_OFFSET)
#define STM32_RCC_ICSCR               (STM32_RCC_BASE+STM32_RCC_ICSCR_OFFSET)
#define STM32_RCC_CFGR                (STM32_RCC_BASE+STM32_RCC_CFGR_OFFSET)
#define STM32_RCC_PLLCFG              (STM32_RCC_BASE+STM32_RCC_PLLCFG_OFFSET)
#define STM32_RCC_CIER                (STM32_RCC_BASE+STM32_RCC_CIER_OFFSET)
#define STM32_RCC_CIFR                (STM32_RCC_BASE+STM32_RCC_CIFR_OFFSET)
#define STM32_RCC_CICR                (STM32_RCC_BASE+STM32_RCC_CICR_OFFSET)
#define STM32_RCC_AHBRSTR             (STM32_RCC_BASE+STM32_RCC_AHBRSTR_OFFSET)
#define STM32_RCC_IOPRSTR             (STM32_RCC_BASE+STM32_RCC_IOPRSTR_OFFSET)
#define STM32_RCC_APB1RSTR            (STM32_RCC_BASE+STM32_RCC_APB1RSTR_OFFSET)
#define STM32_RCC_APB2RSTR            (STM32_RCC_BASE+STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_AHBENR              (STM32_RCC_BASE+STM32_RCC_AHBENR_OFFSET)
#define STM32_RCC_IOPENR              (STM32_RCC_BASE+STM32_RCC_IOPENR_OFFSET)
#define STM32_RCC_DBGCFGR             (STM32_RCC_BASE+STM32_RCC_DBGCFGR_OFFSET)
#define STM32_RCC_APB1ENR             (STM32_RCC_BASE+STM32_RCC_APB1ENR_OFFSET)
#define STM32_RCC_APB2ENR             (STM32_RCC_BASE+STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_AHBSMENR            (STM32_RCC_BASE+STM32_RCC_AHBSMENR_OFFSET)
#define STM32_RCC_IOPSMENR            (STM32_RCC_BASE+STM32_RCC_IOPSMENR_OFFSET)
#define STM32_RCC_APB1SMENR           (STM32_RCC_BASE+STM32_RCC_APB1SMENR_OFFSET)
#define STM32_RCC_APB2SMENR           (STM32_RCC_BASE+STM32_RCC_APB2SMENR_OFFSET)
#define STM32_RCC_CCIPR               (STM32_RCC_BASE+STM32_RCC_CCIPR_OFFSET)
#define STM32_RCC_BDCR                (STM32_RCC_BASE+STM32_RCC_BDCR_OFFSET)
#define STM32_RCC_CSR                 (STM32_RCC_BASE+STM32_RCC_CSR_OFFSET)
#define STM32_RCC_CRRCR               (STM32_RCC_BASE+STM32_RCC_CRRCR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Clock control register */

#define RCC_CR_MSION                  (1 << 0) /* Bit 0: MSI clock enable */
#define RCC_CR_MSIRDY                 (1 << 1) /* Bit 1: MSI clock ready flag */
#define RCC_CR_MSIPLLEN               (1 << 2) /* Bit 2: MSI clock PLL enable */
#define RCC_CR_MSIRGSEL               (1 << 3) /* Bit 3: MSI clock range selection */
#define RCC_CR_MSIRANGE_SHIFT         (4)      /* Bits 4-7: MSI clock ranges */
#define RCC_CR_MSIRANGE_MASK          (0x0f << RCC_CR_MSIRANGE_SHIFT)
#  define RCC_CR_MSIRANGE_0           (0 << RCC_CR_MSIRANGE_SHIFT)  /* 100 kHz */
#  define RCC_CR_MSIRANGE_1           (1 << RCC_CR_MSIRANGE_SHIFT)  /* 200 kHz */
#  define RCC_CR_MSIRANGE_2           (2 << RCC_CR_MSIRANGE_SHIFT)  /* 400 kHz */
#  define RCC_CR_MSIRANGE_3           (3 << RCC_CR_MSIRANGE_SHIFT)  /* 800 kHz */
#  define RCC_CR_MSIRANGE_4           (4 << RCC_CR_MSIRANGE_SHIFT)  /* 1 MHz */
#  define RCC_CR_MSIRANGE_5           (5 << RCC_CR_MSIRANGE_SHIFT)  /* 2 MHz */
#  define RCC_CR_MSIRANGE_6           (6 << RCC_CR_MSIRANGE_SHIFT)  /* 4 MHz */
#  define RCC_CR_MSIRANGE_7           (7 << RCC_CR_MSIRANGE_SHIFT)  /* 8 MHz */
#  define RCC_CR_MSIRANGE_8           (8 << RCC_CR_MSIRANGE_SHIFT)  /* 16 MHz */
#  define RCC_CR_MSIRANGE_9           (9 << RCC_CR_MSIRANGE_SHIFT)  /* 24 MHz */
#  define RCC_CR_MSIRANGE_10          (10 << RCC_CR_MSIRANGE_SHIFT) /* 32 MHz */
#  define RCC_CR_MSIRANGE_11          (11 << RCC_CR_MSIRANGE_SHIFT) /* 48 MHz */
#define RCC_CR_HSION                  (1 << 8)                      /* Bit 8: HSI16 clock enable */
#define RCC_CR_HSIKERON               (1 << 9)                      /* Bit 9: HSI16 clock enable for some IP kernels */
#define RCC_CR_HSIRDY                 (1 << 10)                     /* Bit 10: HSI16 clock ready flag */
#define RCC_CR_HSIASFS                (1 << 11)                     /* Bit 11: HSI16 automatic start from Stop */
                                                                    /* Bits 12-15: Reserved */
#define RCC_CR_HSEON                  (1 << 16)                     /* Bit 16: HSE clock enable */
#define RCC_CR_HSERDY                 (1 << 17)                     /* Bit 17: HSE clock ready flag */
#define RCC_CR_HSEBYP                 (1 << 18)                     /* Bit 18: HSE clock bypass */
#define RCC_CR_CSSON                  (1 << 19)                     /* Bit 19: HSE clock security system enable */
                                                                    /* Bits 20-23: Reserved */
#define RCC_CR_PLLON                  (1 << 24)                     /* Bit 24: PLL enable */
#define RCC_CR_PLLRDY                 (1 << 25)                     /* Bit 25: PLL clock ready flag */
                                                                    /* Bits 26-31: Reserved */

/* Internal clock sources calibration register */

#define RCC_ICSCR_MSICAL_SHIFT        (0) /* Bits 0-7: MSI clock calibration */
#define RCC_ICSCR_MSICAL_MASK         (0xff << RCC_ICSCR_MSICAL_SHIFT)
#define RCC_ICSCR_MSITRIM_SHIFT       (8) /* Bits 8-15: MSI clock trimming */
#define RCC_ICSCR_MSITRIM_MASK        (0xff << RCC_ICSCR_MSITRIM_SHIFT)
#define RCC_ICSCR_HSICAL_SHIFT        (16) /* Bits 16-23: HSI16 clock calibration */
#define RCC_ICSCR_HSICAL_MASK         (0xff << RCC_ICSCR_HSICAL_SHIFT)
#define RCC_ICSCR_HSITRIM_SHIFT       (24) /* Bits 24-30: HSI16 clock trimming */
#define RCC_ICSCR_HSITRIM_MASK        (0x7f << RCC_ICSCR_HSITRIM_SHIFT)

/* Clock configuration register */

#define RCC_CFGR_SW_SHIFT             (0) /* Bits 0-2: System clock switch */
#define RCC_CFGR_SW_MASK              (7 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_MSI             (0 << RCC_CFGR_SW_SHIFT) /* 000: MSI */
#  define RCC_CFGR_SW_HSI             (1 << RCC_CFGR_SW_SHIFT) /* 001: HSI16 */
#  define RCC_CFGR_SW_HSE             (2 << RCC_CFGR_SW_SHIFT) /* 010: HSE */
#  define RCC_CFGR_SW_PLL             (3 << RCC_CFGR_SW_SHIFT) /* 011: PLLRCLK */
#  define RCC_CFGR_SW_LSI             (4 << RCC_CFGR_SW_SHIFT) /* 100: LSI */
#  define RCC_CFGR_SW_LSE             (5 << RCC_CFGR_SW_SHIFT) /* 101: LSE */
#define RCC_CFGR_SWS_SHIFT            (3)                      /* Bits 3-5: System clock switch status */
#define RCC_CFGR_SWS_MASK             (7 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_MSI            (0 << RCC_CFGR_SWS_SHIFT) /* 000: MSI */
#  define RCC_CFGR_SWS_HSI            (1 << RCC_CFGR_SWS_SHIFT) /* 001: HSI16 */
#  define RCC_CFGR_SWS_HSE            (2 << RCC_CFGR_SWS_SHIFT) /* 010: HSE */
#  define RCC_CFGR_SWS_PLL            (3 << RCC_CFGR_SWS_SHIFT) /* 011: PLLRCLK */
#  define RCC_CFGR_SWS_LSI            (4 << RCC_CFGR_SWS_SHIFT) /* 100: LSI */
#  define RCC_CFGR_SWS_LSE            (5 << RCC_CFGR_SWS_SHIFT) /* 101: LSE */
                                                                /* Bits 6-7: Reserved */
#define RCC_CFGR_HPRE_SHIFT           (8)                       /* Bits 8-11: AHB prescaler */
#define RCC_CFGR_HPRE_MASK            (0x0f << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLK        (0 << RCC_CFGR_HPRE_SHIFT)  /* 0xxx: SYSCLK not divided */
#  define RCC_CFGR_HPRE_SYSCLKd2      (8 << RCC_CFGR_HPRE_SHIFT)  /* 1000: SYSCLK divided by 2 */
#  define RCC_CFGR_HPRE_SYSCLKd4      (9 << RCC_CFGR_HPRE_SHIFT)  /* 1001: SYSCLK divided by 4 */
#  define RCC_CFGR_HPRE_SYSCLKd8      (10 << RCC_CFGR_HPRE_SHIFT) /* 1010: SYSCLK divided by 8 */
#  define RCC_CFGR_HPRE_SYSCLKd16     (11 << RCC_CFGR_HPRE_SHIFT) /* 1011: SYSCLK divided by 16 */
#  define RCC_CFGR_HPRE_SYSCLKd64     (12 << RCC_CFGR_HPRE_SHIFT) /* 1100: SYSCLK divided by 64 */
#  define RCC_CFGR_HPRE_SYSCLKd128    (13 << RCC_CFGR_HPRE_SHIFT) /* 1101: SYSCLK divided by 128 */
#  define RCC_CFGR_HPRE_SYSCLKd256    (14 << RCC_CFGR_HPRE_SHIFT) /* 1110: SYSCLK divided by 256 */
#  define RCC_CFGR_HPRE_SYSCLKd512    (15 << RCC_CFGR_HPRE_SHIFT) /* 1111: SYSCLK divided by 512 */
#define RCC_CFGR_PPRE1_SHIFT          (12)                        /* Bits 12-14: APB prescaler (PPRE) */
#define RCC_CFGR_PPRE1_MASK           (7 << RCC_CFGR_PPRE1_SHIFT)
#  define RCC_CFGR_PPRE1_HCLK         (0 << RCC_CFGR_PPRE1_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE1_HCLKd2       (4 << RCC_CFGR_PPRE1_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE1_HCLKd4       (5 << RCC_CFGR_PPRE1_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE1_HCLKd8       (6 << RCC_CFGR_PPRE1_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE1_HCLKd16      (7 << RCC_CFGR_PPRE1_SHIFT) /* 111: HCLK divided by 16 */
#define RCC_CFGR_STOPWUCK             (1 << 15)                   /* Bit 15: Wakeup from Stop and CSS backup clock selection */
#define RCC_CFGR_MCO2SEL_SHIFT        (16)                        /* Bits 16-19: MCO2 clock output selection */
#define RCC_CFGR_MCO2SEL_MASK         (0x0f << RCC_CFGR_MCO2SEL_SHIFT)
#define RCC_CFGR_MCO2PRE_SHIFT        (20) /* Bits 20-23: MCO2 prescaler */
#define RCC_CFGR_MCO2PRE_MASK         (0x0f << RCC_CFGR_MCO2PRE_SHIFT)
#define RCC_CFGR_MCO1SEL_SHIFT        (24) /* Bits 24-27: MCO1 clock output selection */
#define RCC_CFGR_MCO1SEL_MASK         (0x0f << RCC_CFGR_MCO1SEL_SHIFT)
#define RCC_CFGR_MCO1PRE_SHIFT        (28) /* Bits 28-31: MCO1 prescaler */
#define RCC_CFGR_MCO1PRE_MASK         (0x0f << RCC_CFGR_MCO1PRE_SHIFT)

/* Compatibility aliases for MCO (MCO1) */

#define RCC_CFGR_MCOSEL_SHIFT         RCC_CFGR_MCO1SEL_SHIFT
#define RCC_CFGR_MCOSEL_MASK          RCC_CFGR_MCO1SEL_MASK
#define RCC_CFGR_MCOPRE_SHIFT         RCC_CFGR_MCO1PRE_SHIFT
#define RCC_CFGR_MCOPRE_MASK          RCC_CFGR_MCO1PRE_MASK

/* PLL clock configuration register */

#define RCC_PLLCFG_PLLSRC_SHIFT       (0) /* Bits 0-1: PLL entry clock source */
#define RCC_PLLCFG_PLLSRC_MASK        (3 << RCC_PLLCFG_PLLSRC_SHIFT)
#  define RCC_PLLCFG_PLLSRC_NOCLK     (0 << RCC_PLLCFG_PLLSRC_SHIFT) /* 00: No clock */
#  define RCC_PLLCFG_PLLSRC_MSI       (1 << RCC_PLLCFG_PLLSRC_SHIFT) /* 01: MSI */
#  define RCC_PLLCFG_PLLSRC_HSI       (2 << RCC_PLLCFG_PLLSRC_SHIFT) /* 10: HSI16 */
#  define RCC_PLLCFG_PLLSRC_HSE       (3 << RCC_PLLCFG_PLLSRC_SHIFT) /* 11: HSE */
#define RCC_PLLCFG_PLLM_SHIFT         (4)                            /* Bits 4-6: Division factor M of the PLL input clock divider */
#define RCC_PLLCFG_PLLM_MASK          (7 << RCC_PLLCFG_PLLM_SHIFT)
#define RCC_PLLCFG_PLLM(n)            ((n-1) << RCC_PLLCFG_PLLM_SHIFT) /* n=1,...,8 */
                                                                       /* Bit 7: Reserved */
#define RCC_PLLCFG_PLLN_SHIFT         (8)                              /* Bits 8-14: PLL frequency multiplication factor N */
#define RCC_PLLCFG_PLLN_MASK          (0x7f << RCC_PLLCFG_PLLN_SHIFT)
#define RCC_PLLCFG_PLLN(n)            ((n) << RCC_PLLCFG_PLLN_SHIFT) /* n=4,...,127 */
                                                                     /* Bit 15: Reserved */
#define RCC_PLLCFG_PLLPEN             (1 << 16)                      /* Bit 16: PLLPCLK clock output enable */
#define RCC_PLLCFG_PLLP_SHIFT         (17)                           /* Bits 17-21: PLL VCO division factor P for PLLPCLK clock output */
#define RCC_PLLCFG_PLLP_MASK          (0x1f << RCC_PLLCFG_PLLP_SHIFT)
#define RCC_PLLCFG_PLLP(n)            ((n-1) << RCC_PLLCFG_PLLP_SHIFT) /* n=2,...,32 */
                                                                       /* Bits 22-23: Reserved */
#define RCC_PLLCFG_PLLQEN             (1 << 24)                        /* Bit 24: PLLQCLK clock output enable */
#define RCC_PLLCFG_PLLQ_SHIFT         (25)                             /* Bits 25-27: PLL VCO division factor Q for PLLQCLK clock output */
#define RCC_PLLCFG_PLLQ_MASK          (7 << RCC_PLLCFG_PLLQ_SHIFT)
#define RCC_PLLCFG_PLLQ(n)            ((n-1) << RCC_PLLCFG_PLLQ_SHIFT) /* n=2,...,8 */
#define RCC_PLLCFG_PLLREN             (1 << 28)                        /* Bit 28: PLLRCLK clock output enable */
#define RCC_PLLCFG_PLLR_SHIFT         (29)                             /* Bits 29-31: PLL VCO division factor R for PLLRCLK clock output */
#define RCC_PLLCFG_PLLR_MASK          (7 << RCC_PLLCFG_PLLR_SHIFT)
#define RCC_PLLCFG_PLLR(n)            ((n-1) << RCC_PLLCFG_PLLR_SHIFT) /* n=2,...,8 */

/* Clock interrupt enable register */

#define RCC_CIER_LSIRDYIE             (1 << 0)  /* Bit 0: LSI ready interrupt enable */
#define RCC_CIER_LSERDYIE             (1 << 1)  /* Bit 1: LSE ready interrupt enable */
#define RCC_CIER_MSIRDYIE             (1 << 2)  /* Bit 2: MSI ready interrupt enable */
#define RCC_CIER_HSIRDYIE             (1 << 3)  /* Bit 3: HSI16 ready interrupt enable */
#define RCC_CIER_HSERDYIE             (1 << 4)  /* Bit 4: HSE ready interrupt enable */
#define RCC_CIER_PLLRDYIE             (1 << 5)  /* Bit 5: PLL ready interrupt enable */
                                                /* Bits 6-8: Reserved */
#define RCC_CIER_LSECSSIE             (1 << 9)  /* Bit 9: LSE clock security system interrupt enable */
#define RCC_CIER_HSI48RDYIE           (1 << 10) /* Bit 10: HSI48 ready interrupt enable */

/* Clock interrupt flag register */

#define RCC_CIFR_LSIRDYF              (1 << 0)  /* Bit 0: LSI ready interrupt flag */
#define RCC_CIFR_LSERDYF              (1 << 1)  /* Bit 1: LSE ready interrupt flag */
#define RCC_CIFR_MSIRDYF              (1 << 2)  /* Bit 2: MSI ready interrupt flag */
#define RCC_CIFR_HSIRDYF              (1 << 3)  /* Bit 3: HSI16 ready interrupt flag */
#define RCC_CIFR_HSERDYF              (1 << 4)  /* Bit 4: HSE ready interrupt flag */
#define RCC_CIFR_PLLRDYF              (1 << 5)  /* Bit 5: PLL ready interrupt flag */
                                                /* Bits 6-7: Reserved */
#define RCC_CIFR_CSSF                 (1 << 8)  /* Bit 8: HSE clock security system interrupt flag */
#define RCC_CIFR_LSECSSF              (1 << 9)  /* Bit 9: LSE clock security system interrupt flag */
#define RCC_CIFR_HSI48RDYF            (1 << 10) /* Bit 10: HSI48 ready interrupt flag */

/* Clock interrupt clear register */

#define RCC_CICR_LSIRDYC              (1 << 0)  /* Bit 0: LSI ready interrupt clear */
#define RCC_CICR_LSERDYC              (1 << 1)  /* Bit 1: LSE ready interrupt clear */
#define RCC_CICR_MSIRDYC              (1 << 2)  /* Bit 2: MSI ready interrupt clear */
#define RCC_CICR_HSIRDYC              (1 << 3)  /* Bit 3: HSI16 ready interrupt clear */
#define RCC_CICR_HSERDYC              (1 << 4)  /* Bit 4: HSE ready interrupt clear */
#define RCC_CICR_PLLRDYC              (1 << 5)  /* Bit 5: PLL ready interrupt clear */
                                                /* Bits 6-7: Reserved */
#define RCC_CICR_CSSC                 (1 << 8)  /* Bit 8: HSE clock security system interrupt clear */
#define RCC_CICR_LSECSSC              (1 << 9)  /* Bit 9: LSE clock security system interrupt clear */
#define RCC_CICR_HSI48RDYC            (1 << 10) /* Bit 10: HSI48 ready interrupt clear */

/* AHB peripheral reset register */

#define RCC_AHBRSTR_DMA1RST           (1 << 0)  /* Bit 0: DMA1 reset */
#define RCC_AHBRSTR_DMA2RST           (1 << 1)  /* Bit 1: DMA2 reset */
                                                /* Bits 2-7: Reserved */
#define RCC_AHBRSTR_FLASHRST          (1 << 8)  /* Bit 8: Flash memory interface reset */
                                                /* Bits 9-11: Reserved */
#define RCC_AHBRSTR_CRCRST            (1 << 12) /* Bit 12: CRC reset */
                                                /* Bits 13-15: Reserved */
#define RCC_AHBRSTR_AESRST            (1 << 16) /* Bit 16: AES hardware accelerator reset */
                                                /* Bit 17: Reserved */
#define RCC_AHBRSTR_RNGRST            (1 << 18) /* Bit 18: Random number generator reset */
                                                /* Bits 19-23: Reserved */
#define RCC_AHBRSTR_TSCRST            (1 << 24) /* Bit 24: Touch sensing controller reset */
                                                /* Bits 25-31: Reserved */

/* GPIO reset register */

#define RCC_IOPRSTR_IOPARST           (1 << 0) /* Bit 0: IO port A reset */
#define RCC_IOPRSTR_IOPBRST           (1 << 1) /* Bit 1: IO port B reset */
#define RCC_IOPRSTR_IOPCRST           (1 << 2) /* Bit 2: IO port C reset */
#define RCC_IOPRSTR_IOPDRST           (1 << 3) /* Bit 3: IO port D reset */
#define RCC_IOPRSTR_IOPERST           (1 << 4) /* Bit 4: IO port E reset */
#define RCC_IOPRSTR_IOPFRST           (1 << 5) /* Bit 5: IO port F reset */

/* APB1 peripheral reset register (APBRSTR1) */

#define RCC_APB1RSTR_TIM2RST          (1 << 0)  /* Bit 0:  Timer 2 reset */
#define RCC_APB1RSTR_TIM3RST          (1 << 1)  /* Bit 1:  Timer 3 reset */
                                                /* Bits 2-3: Reserved */
#define RCC_APB1RSTR_TIM6RST          (1 << 4)  /* Bit 4:  Timer 6 reset */
#define RCC_APB1RSTR_TIM7RST          (1 << 5)  /* Bit 5:  Timer 7 reset */
                                                /* Bit 6: Reserved */
#define RCC_APB1RSTR_LPUART2RST       (1 << 7)  /* Bit 7:  Low-power UART 2 reset */
                                                /* Bit 8: Reserved */
#define RCC_APB1RSTR_LCDRST           (1 << 9)  /* Bit 9:  LCD controller reset */
                                                /* Bits 10-11: Reserved */
#define RCC_APB1RSTR_LPUART3RST       (1 << 12) /* Bit 12: Low-power UART 3 reset */
#define RCC_APB1RSTR_USBRST           (1 << 13) /* Bit 13: USB reset */
#define RCC_APB1RSTR_SPI2RST          (1 << 14) /* Bit 14: SPI2 reset */
#define RCC_APB1RSTR_SPI3RST          (1 << 15) /* Bit 15: SPI3 reset */
#define RCC_APB1RSTR_CRSRST           (1 << 16) /* Bit 16: CRS reset */
#define RCC_APB1RSTR_USART2RST        (1 << 17) /* Bit 17: USART2 reset */
#define RCC_APB1RSTR_USART3RST        (1 << 18) /* Bit 18: USART3 reset */
#define RCC_APB1RSTR_USART4RST        (1 << 19) /* Bit 19: USART4 reset */
#define RCC_APB1RSTR_LPUART1RST       (1 << 20) /* Bit 20: Low-power UART 1 reset */
#define RCC_APB1RSTR_I2C1RST          (1 << 21) /* Bit 21: I2C1 reset */
#define RCC_APB1RSTR_I2C2RST          (1 << 22) /* Bit 22: I2C2 reset */
#define RCC_APB1RSTR_I2C3RST          (1 << 23) /* Bit 23: I2C3 reset */
#define RCC_APB1RSTR_OPAMPRST         (1 << 24) /* Bit 24: OPAMP reset */
#define RCC_APB1RSTR_I2C4RST          (1 << 25) /* Bit 25: I2C4 reset */
#define RCC_APB1RSTR_LPTIM3RST        (1 << 26) /* Bit 26: Low-power timer 3 reset */
                                                /* Bit 27: Reserved */
#define RCC_APB1RSTR_PWRRST           (1 << 28) /* Bit 28: PWR reset */
#define RCC_APB1RSTR_DAC1RST          (1 << 29) /* Bit 29: DAC1 reset */
#define RCC_APB1RSTR_LPTIM2RST        (1 << 30) /* Bit 30: Low-power timer 2 reset */
#define RCC_APB1RSTR_LPTIM1RST        (1 << 31) /* Bit 31: Low-power timer 1 reset */

/* APB2 peripheral reset register (APBRSTR2) */

#define RCC_APB2RSTR_SYSCFGRST        (1 << 0)  /* Bit 0:  SYSCFG reset */
                                                /* Bits 1-10: Reserved */
#define RCC_APB2RSTR_TIM1RST          (1 << 11) /* Bit 11: TIM1 timer reset */
#define RCC_APB2RSTR_SPI1RST          (1 << 12) /* Bit 12: SPI1 reset */
                                                /* Bit 13: Reserved */
#define RCC_APB2RSTR_USART1RST        (1 << 14) /* Bit 14: USART1 reset */
                                                /* Bit 15: Reserved */
#define RCC_APB2RSTR_TIM15RST         (1 << 16) /* Bit 16: TIM15 timer reset */
#define RCC_APB2RSTR_TIM16RST         (1 << 17) /* Bit 17: TIM16 timer reset */
                                                /* Bits 18-19: Reserved */
#define RCC_APB2RSTR_ADC1RST          (1 << 20) /* Bit 20: ADC reset */

/* AHB peripheral clock enable register */

#define RCC_AHBENR_DMA1EN             (1 << 0)  /* Bit 0: DMA1 clock enable */
#define RCC_AHBENR_DMA2EN             (1 << 1)  /* Bit 1: DMA2 clock enable */
                                                /* Bits 2-7: Reserved */
#define RCC_AHBENR_FLASHEN            (1 << 8)  /* Bit 8: Flash memory interface clock enable */
                                                /* Bits 9-11: Reserved */
#define RCC_AHBENR_CRCEN              (1 << 12) /* Bit 12: CRC clock enable */
                                                /* Bits 13-15: Reserved */
#define RCC_AHBENR_AESEN              (1 << 16) /* Bit 16: AES hardware accelerator clock enable */
                                                /* Bit 17: Reserved */
#define RCC_AHBENR_RNGEN              (1 << 18) /* Bit 18: Random number generator clock enable */
                                                /* Bits 19-23: Reserved */
#define RCC_AHBENR_TSCEN              (1 << 24) /* Bit 24: Touch sensing controller clock enable */
                                                /* Bits 25-31: Reserved */

/* GPIO clock enable register */

#define RCC_IOPENR_IOPAEN             (1 << 0) /* Bit 0: IO port A clock enable */
#define RCC_IOPENR_IOPBEN             (1 << 1) /* Bit 1: IO port B clock enable */
#define RCC_IOPENR_IOPCEN             (1 << 2) /* Bit 2: IO port C clock enable */
#define RCC_IOPENR_IOPDEN             (1 << 3) /* Bit 3: IO port D clock enable */
#define RCC_IOPENR_IOPEEN             (1 << 4) /* Bit 4: IO port E clock enable */
#define RCC_IOPENR_IOPFEN             (1 << 5) /* Bit 5: IO port F clock enable */

/* Debug configuration register */

#define RCC_DBGCFGR_DBGEN             (1 << 0) /* Bit 0: Debug support clock enable */
#define RCC_DBGCFGR_DBGRST            (1 << 1) /* Bit 1: Debug support reset */

/* APB1 peripheral clock enable register (APBENR1) */

#define RCC_APB1ENR_TIM2EN            (1 << 0)  /* Bit 0:  Timer 2 clock enable */
#define RCC_APB1ENR_TIM3EN            (1 << 1)  /* Bit 1:  Timer 3 clock enable */
                                                /* Bits 2-3: Reserved */
#define RCC_APB1ENR_TIM6EN            (1 << 4)  /* Bit 4:  Timer 6 clock enable */
#define RCC_APB1ENR_TIM7EN            (1 << 5)  /* Bit 5:  Timer 7 clock enable */
                                                /* Bit 6: Reserved */
#define RCC_APB1ENR_LPUART2EN         (1 << 7)  /* Bit 7:  Low-power UART 2 clock enable */
                                                /* Bit 8: Reserved */
#define RCC_APB1ENR_LCDEN             (1 << 9)  /* Bit 9:  LCD controller clock enable */
#define RCC_APB1ENR_RTCAPBEN          (1 << 10) /* Bit 10: RTC APB clock enable */
#define RCC_APB1ENR_WWDGEN            (1 << 11) /* Bit 11: Window watchdog clock enable */
#define RCC_APB1ENR_LPUART3EN         (1 << 12) /* Bit 12: Low-power UART 3 clock enable */
#define RCC_APB1ENR_USBEN             (1 << 13) /* Bit 13: USB clock enable */
#define RCC_APB1ENR_SPI2EN            (1 << 14) /* Bit 14: SPI2 clock enable */
#define RCC_APB1ENR_SPI3EN            (1 << 15) /* Bit 15: SPI3 clock enable */
#define RCC_APB1ENR_CRSEN             (1 << 16) /* Bit 16: CRS clock enable */
#define RCC_APB1ENR_USART2EN          (1 << 17) /* Bit 17: USART2 clock enable */
#define RCC_APB1ENR_USART3EN          (1 << 18) /* Bit 18: USART3 clock enable */
#define RCC_APB1ENR_USART4EN          (1 << 19) /* Bit 19: USART4 clock enable */
#define RCC_APB1ENR_LPUART1EN         (1 << 20) /* Bit 20: Low-power UART 1 clock enable */
#define RCC_APB1ENR_I2C1EN            (1 << 21) /* Bit 21: I2C1 clock enable */
#define RCC_APB1ENR_I2C2EN            (1 << 22) /* Bit 22: I2C2 clock enable */
#define RCC_APB1ENR_I2C3EN            (1 << 23) /* Bit 23: I2C3 clock enable */
#define RCC_APB1ENR_OPAMPEN           (1 << 24) /* Bit 24: OPAMP clock enable */
#define RCC_APB1ENR_I2C4EN            (1 << 25) /* Bit 25: I2C4 clock enable */
#define RCC_APB1ENR_LPTIM3EN          (1 << 26) /* Bit 26: Low-power timer 3 clock enable */
                                                /* Bit 27: Reserved */
#define RCC_APB1ENR_PWREN             (1 << 28) /* Bit 28: PWR clock enable */
#define RCC_APB1ENR_DAC1EN            (1 << 29) /* Bit 29: DAC1 clock enable */
#define RCC_APB1ENR_LPTIM2EN          (1 << 30) /* Bit 30: Low-power timer 2 clock enable */
#define RCC_APB1ENR_LPTIM1EN          (1 << 31) /* Bit 31: Low-power timer 1 clock enable */

/* APB2 peripheral clock enable register (APBENR2) */

#define RCC_APB2ENR_SYSCFGEN          (1 << 0)  /* Bit 0:  SYSCFG clock enable */
                                                /* Bits 1-10: Reserved */
#define RCC_APB2ENR_TIM1EN            (1 << 11) /* Bit 11: TIM1 timer clock enable */
#define RCC_APB2ENR_SPI1EN            (1 << 12) /* Bit 12: SPI1 clock enable */
                                                /* Bit 13: Reserved */
#define RCC_APB2ENR_USART1EN          (1 << 14) /* Bit 14: USART1 clock enable */
                                                /* Bit 15: Reserved */
#define RCC_APB2ENR_TIM15EN           (1 << 16) /* Bit 16: TIM15 timer clock enable */
#define RCC_APB2ENR_TIM16EN           (1 << 17) /* Bit 17: TIM16 timer clock enable */
                                                /* Bits 18-19: Reserved */
#define RCC_APB2ENR_ADC1EN            (1 << 20) /* Bit 20: ADC clock enable */

/* AHB peripheral clock enable in Sleep mode register */

#define RCC_AHBSMENR_DMA1SMEN         (1 << 0)  /* Bit 0: DMA1 clock enable in Sleep mode */
#define RCC_AHBSMENR_DMA2SMEN         (1 << 1)  /* Bit 1: DMA2 clock enable in Sleep mode */
                                                /* Bits 2-7: Reserved */
#define RCC_AHBSMENR_FLASHSMEN        (1 << 8)  /* Bit 8: Flash memory interface clock enable in Sleep mode */
#define RCC_AHBSMENR_SRAM1SMEN        (1 << 9)  /* Bit 9: SRAM1 clock enable in Sleep mode */
                                                /* Bits 10-11: Reserved */
#define RCC_AHBSMENR_CRCSMEN          (1 << 12) /* Bit 12: CRC clock enable in Sleep mode */
                                                /* Bits 13-15: Reserved */
#define RCC_AHBSMENR_AESSMEN          (1 << 16) /* Bit 16: AES clock enable in Sleep mode */
                                                /* Bit 17: Reserved */
#define RCC_AHBSMENR_RNGSMEN          (1 << 18) /* Bit 18: RNG clock enable in Sleep mode */
                                                /* Bits 19-23: Reserved */
#define RCC_AHBSMENR_TSCSMEN          (1 << 24) /* Bit 24: TSC clock enable in Sleep mode */

/* GPIO clock enable in Sleep mode register */

#define RCC_IOPSMENR_IOPASMEN         (1 << 0) /* Bit 0: IO port A clock enable in Sleep mode */
#define RCC_IOPSMENR_IOPBSMEN         (1 << 1) /* Bit 1: IO port B clock enable in Sleep mode */
#define RCC_IOPSMENR_IOPCSMEN         (1 << 2) /* Bit 2: IO port C clock enable in Sleep mode */
#define RCC_IOPSMENR_IOPDSMEN         (1 << 3) /* Bit 3: IO port D clock enable in Sleep mode */
#define RCC_IOPSMENR_IOPESMEN         (1 << 4) /* Bit 4: IO port E clock enable in Sleep mode */
#define RCC_IOPSMENR_IOPFSMEN         (1 << 5) /* Bit 5: IO port F clock enable in Sleep mode */

/* APB1 peripheral clock enable in Sleep mode register (APBSMENR1) */

#define RCC_APB1SMENR_TIM2SMEN        (1 << 0)  /* Bit 0:  Timer 2 clock enable in Sleep mode */
#define RCC_APB1SMENR_TIM3SMEN        (1 << 1)  /* Bit 1:  Timer 3 clock enable in Sleep mode */
                                                /* Bits 2-3: Reserved */
#define RCC_APB1SMENR_TIM6SMEN        (1 << 4)  /* Bit 4:  Timer 6 clock enable in Sleep mode */
#define RCC_APB1SMENR_TIM7SMEN        (1 << 5)  /* Bit 5:  Timer 7 clock enable in Sleep mode */
                                                /* Bit 6: Reserved */
#define RCC_APB1SMENR_LPUART2SMEN     (1 << 7)  /* Bit 7:  LPUART2 clock enable in Sleep mode */
                                                /* Bit 8: Reserved */
#define RCC_APB1SMENR_LCDSMEN         (1 << 9)  /* Bit 9:  LCD clock enable in Sleep mode */
#define RCC_APB1SMENR_RTCAPBSMEN      (1 << 10) /* Bit 10: RTC APB clock enable in Sleep mode */
#define RCC_APB1SMENR_WWDGSMEN        (1 << 11) /* Bit 11: WWDG clock enable in Sleep mode */
#define RCC_APB1SMENR_LPUART3SMEN     (1 << 12) /* Bit 12: LPUART3 clock enable in Sleep mode */
#define RCC_APB1SMENR_USBSMEN         (1 << 13) /* Bit 13: USB clock enable in Sleep mode */
#define RCC_APB1SMENR_SPI2SMEN        (1 << 14) /* Bit 14: SPI2 clock enable in Sleep mode */
#define RCC_APB1SMENR_SPI3SMEN        (1 << 15) /* Bit 15: SPI3 clock enable in Sleep mode */
#define RCC_APB1SMENR_CRSSMEN         (1 << 16) /* Bit 16: CRS clock enable in Sleep mode */
#define RCC_APB1SMENR_USART2SMEN      (1 << 17) /* Bit 17: USART2 clock enable in Sleep mode */
#define RCC_APB1SMENR_USART3SMEN      (1 << 18) /* Bit 18: USART3 clock enable in Sleep mode */
#define RCC_APB1SMENR_USART4SMEN      (1 << 19) /* Bit 19: USART4 clock enable in Sleep mode */
#define RCC_APB1SMENR_LPUART1SMEN     (1 << 20) /* Bit 20: LPUART1 clock enable in Sleep mode */
#define RCC_APB1SMENR_I2C1SMEN        (1 << 21) /* Bit 21: I2C1 clock enable in Sleep mode */
#define RCC_APB1SMENR_I2C2SMEN        (1 << 22) /* Bit 22: I2C2 clock enable in Sleep mode */
#define RCC_APB1SMENR_I2C3SMEN        (1 << 23) /* Bit 23: I2C3 clock enable in Sleep mode */
#define RCC_APB1SMENR_OPAMPSMEN       (1 << 24) /* Bit 24: OPAMP clock enable in Sleep mode */
#define RCC_APB1SMENR_I2C4SMEN        (1 << 25) /* Bit 25: I2C4 clock enable in Sleep mode */
#define RCC_APB1SMENR_LPTIM3SMEN      (1 << 26) /* Bit 26: LPTIM3 clock enable in Sleep mode */
                                                /* Bit 27: Reserved */
#define RCC_APB1SMENR_PWRSMEN         (1 << 28) /* Bit 28: PWR clock enable in Sleep mode */
#define RCC_APB1SMENR_DAC1SMEN        (1 << 29) /* Bit 29: DAC1 clock enable in Sleep mode */
#define RCC_APB1SMENR_LPTIM2SMEN      (1 << 30) /* Bit 30: LPTIM2 clock enable in Sleep mode */
#define RCC_APB1SMENR_LPTIM1SMEN      (1 << 31) /* Bit 31: LPTIM1 clock enable in Sleep mode */

/* APB2 peripheral clock enable in Sleep mode register (APBSMENR2) */

#define RCC_APB2SMENR_SYSCFGSMEN      (1 << 0)  /* Bit 0:  SYSCFG clock enable in Sleep mode */
                                                /* Bits 1-10: Reserved */
#define RCC_APB2SMENR_TIM1SMEN        (1 << 11) /* Bit 11: TIM1 clock enable in Sleep mode */
#define RCC_APB2SMENR_SPI1SMEN        (1 << 12) /* Bit 12: SPI1 clock enable in Sleep mode */
                                                /* Bit 13: Reserved */
#define RCC_APB2SMENR_USART1SMEN      (1 << 14) /* Bit 14: USART1 clock enable in Sleep mode */
                                                /* Bit 15: Reserved */
#define RCC_APB2SMENR_TIM15SMEN       (1 << 16) /* Bit 16: TIM15 clock enable in Sleep mode */
#define RCC_APB2SMENR_TIM16SMEN       (1 << 17) /* Bit 17: TIM16 clock enable in Sleep mode */
                                                /* Bits 18-19: Reserved */
#define RCC_APB2SMENR_ADC1SMEN        (1 << 20) /* Bit 20: ADC clock enable in Sleep mode */

/* Peripherals independent clock configuration register */

#define RCC_CCIPR_USART1SEL_SHIFT     (0) /* Bits 0-1: USART1 clock source selection */
#define RCC_CCIPR_USART1SEL_MASK      (3 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_PCLK    (0 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_SYSCLK  (1 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_HSI16   (2 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_LSE     (3 << RCC_CCIPR_USART1SEL_SHIFT)
#define RCC_CCIPR_USART2SEL_SHIFT     (2) /* Bits 2-3: USART2 clock source selection */
#define RCC_CCIPR_USART2SEL_MASK      (3 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_PCLK    (0 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_SYSCLK  (1 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_HSI16   (2 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_LSE     (3 << RCC_CCIPR_USART2SEL_SHIFT)
                                          /* Bits 4-5: Reserved */
#define RCC_CCIPR_LPUART3SEL_SHIFT    (6) /* Bits 6-7: LPUART3 clock source selection */
#define RCC_CCIPR_LPUART3SEL_MASK     (3 << RCC_CCIPR_LPUART3SEL_SHIFT)
#  define RCC_CCIPR_LPUART3SEL_PCLK   (0 << RCC_CCIPR_LPUART3SEL_SHIFT)
#  define RCC_CCIPR_LPUART3SEL_SYSCLK (1 << RCC_CCIPR_LPUART3SEL_SHIFT)
#  define RCC_CCIPR_LPUART3SEL_HSI16  (2 << RCC_CCIPR_LPUART3SEL_SHIFT)
#  define RCC_CCIPR_LPUART3SEL_LSE    (3 << RCC_CCIPR_LPUART3SEL_SHIFT)
#define RCC_CCIPR_LPUART2SEL_SHIFT    (8) /* Bits 8-9: LPUART2 clock source selection */
#define RCC_CCIPR_LPUART2SEL_MASK     (3 << RCC_CCIPR_LPUART2SEL_SHIFT)
#  define RCC_CCIPR_LPUART2SEL_PCLK   (0 << RCC_CCIPR_LPUART2SEL_SHIFT)
#  define RCC_CCIPR_LPUART2SEL_SYSCLK (1 << RCC_CCIPR_LPUART2SEL_SHIFT)
#  define RCC_CCIPR_LPUART2SEL_HSI16  (2 << RCC_CCIPR_LPUART2SEL_SHIFT)
#  define RCC_CCIPR_LPUART2SEL_LSE    (3 << RCC_CCIPR_LPUART2SEL_SHIFT)
#define RCC_CCIPR_LPUART1SEL_SHIFT    (10) /* Bits 10-11: LPUART1 clock source selection */
#define RCC_CCIPR_LPUART1SEL_MASK     (3 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_PCLK   (0 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_SYSCLK (1 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_HSI16  (2 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_LSE    (3 << RCC_CCIPR_LPUART1SEL_SHIFT)
#define RCC_CCIPR_I2C1SEL_SHIFT       (12) /* Bits 12-13: I2C1 clock source selection */
#define RCC_CCIPR_I2C1SEL_MASK        (3 << RCC_CCIPR_I2C1SEL_SHIFT)
#  define RCC_CCIPR_I2C1SEL_PCLK      (0 << RCC_CCIPR_I2C1SEL_SHIFT)
#  define RCC_CCIPR_I2C1SEL_SYSCLK    (1 << RCC_CCIPR_I2C1SEL_SHIFT)
#  define RCC_CCIPR_I2C1SEL_HSI16     (2 << RCC_CCIPR_I2C1SEL_SHIFT)
                                           /* Bits 14-15: Reserved */
#define RCC_CCIPR_I2C3SEL_SHIFT       (16) /* Bits 16-17: I2C3 clock source selection */
#define RCC_CCIPR_I2C3SEL_MASK        (3 << RCC_CCIPR_I2C3SEL_SHIFT)
#  define RCC_CCIPR_I2C3SEL_PCLK      (0 << RCC_CCIPR_I2C3SEL_SHIFT)
#  define RCC_CCIPR_I2C3SEL_SYSCLK    (1 << RCC_CCIPR_I2C3SEL_SHIFT)
#  define RCC_CCIPR_I2C3SEL_HSI16     (2 << RCC_CCIPR_I2C3SEL_SHIFT)
#define RCC_CCIPR_LPTIM1SEL_SHIFT     (18) /* Bits 18-19: LPTIM1 clock source selection */
#define RCC_CCIPR_LPTIM1SEL_MASK      (3 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_PCLK    (0 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_LSI     (1 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_HSI16   (2 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_LSE     (3 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#define RCC_CCIPR_LPTIM2SEL_SHIFT     (20) /* Bits 20-21: LPTIM2 clock source selection */
#define RCC_CCIPR_LPTIM2SEL_MASK      (3 << RCC_CCIPR_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR_LPTIM2SEL_PCLK    (0 << RCC_CCIPR_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR_LPTIM2SEL_LSI     (1 << RCC_CCIPR_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR_LPTIM2SEL_HSI16   (2 << RCC_CCIPR_LPTIM2SEL_SHIFT)
#  define RCC_CCIPR_LPTIM2SEL_LSE     (3 << RCC_CCIPR_LPTIM2SEL_SHIFT)
#define RCC_CCIPR_LPTIM3SEL_SHIFT     (22) /* Bits 22-23: LPTIM3 clock source selection */
#define RCC_CCIPR_LPTIM3SEL_MASK      (3 << RCC_CCIPR_LPTIM3SEL_SHIFT)
#  define RCC_CCIPR_LPTIM3SEL_PCLK    (0 << RCC_CCIPR_LPTIM3SEL_SHIFT)
#  define RCC_CCIPR_LPTIM3SEL_LSI     (1 << RCC_CCIPR_LPTIM3SEL_SHIFT)
#  define RCC_CCIPR_LPTIM3SEL_HSI16   (2 << RCC_CCIPR_LPTIM3SEL_SHIFT)
#  define RCC_CCIPR_LPTIM3SEL_LSE     (3 << RCC_CCIPR_LPTIM3SEL_SHIFT)
#define RCC_CCIPR_TIM1SEL             (1 << 24) /* Bit 24: TIM1 clock source (0=TIMPCLK, 1=PLLQCLK) */
#define RCC_CCIPR_TIM15SEL            (1 << 25) /* Bit 25: TIM15 clock source (0=TIMPCLK, 1=PLLQCLK) */
#define RCC_CCIPR_CLK48SEL_SHIFT      (26)      /* Bits 26-27: 48 MHz clock source selection */
#define RCC_CCIPR_CLK48SEL_MASK       (3 << RCC_CCIPR_CLK48SEL_SHIFT)
#  define RCC_CCIPR_CLK48SEL_HSI48    (0 << RCC_CCIPR_CLK48SEL_SHIFT)
#  define RCC_CCIPR_CLK48SEL_MSI      (1 << RCC_CCIPR_CLK48SEL_SHIFT)
#  define RCC_CCIPR_CLK48SEL_PLLQCLK  (2 << RCC_CCIPR_CLK48SEL_SHIFT)
#define RCC_CCIPR_ADCSEL_SHIFT        (28) /* Bits 28-29: ADC clock source selection */
#define RCC_CCIPR_ADCSEL_MASK         (3 << RCC_CCIPR_ADCSEL_SHIFT)
#  define RCC_CCIPR_ADCSEL_SYSCLK     (0 << RCC_CCIPR_ADCSEL_SHIFT)
#  define RCC_CCIPR_ADCSEL_PLLPCLK    (1 << RCC_CCIPR_ADCSEL_SHIFT)
#  define RCC_CCIPR_ADCSEL_HSI16      (2 << RCC_CCIPR_ADCSEL_SHIFT)

/* RTC domain control register */

#define RCC_BDCR_LSEON                (1 << 0) /* Bit 0: LSE enable */
#define RCC_BDCR_LSERDY               (1 << 1) /* Bit 1: LSE ready */
#define RCC_BDCR_LSEBYP               (1 << 2) /* Bit 2: LSE bypass */
#define RCC_BDCR_LSEDRV_SHIFT         (3)      /* Bits 3-4: LSE driving capability */
#define RCC_BDCR_LSEDRV_MASK          (3 << RCC_BDCR_LSEDRV_SHIFT)
#  define RCC_BDCR_LSEDRV_LOW         (0 << RCC_BDCR_LSEDRV_SHIFT) /* 00: Low */
#  define RCC_BDCR_LSEDRV_MEDLO       (1 << RCC_BDCR_LSEDRV_SHIFT) /* 01: Medium low */
#  define RCC_BDCR_LSEDRV_MEDHI       (2 << RCC_BDCR_LSEDRV_SHIFT) /* 10: Medium high */
#  define RCC_BDCR_LSEDRV_HIGH        (3 << RCC_BDCR_LSEDRV_SHIFT) /* 11: High */
#define RCC_BDCR_LSECSSON             (1 << 5)                     /* Bit 5: CSS on LSE enable */
#define RCC_BDCR_LSECSSD              (1 << 6)                     /* Bit 6: CSS on LSE failure detection */
#define RCC_BDCR_LSESYSEN             (1 << 7)                     /* Bit 7: LSE clock enable for system usage */
#define RCC_BDCR_RTCSEL_SHIFT         (8)                          /* Bits 8-9: RTC clock source selection */
#define RCC_BDCR_RTCSEL_MASK          (3 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_NOCLK       (0 << RCC_BDCR_RTCSEL_SHIFT) /* 00: No clock */
#  define RCC_BDCR_RTCSEL_LSE         (1 << RCC_BDCR_RTCSEL_SHIFT) /* 01: LSE */
#  define RCC_BDCR_RTCSEL_LSI         (2 << RCC_BDCR_RTCSEL_SHIFT) /* 10: LSI */
#  define RCC_BDCR_RTCSEL_HSE         (3 << RCC_BDCR_RTCSEL_SHIFT) /* 11: HSE/32 */
                                                                   /* Bit 10: Reserved */
#define RCC_BDCR_LSESYSRDY            (1 << 11)                    /* Bit 11: LSE clock ready for system usage */
                                                                   /* Bits 12-14: Reserved */
#define RCC_BDCR_RTCEN                (1 << 15)                    /* Bit 15: RTC clock enable */
#define RCC_BDCR_BDRST                (1 << 16)                    /* Bit 16: RTC domain software reset */
                                                                   /* Bits 17-23: Reserved */
#define RCC_BDCR_LSCOEN               (1 << 24)                    /* Bit 24: Low-speed clock output enable */
#define RCC_BDCR_LSCOSEL              (1 << 25)                    /* Bit 25: Low-speed clock output selection (0=LSI, 1=LSE) */

/* Control/status register */

#define RCC_CSR_LSION                 (1 << 0) /* Bit 0: LSI enable */
#define RCC_CSR_LSIRDY                (1 << 1) /* Bit 1: LSI ready */
#define RCC_CSR_LSIPREDIV             (1 << 2) /* Bit 2: LSI prescaler (0=div 1, 1=div 128) */
                                               /* Bits 3-7: Reserved */
#define RCC_CSR_MSISTBYRG_SHIFT       (8)      /* Bits 8-11: MSI range after Standby mode */
#define RCC_CSR_MSISTBYRG_MASK        (0x0f << RCC_CSR_MSISTBYRG_SHIFT)
#  define RCC_CSR_MSISTBYRG_1M        (4 << RCC_CSR_MSISTBYRG_SHIFT) /* 1 MHz */
#  define RCC_CSR_MSISTBYRG_2M        (5 << RCC_CSR_MSISTBYRG_SHIFT) /* 2 MHz */
#  define RCC_CSR_MSISTBYRG_4M        (6 << RCC_CSR_MSISTBYRG_SHIFT) /* 4 MHz */
#  define RCC_CSR_MSISTBYRG_8M        (7 << RCC_CSR_MSISTBYRG_SHIFT) /* 8 MHz */
                                                                     /* Bits 12-22: Reserved */
#define RCC_CSR_RMVF                  (1 << 23)                      /* Bit 23: Remove reset flags */
                                                                     /* Bit 24: Reserved */
#define RCC_CSR_OBLRSTF               (1 << 25)                      /* Bit 25: Option bytes loading reset flag */
#define RCC_CSR_PINRSTF               (1 << 26)                      /* Bit 26: PIN reset flag */
#define RCC_CSR_PWRRSTF               (1 << 27)                      /* Bit 27: BOR or POR/PDR reset flag */
#define RCC_CSR_SFTRSTF               (1 << 28)                      /* Bit 28: Software reset flag */
#define RCC_CSR_IWDGRSTF              (1 << 29)                      /* Bit 29: IWDG reset flag */
#define RCC_CSR_WWDGRSTF              (1 << 30)                      /* Bit 30: WWDG reset flag */
#define RCC_CSR_LPWRRSTF              (1 << 31)                      /* Bit 31: Low-power reset flag */

/* Clock recovery RC register */

#define RCC_CRRCR_HSI48ON             (1 << 0) /* Bit 0: HSI48 clock enable */
#define RCC_CRRCR_HSI48RDY            (1 << 1) /* Bit 1: HSI48 clock ready flag */
                                               /* Bits 2-6: Reserved */
#define RCC_CRRCR_HSI48CAL_SHIFT      (7)      /* Bits 7-15: HSI48 clock calibration */
#define RCC_CRRCR_HSI48CAL_MASK       (0x1ff << RCC_CRRCR_HSI48CAL_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32U0_HARDWARE_STM32U0_RCC_H */
