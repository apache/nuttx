/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32g4xxxx_rcc.h
 *  Register offsets, addresses, and bitfield defines for STM32G4Xxxx RCC
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_RCC_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_RCC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Abbreviations:
 * HSE - High Speed External (clock)
 * HSI - High Speed Internal (clock)
 * POR - Power On Reset
 * RCC - Reset and Clock Control
 */

/* Register Offsets *********************************************************/

#define STM32_RCC_CR_OFFSET            0x0000              /* RCC Clock Control */
#define STM32_RCC_ICSCR_OFFSET         0x0004              /* RCC Internal Clock Sources Calibration */
#define STM32_RCC_CFGR_OFFSET          0x0008              /* RCC Clock Configuration */
#define STM32_RCC_PLLCFGR_OFFSET       0x000c              /* RCC PLL configuration register */
                                                           /* Offset 0x0010 Reserved */
                                                           /* Offset 0x0014 Reserved */
#define STM32_RCC_CIER_OFFSET          0x0018              /* RCC Clock Interrupt Enable */
#define STM32_RCC_CIFR_OFFSET          0x001c              /* RCC Clock Interrupt Flag */
#define STM32_RCC_CICR_OFFSET          0x0020              /* RCC Clock Interrupt Clear */
                                                           /* Offset 0x0024 Reserved */
#define STM32_RCC_AHB1RSTR_OFFSET      0x0028              /* RCC AHB1 Peripheral Reset */
#define STM32_RCC_AHB2RSTR_OFFSET      0x002c              /* RCC AHB2 Peripheral Reset */
#define STM32_RCC_AHB3RSTR_OFFSET      0x0030              /* RCC AHB3 Peripheral Reset */
                                                           /* Offset 0x0034 Reserved */
#define STM32_RCC_APB1RSTR1_OFFSET     0x0038              /* RCC APB1 Peripheral Reset Register 1 */
#define STM32_RCC_APB1RSTR2_OFFSET     0x003c              /* RCC APB1 Peripheral Reset Register 2 */
#define STM32_RCC_APB2RSTR_OFFSET      0x0040              /* RCC APB2 Peripheral Reset */
                                                           /* Offset 0x0044 Reserved */
#define STM32_RCC_AHB1ENR_OFFSET       0x0048              /* RCC AHB1 Peripheral Clock Enable */
#define STM32_RCC_AHB2ENR_OFFSET       0x004c              /* RCC AHB2 Peripheral Clock Enable */
#define STM32_RCC_AHB3ENR_OFFSET       0x0050              /* RCC AHB3 Peripheral Clock Enable */
                                                           /* Offset 0x0054 Reserved */
#define STM32_RCC_APB1ENR1_OFFSET      0x0058              /* RCC APB1 Peripheral Clock Enable Register 1 */
#define STM32_RCC_APB1ENR2_OFFSET      0x005c              /* RCC APB1 Peripheral Clock Enable Register 2 */
#define STM32_RCC_APB2ENR_OFFSET       0x0060              /* RCC APB2 Peripheral Clock Enable */
                                                           /* Offset 0x0064 Reserved */
#define STM32_RCC_AHB1SMENR_OFFSET     0x0068              /* RCC AHB1 Peripheral Clock Enable in Sleep/Stop Modes */
#define STM32_RCC_AHB2SMENR_OFFSET     0x006c              /* RCC AHB2 Peripheral Clock Enable in Sleep/Stop Modes */
#define STM32_RCC_AHB3SMENR_OFFSET     0x0070              /* RCC AHB3 Peripheral Clock Enable in Sleep/Stop Modes */
                                                           /* Offset 0x0074 Reserved */
#define STM32_RCC_APB1SMENR1_OFFSET    0x0078              /* RCC APB1 Peripheral Clock Enable in Sleep/Stop Modes (1 of 2) */
#define STM32_RCC_APB1SMENR2_OFFSET    0x007c              /* RCC APB1 Peripheral Clock Enable in Sleep/Stop Modes (2 of 2) */
#define STM32_RCC_APB2SMENR_OFFSET     0x0080              /* RCC APB2 Peripheral Clock Enable in Sleep/Stop Modes */
                                                           /* Offset 0x0084 Reserved */
#define STM32_RCC_CCIPR_OFFSET         0x0088              /* RCC Peripherals Independent Clock Configuration (1 of 2, see CCIPR2) */
                                                           /* Offset 0x008c Reserved */
#define STM32_RCC_BDCR_OFFSET          0x0090              /* RCC RTC Domain Control Register */
#define STM32_RCC_CSR_OFFSET           0x0094              /* RCC Clock Control / Status Register */
#define STM32_RCC_CRRCR_OFFSET         0x0098              /* RCC Clock Recovery RC Register */
#define STM32_RCC_CCIPR2_OFFSET        0x009c              /* RCC Peripherals Independent Clock Configuration (2 of 2, see CCIPR) */

/* Register Addresses *******************************************************/

#define STM32_RCC_CR                   (STM32_RCC_BASE + STM32_RCC_CR_OFFSET)
#define STM32_RCC_ICSCR                (STM32_RCC_BASE + STM32_RCC_ICSCR_OFFSET)
#define STM32_RCC_CFGR                 (STM32_RCC_BASE + STM32_RCC_CFGR_OFFSET)
#define STM32_RCC_PLLCFGR              (STM32_RCC_BASE + STM32_RCC_PLLCFGR_OFFSET)
#define STM32_RCC_CIER                 (STM32_RCC_BASE + STM32_RCC_CIER_OFFSET)
#define STM32_RCC_CIFR                 (STM32_RCC_BASE + STM32_RCC_CIFR_OFFSET)
#define STM32_RCC_CICR                 (STM32_RCC_BASE + STM32_RCC_CICR_OFFSET)
#define STM32_RCC_AHB1RSTR             (STM32_RCC_BASE + STM32_RCC_AHB1RSTR_OFFSET)
#define STM32_RCC_AHB2RSTR             (STM32_RCC_BASE + STM32_RCC_AHB2RSTR_OFFSET)
#define STM32_RCC_AHB3RSTR             (STM32_RCC_BASE + STM32_RCC_AHB3RSTR_OFFSET)
#define STM32_RCC_APB1RSTR1            (STM32_RCC_BASE + STM32_RCC_APB1RSTR1_OFFSET)
#define STM32_RCC_APB1RSTR2            (STM32_RCC_BASE + STM32_RCC_APB1RSTR2_OFFSET)
#define STM32_RCC_APB2RSTR             (STM32_RCC_BASE + STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_AHB1ENR              (STM32_RCC_BASE + STM32_RCC_AHB1ENR_OFFSET)
#define STM32_RCC_AHB2ENR              (STM32_RCC_BASE + STM32_RCC_AHB2ENR_OFFSET)
#define STM32_RCC_AHB3ENR              (STM32_RCC_BASE + STM32_RCC_AHB3ENR_OFFSET)
#define STM32_RCC_APB1ENR1             (STM32_RCC_BASE + STM32_RCC_APB1ENR1_OFFSET)
#define STM32_RCC_APB1ENR2             (STM32_RCC_BASE + STM32_RCC_APB1ENR2_OFFSET)
#define STM32_RCC_APB2ENR              (STM32_RCC_BASE + STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_AHB1SMENR            (STM32_RCC_BASE + STM32_RCC_AHB1SMENR_OFFSET)
#define STM32_RCC_AHB2SMENR            (STM32_RCC_BASE + STM32_RCC_AHB2SMENR_OFFSET)
#define STM32_RCC_AHB3SMENR            (STM32_RCC_BASE + STM32_RCC_AHB3SMENR_OFFSET)
#define STM32_RCC_APB1SMENR1           (STM32_RCC_BASE + STM32_RCC_APB1SMENR1_OFFSET)
#define STM32_RCC_APB1SMENR2           (STM32_RCC_BASE + STM32_RCC_APB1SMENR2_OFFSET)
#define STM32_RCC_APB2SMENR            (STM32_RCC_BASE + STM32_RCC_APB2SMENR_OFFSET)
#define STM32_RCC_CCIPR                (STM32_RCC_BASE + STM32_RCC_CCIPR_OFFSET)
#define STM32_RCC_BDCR                 (STM32_RCC_BASE + STM32_RCC_BDCR_OFFSET)
#define STM32_RCC_CSR                  (STM32_RCC_BASE + STM32_RCC_CSR_OFFSET)
#define STM32_RCC_CRRCR                (STM32_RCC_BASE + STM32_RCC_CRRCR_OFFSET)
#define STM32_RCC_CCIPR2               (STM32_RCC_BASE + STM32_RCC_CCIPR2_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* CR - Clock Control Register */

#define RCC_CR_HSION                   (1 << 8)            /* Bit  8: HSI16 clock enable */
#define RCC_CR_HSIKERON                (1 << 9)            /* Bit  9: HSI16 always enable for peripheral kernel */
#define RCC_CR_HSIRDY                  (1 << 10)           /* Bit 10: HSI16 clock ready flag */
#define RCC_CR_HSEON                   (1 << 16)           /* Bit 16: HSE clock enable */
#define RCC_CR_HSERDY                  (1 << 17)           /* Bit 17: HSE clock ready flag */
#define RCC_CR_HSEBYP                  (1 << 18)           /* Bit 18: HSE crystal oscillator bypass */
#define RCC_CR_CSSON                   (1 << 19)           /* Bit 19: HSE clock security system enable */
#define RCC_CR_PLLON                   (1 << 24)           /* Bit 24: Main PLL enable */
#define RCC_CR_PLLRDY                  (1 << 25)           /* Bit 25: Main PLL ready */

#define RCC_CR_RESERVED_MASK           0xfcf0f8ff          /* Bits 31-26, 23-20, 15-11, and 7-0 should be kept at reset value */
#define RCC_CR_RESET                   0x00000063          /* Value at POR */

/* ICSCR - Internal Clock Sources Calibration Register */

#define RCC_ICSCR_HSICAL_SHIFT         (16)                /* Bits 23-16: HSI16 calibration (factory programmed, read only) */
#define RCC_ICSCR_HSICAL_MASK          (0xff << RCC_ICSCR_HSICAL_SHIFT)

#define RCC_ICSCR_HSITRIM_SHIFT        (24)                /* Bits 30-24: HSI16 clock trimming */
#define RCC_ICSCR_HSITRIM_MASK         (0x7f << RCC_ICSCR_HSITRIM_SHIFT)
#define RCC_ICSCR_HSITRIM_RESET        (0x40 << RCC_ICSCR_HSITRIM_SHIFT)

/* CFGR - Clock Configuration Register */

#define RCC_CFGR_SW_SHIFT              (0)                 /* Bits 1-0: System clock switch */
#define RCC_CFGR_SW_MASK               (0x3 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_HSI              (1 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_HSE              (2 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_PLL              (3 << RCC_CFGR_SW_SHIFT)
#define RCC_CFGR_SWS_SHIFT             (2)                 /* Bits 3-2: System clock switch status */
#define RCC_CFGR_SWS_MASK              (0x3 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_HSI             (1 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_HSE             (2 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_PLL             (3 << RCC_CFGR_SWS_SHIFT)
#define RCC_CFGR_HPRE_SHIFT            (4)                 /* Bits 7-4: AHB prescaler */
#define RCC_CFGR_HPRE_MASK             (0xf << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLK         (0 << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLKd2       (8 << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLKd4       (9 << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLKd8       (10 << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLKd16      (11 << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLKd64      (12 << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLKd128     (13 << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLKd256     (14 << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLKd512     (15 << RCC_CFGR_HPRE_SHIFT)
#define RCC_CFGR_PPRE1_SHIFT           (8)                 /* Bits 10-8: AHB1 prescaler */
#define RCC_CFGR_PPRE1_MASK            (0x7 << RCC_CFGR_PPRE1_SHIFT)
#  define RCC_CFGR_PPRE1_HCLK          (0 << RCC_CFGR_PPRE1_SHIFT)
#  define RCC_CFGR_PPRE1_HCLKd2        (4 << RCC_CFGR_PPRE1_SHIFT)
#  define RCC_CFGR_PPRE1_HCLKd4        (5 << RCC_CFGR_PPRE1_SHIFT)
#  define RCC_CFGR_PPRE1_HCLKd8        (6 << RCC_CFGR_PPRE1_SHIFT)
#  define RCC_CFGR_PPRE1_HCLKd16       (7 << RCC_CFGR_PPRE1_SHIFT)
#define RCC_CFGR_PPRE2_SHIFT           (11)                /* Bits 13-11: AHB2 prescaler */
#define RCC_CFGR_PPRE2_MASK            (0x7 << RCC_CFGR_PPRE2_SHIFT)
#  define RCC_CFGR_PPRE2_HCLK          (0 << RCC_CFGR_PPRE2_SHIFT)
#  define RCC_CFGR_PPRE2_HCLKd2        (4 << RCC_CFGR_PPRE2_SHIFT)
#  define RCC_CFGR_PPRE2_HCLKd4        (5 << RCC_CFGR_PPRE2_SHIFT)
#  define RCC_CFGR_PPRE2_HCLKd8        (6 << RCC_CFGR_PPRE2_SHIFT)
#  define RCC_CFGR_PPRE2_HCLKd16       (7 << RCC_CFGR_PPRE2_SHIFT)
                                                           /* Bits 23-14: Reserved */
#define RCC_CFGR_MCOSEL_SHIFT          (24)                /* Bits 27-24: MCU clock output*/
#define RCC_CFGR_MCOSEL_MASK           (0xf << RCC_CFGR_MCOSEL_SHIFT)
#define RCC_CFGR_MCOPRE_SHIFT          (28)                /* Bit 28: */
#define RCC_CFGR_MCOPRE_MASK           (0x7 << RCC_CFGR_MCOPRE_SHIFT)
                                                           /* Bit 31: Reserved */

#define RCC_CFGR_RESERVED_MASK         0x00ffc000          /* Bits 23-14 should be kept at reset value */
#define RCC_CFGR_RESET                 0x00000001          /* It's actually 0x5 but bits 3-2 are read only */

/* PLLCFGR - System PLL Configuration Register */

#define RCC_PLLCFGR_PLLSRC_SHIFT       (0)                 /* Bits 1-0: Main PLL entry clock source */
#define RCC_PLLCFGR_PLLSRC_MASK        (3 << RCC_PLLCFGR_PLLSRC_SHIFT)
#  define RCC_PLLCFGR_PLLSRC_NOCLK     (0 << RCC_PLLCFGR_PLLSRC_SHIFT)
#  define RCC_PLLCFGR_PLLSRC_HSI       (2 << RCC_PLLCFGR_PLLSRC_SHIFT)
#  define RCC_PLLCFGR_PLLSRC_HSE       (3 << RCC_PLLCFGR_PLLSRC_SHIFT)
                                                           /* Bits 3-2: Reserved */
#define RCC_PLLCFGR_PLLM_SHIFT         (4)                 /* Bits 7-4: Division factor M of main PLL input clock */
#define RCC_PLLCFGR_PLLM_MASK          (0xf << RCC_PLLCFGR_PLLM_SHIFT)
#define RCC_PLLCFGR_PLLM(n)            ((((n) - 1) << RCC_PLLCFGR_PLLM_SHIFT) & RCC_PLLCFGR_PLLM_MASK) /* n=1..16 */

#define RCC_PLLCFGR_PLLN_SHIFT         (8)                 /* Bits 14-8: Main PLL multiplication factor N for VCO, n=8..127 */
#define RCC_PLLCFGR_PLLN_MASK          (0x7f << RCC_PLLCFGR_PLLN_SHIFT)
#define RCC_PLLCFGR_PLLN(n)            (((n) << RCC_PLLCFGR_PLLN_SHIFT) & RCC_PLLCFGR_PLLN_MASK)
                                                           /* Bit 15: Reserved */
#define RCC_PLLCFGR_PLLPEN             (1 << 16)           /* Bit 16: Main PLL PCLK output enable */
#define RCC_PLLCFGR_PLLP               (1 << 17)           /* Bit 17: Main PLL division factor for PLL P clock, 0:PLLP=7, 1:PLLP=17 */
                                                           /* Bits 19-18: Reserved */
#define RCC_PLLCFGR_PLLQEN             (1 << 20)           /* Bit 20: Main PLL Q clock output enable */
#define RCC_PLLCFGR_PLLQ_SHIFT         (21)                /* Bits 22-21: Main PLL division factor for PLL Q clock */
#define RCC_PLLCFGR_PLLQ_MASK          (0x3 << RCC_PLLCFGR_PLLQ_SHIFT)
#  define RCC_PLLCFGR_PLLQ_2           (0x0 << RCC_PLLCFGR_PLLQ_SHIFT)
#  define RCC_PLLCFGR_PLLQ_4           (0x1 << RCC_PLLCFGR_PLLQ_SHIFT)
#  define RCC_PLLCFGR_PLLQ_6           (0x2 << RCC_PLLCFGR_PLLQ_SHIFT)
#  define RCC_PLLCFGR_PLLQ_8           (0x3 << RCC_PLLCFGR_PLLQ_SHIFT)
                                                           /* Bits 23: Reserved */
#define RCC_PLLCFGR_PLLREN             (1 << 24)           /* Bit 24: PLL R clock output enable */
#define RCC_PLLCFGR_PLLR_SHIFT         (25)                /* Bits 26-25: Main PLL division factor for PLL R clock (system clock) */
#define RCC_PLLCFGR_PLLR_MASK          (3 << RCC_PLLCFGR_PLLR_SHIFT)
#  define RCC_PLLCFGR_PLLR_2           (0x0 << RCC_PLLCFGR_PLLR_SHIFT)
#  define RCC_PLLCFGR_PLLR_4           (0x1 << RCC_PLLCFGR_PLLR_SHIFT)
#  define RCC_PLLCFGR_PLLR_6           (0x2 << RCC_PLLCFGR_PLLR_SHIFT)
#  define RCC_PLLCFGR_PLLR_8           (0x3 << RCC_PLLCFGR_PLLR_SHIFT)

#define RCC_PLLCFGR_PLLPDIV_SHIFT      (27)                /* Bits 31-27: Main PLL division factor for PLL P clock */
#define RCC_PLLCFGR_PLLPDIV_MASK       (0x1f << RCC_PLLCFGR_PLLPDIV_SHIFT)
#define RCC_PLLCFGR_PLLPDIV(n)         (((n) << RCC_PLLCFGR_PLLPDIV_SHIFT) & RCC_PLLCFGR_PLLPDIV_MASK) /* n=2..31 */

#define RCC_PLLCFGR_RESERVED_MASK      0x008c800c          /* Bits 23, 19-18, 15, and 3-2 should be kept at reset value */
#define RCC_PLLCFGR_RESET              0x00001000

/* CIER - Clock Interrupt Enable Register */

#define RCC_CIER_LSIRDYIE              (1 << 0)
#define RCC_CIER_LSERDYIE              (1 << 1)
#define RCC_CIER_HSIRDYIE              (1 << 3)
#define RCC_CIER_HSERDYIE              (1 << 4)
#define RCC_CIER_PLLRDYIE              (1 << 5)
#define RCC_CIER_LSECSSIE              (1 << 9)
#define RCC_CIER_HSI48RDYIE            (1 << 10)

/* CIFR - Clock Interrupt Flag Register */

#define RCC_CIFR_LSIRDYF               (1 << 0)
#define RCC_CIFR_LSERDYF               (1 << 1)
#define RCC_CIFR_HSIRDYF               (1 << 3)
#define RCC_CIFR_HSERDYF               (1 << 4)
#define RCC_CIFR_PLLRDYF               (1 << 5)
#define RCC_CIFR_CSSF                  (1 << 8)
#define RCC_CIFR_LSECSSF               (1 << 9)
#define RCC_CIFR_HSI48RDYF             (1 << 10)

/* CICR - Clock Interrupt Clear Register */

#define RCC_CICR_LSIRDYC               (1 << 0)
#define RCC_CICR_LSERDYC               (1 << 1)
#define RCC_CICR_HSIRDYC               (1 << 3)
#define RCC_CICR_HSERDYC               (1 << 4)
#define RCC_CICR_PLLRDYC               (1 << 5)
#define RCC_CICR_CSSC                  (1 << 8)
#define RCC_CICR_LSECSSC               (1 << 9)
#define RCC_CICR_HSI48RDYC             (1 << 10)

/* AHB1RSTR - AHB1 Peripheral Reset Register */

#define RCC_AHB1RSTR_DMA1RST           (1 << 0)
#define RCC_AHB1RSTR_DMA2RST           (1 << 1)
#define RCC_AHB1RSTR_DMAMUX1RST        (1 << 2)
#define RCC_AHB1RSTR_CORDICRST         (1 << 3)
#define RCC_AHB1RSTR_FMACRST           (1 << 4)
#define RCC_AHB1RSTR_FLASHRST          (1 << 8)
#define RCC_AHB1RSTR_CRCRST            (1 << 12)

/* AHB2RSTR - AHB2 Peripheral Reset Register */

#define RCC_AHB2RSTR_GPIOARST          (1 << 0)
#define RCC_AHB2RSTR_GPIOBRST          (1 << 1)
#define RCC_AHB2RSTR_GPIOCRST          (1 << 2)
#define RCC_AHB2RSTR_GPIODRST          (1 << 3)
#define RCC_AHB2RSTR_GPIOERST          (1 << 4)
#define RCC_AHB2RSTR_GPIOFRST          (1 << 5)
#define RCC_AHB2RSTR_GPIOGRST          (1 << 6)
#define RCC_AHB2RSTR_ADC12RST          (1 << 13)
#define RCC_AHB2RSTR_ADC345RST         (1 << 14)
#define RCC_AHB2RSTR_DAC1RST           (1 << 16)
#define RCC_AHB2RSTR_DAC2RST           (1 << 17)
#define RCC_AHB2RSTR_DAC3RST           (1 << 18)
#define RCC_AHB2RSTR_DAC4RST           (1 << 19)
#define RCC_AHB2RSTR_RNGRST            (1 << 26)

/* AHB3RSTR - AHB3 Peripheral Reset Register */

#define RCC_AHB3RSTR_FMCRST            (1 << 0)
#define RCC_AHB3RSTR_QSPIRST           (1 << 8)

/* APB1RSTR1 - APB1 Peripheral Reset Register 1 */

#define RCC_APB1RSTR1_TIM2RST          (1 << 0)
#define RCC_APB1RSTR1_TIM3RST          (1 << 1)
#define RCC_APB1RSTR1_TIM4RST          (1 << 2)
#define RCC_APB1RSTR1_TIM5RST          (1 << 3)
#define RCC_APB1RSTR1_TIM6RST          (1 << 4)
#define RCC_APB1RSTR1_TIM7RST          (1 << 5)
#define RCC_APB1RSTR1_CRSRST           (1 << 8)
#define RCC_APB1RSTR1_SPI2RST          (1 << 14)
#define RCC_APB1RSTR1_SPI3RST          (1 << 15)
#define RCC_APB1RSTR1_USART2RST        (1 << 17)
#define RCC_APB1RSTR1_USART3RST        (1 << 18)
#define RCC_APB1RSTR1_UART4RST         (1 << 19)
#define RCC_APB1RSTR1_UART5RST         (1 << 20)
#define RCC_APB1RSTR1_I2C1RST          (1 << 21)
#define RCC_APB1RSTR1_I2C2RST          (1 << 22)
#define RCC_APB1RSTR1_USBRST           (1 << 23)
#define RCC_APB1RSTR1_FDCANRST         (1 << 25)
#define RCC_APB1RSTR1_PWRRST           (1 << 28)
#define RCC_APB1RSTR1_I2C3RST          (1 << 30)
#define RCC_APB1RSTR1_LPTIM1RST        (1 << 31)

/* APB1RSTR2 - APB1 Peripheral Reset Register 2 */

#define RCC_APB1RSTR2_LPUART1RST       (1 << 0)
#define RCC_APB1RSTR2_I2C4RST          (1 << 1)
#define RCC_APB1RSTR2_UCPD1RST         (1 << 8)

/* APB2RSTR - APB2 Peripheral Reset Register */

#define RCC_APB2RSTR_SYSCFGRST         (1 << 0)
#define RCC_APB2RSTR_TIM1RST           (1 << 11)
#define RCC_APB2RSTR_SPI1RST           (1 << 12)
#define RCC_APB2RSTR_TIM8RST           (1 << 13)
#define RCC_APB2RSTR_USART1RST         (1 << 14)
#define RCC_APB2RSTR_SPI4RST           (1 << 15)
#define RCC_APB2RSTR_TIM15RST          (1 << 16)
#define RCC_APB2RSTR_TIM16RST          (1 << 17)
#define RCC_APB2RSTR_TIM17RST          (1 << 18)
#define RCC_APB2RSTR_TIM20RST          (1 << 20)
#define RCC_APB2RSTR_SAI1RST           (1 << 21)
#define RCC_APB2RSTR_HRTIM1RST         (1 << 26)

/* AHB1ENR - AHB1 Peripheral Clocks Enable Register */

#define RCC_AHB1ENR_DMA1EN             (1 << 0)
#define RCC_AHB1ENR_DMA2EN             (1 << 1)
#define RCC_AHB1ENR_DMAMUX1EN          (1 << 2)
#define RCC_AHB1ENR_CORDICEN           (1 << 3)
#define RCC_AHB1ENR_FMACEN             (1 << 4)
#define RCC_AHB1ENR_FLASHEN            (1 << 8)
#define RCC_AHB1ENR_CRCEN              (1 << 12)

/* AHB2ENR - AHB2 Peripheral Clocks Enable Register */

#define RCC_AHB2ENR_GPIOEN(n)          ((1 << (n)) & 0x7f)
#define RCC_AHB2ENR_GPIOAEN            (1 << 0)
#define RCC_AHB2ENR_GPIOBEN            (1 << 1)
#define RCC_AHB2ENR_GPIOCEN            (1 << 2)
#define RCC_AHB2ENR_GPIODEN            (1 << 3)
#define RCC_AHB2ENR_GPIOEEN            (1 << 4)
#define RCC_AHB2ENR_GPIOFEN            (1 << 5)
#define RCC_AHB2ENR_GPIOGEN            (1 << 6)
#define RCC_AHB2ENR_ADC12EN            (1 << 13)
#define RCC_AHB2ENR_ADC345EN           (1 << 14)
#define RCC_AHB2ENR_DAC1EN             (1 << 16)
#define RCC_AHB2ENR_DAC2EN             (1 << 17)
#define RCC_AHB2ENR_DAC3EN             (1 << 18)
#define RCC_AHB2ENR_DAC4EN             (1 << 19)
#define RCC_AHB2ENR_RNGEN              (1 << 26)

/* AHB3ENR - AHB3 Peripheral Clocks Enable Register */

#define RCC_AHB3ENR_FMCEN              (1 << 0)
#define RCC_AHB3ENR_QSPIEN             (1 << 8)

/* APB1ENR1 - APB1 Peripheral Clocks Enable Register 1 */

#define RCC_APB1ENR1_TIM2EN            (1 << 0)
#define RCC_APB1ENR1_TIM3EN            (1 << 1)
#define RCC_APB1ENR1_TIM4EN            (1 << 2)
#define RCC_APB1ENR1_TIM5EN            (1 << 3)
#define RCC_APB1ENR1_TIM6EN            (1 << 4)
#define RCC_APB1ENR1_TIM7EN            (1 << 5)
#define RCC_APB1ENR1_CRSEN             (1 << 8)
#define RCC_APB1ENR1_RTCAPBEN          (1 << 10)
#define RCC_APB1ENR1_WWDGEN            (1 << 11)
#define RCC_APB1ENR1_SPI2EN            (1 << 14)
#define RCC_APB1ENR1_SPI3EN            (1 << 15)
#define RCC_APB1ENR1_USART2EN          (1 << 17)
#define RCC_APB1ENR1_USART3EN          (1 << 18)
#define RCC_APB1ENR1_UART4EN           (1 << 19)
#define RCC_APB1ENR1_UART5EN           (1 << 20)
#define RCC_APB1ENR1_I2C1EN            (1 << 21)
#define RCC_APB1ENR1_I2C2EN            (1 << 22)
#define RCC_APB1ENR1_USBEN             (1 << 23)
#define RCC_APB1ENR1_FDCANEN           (1 << 25)
#define RCC_APB1ENR1_PWREN             (1 << 28)
#define RCC_APB1ENR1_I2C3EN            (1 << 30)
#define RCC_APB1ENR1_LPTIM1EN          (1 << 31)

/* APB1ENR2 - APB1 Peripheral Clocks Enable Register 2 */

#define RCC_APB1ENR2_LPUART1EN         (1 << 0)
#define RCC_APB1ENR2_I2C4EN            (1 << 1)
#define RCC_APB1ENR2_UCPD1EN           (1 << 8)

/* APB2ENR - APB2 Peripheral Clocks Enable Register */

#define RCC_APB2ENR_SYSCFGEN           (1 << 0)
#define RCC_APB2ENR_TIM1EN             (1 << 11)
#define RCC_APB2ENR_SPI1EN             (1 << 12)
#define RCC_APB2ENR_TIM8EN             (1 << 13)
#define RCC_APB2ENR_USART1EN           (1 << 14)
#define RCC_APB2ENR_SPI4EN             (1 << 15)
#define RCC_APB2ENR_TIM15EN            (1 << 16)
#define RCC_APB2ENR_TIM16EN            (1 << 17)
#define RCC_APB2ENR_TIM17EN            (1 << 18)
#define RCC_APB2ENR_TIM20EN            (1 << 20)
#define RCC_APB2ENR_SAI1EN             (1 << 21)
#define RCC_APB2ENR_HRTIM1EN           (1 << 26)

/* AHB1SMENR - AHB1 Peripheral Clocks Enable In Sleep/Stop Mode Register */

#define RCC_AHB1SMENR_DMA1SMEN         (1 << 0)
#define RCC_AHB1SMENR_DMA2SMEN         (1 << 1)
#define RCC_AHB1SMENR_DMAMUX1SMEN      (1 << 2)
#define RCC_AHB1SMENR_CORDICSMEN       (1 << 3)
#define RCC_AHB1SMENR_FMACSMEN         (1 << 4)
#define RCC_AHB1SMENR_FLASHSMEN        (1 << 8)
#define RCC_AHB1SMENR_SRAM1SMEN        (1 << 9)
#define RCC_AHB1SMENR_CRCSMEN          (1 << 12)

/* AHB2SMENR - AHB2 Peripheral Clocks Enable In Sleep/Stop Mode Register */

#define RCC_AHB2SMENR_GPIOASMEN        (1 << 0)
#define RCC_AHB2SMENR_GPIOBSMEN        (1 << 1)
#define RCC_AHB2SMENR_GPIOCSMEN        (1 << 2)
#define RCC_AHB2SMENR_GPIODSMEN        (1 << 3)
#define RCC_AHB2SMENR_GPIOESMEN        (1 << 4)
#define RCC_AHB2SMENR_GPIOFSMEN        (1 << 5)
#define RCC_AHB2SMENR_GPIOGSMEN        (1 << 6)
#define RCC_AHB2SMENR_CCMSRAMSMEN      (1 << 9)
#define RCC_AHB2SMENR_SRAM2SMEN        (1 << 10)
#define RCC_AHB2SMENR_ADC12SMEN        (1 << 13)
#define RCC_AHB2SMENR_ADC345SMEN       (1 << 14)
#define RCC_AHB2SMENR_DAC1SMEN         (1 << 16)
#define RCC_AHB2SMENR_DAC2SMEN         (1 << 17)
#define RCC_AHB2SMENR_DAC3SMEN         (1 << 18)
#define RCC_AHB2SMENR_DAC4SMEN         (1 << 19)
#define RCC_AHB2SMENR_RNGSMEN          (1 << 26)

/* AHB3SMENR - AHB3 Peripheral Clocks Enable In Sleep/Stop Mode Register */

#define RCC_AHB3SMENR_FMCSMEN          (1 << 0)
#define RCC_AHB3SMENR_QSPISMEN         (1 << 8)

/* APB1SMENR1 - APB1 Peripheral Clocks Enable In Sleep/Stop Mode Register 1 */

#define RCC_APB1SMENR1_TIM2SMEN        (1 << 0)
#define RCC_APB1SMENR1_TIM3SMEN        (1 << 1)
#define RCC_APB1SMENR1_TIM4SMEN        (1 << 2)
#define RCC_APB1SMENR1_TIM5SMEN        (1 << 3)
#define RCC_APB1SMENR1_TIM6SMEN        (1 << 4)
#define RCC_APB1SMENR1_TIM7SMEN        (1 << 5)
#define RCC_APB1SMENR1_CRSSMEN         (1 << 8)
#define RCC_APB1SMENR1_RTCAPBSMEN      (1 << 10)
#define RCC_APB1SMENR1_WWDGSMEN        (1 << 11)
#define RCC_APB1SMENR1_SPI2SMEN        (1 << 14)
#define RCC_APB1SMENR1_SPI3SMEN        (1 << 15)
#define RCC_APB1SMENR1_USART2SMEN      (1 << 17)
#define RCC_APB1SMENR1_USART3SMEN      (1 << 18)
#define RCC_APB1SMENR1_UART4SMEN       (1 << 19)
#define RCC_APB1SMENR1_UART5SMEN       (1 << 20)
#define RCC_APB1SMENR1_I2C1SMEN        (1 << 21)
#define RCC_APB1SMENR1_I2C2SMEN        (1 << 22)
#define RCC_APB1SMENR1_USBSMEN         (1 << 23)
#define RCC_APB1SMENR1_FDCANSMEN       (1 << 25)
#define RCC_APB1SMENR1_PWRSMEN         (1 << 28)
#define RCC_APB1SMENR1_I2C3SMEN        (1 << 30)
#define RCC_APB1SMENR1_LPTIM1SMEN      (1 << 31)

/* APB1SMENR2 - APB1 Peripheral Clocks Enable In Sleep/Stop Mode Register 2 */

#define RCC_APB1SMENR2_LPUART1SMEN     (1 << 0)
#define RCC_APB1SMENR2_I2C4SMEN        (1 << 1)
#define RCC_APB1SMENR2_UCPD1SMEN       (1 << 8)

/* APB2SMENR - APB2 Peripheral Clocks Enable In Sleep/Stop Mode Register */

#define RCC_APB2SMENR_SYSCFGSMEN       (1 << 0)
#define RCC_APB2SMENR_TIM1SMEN         (1 << 11)
#define RCC_APB2SMENR_SPI1SMEN         (1 << 12)
#define RCC_APB2SMENR_TIM8SMEN         (1 << 13)
#define RCC_APB2SMENR_USART1SMEN       (1 << 14)
#define RCC_APB2SMENR_SPI4SMEN         (1 << 15)
#define RCC_APB2SMENR_TIM15SMEN        (1 << 16)
#define RCC_APB2SMENR_TIM16SMEN        (1 << 17)
#define RCC_APB2SMENR_TIM17SMEN        (1 << 18)
#define RCC_APB2SMENR_TIM20SMEN        (1 << 20)
#define RCC_APB2SMENR_SAI1SMEN         (1 << 21)
#define RCC_APB2SMENR_HRTIM1SMEN       (1 << 26)

/* CCIPR - Peripherals Independent Clock Configuration Register */

#define RCC_CCIPR_USART1SEL_SHIFT      (0)
#define RCC_CCIPR_USART1SEL_MASK       (3 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_PCLK     (0 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_SYSCLK   (1 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_HSI16    (2 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_LSE      (3 << RCC_CCIPR_USART1SEL_SHIFT)

#define RCC_CCIPR_USART2SEL_SHIFT      (2)
#define RCC_CCIPR_USART2SEL_MASK       (3 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_PCLK     (0 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_SYSCLK   (1 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_HSI16    (2 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_LSE      (3 << RCC_CCIPR_USART2SEL_SHIFT)

#define RCC_CCIPR_USART3SEL_SHIFT      (4)
#define RCC_CCIPR_USART3SEL_MASK       (3 << RCC_CCIPR_USART3SEL_SHIFT)
#  define RCC_CCIPR_USART3SEL_PCLK     (0 << RCC_CCIPR_USART3SEL_SHIFT)
#  define RCC_CCIPR_USART3SEL_SYSCLK   (1 << RCC_CCIPR_USART3SEL_SHIFT)
#  define RCC_CCIPR_USART3SEL_HSI16    (2 << RCC_CCIPR_USART3SEL_SHIFT)
#  define RCC_CCIPR_USART3SEL_LSE      (3 << RCC_CCIPR_USART3SEL_SHIFT)

#define RCC_CCIPR_UART4SEL_SHIFT       (6)
#define RCC_CCIPR_UART4SEL_MASK        (3 << RCC_CCIPR_UART4SEL_SHIFT)
#  define RCC_CCIPR_UART4SEL_PCLK      (0 << RCC_CCIPR_UART4SEL_SHIFT)
#  define RCC_CCIPR_UART4SEL_SYSCLK    (1 << RCC_CCIPR_UART4SEL_SHIFT)
#  define RCC_CCIPR_UART4SEL_HSI16     (2 << RCC_CCIPR_UART4SEL_SHIFT)
#  define RCC_CCIPR_UART4SEL_LSE       (3 << RCC_CCIPR_UART4SEL_SHIFT)

#define RCC_CCIPR_UART5SEL_SHIFT       (8)
#define RCC_CCIPR_UART5SEL_MASK        (3 << RCC_CCIPR_UART5SEL_SHIFT)
#  define RCC_CCIPR_UART5SEL_PCLK      (0 << RCC_CCIPR_UART5SEL_SHIFT)
#  define RCC_CCIPR_UART5SEL_SYSCLK    (1 << RCC_CCIPR_UART5SEL_SHIFT)
#  define RCC_CCIPR_UART5SEL_HSI16     (2 << RCC_CCIPR_UART5SEL_SHIFT)
#  define RCC_CCIPR_UART5SEL_LSE       (3 << RCC_CCIPR_UART5SEL_SHIFT)

#define RCC_CCIPR_LPUART1SEL_SHIFT     (10)
#define RCC_CCIPR_LPUART1SEL_MASK      (3 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_PCLK    (0 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_SYSCLK  (1 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_HSI16   (2 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_LSE     (3 << RCC_CCIPR_LPUART1SEL_SHIFT)

#define RCC_CCIPR_I2C1SEL_SHIFT        (12)
#define RCC_CCIPR_I2C1SEL_MASK         (3 << RCC_CCIPR_I2C1SEL_SHIFT)
#  define RCC_CCIPR_I2C1SEL_PCLK       (0 << RCC_CCIPR_I2C1SEL_SHIFT)
#  define RCC_CCIPR_I2C1SEL_SYSCLK     (1 << RCC_CCIPR_I2C1SEL_SHIFT)
#  define RCC_CCIPR_I2C1SEL_HSI16      (2 << RCC_CCIPR_I2C1SEL_SHIFT)

#define RCC_CCIPR_I2C2SEL_SHIFT        (14)
#define RCC_CCIPR_I2C2SEL_MASK         (3 << RCC_CCIPR_I2C2SEL_SHIFT)
#  define RCC_CCIPR_I2C2SEL_PCLK       (0 << RCC_CCIPR_I2C2SEL_SHIFT)
#  define RCC_CCIPR_I2C2SEL_SYSCLK     (1 << RCC_CCIPR_I2C2SEL_SHIFT)
#  define RCC_CCIPR_I2C2SEL_HSI16      (2 << RCC_CCIPR_I2C2SEL_SHIFT)

#define RCC_CCIPR_I2C3SEL_SHIFT        (16)
#define RCC_CCIPR_I2C3SEL_MASK         (3 << RCC_CCIPR_I2C3SEL_SHIFT)
#  define RCC_CCIPR_I2C3SEL_PCLK       (0 << RCC_CCIPR_I2C3SEL_SHIFT)
#  define RCC_CCIPR_I2C3SEL_SYSCLK     (1 << RCC_CCIPR_I2C3SEL_SHIFT)
#  define RCC_CCIPR_I2C3SEL_HSI16      (2 << RCC_CCIPR_I2C3SEL_SHIFT)

#define RCC_CCIPR_LPTIM1SEL_SHIFT      (18)
#define RCC_CCIPR_LPTIM1SEL_MASK       (3 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_PCLK     (0 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_SYSCLK   (1 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_HSI16    (2 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_LSE      (3 << RCC_CCIPR_LPTIM1SEL_SHIFT)

#define RCC_CCIPR_SAI1SEL_SHIFT        (20)
#define RCC_CCIPR_SAI1SEL_MASK         (3 << RCC_CCIPR_SAI1SEL_SHIFT)
#  define RCC_CCIPR_SAI1SEL_SYSCLK     (0 << RCC_CCIPR_SAI1SEL_SHIFT)
#  define RCC_CCIPR_SAI1SEL_PLLQ       (1 << RCC_CCIPR_SAI1SEL_SHIFT)
#  define RCC_CCIPR_SAI1SEL_I2S_CKIN   (2 << RCC_CCIPR_SAI1SEL_SHIFT)
#  define RCC_CCIPR_SAI1SEL_HSI16      (3 << RCC_CCIPR_SAI1SEL_SHIFT)

#define RCC_CCIPR_I2S23SEL_SHIFT       (22)
#define RCC_CCIPR_I2S23SEL_MASK        (3 << RCC_CCIPR_I2S23SEL_SHIFT)
#  define RCC_CCIPR_I2S23SEL_SYSCLK    (0 << RCC_CCIPR_I2S23SEL_SHIFT)
#  define RCC_CCIPR_I2S23SEL_PLLQ      (1 << RCC_CCIPR_I2S23SEL_SHIFT)
#  define RCC_CCIPR_I2S23SEL_I2S_CKIN  (2 << RCC_CCIPR_I2S23SEL_SHIFT)
#  define RCC_CCIPR_I2S23SEL_HSI16     (3 << RCC_CCIPR_I2S23SEL_SHIFT)

#define RCC_CCIPR_FDCANSEL_SHIFT       (24)
#define RCC_CCIPR_FDCANSEL_MASK        (3 << RCC_CCIPR_FDCANSEL_SHIFT)
#  define RCC_CCIPR_FDCANSEL_HSE       (0 << RCC_CCIPR_FDCANSEL_SHIFT)
#  define RCC_CCIPR_FDCANSEL_PLLQ      (1 << RCC_CCIPR_FDCANSEL_SHIFT)
#  define RCC_CCIPR_FDCANSEL_PCLK      (2 << RCC_CCIPR_FDCANSEL_SHIFT)

#define RCC_CCIPR_CLK48SEL_SHIFT       (26)
#define RCC_CCIPR_CLK48SEL_MASK        (3 << RCC_CCIPR_CLK48SEL_SHIFT)
#  define RCC_CCIPR_CLK48SEL_HSI48     (0 << RCC_CCIPR_CLK48SEL_SHIFT)
#  define RCC_CCIPR_CLK48SEL_PLLQ      (2 << RCC_CCIPR_CLK48SEL_SHIFT)

#define RCC_CCIPR_ADC12SEL_SHIFT       (28)
#define RCC_CCIPR_ADC12SEL_MASK        (3 << RCC_CCIPR_ADC12SEL_SHIFT)
#  define RCC_CCIPR_ADC12SEL_NO_CLK    (0 << RCC_CCIPR_ADC12SEL_SHIFT)
#  define RCC_CCIPR_ADC12SEL_PLLP      (1 << RCC_CCIPR_ADC12SEL_SHIFT)
#  define RCC_CCIPR_ADC12SEL_SYSCLK    (2 << RCC_CCIPR_ADC12SEL_SHIFT)

#define RCC_CCIPR_ADC345SEL_SHIFT      (30)
#define RCC_CCIPR_ADC345SEL_MASK       (3 << RCC_CCIPR_ADC345SEL_SHIFT)
#  define RCC_CCIPR_ADC345SEL_NO_CLK   (0 << RCC_CCIPR_ADC345SEL_SHIFT)
#  define RCC_CCIPR_ADC345SEL_PLLP     (1 << RCC_CCIPR_ADC345SEL_SHIFT)
#  define RCC_CCIPR_ADC345SEL_SYSCLK   (2 << RCC_CCIPR_ADC345SEL_SHIFT)

/* BDCR - Backup Domain Control Register */

#define RCC_BDCR_LSEON                 (1 << 0)
#define RCC_BDCR_LSERDY                (1 << 1)
#define RCC_BDCR_LSEBYP                (1 << 2)

#define RCC_BDCR_LSEDRV_SHIFT          (3)
#define RCC_BDCR_LSEDRV_MASK           (3 << RCC_BDCR_LSEDRV_SHIFT)
#  define RCC_BDCR_LSEDRV_LOW          (0 << RCC_BDCR_LSEDRV_SHIFT)
#  define RCC_BDCR_LSEDRV_MEDIUM_LOW   (1 << RCC_BDCR_LSEDRV_SHIFT)
#  define RCC_BDCR_LSEDRV_MEDIUM_HIGH  (2 << RCC_BDCR_LSEDRV_SHIFT)
#  define RCC_BDCR_LSEDRV_HIGH         (3 << RCC_BDCR_LSEDRV_SHIFT)

#define RCC_BDCR_LSECSSON              (1 << 5)
#define RCC_BDCR_LSECSSD               (1 << 6)

#define RCC_BDCR_RTCSEL_SHIFT          (8)
#define RCC_BDCR_RTCSEL_MASK           (3 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_NO_CLK       (0 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_LSE          (1 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_LSI          (2 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_HSE          (3 << RCC_BDCR_RTCSEL_SHIFT)

#define RCC_BDCR_RTCEN                 (1 << 15)
#define RCC_BDCR_BDRST                 (1 << 16)
#define RCC_BDCR_LSCOEN                (1 << 24)
#define RCC_BDCR_LSCOSEL               (1 << 25)

/* CSR - Clock Control & Status Register */

#define RCC_CSR_LSION                  (1 << 0)
#define RCC_CSR_LSIRDY                 (1 << 1)
#define RCC_CSR_RMVF                   (1 << 23)
#define RCC_CSR_OBLRSTF                (1 << 25)
#define RCC_CSR_PINRSTF                (1 << 26)
#define RCC_CSR_BORRSTF                (1 << 27)
#define RCC_CSR_SFTRSTF                (1 << 28)
#define RCC_CSR_IWDGRSTF               (1 << 29)
#define RCC_CSR_WWDGRSTF               (1 << 30)
#define RCC_CSR_LPWRRSTF               (1 << 31)

/* CRRCR - Clock Recovery RC Register */

#define RCC_CRRCR_HSI48ON              (1 << 0)
#define RCC_CRRCR_HSI48RDY             (1 << 1)

#define RCC_CRRCR_HSI48CAL_SHIFT       (7)
#define RCC_CRRCR_HSI48CAL_MASK        (0x1ff << RCC_CRRCR_HSI48CAL_SHIFT)
#define RCC_CRRCR_HSI48CAL(n)          (((n) << RCC_CRRCR_HSI48CAL_SHIFT) & RCC_CRRCR_HSI48CAL_MASK)

/* CCIPR2 - Peripherals Independent Clock Configuration Register 2 */

#define RCC_CCIPR2_I2C4SEL_SHIFT       (0)
#define RCC_CCIPR2_I2C4SEL_MASK        (3 << RCC_CCIPR2_I2C4SEL_SHIFT)
#define RCC_CCIPR2_I2C4SEL_PCLK        (0 << RCC_CCIPR2_I2C4SEL_SHIFT)
#define RCC_CCIPR2_I2C4SEL_SYSCLK      (1 << RCC_CCIPR2_I2C4SEL_SHIFT)
#define RCC_CCIPR2_I2C4SEL_HSI16       (2 << RCC_CCIPR2_I2C4SEL_SHIFT)

#define RCC_CCIPR2_QSPISEL_SHIFT       (20)
#define RCC_CCIPR2_QSPISEL_MASK        (3 << RCC_CCIPR2_QSPISEL_SHIFT)
#define RCC_CCIPR2_QSPISEL_SYSCLK      (0 << RCC_CCIPR2_QSPISEL_SHIFT)
#define RCC_CCIPR2_QSPISEL_HSI16       (1 << RCC_CCIPR2_QSPISEL_SHIFT)
#define RCC_CCIPR2_QSPISEL_PLLQ        (2 << RCC_CCIPR2_QSPISEL_SHIFT)

/* Compatibility definitions ************************************************/

/* Compatibility with F1/F2/F4 Status Register names */

#define STM32_RCC_APB1ENR_OFFSET       STM32_RCC_APB1ENR1_OFFSET
#define STM32_RCC_APB1ENR              STM32_RCC_APB1ENR1
#define STM32_RCC_APB1RSTR             STM32_RCC_APB1RSTR1

#define RCC_APB1ENR_USART2EN           RCC_APB1ENR1_USART2EN
#define RCC_APB1ENR_USART3EN           RCC_APB1ENR1_USART3EN
#define RCC_APB1ENR_UART4EN            RCC_APB1ENR1_UART4EN
#define RCC_APB1ENR_UART5EN            RCC_APB1ENR1_UART5EN

#define RCC_APB1ENR_I2C1EN             RCC_APB1ENR1_I2C1EN
#define RCC_APB1RSTR_I2C1RST           RCC_APB1RSTR1_I2C1RST
#define RCC_APB1ENR_I2C2EN             RCC_APB1ENR1_I2C2EN
#define RCC_APB1RSTR_I2C2RST           RCC_APB1RSTR1_I2C2RST
#define RCC_APB1ENR_I2C3EN             RCC_APB1ENR1_I2C3EN
#define RCC_APB1RSTR_I2C3RST           RCC_APB1RSTR1_I2C3RST

#define RCC_APB1ENR_TIM2EN             RCC_APB1ENR1_TIM2EN
#define RCC_APB1ENR_TIM3EN             RCC_APB1ENR1_TIM3EN
#define RCC_APB1ENR_TIM4EN             RCC_APB1ENR1_TIM4EN
#define RCC_APB1ENR_TIM5EN             RCC_APB1ENR1_TIM5EN

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_RCC_H */
