/************************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32l0_rcc.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32L0_RCC_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32L0_RCC_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_RCC_CR_OFFSET             0x0000  /* Clock control register */
#define STM32_RCC_ICSCR_OFFSET          0x0004  /* Internal clock sources calibration register */
#define STM32_RCC_CRRCR_OFFSET          0x0008  /* Clock recovery RC register */
#define STM32_RCC_CFGR_OFFSET           0x000C  /* Clock configuration register */
#define STM32_RCC_CIER_OFFSET           0x0010  /* Clock Source Interrupt enable register */
#define STM32_RCC_CIFR_OFFSET           0x0014  /* Clock Source Interrupt Flag register */
#define STM32_RCC_CICR_OFFSET           0x0018  /* Clock Source Interrupt Clear register */
#define STM32_RCC_IOPRSTR_OFFSET        0x001C  /* GPIO reset register */
#define STM32_RCC_AHBRSTR_OFFSET        0x0020  /* AHB peripheral reset register */
#define STM32_RCC_APB2RSTR_OFFSET       0x0024  /* APB2 Peripheral reset register */
#define STM32_RCC_APB1RSTR_OFFSET       0x0028  /* APB1 Peripheral reset register */
#define STM32_RCC_IOPENR_OFFSET         0x002C  /* GPIO clock enable register */
#define STM32_RCC_AHBENR_OFFSET         0x0030  /* AHB Peripheral Clock enable register */
#define STM32_RCC_APB2ENR_OFFSET        0x0034  /* APB2 Peripheral Clock enable register */
#define STM32_RCC_APB1ENR_OFFSET        0x0038  /* APB1 Peripheral Clock enable register */
#define STM32_RCC_IOPSMEN_OFFSET        0x003C  /* GPIO clock enable in Sleep mode register */
#define STM32_RCC_AHBSMENR_OFFSET       0x0040  /* AHB peripheral clock enable in Sleep mode register */
#define STM32_RCC_APB2SMENR_OFFSET      0x0044  /* APB2 peripheral clock enable in Sleep mode register */
#define STM32_RCC_APB1SMENR_OFFSET      0x0048  /* APB1 peripheral clock enable in Sleep mode register */
#define STM32_RCC_CCIPR_OFFSET          0x004C  /* Clock configuration register */
#define STM32_RCC_CSR_OFFSET            0x0050  /* Control/status register */

/* Register Addresses ***************************************************************/

#define STM32_RCC_CR                (STM32_RCC_BASE+STM32_RCC_CR_OFFSET)
#define STM32_RCC_ICSCR             (STM32_RCC_BASE+STM32_RCC_ICSCR_OFFSET)
#define STM32_RCC_CRRCR             (STM32_RCC_BASE+STM32_RCC_CRRCR_OFFSET)
#define STM32_RCC_CFGR              (STM32_RCC_BASE+STM32_RCC_CFGR_OFFSET)
#define STM32_RCC_CIER              (STM32_RCC_BASE+STM32_RCC_CIER_OFFSET)
#define STM32_RCC_CIFR              (STM32_RCC_BASE+STM32_RCC_CIFR_OFFSET)
#define STM32_RCC_CICR              (STM32_RCC_BASE+STM32_RCC_CICR_OFFSET)
#define STM32_RCC_IOPRSTR           (STM32_RCC_BASE+STM32_RCC_IOPRSTR_OFFSET)
#define STM32_RCC_AHBRSTR           (STM32_RCC_BASE+STM32_RCC_AHBRSTR_OFFSET)
#define STM32_RCC_APB2RSTR          (STM32_RCC_BASE+STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_APB1RSTR          (STM32_RCC_BASE+STM32_RCC_APB1RSTR_OFFSET)
#define STM32_RCC_IOPENR            (STM32_RCC_BASE+STM32_RCC_IOPENR_OFFSET)
#define STM32_RCC_AHBENR            (STM32_RCC_BASE+STM32_RCC_AHBENR_OFFSET)
#define STM32_RCC_APB2ENR           (STM32_RCC_BASE+STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_APB1ENR           (STM32_RCC_BASE+STM32_RCC_APB1ENR_OFFSET)
#define STM32_RCC_IOPSMEN           (STM32_RCC_BASE+STM32_RCC_IOPSMEN_OFFSET)
#define STM32_RCC_AHBSMENR          (STM32_RCC_BASE+STM32_RCC_AHBSMENR_OFFSET)
#define STM32_RCC_APB2SMENR         (STM32_RCC_BASE+STM32_RCC_APB2SMENR_OFFSET)
#define STM32_RCC_APB1SMENR         (STM32_RCC_BASE+STM32_RCC_APB1SMENR_OFFSET)
#define STM32_RCC_CCIPR             (STM32_RCC_BASE+STM32_RCC_CCIPR_OFFSET)
#define STM32_RCC_CSR               (STM32_RCC_BASE+STM32_RCC_CSR_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* Clock control register */

#define RCC_CR_HSION                (1 << 0)  /* Bit 0: Internal high speed clock enable */
#define RCC_CR_HSIKERON             (1 << 1)  /* Bit 1: Internal high speed clock enable for some IP kernels */
#define RCC_CR_HSIRDY               (1 << 2)  /* Bit 2: Internal high speed clock ready flag */
#define RCC_CR_HSIDIV               (1 << 3)  /* Bit 3: Internal high speed clock divider enable */
#define RCC_CR_HSIDIVF              (1 << 4)  /* Bit 4: Internal high speed clock divider flag */
#define RCC_CR_HSIOUTEN             (1 << 5)  /* Bit 5: Internal high speed clock output enable */
                                              /* Bits 6-7: Reserved */
#define RCC_CR_MSION                (1 << 8)  /* Bit 8: MSI clock enable */
#define RCC_CR_MSIRDY               (1 << 9)  /* Bit 9: MSI clock ready flag */
                                              /* Bits 10-15: Reserved */
#define RCC_CR_HSEON                (1 << 16) /* Bit 16: External high speed clock enable */
#define RCC_CR_HSERDY               (1 << 17) /* Bit 17: External high speed clock ready flag */
#define RCC_CR_HSEBYP               (1 << 18) /* Bit 18: External high speed clock bypass */
#define RCC_CR_CSSHSEON             (1 << 19) /* Bit 19: Clock security system on HSE enable */
#define RCC_CR_RTCPRE_SHIFT         (20)      /* Bits 20-21: RTC prescaler */
#define RCC_CR_RTCPRE_MASK          (3 << RCC_CR_RTCPRE_SHIFT)
                                              /* Bits 22-23: Reserved */
#define RCC_CR_PLLON                (1 << 24) /* Bit 24: PLL enable */
#define RCC_CR_PLLRDY               (1 << 25) /* Bit 25: PLL clock ready flag */
                                              /* Bits 26-27: Reserved */

#define RCC_CR_RSTVAL               0x00000b00

/* Internal clock sources calibration register */

#define RCC_ICSCR_HSICAL_SHIFT      (0)                             /* Bits 0-7:  Internal high speed clock calibration */
#define RCC_ICSCR_HSICAL_MASK       (0xff << RCC_ICSCR_HSICAL_SHIFT)
#define RCC_ICSCR_HSITRIM_SHIFT     (8)                             /* Bits 8-12:  High speed internal clock trimming */
#define RCC_ICSCR_HSITRIM_MASK      (0x1f << RCC_ICSCR_HSITRIM_SHIFT)
#define RCC_ICSCR_MSIRANGE_SHIFT    (13)                            /* Bits 13-15:  MSI clock ranges */
#define RCC_ICSCR_MSIRANGE_MASK     (7 << RCC_ICSCR_MSIRANGE_SHIFT)
#  define RCC_ICSCR_MSIRANGE_0      (0 << RCC_ICSCR_MSIRANGE_SHIFT) /* 000: Range 0 around 65.536 kHz */
#  define RCC_ICSCR_MSIRANGE_1      (1 << RCC_ICSCR_MSIRANGE_SHIFT) /* 001: Range 1 around 131.072 kHz */
#  define RCC_ICSCR_MSIRANGE_2      (2 << RCC_ICSCR_MSIRANGE_SHIFT) /* 010: Range 2 around 262.144 kHz */
#  define RCC_ICSCR_MSIRANGE_3      (3 << RCC_ICSCR_MSIRANGE_SHIFT) /* 011: Range 3 around 524.288 kHz */
#  define RCC_ICSCR_MSIRANGE_4      (4 << RCC_ICSCR_MSIRANGE_SHIFT) /* 100: Range 4 around 1.048 MHz */
#  define RCC_ICSCR_MSIRANGE_5      (5 << RCC_ICSCR_MSIRANGE_SHIFT) /* 101: Range 5 around 2.097 MHz (reset value) */
#  define RCC_ICSCR_MSIRANGE_6      (6 << RCC_ICSCR_MSIRANGE_SHIFT) /* 110: Range 6 around 4.194 MHz */
#define RCC_ICSCR_MSICAL_SHIFT      (16)                            /* Bits 16-23:  MSI clock calibration */
#define RCC_ICSCR_MSICAL_MASK       (0xff << RCC_ICSCR_MSICAL_SHIFT)
#define RCC_ICSCR_MSITRIM_SHIFT     (24)                            /* Bits 24-31:  MSI clock trimming */
#define RCC_ICSCR_MSITRIM_MASK      (0xff << RCC_ICSCR_MSITRIM_SHIFT)

#define RCC_ICSR_RSTVAL             0x0000b000

/* Clock recovery RC register */

#define RCC_CRRCR_HSI48ON           (1 << 0)  /* Bits 0: 48MHz HSI clock enable */
#define RCC_CRRCR_HSI48RDY          (1 << 1)  /* Bits 1: 48MHz HSI clock ready */
#define RCC_CRRCR_HSI48DIV6EN       (1 << 2)  /* Bits 2: 48MHz HSI clock divided by 6 output enable */
                                              /* Bits 3-7: Reserved */
#define RCC_CRRCR_HSI48CAL_SHIFT    (8)       /* Bits 8: 48 MHz HSI reset calibration */
#define RCC_CRRCR_HSI48CAL_MASK     (0xff << RCC_CRRCR_HSI48CAL_SHIFT)
                                              /* Bits 16-31: Reserved */

/* Clock configuration register */

#define RCC_CFGR_SW_SHIFT           (0)                         /* Bits 0-1: System clock Switch */
#define RCC_CFGR_SW_MASK            (3 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_MSI           (0 << RCC_CFGR_SW_SHIFT)    /* 00: MSI selected as system clock */
#  define RCC_CFGR_SW_HSI           (1 << RCC_CFGR_SW_SHIFT)    /* 01: HSI selected as system clock */
#  define RCC_CFGR_SW_HSE           (2 << RCC_CFGR_SW_SHIFT)    /* 10: HSE selected as system clock */
#  define RCC_CFGR_SW_PLL           (3 << RCC_CFGR_SW_SHIFT)    /* 11: PLL selected as system clock */
#define RCC_CFGR_SWS_SHIFT          (2)                         /* Bits 2-3: System Clock Switch Status */
#define RCC_CFGR_SWS_MASK           (3 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_MSI          (0 << RCC_CFGR_SWS_SHIFT)   /* 00: MSI oscillator used as system clock */
#  define RCC_CFGR_SWS_HSI          (1 << RCC_CFGR_SWS_SHIFT)   /* 01: HSI oscillator used as system clock */
#  define RCC_CFGR_SWS_HSE          (2 << RCC_CFGR_SWS_SHIFT)   /* 10: HSE oscillator used as system clock */
#  define RCC_CFGR_SWS_PLL          (3 << RCC_CFGR_SWS_SHIFT)   /* 11: PLL used as system clock */
#define RCC_CFGR_HPRE_SHIFT         (4)                         /* Bits 4-7: AHB prescaler */
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
#define RCC_CFGR_PPRE1_SHIFT        (8)                         /* Bits 8-10: APB Low speed prescaler (APB1) */
#define RCC_CFGR_PPRE1_MASK         (7 << RCC_CFGR_PPRE1_SHIFT)
#  define RCC_CFGR_PPRE1_HCLK       (0 << RCC_CFGR_PPRE1_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE1_HCLKd2     (4 << RCC_CFGR_PPRE1_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE1_HCLKd4     (5 << RCC_CFGR_PPRE1_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE1_HCLKd8     (6 << RCC_CFGR_PPRE1_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE1_HCLKd16    (7 << RCC_CFGR_PPRE1_SHIFT) /* 111: HCLK divided by 16 */
#define RCC_CFGR_PPRE2_SHIFT        (11)                        /* Bits 11-13: APB High speed prescaler (APB2) */
#define RCC_CFGR_PPRE2_MASK         (7 << RCC_CFGR_PPRE2_SHIFT)
#  define RCC_CFGR_PPRE2_HCLK       (0 << RCC_CFGR_PPRE2_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE2_HCLKd2     (4 << RCC_CFGR_PPRE2_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE2_HCLKd4     (5 << RCC_CFGR_PPRE2_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE2_HCLKd8     (6 << RCC_CFGR_PPRE2_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE2_HCLKd16    (7 << RCC_CFGR_PPRE2_SHIFT) /* 111: HCLK divided by 16 */
                                                                /* Bits 14: Reserved */
#define RCC_CFGR_STOPWUCK           (15)                        /* Bits 15:  */
#define RCC_CFGR_PLLSRC             (1 << 16)                   /* Bit 16: PLL entry clock source */
                                                                /* Bit 17: Reserved */
#define RCC_CFGR_PLLMUL_SHIFT       (18)                        /* Bits 18-21: PLL Multiplication Factor */
#define RCC_CFGR_PLLMUL_MASK        (15 << RCC_CFGR_PLLMUL_SHIFT)
#  define RCC_CFGR_PLLMUL_CLKx3     (0 << RCC_CFGR_PLLMUL_SHIFT)  /* 0000: PLL clock entry x 3 */
#  define RCC_CFGR_PLLMUL_CLKx4     (1 << RCC_CFGR_PLLMUL_SHIFT)  /* 0001: PLL clock entry x 4 */
#  define RCC_CFGR_PLLMUL_CLKx6     (2 << RCC_CFGR_PLLMUL_SHIFT)  /* 0010: PLL clock entry x 6 */
#  define RCC_CFGR_PLLMUL_CLKx8     (3 << RCC_CFGR_PLLMUL_SHIFT)  /* 0011: PLL clock entry x 8 */
#  define RCC_CFGR_PLLMUL_CLKx12    (4 << RCC_CFGR_PLLMUL_SHIFT)  /* 0100: PLL clock entry x 12 */
#  define RCC_CFGR_PLLMUL_CLKx16    (5 << RCC_CFGR_PLLMUL_SHIFT)  /* 0101: PLL clock entry x 16 */
#  define RCC_CFGR_PLLMUL_CLKx24    (6 << RCC_CFGR_PLLMUL_SHIFT)  /* 0110: PLL clock entry x 24 */
#  define RCC_CFGR_PLLMUL_CLKx32    (7 << RCC_CFGR_PLLMUL_SHIFT)  /* 0111: PLL clock entry x 32 */
#  define RCC_CFGR_PLLMUL_CLKx48    (8 << RCC_CFGR_PLLMUL_SHIFT)  /* 1000: PLL clock entry x 48 */
#define RCC_CFGR_PLLDIV_SHIFT       (22)                          /* Bits 22-23: PLL output division */
#define RCC_CFGR_PLLDIV_MASK        (3 << RCC_CFGR_PLLDIV_SHIFT)
#  define RCC_CFGR_PLLDIV_2         (1 << RCC_CFGR_PLLDIV_SHIFT) /* 01: PLL clock output = PLLVCO / 2 */
#  define RCC_CFGR_PLLDIV_3         (2 << RCC_CFGR_PLLDIV_SHIFT) /* 10: PLL clock output = PLLVCO / 3 */
#  define RCC_CFGR_PLLDIV_4         (3 << RCC_CFGR_PLLDIV_SHIFT) /* 11: PLL clock output = PLLVCO / 4 */
#define RCC_CFGR_MCOSEL_SHIFT       (24)                         /* Bits 24-27: Microcontroller clock output selection */
#define RCC_CFGR_MCOSEL_MASK        (7 << RCC_CFGR_MCOSEL_SHIFT)
#  define RCC_CFGR_MCOSEL_DISABLED  (0 << RCC_CFGR_MCOSEL_SHIFT)  /* 0000: MCO output disabled, no clock on MCO */
#  define RCC_CFGR_MCOSEL_SYSCLK    (1 << RCC_CFGR_MCOSEL_SHIFT)  /* 0001: SYSCLK clock selected */
#  define RCC_CFGR_MCOSEL_HSICLK    (2 << RCC_CFGR_MCOSEL_SHIFT)  /* 0010: HSI16 oscillator clock selected */
#  define RCC_CFGR_MCOSEL_MSICLK    (3 << RCC_CFGR_MCOSEL_SHIFT)  /* 0011: MSI oscillator clock selected */
#  define RCC_CFGR_MCOSEL_HSECLK    (4 << RCC_CFGR_MCOSEL_SHIFT)  /* 0100: HSE oscillator clock selected */
#  define RCC_CFGR_MCOSEL_PLLCLK    (5 << RCC_CFGR_MCOSEL_SHIFT)  /* 0101: PLL clock selected */
#  define RCC_CFGR_MCOSEL_LSICLK    (6 << RCC_CFGR_MCOSEL_SHIFT)  /* 0110: LSI oscillator clock selected */
#  define RCC_CFGR_MCOSEL_LSECLK    (7 << RCC_CFGR_MCOSEL_SHIFT)  /* 0111: LSE oscillator clock selected */
#  define RCC_CFGR_MCOSEL_HSI48CLK  (8 << RCC_CFGR_MCOSEL_SHIFT)  /* 1000: HSI48 oscillator clock selected */
#define RCC_CFGR_MCOPRE_SHIFT       (28)                          /* Bits 28-30: Microcontroller clock output selection */
#define RCC_CFGR_MCOPRE_MASK        (7 << RCC_CFGR_MCOPRE_SHIFT)
#  define RCC_CFGR_MCOPRE_DIV1      (0 << RCC_CFGR_MCOPRE_SHIFT) /* 000: MCO is divided by 1 */
#  define RCC_CFGR_MCOPRE_DIV2      (1 << RCC_CFGR_MCOPRE_SHIFT) /* 001: MCO is divided by 2 */
#  define RCC_CFGR_MCOPRE_DIV4      (2 << RCC_CFGR_MCOPRE_SHIFT) /* 010: MCO is divided by 4 */
#  define RCC_CFGR_MCOPRE_DIV8      (3 << RCC_CFGR_MCOPRE_SHIFT) /* 011: MCO is divided by 8 */
#  define RCC_CFGR_MCOPRE_DIV16     (4 << RCC_CFGR_MCOPRE_SHIFT) /* 100: MCO is divided by 16 */
                                                                 /* Bit 31: Reserved */
#define RCC_CFGR_RESET              0x00000000

/* Clock Source Interrupt enable register */

#define RCC_CIER_LSIRDYF            (1 << 0)  /* Bit 0: LSI ready interrupt flag */
#define RCC_CIER_LSERDYF            (1 << 1)  /* Bit 1: LSE ready interrupt flag */
#define RCC_CIER_HSIRDYF            (1 << 2)  /* Bit 2: HSI16 ready interrupt flag */
#define RCC_CIER_HSERDYF            (1 << 3)  /* Bit 3: HSE ready interrupt flag */
#define RCC_CIER_PLLRDYF            (1 << 4)  /* Bit 4: PLL ready interrupt flag */
#define RCC_CIER_MSIRDYF            (1 << 5)  /* Bit 5: MSI ready interrupt flag */
#define RCC_CIER_HSI48RDYF          (1 << 6)  /* Bit 6: HSI48 ready interrupt flag */
#define RCC_CIER_CSSLSE             (1 << 7)  /* Bit 7: LSE CSS interrupt flag */

/* Clock Source Interrupt Flag register */

#define RCC_CIFR_LSIRDYF            (1 << 0)  /* Bit 0: LSI ready interrupt flag */
#define RCC_CIFR_LSERDYF            (1 << 1)  /* Bit 1: LSE ready interrupt flag */
#define RCC_CIFR_HSIRDYF            (1 << 2)  /* Bit 2: HSI16 ready interrupt flag */
#define RCC_CIFR_HSERDYF            (1 << 3)  /* Bit 3: HSE ready interrupt flag */
#define RCC_CIFR_PLLRDYF            (1 << 4)  /* Bit 4: PLL ready interrupt flag */
#define RCC_CIFR_MSIRDYF            (1 << 5)  /* Bit 5: MSI ready interrupt flag */
#define RCC_CIFR_HSI48RDYF          (1 << 6)  /* Bit 6: HSI48 ready interrupt flag */
#define RCC_CIFR_CSSLSE             (1 << 7)  /* Bit 7: LSE CSS interrupt flag */

/* Clock Source Interrupt Clear register */

#define RCC_CICR_LSIRDYF            (1 << 0)  /* Bit 0: LSI ready interrupt flag */
#define RCC_CICR_LSERDYF            (1 << 1)  /* Bit 1: LSE ready interrupt flag */
#define RCC_CICR_HSIRDYF            (1 << 2)  /* Bit 2: HSI16 ready interrupt flag */
#define RCC_CICR_HSERDYF            (1 << 3)  /* Bit 3: HSE ready interrupt flag */
#define RCC_CICR_PLLRDYF            (1 << 4)  /* Bit 4: PLL ready interrupt flag */
#define RCC_CICR_MSIRDYF            (1 << 5)  /* Bit 5: MSI ready interrupt flag */
#define RCC_CICR_HSI48RDYF          (1 << 6)  /* Bit 6: HSI48 ready interrupt flag */
#define RCC_CICR_CSSLSE             (1 << 7)  /* Bit 7: LSE CSS interrupt flag */

/* GPIO reset register */

#define RCC_IOPRSTR_IOPARST         (1 << 0)  /* Bit 0: IO port A reset */
#define RCC_IOPRSTR_IOPBRST         (1 << 1)  /* Bit 1: IO port B reset */
#define RCC_IOPRSTR_IOPCRST         (1 << 2)  /* Bit 3: IO port C reset */
#define RCC_IOPRSTR_IOPDRST         (1 << 3)  /* Bit 4: IO port D reset */
#define RCC_IOPRSTR_IOPERST         (1 << 4)  /* Bit 4: IO port E reset */
                                              /* Bits 5-6: Reserved */
#define RCC_IOPRSTR_IOPHRST         (1 << 7)  /* Bit 5: IO port H reset */

/* AHB peripheral reset register */

#define RCC_AHBRSTR_DMA1RST         (1 << 0)  /* Bit 0: DMA 1 reset */
                                              /* Bits 1-7: Reserved */
#define RCC_AHBRSTR_MIFRST          (1 << 8)  /* Bit 8: Memory interface reset */
                                              /* Bits 9-11: Reserved */
#define RCC_AHBRSTR_CRCRST          (1 << 12) /* Bit 12: Memory interface reset */
                                              /* Bits 13-15: Reserved */
#define RCC_AHBRSTR_TSCRST          (1 << 16) /* Bit 16: Touch sensing reset */
                                              /* Bits 17-19: Reserved */
#define RCC_AHBRSTR_RNGRST          (1 << 20) /* Bit 20: Random number generator module reset */
                                              /* Bits 21-23: Reserved */
#define RCC_AHBRSTR_AESRST          (1 << 24) /* Bit 24: Crypto module (AES) reset */
                                              /* Bits 25-31: Reserved */

/* APB2 Peripheral reset register */

#define RCC_APB2RSTR_SYSCFGRST      (1 << 0)  /* Bit 0:  System configuration controller reset */
                                              /* Bit 1:  Reserved */
#define RCC_APB2RSTR_TIM21RST       (1 << 2)  /* Bit 2:  TIM21 timer reset */
                                              /* Bits 3-4: Reserved */
#define RCC_APB2RSTR_TIM22RST       (1 << 5)  /* Bit 5:  TIM21 timer reset */
                                              /* Bits 6-8: Reserved */
#define RCC_APB2RSTR_ADC1RST        (1 << 9)  /* Bit 9:  ADC1 interface reset */
                                              /* Bits 10-11: Reserved */
#define RCC_APB2RSTR_SPI1RST        (1 << 12) /* Bit 12: SPI 1 reset */
                                              /* Bit 13: Reserved */
#define RCC_APB2RSTR_USART1RST      (1 << 14) /* Bit 14: USART1 reset */
                                              /* Bits 15-21: Reserved */
#define RCC_APB2RSTR_DBGRST         (1 << 22) /* Bit 22: DBG reset */
                                              /* Bits 23-31: Reserved */

/* APB1 Peripheral reset register */

#define RCC_APB1RSTR_TIM2RST        (1 << 0)  /* Bit 0:  Timer 2 reset */
#define RCC_APB1RSTR_TIM3RST        (1 << 1)  /* Bit 1:  Timer 3 reset */
                                              /* Bits 2-3: Reserved */
#define RCC_APB1RSTR_TIM6RST        (1 << 4)  /* Bit 4:  Timer 6 reset */
#define RCC_APB1RSTR_TIM7RST        (1 << 5)  /* Bit 5:  Timer 7 reset */
                                              /* Bits 6-8: Reserved */
#define RCC_APB1RSTR_LCDRST         (1 << 9)  /* Bit 9:  LCD reset */
                                              /* Bit 10: Reserved */
#define RCC_APB1RSTR_WWDGRST        (1 << 11) /* Bit 11: Window Watchdog reset */
                                              /* Bits 12-13: Reserved */
#define RCC_APB1RSTR_SPI2RST        (1 << 14) /* Bit 14: SPI 2 reset */
                                              /* Bits 15-16: Reserved */
#define RCC_APB1RSTR_USART2RST      (1 << 17) /* Bit 17: USART 2 reset */
#define RCC_APB1RSTR_USART3RST      (1 << 18) /* Bit 18: USART 3 reset */
#define RCC_APB1RSTR_USART4RST      (1 << 19) /* Bit 19: USART 4 reset */
#define RCC_APB1RSTR_USART5RST      (1 << 20) /* Bit 20: USART 5 reset */
#define RCC_APB1RSTR_I2C1RST        (1 << 21) /* Bit 21: I2C 1 reset */
#define RCC_APB1RSTR_I2C2RST        (1 << 22) /* Bit 22: I2C 2 reset */
#define RCC_APB1RSTR_USBRST         (1 << 23) /* Bit 23: USB reset */
                                              /* Bits 24-26: Reserved */
#define RCC_APB1RSTR_CRSRST         (1 << 27) /* Bit 27: Clock recovery system reset */
#define RCC_APB1RSTR_PWRRST         (1 << 28) /* Bit 28: Power interface reset */
#define RCC_APB1RSTR_DAC1RST        (1 << 29) /* Bit 29: DAC 1 interface reset */
#define RCC_APB1RSTR_I2C3RST        (1 << 30) /* Bit 30: I2C3 reset */
#define RCC_APB1RSTR_LPTIM1RST      (1 << 31) /* Bit 31: Low-power timer reset */

/* GPIO clock enable register */

#define RCC_IOPENR_IOPAEN           (1 << 0)  /* Bit 0: IO port A clock enable */
#define RCC_IOPENR_IOPBEN           (1 << 1)  /* Bit 1: IO port B clock enable */
#define RCC_IOPENR_IOPCEN           (1 << 2)  /* Bit 2: IO port C clock enable */
#define RCC_IOPENR_IOPDEN           (1 << 3)  /* Bit 3: IO port D clock enable */
#define RCC_IOPENR_IOPEEN           (1 << 4)  /* Bit 4: IO port E clock enable */
                                              /* Bits 5-6: Reserved */
#define RCC_IOPENR_IOPHEN           (1 << 7)  /* Bit 5: IO port H clock enable */

/* AHB Peripheral Clock enable register */

#define RCC_AHBENR_DMA1EN           (1 << 0)  /* Bit 0: DMA 1 clock enable */
                                              /* Bits 1-7: Reserved */
#define RCC_AHBENR_MIFEN            (1 << 8)  /* Bit 8: Memory interface clock enable */
                                              /* Bits 9-11: Reserved */
#define RCC_AHBENR_CRCEN            (1 << 12) /* Bit 12: Memory interface clock enable */
                                              /* Bits 13-15: Reserved */
#define RCC_AHBENR_TSCEN            (1 << 12) /* Bit 12: Touch sensing clock enable */
                                              /* Bits 17-19: Reserved */
#define RCC_AHBENR_RNGEN            (1 << 20) /* Bit 20: Random number generator module clock enable */
                                              /* Bits 21-23: Reserved */
#define RCC_AHBENR_AESEN            (1 << 24) /* Bit 24: Crypto module (AES) clock enable */
                                              /* Bits 25-31: Reserved */

/* APB2 Peripheral Clock enable register */

#define RCC_APB2ENR_SYSCFGEN        (1 << 0)  /* Bit 0:  System configuration controller clock enable */
                                              /* Bit 1:  Reserved */
#define RCC_APB2ENR_TIM21EN         (1 << 2)  /* Bit 2:  TIM21 timer clock enable */
                                              /* Bits 3-4: Reserved */
#define RCC_APB2ENR_TIM22EN         (1 << 5)  /* Bit 5:  TIM21 timer clock enable */
                                              /* Bits 6-8: Reserved */
#define RCC_APB2ENR_ADC1EN          (1 << 9)  /* Bit 9:  ADC1 interface clock enable */
                                              /* Bits 10-11: Reserved */
#define RCC_APB2ENR_SPI1EN          (1 << 12) /* Bit 12: SPI 1 clock enable */
                                              /* Bit 13: Reserved */
#define RCC_APB2ENR_USART1EN        (1 << 14) /* Bit 14: USART1 clock enable */
                                              /* Bits 15-21: Reserved */
#define RCC_APB2ENR_DBGEN           (1 << 22) /* Bit 22: DBG clock enable */
                                              /* Bits 23-31: Reserved */

/* APB1 Peripheral Clock enable register */

#define RCC_APB1ENR_TIM2EN            (1 << 0)  /* Bit 0:  Timer 2 clock enable */
#define RCC_APB1ENR_TIM3EN            (1 << 1)  /* Bit 1:  Timer 3 clock enable */
                                                /* Bits 2-3: Reserved */
#define RCC_APB1ENR_TIM6EN            (1 << 4)  /* Bit 4:  Timer 6 clock enable */
#define RCC_APB1ENR_TIM7EN            (1 << 5)  /* Bit 5:  Timer 7 clock enable */
                                                /* Bits 6-8: Reserved */
#define RCC_APB1ENR_LCDEN             (1 << 9)  /* Bit 9:  LCD clock enable */
                                                /* Bit 10: Reserved */
#define RCC_APB1ENR_WWDGEN            (1 << 11) /* Bit 11: Window Watchdog clock enable */
                                                /* Bits 12-13: Reserved */
#define RCC_APB1ENR_SPI2EN            (1 << 14) /* Bit 14: SPI 2 clock enable */
                                                /* Bits 15-16: Reserved */
#define RCC_APB1ENR_USART2EN          (1 << 17) /* Bit 17: USART 2 clock enable */
#define RCC_APB1ENR_USART3EN          (1 << 18) /* Bit 18: USART 3 clock enable */
#define RCC_APB1ENR_USART4EN          (1 << 19) /* Bit 19: USART 4 clock enable */
#define RCC_APB1ENR_USART5EN          (1 << 20) /* Bit 20: USART 5 clock enable */
#define RCC_APB1ENR_I2C1EN            (1 << 21) /* Bit 21: I2C 1 clock enable */
#define RCC_APB1ENR_I2C2EN            (1 << 22) /* Bit 22: I2C 2 clock enable */
#define RCC_APB1ENR_USBEN             (1 << 23) /* Bit 23: USB clock enable */
                                                /* Bits 24-26: Reserved */
#define RCC_APB1ENR_CRSEN             (1 << 27) /* Bit 27: Clock recovery system clock enable */
#define RCC_APB1ENR_PWREN             (1 << 28) /* Bit 28: Power interface clock enable */
#define RCC_APB1ENR_DAC1EN            (1 << 29) /* Bit 29: DAC 1 interface clock enable */
#define RCC_APB1ENR_I2C3EN            (1 << 30) /* Bit 30: I2C3 clock enable */
#define RCC_APB1ENR_LPTIM1EN          (1 << 31) /* Bit 31: Low-power timer clock enable */

/* GPIO clock enable in Sleep mode register */

#define RCC_IOPSMENR_IOPASMEN          (1 << 0)  /* Bit 0: IO port A clock enable during Sleep mode */
#define RCC_IOPSMENR_IOPBSMEN          (1 << 1)  /* Bit 1: IO port B clock enable during Sleep mode */
#define RCC_IOPSMENR_IOPCSMEN          (1 << 2)  /* Bit 2: IO port C clock enable during Sleep mode */
#define RCC_IOPSMENR_IOPDSMEN          (1 << 3)  /* Bit 3: IO port D clock enable during Sleep mode */
#define RCC_IOPSMENR_IOPESMEN          (1 << 4)  /* Bit 4: IO port E clock enable during Sleep mode */
                                                 /* Bits 5-6: Reserved */
#define RCC_IOPSMENR_IOPHSMEN          (1 << 7)  /* Bit 7: IO port H clock enable during Sleep mode */

/* AHB peripheral clock enable in Sleep mode register */

#define RCC_AHBSMENR_DMA1SMEN           (1 << 0)  /* Bit 0: DMA 1 clock enable in Sleep mode */
                                                  /* Bits 1-7: Reserved */
#define RCC_AHBSMENR_MIFSMEN            (1 << 8)  /* Bit 8: Memory interface clock enable in Sleep mode */
                                                  /* Bits 9-11: Reserved */
#define RCC_AHBSMENR_CRCSMEN            (1 << 12) /* Bit 12: Memory interface clock enable in Sleep mode */
                                                  /* Bits 13-15: Reserved */
#define RCC_AHBSMENR_TSCSMEN            (1 << 12) /* Bit 12: Touch sensing clock enable in Sleep mode */
                                                  /* Bits 17-19: Reserved */
#define RCC_AHBSMENR_RNGSMEN            (1 << 20) /* Bit 20: Random number generator module clock enable in Sleep mode */
                                                  /* Bits 21-23: Reserved */
#define RCC_AHBSMENR_AESSMEN            (1 << 24) /* Bit 24: Crypto module (AES) clock enable in Sleep mode */
                                                  /* Bits 25-31: Reserved */

/* APB2 peripheral clock enable in Sleep mode register */

#define RCC_APB2SMENR_SYSCFGSMEN        (1 << 0)  /* Bit 0:  System configuration controller clock enable in Sleep mode */
                                                  /* Bit 1:  Reserved */
#define RCC_APB2SMENR_TIM21SMEN         (1 << 2)  /* Bit 2:  TIM21 timer clock enable in Sleep mode */
                                                  /* Bits 3-4: Reserved */
#define RCC_APB2SMENR_TIM22SMEN         (1 << 5)  /* Bit 5:  TIM21 timer clock enable in Sleep mode */
                                                  /* Bits 6-8: Reserved */
#define RCC_APB2SMENR_ADC1SMEN          (1 << 9)  /* Bit 9:  ADC1 interface clock enable in Sleep mode */
                                                  /* Bits 10-11: Reserved */
#define RCC_APB2SMENR_SPI1SMEN          (1 << 12) /* Bit 12: SPI 1 clock enable in Sleep mode */
                                                  /* Bit 13: Reserved */
#define RCC_APB2SMENR_USART1SMEN        (1 << 14) /* Bit 14: USART1 clock enable in Sleep mode */
                                                  /* Bits 15-21: Reserved */
#define RCC_APB2SMENR_DBGSMEN           (1 << 22) /* Bit 22: DBG clock enable in Sleep mode */
                                                  /* Bits 23-31: Reserved */

/* APB1 peripheral clock enable in Sleep mode register */

#define RCC_APB1SMENR_TIM2SMEN          (1 << 0)  /* Bit 0:  Timer 2 clock enable in Sleep mode */
#define RCC_APB1SMENR_TIM3SMEN          (1 << 1)  /* Bit 1:  Timer 3 clock enable in Sleep mode */
                                                  /* Bits 2-3: Reserved */
#define RCC_APB1SMENR_TIM6SMEN          (1 << 4)  /* Bit 4:  Timer 6 clock enable in Sleep mode */
#define RCC_APB1SMENR_TIM7SMEN          (1 << 5)  /* Bit 5:  Timer 7 clock enable in Sleep mode */
                                                  /* Bits 6-8: Reserved */
#define RCC_APB1SMENR_LCDSMEN           (1 << 9)  /* Bit 9: LCD clock enable in Sleep mode */
                                                  /* Bits 10: Reserved */
#define RCC_APB1SMENR_WWDGSMEN          (1 << 11) /* Bit 11: Window Watchdog clock enable in Sleep mode */
                                                  /* Bits 12-13: Reserved */
#define RCC_APB1SMENR_SPI2SMEN          (1 << 14) /* Bit 14: SPI 2 clock enable in Sleep mode */
                                                  /* Bits 15-16: Reserved */
#define RCC_APB1SMENR_USART2SMEN        (1 << 17) /* Bit 17: USART 2 clock enable in Sleep mode */
#define RCC_APB1SMENR_USART3SMEN        (1 << 18) /* Bit 18: USART 3 clock enable in Sleep mode */
#define RCC_APB1SMENR_USART4SMEN        (1 << 19) /* Bit 19: USART 4 clock enable in Sleep mode */
#define RCC_APB1SMENR_USART5SMEN        (1 << 20) /* Bit 20: USART 5 clock enable in Sleep mode */
#define RCC_APB1SMENR_I2C1SMEN          (1 << 21) /* Bit 21: I2C 1 clock enable in Sleep mode */
#define RCC_APB1SMENR_I2C2SMEN          (1 << 22) /* Bit 22: I2C 2 clock enable in Sleep mode */
#define RCC_APB1SMENR_USBSMEN           (1 << 23) /* Bit 23: USB clock enable in Sleep mode */
                                                  /* Bits 24-26: Reserved */
#define RCC_APB1SMENR_CRSSMEN           (1 << 27) /* Bit 27: Clock recovery system clock enable in Sleep mode */
#define RCC_APB1SMENR_PWRSMEN           (1 << 28) /* Bit 28: Power interface clock enable in Sleep mode */
#define RCC_APB1SMENR_DAC1SMEN          (1 << 29) /* Bit 29: DAC 1 interface clock enable in Sleep mode */
#define RCC_APB1SMENR_I2C3SMEN          (1 << 30) /* Bit 30: I2C3 clock enable in Sleep mode */
#define RCC_APB1SMENR_LPTIM1SMEN        (1 << 31) /* Bit 31: Low-power timer clock enable in Sleep mode */

/* Clock configuration register */

#define RCC_CCIPR_USART1SEL_SHIFT       (0)  /* Bits 0-1: USART1 clock source selection */
#define RCC_CCIPR_USART1SEL_MASK        (3 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_APB       (0 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_SYSCLK    (1 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_HSI16     (2 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_LSE       (3 << RCC_CCIPR_USART1SEL_SHIFT)
#define RCC_CCIPR_USART2SEL_SHIFT       (2)  /* Bits 2-3: USART2 clock source selection */
#define RCC_CCIPR_USART2SEL_MASK        (3 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_APB       (0 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_SYSCLK    (1 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_HSI16     (2 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_LSE       (3 << RCC_CCIPR_USART2SEL_SHIFT)
                                              /* Bits 4-9: Reserved */
#define RCC_CCIPR_LPUART1SEL_SHIFT      (10)  /* Bits 10-11: LPUART1 clock source selection */
#define RCC_CCIPR_LPUART1SEL_MASK       (3 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_APB      (0 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_SYSCLK   (1 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_HSI16    (2 << RCC_CCIPR_LPUART1SEL_SHIFT)
#  define RCC_CCIPR_LPUART1SEL_LSE      (3 << RCC_CCIPR_LPUART1SEL_SHIFT)
#define RCC_CCIPR_I2C1SEL_SHIFT         (12)  /* Bits 12-13: I2C1 clock source selection */
#define RCC_CCIPR_I2C1SEL_MASK          (3 << RCC_CCIPR_I2C1SEL_SHIFT)
#  define RCC_CCIPR_I2C1SEL_APB         (0 << RCC_CCIPR_I2C1SEL_SHIFT)
#  define RCC_CCIPR_I2C1SEL_SYSCLK      (1 << RCC_CCIPR_I2C1SEL_SHIFT)
#  define RCC_CCIPR_I2C1SEL_HSI16       (2 << RCC_CCIPR_I2C1SEL_SHIFT)
                                              /* Bits 14-15: Reserved */
#define RCC_CCIPR_I2C3SEL_SHIFT         (16)  /* Bits 16-17: I2C3 clock source selection */
#define RCC_CCIPR_I2C3SEL_MASK          (3 << RCC_CCIPR_I2C3SEL_SHIFT)
#  define RCC_CCIPR_I2C3SEL_APB         (0 << RCC_CCIPR_I2C3SEL_SHIFT)
#  define RCC_CCIPR_I2C3SEL_SYSCLK      (1 << RCC_CCIPR_I2C3SEL_SHIFT)
#  define RCC_CCIPR_I2C3SEL_HSI16       (2 << RCC_CCIPR_I2C3SEL_SHIFT)
#define RCC_CCIPR_LPTIM1SEL_SHIFT       (18)  /* Bits 18-19: LPTIM1 clock source selection */
#define RCC_CCIPR_LPTIM1SEL_MASK        (3 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_APB       (0 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_SYSCLK    (1 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_HSI16     (2 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_LSE       (3 << RCC_CCIPR_LPTIM1SEL_SHIFT)
                                              /* Bits 20-25: Reserved */
#define RCC_CCIPR_CLK48SEL_SHIFT        (26)  /* Bit 26: HSI48 clock source selection */
#define RCC_CCIPR_CLK48SEL_MASK         (1 << RCC_CCIPR_CLK48SEL_SHIFT)
#  define RCC_CCIPR_CLK48SEL_PLL        (0 << RCC_CCIPR_CLK48SEL_SHIFT)
#  define RCC_CCIPR_CLK48SEL_HSI48      (1 << RCC_CCIPR_CLK48SEL_SHIFT)
                                              /* Bits 27-31: Reserved */

/* Control/status register */

#define RCC_CSR_LSION                   (1 << 0)  /* Bit 0: LSI enable */
#define RCC_CSR_LSIRDY                  (1 << 1)  /* Bit 1: ready */
                                                  /* Bits 2-7: Reserved */
#define RCC_CSR_LSEON                   (1 << 8)  /* Bit 8: LSE enable */
#define RCC_CSR_LSERDY                  (1 << 9)  /* Bit 9: LSE ready */
#define RCC_CSR_LSEBPY                  (1 << 10) /* Bit 10: LSE bypass */
#define RCC_CSR_LSEDRV_SHIFT            (11)      /* Bits 11-12: LSE driving capability */
#define RCC_CSR_LSEDRV_MASK             (3 << RCC_CSR_LSEDRV_SHIFT)
#  define RCC_CSR_LSEDRV_LOW            (0 << RCC_CSR_LSEDRV_SHIFT)
#  define RCC_CSR_LSEDRV_MEDLOW         (1 << RCC_CSR_LSEDRV_SHIFT)
#  define RCC_CSR_LSEDRV_MEDHGIH        (2 << RCC_CSR_LSEDRV_SHIFT)
#  define RCC_CSR_LSEDRV_HIGH           (3 << RCC_CSR_LSEDRV_SHIFT)
#define RCC_CSR_CSSLSEON                (1 << 13) /* Bit 13: CSS on LSE enable */
#define RCC_CSR_CSSLSED                 (1 << 14) /* Bit 14: CSS on LSE failure detection flag */
                                                  /* Bit 15: Reserved */
#define RCC_CSR_RTCSEL_SHIFT            (16)      /* Bits 16-17: RTC clock source selection */
#define RCC_CSR_RTCSEL_MASK             (3 << RCC_CSR_RTCSEL_SHIFT)
#  define RCC_CSR_RTCSEL_NOCLK          (0 << RCC_CSR_RTCSEL_SHIFT)
#  define RCC_CSR_RTCSEL_LSE            (1 << RCC_CSR_RTCSEL_SHIFT)
#  define RCC_CSR_RTCSEL_LSI            (2 << RCC_CSR_RTCSEL_SHIFT)
#  define RCC_CSR_RTCSEL_HSE            (3 << RCC_CSR_RTCSEL_SHIFT)
#define RCC_CSR_RTCEN                   (1 << 18) /* Bit 18: RTC clock enable */
#define RCC_CSR_RTCRST                  (1 << 19) /* Bit 19: RTC software reset */
                                                  /* Bits 20-22: Reserved */
#define RCC_CSR_RMVF                    (1 << 23) /* Bit 23: Remove reset flag */
#define RCC_CSR_FWRSTF                  (1 << 24) /* Bit 24: Firewall reset flag */
#define RCC_CSR_OBLRSTF                 (1 << 25) /* Bit 25: Options bytes loading reset flag */
#define RCC_CSR_PINRSTF                 (1 << 26) /* Bit 26: PIN reset flag */
#define RCC_CSR_PORRSTF                 (1 << 27) /* Bit 27: POR/PDR reset flag */
#define RCC_CSR_SFTRSTF                 (1 << 28) /* Bit 28: software reset flag */
#define RCC_CSR_IWDGRSTF                (1 << 29) /* Bit 29: IWDG reset flag */
#define RCC_CSR_WWDGRSTF                (1 << 30) /* Bit 30: WWDG reset flag */
#define RCC_CSR_LPWRRSTF                (1 << 31) /* Bit 31: Low-power reset flag */

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32L0_RCC_H */
