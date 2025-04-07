/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32c0_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32C0_RCC_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32C0_RCC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NOTE: STM32C0 does not have separate APB1 and APB2 buses, but uses single
 * APB bus, however, divided into two sets of registers. For compatibility
 * with other families, we leave the register names as APB1xx and APB2yy.
 */

/* Register Offsets *********************************************************/

#define STM32_RCC_CR_OFFSET         0x0000  /* Clock control register */
#define STM32_RCC_ICSCR_OFFSET      0x0004  /* Internal clock sources calibration register */
#define STM32_RCC_CFGR_OFFSET       0x0008  /* Clock configuration register */
#define STM32_RCC_CRRCR_OFFSET      0x0014  /* Clock recovery RC register */
#define STM32_RCC_CIER_OFFSET       0x0018  /* Clock Source Interrupt enable register */
#define STM32_RCC_CIFR_OFFSET       0x001c  /* Clock Source Interrupt Flag register */
#define STM32_RCC_CICR_OFFSET       0x0020  /* Clock Source Interrupt Clear register */
#define STM32_RCC_IOPRSTR_OFFSET    0x0024  /* GPIO reset register */
#define STM32_RCC_AHBRSTR_OFFSET    0x0028  /* AHB peripheral reset register */
#define STM32_RCC_APB1RSTR_OFFSET   0x002c  /* APB Peripheral reset register 1 */
#define STM32_RCC_APB2RSTR_OFFSET   0x0030  /* APB Peripheral reset register 2 */
#define STM32_RCC_IOPENR_OFFSET     0x0034  /* GPIO clock enable register */
#define STM32_RCC_AHBENR_OFFSET     0x0038  /* AHB Peripheral Clock enable register */
#define STM32_RCC_APB1ENR_OFFSET    0x003c  /* APB Peripheral Clock enable register 1 */
#define STM32_RCC_APB2ENR_OFFSET    0x0040  /* APB Peripheral Clock enable register 2 */
#define STM32_RCC_IOPSMEN_OFFSET    0x0044  /* GPIO clock enable in Sleep mode register */
#define STM32_RCC_AHBSMENR_OFFSET   0x0048  /* AHB peripheral clock enable in Sleep mode register */
#define STM32_RCC_APBSMENR1_OFFSET  0x004c  /* APB peripheral clock enable in Sleep mode register 1 */
#define STM32_RCC_APBSMENR2_OFFSET  0x0050  /* APB peripheral clock enable in Sleep mode register 2 */
#define STM32_RCC_CCIPR1_OFFSET     0x0054  /* Clock configuration register 1 */
#define STM32_RCC_CCIPR2_OFFSET     0x0058  /* Clock configuration register 2 */
#define STM32_RCC_CSR1_OFFSET       0x005c  /* Control/status register 1 */
#define STM32_RCC_CSR2_OFFSET       0x0060  /* Control/status register 2*/

/* Register Addresses *******************************************************/

#define STM32_RCC_CR                (STM32_RCC_BASE+STM32_RCC_CR_OFFSET)
#define STM32_RCC_ICSCR             (STM32_RCC_BASE+STM32_RCC_ICSCR_OFFSET)
#define STM32_RCC_CFGR              (STM32_RCC_BASE+STM32_RCC_CFGR_OFFSET)
#define STM32_RCC_CRRCR             (STM32_RCC_BASE+STM32_RCC_CRRCR_OFFSET)
#define STM32_RCC_CIER              (STM32_RCC_BASE+STM32_RCC_CIER_OFFSET)
#define STM32_RCC_CIFR              (STM32_RCC_BASE+STM32_RCC_CIFR_OFFSET)
#define STM32_RCC_CICR              (STM32_RCC_BASE+STM32_RCC_CICR_OFFSET)
#define STM32_RCC_IOPRSTR           (STM32_RCC_BASE+STM32_RCC_IOPRSTR_OFFSET)
#define STM32_RCC_AHBRSTR           (STM32_RCC_BASE+STM32_RCC_AHBRSTR_OFFSET)
#define STM32_RCC_APB1RSTR          (STM32_RCC_BASE+STM32_RCC_APB1RSTR_OFFSET)
#define STM32_RCC_APB2RSTR          (STM32_RCC_BASE+STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_IOPENR            (STM32_RCC_BASE+STM32_RCC_IOPENR_OFFSET)
#define STM32_RCC_AHBENR            (STM32_RCC_BASE+STM32_RCC_AHBENR_OFFSET)
#define STM32_RCC_APB1ENR           (STM32_RCC_BASE+STM32_RCC_APB1ENR_OFFSET)
#define STM32_RCC_APB2ENR           (STM32_RCC_BASE+STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_IOPSMEN           (STM32_RCC_BASE+STM32_RCC_IOPSMEN_OFFSET)
#define STM32_RCC_AHBSMENR          (STM32_RCC_BASE+STM32_RCC_AHBSMENR_OFFSET)
#define STM32_RCC_APBSMENR1         (STM32_RCC_BASE+STM32_RCC_APBSMENR1_OFFSET)
#define STM32_RCC_APBSMENR2         (STM32_RCC_BASE+STM32_RCC_APBSMENR2_OFFSET)
#define STM32_RCC_CCIPR1            (STM32_RCC_BASE+STM32_RCC_CCIPR1_OFFSET)
#define STM32_RCC_CCIPR2            (STM32_RCC_BASE+STM32_RCC_CCIPR2_OFFSET)
#define STM32_RCC_CSR1              (STM32_RCC_BASE+STM32_RCC_CSR1_OFFSET)
#define STM32_RCC_CSR2              (STM32_RCC_BASE+STM32_RCC_CSR2_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Clock control register */

                                              /* Bits 0-1: Reserved */

#define RCC_CR_SYSDIV_SHIFT         (2)       /* Bit 2-4: Clock division factor for system clock */
#define RCC_CR_HSIKERDIV_SHIFT      (5)       /* Bit 5-7: HSI48 kernel clock division factor */
#define RCC_CR_HSION                (1 << 8)  /* Bit 8: Internal high speed clock enable */
#define RCC_CR_HSIKERON             (1 << 9)  /* Bit 9: Internal high speed clock enable for some IP kernels */
#define RCC_CR_HSIRDY               (1 << 10) /* Bit 10: Internal high speed clock ready flag */
#define RCC_CR_HSIDIV_SHIFT         (11)      /* Bit 11: Internal high speed clock divider */
#define RCC_CR_HSIDIV_MASK          (7 << RCC_CR_HSIDIV_SHIFT)
#define RCC_CR_HSIDIV_HSI           (0 << RCC_CR_HSIDIV_SHIFT)
#define RCC_CR_HSIDIV_HSId2         (1 << RCC_CR_HSIDIV_SHIFT)
#define RCC_CR_HSIDIV_HSId4         (2 << RCC_CR_HSIDIV_SHIFT)
#define RCC_CR_HSIDIV_HSId8         (3 << RCC_CR_HSIDIV_SHIFT)
#define RCC_CR_HSIDIV_HSId16        (4 << RCC_CR_HSIDIV_SHIFT)
#define RCC_CR_HSIDIV_HSId32        (5 << RCC_CR_HSIDIV_SHIFT)
#define RCC_CR_HSIDIV_HSId64        (6 << RCC_CR_HSIDIV_SHIFT)
#define RCC_CR_HSIDIV_HSId128       (7 << RCC_CR_HSIDIV_SHIFT)
                                              /* Bits 14-15: Reserved */
#define RCC_CR_HSEON                (1 << 16) /* Bit 16: External high speed clock enable */
#define RCC_CR_HSERDY               (1 << 17) /* Bit 17: External high speed clock ready flag */
#define RCC_CR_HSEBYP               (1 << 18) /* Bit 18: External high speed clock bypass */

#define RCC_CR_CSSON                (1 << 19) /* Bit 19: Clock security system enable */
                                              /* Bits 20-21: Reserved */
#define RCC_CR_HSIUSB48ON           (1 << 22) /* Bit 22: HSIUSB48 clock enable */
#define RCC_CR_HSIUSB48RDY          (1 << 23) /* Bit 23: HSIUSB48 clock ready flag */
                                              /* Bits 24-31: Reserved */

#define RCC_CR_RESET                0x00001540

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

#define RCC_CFGR_SW_SHIFT           (0)                         /* Bits 0-2: System clock Switch */
#define RCC_CFGR_SW_MASK            (7 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_HSI           (0 << RCC_CFGR_SW_SHIFT)    /* 000: HSI selected as system clock */
#  define RCC_CFGR_SW_HSE           (1 << RCC_CFGR_SW_SHIFT)    /* 001: HSE selected as system clock */
#  define RCC_CFGR_SW_HSIUSB48      (2 << RCC_CFGR_SW_SHIFT)    /* 010: HSIUSB48 selected as system clock */
#  define RCC_CFGR_SW_LSI           (3 << RCC_CFGR_SW_SHIFT)    /* 011: LSI selected as system clock */
#  define RCC_CFGR_SW_LSE           (4 << RCC_CFGR_SW_SHIFT)    /* 100: LSE selected as system clock */
#define RCC_CFGR_SWS_SHIFT          (3)                         /* Bits 2-3: System Clock Switch Status */
#define RCC_CFGR_SWS_MASK           (7 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_HSI          (0 << RCC_CFGR_SWS_SHIFT)   /* 000: HSI used as system clock */
#  define RCC_CFGR_SWS_HSE          (1 << RCC_CFGR_SWS_SHIFT)   /* 001: HSE used as system clock */
#  define RCC_CFGR_SWS_HSIUSB48     (2 << RCC_CFGR_SWS_SHIFT)   /* 010: HSIUSB48 used as system clock */
#  define RCC_CFGR_SWS_LSI          (3 << RCC_CFGR_SWS_SHIFT)   /* 011: LSI used as system clock */
#  define RCC_CFGR_SWS_LSE          (4 << RCC_CFGR_SWS_SHIFT)   /* 100: LSE used as system clock */
                                                                /* Bits 6-7: Reserved */
#define RCC_CFGR_HPRE_SHIFT         (8)                         /* Bits 8-11: AHB prescaler */
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

#define RCC_CFGR_PPRE_SHIFT         (12)                        /* Bits 12-14: APB Low speed prescaler (APB) */
#define RCC_CFGR_PPRE_MASK          (7 << RCC_CFGR_PPRE_SHIFT)
#  define RCC_CFGR_PPRE_HCLK        (0 << RCC_CFGR_PPRE_SHIFT)  /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE_HCLKd2      (4 << RCC_CFGR_PPRE_SHIFT)  /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE_HCLKd4      (5 << RCC_CFGR_PPRE_SHIFT)  /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE_HCLKd8      (6 << RCC_CFGR_PPRE_SHIFT)  /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE_HCLKd16     (7 << RCC_CFGR_PPRE_SHIFT)  /* 111: HCLK divided by 16 */
                                                                /* Bit 15: Reserved */
                                                                /* TODO: MCO bits */
#define RCC_CFGR_RESET              0x00000000

/* Clock Source Interrupt enable register */

#define RCC_CIER_LSIRDYF            (1 << 0)  /* Bit 0: LSI ready interrupt flag */
#define RCC_CIER_LSERDYF            (1 << 1)  /* Bit 1: LSE ready interrupt flag */
#define RCC_CIER_HSIUSB48RDYE       (1 << 2)  /* Bit 2: HSI48USB ready interrupt flag */
#define RCC_CIER_HSIRDYF            (1 << 3)  /* Bit 3: HSI ready interrupt flag */
#define RCC_CIER_HSERDYF            (1 << 5)  /* Bit 5: MSE ready interrupt flag */

/* Clock Source Interrupt Flag register */

#define RCC_CIFR_LSIRDYF            (1 << 0)  /* Bit 0: LSI ready interrupt flag */
#define RCC_CIFR_LSERDYF            (1 << 1)  /* Bit 1: LSE ready interrupt flag */
#define RCC_CIFR_HSIUSB48RDYE       (1 << 2)  /* Bit 2: HSI16 ready interrupt flag */
#define RCC_CIFR_HSIRDYF            (1 << 3)  /* Bit 3: HSI ready interrupt flag */
#define RCC_CIFR_HSERDYF            (1 << 5)  /* Bit 5: MSE ready interrupt flag */

/* Clock Source Interrupt Clear register */

#define RCC_CICR_LSIRDYF            (1 << 0)  /* Bit 0: LSI ready interrupt flag */
#define RCC_CICR_LSERDYF            (1 << 1)  /* Bit 1: LSE ready interrupt flag */
#define RCC_CICR_HSIUSB48RDYE       (1 << 2)  /* Bit 2: HSI16 ready interrupt flag */
#define RCC_CICR_HSIRDYF            (1 << 3)  /* Bit 3: HSI ready interrupt flag */
#define RCC_CICR_HSERDYF            (1 << 5)  /* Bit 5: MSE ready interrupt flag */

/* GPIO reset register */

#define RCC_IOPRSTR_IOPARST         (1 << 0)  /* Bit 0: IO port A reset */
#define RCC_IOPRSTR_IOPBRST         (1 << 1)  /* Bit 1: IO port B reset */
#define RCC_IOPRSTR_IOPCRST         (1 << 2)  /* Bit 2: IO port C reset */
#define RCC_IOPRSTR_IOPDRST         (1 << 3)  /* Bit 3: IO port D reset */
#define RCC_IOPRSTR_IOPFRST         (1 << 5)  /* Bit 5: IO port F reset */

/* AHB peripheral reset register */

#define RCC_AHBRSTR_DMA1RST         (1 << 0)  /* Bit 0: DMA 1 reset */
                                              /* Bits 1-7: Reserved */
#define RCC_AHBRSTR_MIFRST          (1 << 8)  /* Bit 8: Memory interface reset */
                                              /* Bits 9-11: Reserved */
#define RCC_AHBRSTR_CRCRST          (1 << 12) /* Bit 12: Memory interface reset */
                                              /* Bits 13-31: Reserved */

/* APB Peripheral reset register 1 */

#define RCC_APB1RSTR_TIM2RST        (1 << 0)  /* Bit 0:  Timer 2 reset */
#define RCC_APB1RSTR_TIM3RST        (1 << 1)  /* Bit 1:  Timer 3 reset */
                                              /* Bits 2-11: Reserved */
#define RCC_APB1RSTR_FDCANRST       (1 << 12) /* Bit 12: FDCAN reset */
#define RCC_APB1RSTR_USBRST         (1 << 13) /* Bit 13: USB reset */
#define RCC_APB1RSTR_SPI2RST        (1 << 14) /* Bit 14: SPI 2 reset */
                                              /* Bit 15: Reserved */
#define RCC_APB1RSTR_CRCRST         (1 << 16) /* Bit 15: CRC reset */
#define RCC_APB1RSTR_USART2RST      (1 << 17) /* Bit 17: USART 2 reset */
#define RCC_APB1RSTR_USART3RST      (1 << 18) /* Bit 18: USART 3 reset */
#define RCC_APB1RSTR_USART4RST      (1 << 19) /* Bit 19: USART 4 reset */
                                              /* Bit 20: Reserved */
#define RCC_APB1RSTR_I2C1RST        (1 << 21) /* Bit 21: I2C 1 reset */
                                              /* Bits 22-26: Reserved */
#define RCC_APB1RSTR_DBGRST         (1 << 27) /* Bit 27: Debug interface reset */
#define RCC_APB1RSTR_PWRRST         (1 << 28) /* Bit 28: Power interface reset */
                                              /* Bits 29-31: Reserved */

/* APB Peripheral reset register 2 */

#define RCC_APB2RSTR_SYSCFGRST      (1 << 0)  /* Bit 0: SYSCFG reset */
                                              /* Bits 1-10: Reserved */
#define RCC_APB2RSTR_TIM1RST        (1 << 11) /* Bit 11: TIM1 reset */
#define RCC_APB2RSTR_SPI1RST        (1 << 12) /* Bit 12: SPI1 reset */
                                              /* Bit 13: Reserved */
#define RCC_APB2RSTR_USART1RST      (1 << 14) /* Bit 14: USART1 reset */
#define RCC_APB2RSTR_TIM14RST       (1 << 14) /* Bit 14: TIM14 reset */
#define RCC_APB2RSTR_TIM15RST       (1 << 15) /* Bit 15: TIM15 reset */
#define RCC_APB2RSTR_TIM16RST       (1 << 16) /* Bit 16: TIM16 reset */
#define RCC_APB2RSTR_TIM17RST       (1 << 17) /* Bit 17: TIM17 reset */
                                              /* Bit 19: Reserved */
#define RCC_APB2RSTR_ADC1RST        (1 << 20) /* Bit 20: ADC reset */
                                              /* Bits 21-31: Reserved */

/* GPIO clock enable register */

#define RCC_IOPENR_IOPAEN           (1 << 0)  /* Bit 0: IO port A clock enable */
#define RCC_IOPENR_IOPBEN           (1 << 1)  /* Bit 1: IO port B clock enable */
#define RCC_IOPENR_IOPCEN           (1 << 2)  /* Bit 2: IO port C clock enable */
#define RCC_IOPENR_IOPDEN           (1 << 3)  /* Bit 3: IO port D clock enable */
#define RCC_IOPENR_IOPFEN           (1 << 5)  /* Bit 5: IO port F clock enable */

/* AHB Peripheral Clock enable register */

#define RCC_AHBENR_DMA1EN           (1 << 0)  /* Bit 0: DMA 1 clock enable */
                                              /* Bits 1-7: Reserved */
#define RCC_AHBENR_MIFEN            (1 << 8)  /* Bit 8: Memory interface clock enable */
                                              /* Bits 9-11: Reserved */
#define RCC_AHBENR_CRCEN            (1 << 12) /* Bit 12: Memory interface clock enable */
                                              /* Bits 13-31: Reserved */

/* APB Peripheral Clock enable register 1 */

#define RCC_APB1ENR_TIM2EN           (1 << 0)  /* Bit 0:  Timer 2 clock enable */
#define RCC_APB1ENR_TIM3EN           (1 << 1)  /* Bit 1:  Timer 3 clock enable */
                                               /* Bits 2-11: Reserved */
#define RCC_APB1ENR_FDCANEN          (1 << 12) /* Bit 12: FDCAN clock enable */
#define RCC_APB1ENR_USBEN            (1 << 13) /* Bit 13: USB clock enable */
#define RCC_APB1ENR_SPI2EN           (1 << 14) /* Bit 14: SPI 2 clock enable */
                                               /* Bit 15: Reserved */
#define RCC_APB1ENR_CRCEN            (1 << 16) /* Bit 16: CRC clock enable */
#define RCC_APB1ENR_USART2EN         (1 << 17) /* Bit 17: USART 2 clock enable */
#define RCC_APB1ENR_USART3EN         (1 << 18) /* Bit 18: USART 3 clock enable */
#define RCC_APB1ENR_USART4EN         (1 << 19) /* Bit 19: USART 4 clock enable */
                                               /* Bit 20: Reserved */
#define RCC_APB1ENR_I2C1EN           (1 << 21) /* Bit 21: I2C 1 clock enable */
                                               /* Bits 22-26: Reserved */
#define RCC_APB1ENR_DBGEN            (1 << 27) /* Bit 27: Debug interface clock enable */
#define RCC_APB1ENR_PWREN            (1 << 28) /* Bit 28: Power interface clock enable */
                                               /* Bits 29-31: Reserved */

/* APB Peripheral Clock enable register 2 */

#define RCC_APB2ENR_SYSCFGEN        (1 << 0)  /* Bit 0: SYSCFG clock enable */
                                              /* Bits 1-10: Reserved */
#define RCC_APB2ENR_TIM1EN          (1 << 11) /* Bit 11: TIM1 clock enable */
#define RCC_APB2ENR_SPI1EN          (1 << 12) /* Bit 12: SPI1 clock enable */
                                              /* Bit 13: Reserved */
#define RCC_APB2ENR_USART1EN        (1 << 14) /* Bit 14: USART1 clock enable */
#define RCC_APB2ENR_TIM14EN         (1 << 14) /* Bit 14: TIM14 clock enable */
#define RCC_APB2ENR_TIM15EN         (1 << 15) /* Bit 15: TIM15 clock enable */
#define RCC_APB2ENR_TIM16EN         (1 << 16) /* Bit 16: TIM16 clock enable */
#define RCC_APB2ENR_TIM17EN         (1 << 17) /* Bit 17: TIM17 clock enable */
                                              /* Bit 19: Reserved */
#define RCC_APB2ENR_ADC1EN          (1 << 20) /* Bit 20: ADC clock enable */
                                              /* Bits 21-31: Reserved */

/* TODO: GPIO clock enable in Sleep mode register */

/* TODO: AHB peripheral clock enable in Sleep mode register */

/* TODO: APB1 peripheral clock enable in Sleep mode register */

/* RCC peripherals independent clock configuration register 1 */

#define RCC_CCIPR1_USART1SEL_SHIFT    (0) /* Bits 0-1: USART1 clock source selection */
#  define RCC_CCIPR1_USART1SEL_PCLK   (0 << RCC_CCIPR1_USART1SEL_SHIFT)
#  define RCC_CCIPR1_USART1SEL_SYSCLK (1 << RCC_CCIPR1_USART1SEL_SHIFT)
#  define RCC_CCIPR1_USART1SEL_HSIKER (2 << RCC_CCIPR1_USART1SEL_SHIFT)
#  define RCC_CCIPR1_USART1SEL_LSE    (3 << RCC_CCIPR1_USART1SEL_SHIFT)
#define RCC_CCIPR1_FDCAN1SEL_SHIFT    (8) /* Bits 8-9: FDCAN1 clock source selection */
#  define RCC_CCIPR1_FDCAN1SEL_PCLK   (0 << RCC_CCIPR1_FDCAN1SEL_SHIFT)
#  define RCC_CCIPR1_FDCAN1SEL_SYSCLK (1 << RCC_CCIPR1_FDCAN1SEL_SHIFT)
#  define RCC_CCIPR1_FDCAN1SEL_HSIKER (2 << RCC_CCIPR1_FDCAN1SEL_SHIFT)
#define RCC_CCIPR1_I2C1SEL_SHIFT      (12) /* Bits 12-13: I2C1 clock source selection */
#  define RCC_CCIPR1_I2C1SEL_PCLK     (0 << RCC_CCIPR1_I2C1SEL_SHIFT)
#  define RCC_CCIPR1_I2C1SEL_SYSCLK   (1 << RCC_CCIPR1_I2C1SEL_SHIFT)
#  define RCC_CCIPR1_I2C1SEL_HSIKER   (2 << RCC_CCIPR1_I2C1SEL_SHIFT)
#define RCC_CCIPR1_I2S1SEL_SHIFT      (14) /* Bits 14-15: I2S1 clock source selection */
#  define RCC_CCIPR1_I2S1SEL_PCLK     (0 << RCC_CCIPR1_I2S1SEL_SHIFT)
#  define RCC_CCIPR1_I2S1SEL_HSIKER   (2 << RCC_CCIPR1_I2S1SEL_SHIFT)
#  define RCC_CCIPR1_I2S1SEL_I2S      (3 << RCC_CCIPR1_I2S1SEL_SHIFT)
#define RCC_CCIPR1_ADC1SEL_SHIFT      (30) /* Bits 30-31: ADC1 clock source selection */
#  define RCC_CCIPR1_ADC1SEL_SYSCLK   (0 << RCC_CCIPR1_ADC1SEL_SHIFT)
#  define RCC_CCIPR1_ADC1SEL_HSIKER   (2 << RCC_CCIPR1_ADC1SEL_SHIFT)

/* RCC peripherals independent clock configuration register 2 */

#define RCC_CCIPR2_USBSEL_SHIFT       (12) /* Bit 12: SB clock source selection */
#define RCC_CCIPR2_USBSEL_HSIUSB48    (0 << RCC_CCIPR2_USBSEL_SHIFT)
#define RCC_CCIPR2_USBSEL_HSE         (1 << RCC_CCIPR2_USBSEL_SHIFT)

/* Clock configuration register 1 */

#define RCC_CSR1_LSEON              (1 << 0)  /* Bit 0: LSE enable */
#define RCC_CSR1_LSERDY             (1 << 1)  /* Bit 1: LSE ready */
#define RCC_CSR1_LSEBPY             (1 << 2)  /* Bit 2: LSE bypass */
#define RCC_CSR1_LSEDRV_SHIFT       (1 << 3)  /* Bit 3: LSE driving capability */
                                              /* Bit 4: Reserved */
#define RCC_CSR1_CSSLSEON           (1 << 5)  /* Bit 5: CSS on LSE enable */
#define RCC_CSR1_CSSLSED            (1 << 6)  /* Bit 6: CSS on LSE failure detection flag */
                                              /* Bit 7: Reserved */
#define RCC_CSR1_RTCSEL_SHIFT       (8)       /* Bits 8-9: RTC clock source selection */
#define RCC_CSR1_RTCSEL_MASK        (3 << RCC_CSR1_RTCSEL_SHIFT)
#  define RCC_CSR1_RTCSEL_NOCLK     (0 << RCC_CSR1_RTCSEL_SHIFT)
#  define RCC_CSR1_RTCSEL_LSE       (1 << RCC_CSR1_RTCSEL_SHIFT)
#  define RCC_CSR1_RTCSEL_LSI       (2 << RCC_CSR1_RTCSEL_SHIFT)
#  define RCC_CSR1_RTCSEL_HSEd32    (3 << RCC_CSR1_RTCSEL_SHIFT)
                                              /* Bits 10-14: Reserved */
#define RCC_CSR1_RTCEN              (1 << 15) /* Bit 15: RTC clock enable */
#define RCC_CSR1_RTCRST             (1 << 16) /* Bit 16: RTC software reset */
                                              /* Bits 17-23: Reserved */
#define RCC_CSR1_LSCOEN             (1 << 24) /* Bit 24: Low-speed clock output (LSCO) enable */
#define RCC_CSR1_LSCOSEL            (1 << 25) /* Bit 25: Low-speed clock output selection */
                                              /* Bits 26-31: Reserved */

/* Clock configuration register 2 */

#define RCC_CSR2_LSION              (1 << 0)  /* Bit 0: LSI oscillator enable */
#define RCC_CSR2_LSIRDY             (1 << 1)  /* Bit 1: LSI oscillator ready */
                                              /* Bits 2-22: Reserved */
#define RCC_CSR2_RMVF               (1 << 23) /* Bit 23: Remove reset flag */
                                              /* Bit 24: Reserved */
#define RCC_CSR2_OBLRSTF            (1 << 25) /* Bit 25: Options bytes loading reset flag */
#define RCC_CSR2_PINRSTF            (1 << 26) /* Bit 26: PIN reset flag */
#define RCC_CSR2_PORRSTF            (1 << 27) /* Bit 27: POR/PDR reset flag */
#define RCC_CSR2_SFTRSTF            (1 << 28) /* Bit 28: software reset flag */
#define RCC_CSR2_IWDGRSTF           (1 << 29) /* Bit 29: IWDG reset flag */
#define RCC_CSR2_WWDGRSTF           (1 << 30) /* Bit 30: WWDG reset flag */
#define RCC_CSR2_LPWRRSTF           (1 << 31) /* Bit 31: Low-power reset flag */

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32C0_RCC_H */
