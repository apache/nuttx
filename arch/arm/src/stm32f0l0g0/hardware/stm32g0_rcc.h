/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32g0_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_RCC_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_RCC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_RCC_CR_OFFSET             0x0000  /* Clock control register */
#define STM32_RCC_ICSCR_OFFSET          0x0004  /* Internal clock sources calibration register */
#define STM32_RCC_CFGR_OFFSET           0x0008  /* Clock configuration register */
#define STM32_RCC_PLLCFG_OFFSET         0x000C  /* PLL clock configuration register */
#define STM32_RCC_CIER_OFFSET           0x0018  /* Clock Source Interrupt enable register */
#define STM32_RCC_CIFR_OFFSET           0x001c  /* Clock Source Interrupt Flag register */
#define STM32_RCC_CICR_OFFSET           0x0020  /* Clock Source Interrupt Clear register */
#define STM32_RCC_IOPRSTR_OFFSET        0x0024  /* GPIO reset register */
#define STM32_RCC_AHBRSTR_OFFSET        0x0028  /* AHB peripheral reset register */
#define STM32_RCC_APB1RSTR_OFFSET       0x002C  /* APB1 Peripheral reset register */
#define STM32_RCC_APB2RSTR_OFFSET       0x0030  /* APB2 Peripheral reset register */
#define STM32_RCC_IOPENR_OFFSET         0x0034  /* GPIO clock enable register */
#define STM32_RCC_AHBENR_OFFSET         0x0038  /* AHB Peripheral Clock enable register */
#define STM32_RCC_APB1ENR_OFFSET        0x003C  /* APB1 Peripheral Clock enable register */
#define STM32_RCC_APB2ENR_OFFSET        0x0040  /* APB2 Peripheral Clock enable register */
#define STM32_RCC_IOPSMEN_OFFSET        0x0044  /* GPIO clock enable in Sleep mode register */
#define STM32_RCC_AHBSMENR_OFFSET       0x0048  /* AHB peripheral clock enable in Sleep mode register */
#define STM32_RCC_APB1SMENR_OFFSET      0x004C  /* APB1 peripheral clock enable in Sleep mode register */
#define STM32_RCC_APB2SMENR_OFFSET      0x0050  /* APB2 peripheral clock enable in Sleep mode register */
#define STM32_RCC_CCIPR_OFFSET          0x0054  /* Clock configuration register */
#define STM32_RCC_BDCR_OFFSET           0x005c  /* RTC domain control register */
#define STM32_RCC_CSR_OFFSET            0x0060  /* Control/status register */

/* Register Addresses *******************************************************/

#define STM32_RCC_CR                (STM32_RCC_BASE+STM32_RCC_CR_OFFSET)
#define STM32_RCC_ICSCR             (STM32_RCC_BASE+STM32_RCC_ICSCR_OFFSET)
#define STM32_RCC_CFGR              (STM32_RCC_BASE+STM32_RCC_CFGR_OFFSET)
#define STM32_RCC_PLLCFG            (STM32_RCC_BASE+STM32_RCC_PLLCFG_OFFSET)
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
#define STM32_RCC_APB1SMENR         (STM32_RCC_BASE+STM32_RCC_APB1SMENR_OFFSET)
#define STM32_RCC_APB2SMENR         (STM32_RCC_BASE+STM32_RCC_APB2SMENR_OFFSET)
#define STM32_RCC_CCIPR             (STM32_RCC_BASE+STM32_RCC_CCIPR_OFFSET)
#define STM32_RCC_BDCR              (STM32_RCC_BASE+STM32_RCC_BDCR_OFFSET)
#define STM32_RCC_CSR               (STM32_RCC_BASE+STM32_RCC_CSR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Clock control register */

                                              /* Bits 0-7: Reserved */
#define RCC_CR_HSION                (1 << 8)  /* Bit 8: Internal high speed clock enable */
#define RCC_CR_HSIKERON             (1 << 9)  /* Bit 9: Internal high speed clock enable for some IP kernels */
#define RCC_CR_HSIRDY               (1 << 10) /* Bit 10: Internal high speed clock ready flag */
#define RCC_CR_HSIDIV_SHIFT         (11)      /* Bit 11: Internal high speed clock divider */
#define RCC_CR_HSIDIV_MASK          (7 << RCC_CR_HSIDIV_SHIFT)
                                              /* Bits 14-15: Reserved */
#define RCC_CR_HSEON                (1 << 16) /* Bit 16: External high speed clock enable */
#define RCC_CR_HSERDY               (1 << 17) /* Bit 17: External high speed clock ready flag */
#define RCC_CR_HSEBYP               (1 << 18) /* Bit 18: External high speed clock bypass */

#define RCC_CR_CSSON                (1 << 19) /* Bit 19: Clock security system enable */
                                              /* Bits 20-23: Reserved */
#define RCC_CR_PLLON                (1 << 24) /* Bit 24: PLL enable */
#define RCC_CR_PLLRDY               (1 << 25) /* Bit 25: PLL clock ready flag */
                                              /* Bits 26-27: Reserved */

#define RCC_CR_RESET                 0x00000500

/* Internal clock sources calibration register */

#define RCC_ICSCR_HSICAL_SHIFT      (0)       /* Bits 0-7:  Internal high speed clock calibration */
#define RCC_ICSCR_HSICAL_MASK       (0xff << RCC_ICSCR_HSICAL_SHIFT)
#define RCC_ICSCR_HSITRIM_SHIFT     (8)       /* Bits 8-14:  High speed internal clock trimming */
#define RCC_ICSCR_HSITRIM_MASK      (0x7f << RCC_ICSCR_HSITRIM_SHIFT)

/* Clock configuration register */

#define RCC_CFGR_SW_SHIFT           (0)                      /* Bits 0-2: System clock Switch */
#define RCC_CFGR_SW_MASK            (7 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_HSI           (0 << RCC_CFGR_SW_SHIFT) /* 000: HSI selected as system clock */
#  define RCC_CFGR_SW_HSE           (1 << RCC_CFGR_SW_SHIFT) /* 001: HSE selected as system clock */
#  define RCC_CFGR_SW_PLL           (2 << RCC_CFGR_SW_SHIFT) /* 010: PLL selected as system clock */
#  define RCC_CFGR_SW_LSI           (3 << RCC_CFGR_SW_SHIFT) /* 011: LSI selected as system clock */
#  define RCC_CFGR_SW_LSE           (4 << RCC_CFGR_SW_SHIFT) /* 100: LSE selected as system clock */
#define RCC_CFGR_SWS_SHIFT          (3)                      /* Bits 3-5: System Clock Switch Status */
#define RCC_CFGR_SWS_MASK           (3 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_HSI          (0 << RCC_CFGR_SWS_SHIFT) /* 000: HSI oscillator used as system clock */
#  define RCC_CFGR_SWS_HSE          (1 << RCC_CFGR_SWS_SHIFT) /* 001: HSE oscillator used as system clock */
#  define RCC_CFGR_SWS_PLL          (2 << RCC_CFGR_SWS_SHIFT) /* 010: PLL oscillator used as system clock */
#  define RCC_CFGR_SWS_LSI          (3 << RCC_CFGR_SWS_SHIFT) /* 011: LSI used as system clock */
#  define RCC_CFGR_SWS_LSE          (4 << RCC_CFGR_SWS_SHIFT) /* 100: LSE used as system clock */
#define RCC_CFGR_HPRE_SHIFT         (8)                       /* Bits 8-11: AHB prescaler */
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
#define RCC_CFGR_PPRE1_SHIFT        (12)                        /* Bits 12-14: APB Low speed prescaler (APB1) */
#define RCC_CFGR_PPRE1_MASK         (7 << RCC_CFGR_PPRE1_SHIFT)
#  define RCC_CFGR_PPRE1_HCLK       (0 << RCC_CFGR_PPRE1_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE1_HCLKd2     (4 << RCC_CFGR_PPRE1_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE1_HCLKd4     (5 << RCC_CFGR_PPRE1_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE1_HCLKd8     (6 << RCC_CFGR_PPRE1_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE1_HCLKd16    (7 << RCC_CFGR_PPRE1_SHIFT) /* 111: HCLK divided by 16 */
                                                                /* Bits 15-23: Reserved */
#define RCC_CFGR_MCOSEL_SHIFT       (24)                        /* Bits 24-26: Microcontroller clock output selection */
#define RCC_CFGR_MCOSEL_MASK        (7 << RCC_CFGR_MCOSEL_SHIFT)
                                                                /* Bit 27: Reserved */
#define RCC_CFGR_MCOPRE_SHIFT       (28)                        /* Bits 28-30: Microcontroller clock output selection */
#define RCC_CFGR_MCOPRE_MASK        (7 << RCC_CFGR_MCOPRE_SHIFT)

#define RCC_CFGR_RESET              0x00000000

/* PLL clock configuration register */

#define RCC_PLLCFG_PLLSRC_SHIFT    (0)                              /* Bits 0-1: PLL entry clock source */
#define RCC_PLLCFG_PLLSRC_MASK     (3 << RCC_PLLCFG_PLLSRC_SHIFT)
#  define RCC_PLLCFG_PLLSRC_NOCLK  (0 << RCC_PLLCFG_PLLSRC_SHIFT)
#  define RCC_PLLCFG_PLLSRC_HSI    (2 << RCC_PLLCFG_PLLSRC_SHIFT)
#  define RCC_PLLCFG_PLLSRC_HSE    (3 << RCC_PLLCFG_PLLSRC_SHIFT)
                                                                    /* Bits 2-3: Reserved */
#define RCC_PLLCFG_PLLM_SHIFT      (4)                              /* Bits 4-6: Division factor M of the PLL input clock divider */
#define RCC_PLLCFG_PLLM_MASK       (7 << RCC_PLLCFG_PLLM_SHIFT)
#define RCC_PLLCFG_PLLM(n)         ((n-1) << RCC_PLLCFG_PLLM_SHIFT)
                                                                    /* Bit 7: Reserved */
#define RCC_PLLCFG_PLLN_SHIFT      (8)                              /* Bits 8-14: PLL frequency multiplication factor N */
#define RCC_PLLCFG_PLLN_MASK       (0x7f << RCC_PLLCFG_PLLN_SHIFT)
#define RCC_PLLCFG_PLLN(n)         ((n) << RCC_PLLCFG_PLLN_SHIFT)
                                                                    /* Bit 15: Reserved */
#define RCC_PLLCFG_PLLPEN          (1 << 16)                        /* Bit 16: PLLPCLK clock output enable */
#define RCC_PLLCFG_PLLP_SHIFT      (17)                             /* Bits 17-21: PLL VCO division factor P for PLLPCLK clock output */
#define RCC_PLLCFG_PLLP_MASK       (0x1f << RCC_PLLCFG_PLLP_SHIFT)
#define RCC_PLLCFG_PLLP(n)         ((n-1) << RCC_PLLCFG_PLLP_SHIFT) /* n=2,...,32 */
                                                                    /* Bits 22-23: Reserved */
#define RCC_PLLCFG_PLLQEN          (1 << 24)                        /* Bit 24: PLLQCLK clock output enable */
#define RCC_PLLCFG_PLLQ_SHIFT      (25)                             /* Bits 25-27: Division factor Q of the PLL input clock divider */
#define RCC_PLLCFG_PLLQ_MASK       (7 << RCC_PLLCFG_PLLQ_SHIFT)
#define RCC_PLLCFG_PLLQ(n)         ((n-1) << RCC_PLLCFG_PLLQ_SHIFT)
#define RCC_PLLCFG_PLLREN          (1 << 28)                        /* Bit 28: PLLRCLK clock output enable */
#define RCC_PLLCFG_PLLR_SHIFT      (29)                             /* Bits 29-31: Division factor R of the PLL input clock divider */
#define RCC_PLLCFG_PLLR_MASK       (7 << RCC_PLLCFG_PLLR_SHIFT)
#define RCC_PLLCFG_PLLR(n)         ((n-1) << RCC_PLLCFG_PLLR_SHIFT)

#define RCC_PLLCFGR_RESET           0x00001000

/* TODO: Clock Source Interrupt enable register */

/* TODO: Clock Source Interrupt Flag register */

/* TODO: Clock Source Interrupt Clear register */

/* GPIO reset register */

#define RCC_IOPRSTR_IOPARST         (1 << 0)  /* Bit 0: IO port A reset */
#define RCC_IOPRSTR_IOPBRST         (1 << 1)  /* Bit 1: IO port B reset */
#define RCC_IOPRSTR_IOPCRST         (1 << 2)  /* Bit 2: IO port C reset */
#define RCC_IOPRSTR_IOPDRST         (1 << 3)  /* Bit 3: IO port D reset */
                                              /* Bits 4: Reserved */
#define RCC_IOPRSTR_IOPFRST         (1 << 5)  /* Bit 5: IO port F reset */

/* AHB peripheral reset register */

#define RCC_AHBRSTR_DMA1RST         (1 << 0)  /* Bit 0: DMA 1 reset */
                                              /* Bits 1-7: Reserved */
#define RCC_AHBRSTR_MIFRST          (1 << 8)  /* Bit 8: Memory interface reset */
                                              /* Bits 9-11: Reserved */
#define RCC_AHBRSTR_CRCRST          (1 << 12) /* Bit 12: Memory interface reset */
                                              /* Bits 13-15: Reserved */
#define RCC_AHBRSTR_AESRST          (1 << 16) /* Bit 16: Touch sensing reset */
                                              /* Bit 17: Reserved */
#define RCC_AHBRSTR_RNGRST          (1 << 18) /* Bit 18: Random number generator module reset */
                                              /* Bits 19-31: Reserved */

/* APB1 Peripheral reset register */

#define RCC_APB1RSTR_TIM2RST        (1 << 0)  /* Bit 0:  Timer 2 reset */
#define RCC_APB1RSTR_TIM3RST        (1 << 1)  /* Bit 1:  Timer 3 reset */
                                              /* Bits 2-3: Reserved */
#define RCC_APB1RSTR_TIM6RST        (1 << 4)  /* Bit 4:  Timer 6 reset */
#define RCC_APB1RSTR_TIM7RST        (1 << 5)  /* Bit 5:  Timer 7 reset */
                                              /* Bits 6-13: Reserved */
#define RCC_APB1RSTR_SPI2RST        (1 << 14) /* Bit 14: SPI 2 reset */
                                              /* Bits 15-16: Reserved */
#define RCC_APB1RSTR_USART2RST      (1 << 17) /* Bit 17: USART 2 reset */
#define RCC_APB1RSTR_USART3RST      (1 << 18) /* Bit 18: USART 3 reset */
#define RCC_APB1RSTR_USART4RST      (1 << 19) /* Bit 19: USART 4 reset */
#define RCC_APB1RSTR_LPUSART1RST    (1 << 20) /* Bit 20: Low-power USART 1 reset */
#define RCC_APB1RSTR_I2C1RST        (1 << 21) /* Bit 21: I2C 1 reset */
#define RCC_APB1RSTR_I2C2RST        (1 << 22) /* Bit 22: I2C 2 reset */
                                              /* Bit 23: Reserved */
#define RCC_APB1RSTR_CECRST         (1 << 24) /* Bit 24: HDMI CEC reset */
#define RCC_APB1RSTR_UCPD1RST       (1 << 25) /* Bit 25: UCPD1 reset */
#define RCC_APB1RSTR_UCPD2RST       (1 << 26) /* Bit 26: UCPD2 reset */
#define RCC_APB1RSTR_DBGRST         (1 << 27) /* Bit 27: DBG reset */
#define RCC_APB1RSTR_PWRRST         (1 << 28) /* Bit 28: PWR reset */
#define RCC_APB1RSTR_DAC1RST        (1 << 29) /* Bit 29: DAC1 reset */
#define RCC_APB1RSTR_LPTIM2RST      (1 << 30) /* Bit 30: LPTIM2 reset */
#define RCC_APB1RSTR_LPTIM1RST      (1 << 31) /* Bit 31: LPTIM1 reset */

/* APB2 Peripheral reset register */

#define RCC_APB2RSTR_SYSCFGRST      (1 << 0)  /* Bit 0:  System configuration controller reset */
                                              /* Bits 1-10: Reserved */
#define RCC_APB2RSTR_TIM1RST        (1 << 11) /* Bit 11: TIM1 timer reset */
#define RCC_APB2RSTR_SPI1RST        (1 << 12) /* Bit 12: SPI 1 reset */
                                              /* Bit 13: Reserved */
#define RCC_APB2RSTR_USART1RST      (1 << 14) /* Bit 14: USART1 reset */
#define RCC_APB2RSTR_TIM14RST       (1 << 15) /* Bit 15: TIM14 timer reset */
#define RCC_APB2RSTR_TIM15RST       (1 << 16) /* Bit 16: TIM15 timer reset */
#define RCC_APB2RSTR_TIM16RST       (1 << 17) /* Bit 17: TIM16 timer reset */
#define RCC_APB2RSTR_TIM17RST       (1 << 18) /* Bit 18: TIM17 timer reset */
                                              /* Bit 19: Reserved */
#define RCC_APB2RSTR_ADC1RST        (1 << 20) /* Bit 20: ADC1 timer reset */

/* GPIO clock enable register */

#define RCC_IOPENR_IOPAEN           (1 << 0)  /* Bit 0: IO port A enable */
#define RCC_IOPENR_IOPBEN           (1 << 1)  /* Bit 1: IO port B enable */
#define RCC_IOPENR_IOPCEN           (1 << 2)  /* Bit 2: IO port C enable */
#define RCC_IOPENR_IOPDEN           (1 << 3)  /* Bit 3: IO port D enable */
                                              /* Bits 4: Reserved */
#define RCC_IOPENR_IOPFEN           (1 << 5)  /* Bit 5: IO port F enable */

/* AHB Peripheral Clock enable register */

#define RCC_AHBENR_DMA1EN           (1 << 0)  /* Bit 0: DMA 1 enable */
                                              /* Bits 1-7: Reserved */
#define RCC_AHBENR_MIFEN            (1 << 8)  /* Bit 8: Memory interface enable */
                                              /* Bits 9-11: Reserved */
#define RCC_AHBENR_CRCEN            (1 << 12) /* Bit 12: Memory interface enable */
                                              /* Bits 13-15: Reserved */
#define RCC_AHBENR_AESEN            (1 << 16) /* Bit 16: Touch sensing enable */
                                              /* Bit 17: Reserved */
#define RCC_AHBENR_RNGEN            (1 << 18) /* Bit 18: Random number generator module enable */
                                              /* Bits 19-31: Reserved */

/* APB1 Peripheral Clock enable register */

#define RCC_APB1ENR_TIM2EN          (1 << 0)  /* Bit 0:  Timer 2 enable */
#define RCC_APB1ENR_TIM3EN          (1 << 1)  /* Bit 1:  Timer 3 enable */
                                              /* Bits 2-3: Reserved */
#define RCC_APB1ENR_TIM6EN          (1 << 4)  /* Bit 4:  Timer 6 enable */
#define RCC_APB1ENR_TIM7EN          (1 << 5)  /* Bit 5:  Timer 7 enable */
                                              /* Bits 6-13: Reserved */
#define RCC_APB1ENR_SPI2EN          (1 << 14) /* Bit 14: SPI 2 enable */
                                              /* Bits 15-16: Reserved */
#define RCC_APB1ENR_USART2EN        (1 << 17) /* Bit 17: USART 2 enable */
#define RCC_APB1ENR_USART3EN        (1 << 18) /* Bit 18: USART 3 enable */
#define RCC_APB1ENR_USART4EN        (1 << 19) /* Bit 19: USART 4 enable */
#define RCC_APB1ENR_LPUSART1EN      (1 << 20) /* Bit 20: Low-power USART 1 enable */
#define RCC_APB1ENR_I2C1EN          (1 << 21) /* Bit 21: I2C 1 enable */
#define RCC_APB1ENR_I2C2EN          (1 << 22) /* Bit 22: I2C 2 enable */
                                              /* Bit 23: Reserved */
#define RCC_APB1ENR_CECEN           (1 << 24) /* Bit 24: HDMI CEC enable */
#define RCC_APB1ENR_UCPD1EN         (1 << 25) /* Bit 25: UCPD1 enable */
#define RCC_APB1ENR_UCPD2EN         (1 << 26) /* Bit 26: UCPD2 enable */
#define RCC_APB1ENR_DBGEN           (1 << 27) /* Bit 27: DBG enable */
#define RCC_APB1ENR_PWREN           (1 << 28) /* Bit 28: PWR enable */
#define RCC_APB1ENR_DAC1EN          (1 << 29) /* Bit 29: DAC1 enable */
#define RCC_APB1ENR_LPTIM2EN        (1 << 30) /* Bit 30: LPTIM2 enable */
#define RCC_APB1ENR_LPTIM1EN        (1 << 31) /* Bit 31: LPTIM1 enable */

/* APB2 Peripheral Clock enable register */

#define RCC_APB2ENR_SYSCFGEN        (1 << 0)  /* Bit 0:  System configuration controller enable */
                                              /* Bits 1-10: Reserved */
#define RCC_APB2ENR_TIM1EN          (1 << 11) /* Bit 11: TIM1 timer enable */
#define RCC_APB2ENR_SPI1EN          (1 << 12) /* Bit 12: SPI 1 enable */
                                              /* Bit 13: Reserved */
#define RCC_APB2ENR_USART1EN        (1 << 14) /* Bit 14: USART1 enable */
#define RCC_APB2ENR_TIM14EN         (1 << 15) /* Bit 15: TIM14 timer enable */
#define RCC_APB2ENR_TIM15EN         (1 << 16) /* Bit 16: TIM15 timer enable */
#define RCC_APB2ENR_TIM16EN         (1 << 17) /* Bit 17: TIM16 timer enable */
#define RCC_APB2ENR_TIM17EN         (1 << 18) /* Bit 18: TIM17 timer enable */
                                              /* Bit 19: Reserved */
#define RCC_APB2ENR_ADC1EN          (1 << 20) /* Bit 20: ADC1 timer enable */

/* GPIO clock enable in Sleep mode register */

/* AHB peripheral clock enable in Sleep mode register */

#define RCC_AHBSMENR_DMA1SMEN       (1 << 0)  /* Bit 0: DMA 1 enable in Sleep mode */
                                              /* Bits 1-7: Reserved */
#define RCC_AHBSMENR_MIFSMEN        (1 << 8)  /* Bit 8: Memory interface enable in Sleep mode */
                                              /* Bits 9-11: Reserved */
#define RCC_AHBSMENR_CRCSMEN        (1 << 12) /* Bit 12: Memory interface enable in Sleep mode */
                                              /* Bits 13-15: Reserved */
#define RCC_AHBSMENR_AESSMEN        (1 << 16) /* Bit 16: Touch sensing enable in Sleep mode */
                                              /* Bit 17: Reserved */
#define RCC_AHBSMENR_RNGSMEN        (1 << 18) /* Bit 18: Random number generator module enable in Sleep mode */
                                              /* Bits 19-31: Reserved */

/* TODO: APB1 peripheral clock enable in Sleep mode register */

/* TODO: APB2 peripheral clock enable in Sleep mode register */

/* TODO: Clock configuration register */

/* TODO: RTC domain control register */

/* Control/status register */

#define RCC_CSR_LSION                   (1 << 0)  /* Bit 0: LSI enable */
#define RCC_CSR_LSIRDY                  (1 << 1)  /* Bit 1: ready */
                                                  /* Bits 2-22: Reserved */
#define RCC_CSR_RMVF                    (1 << 23) /* Bit 23: Remove reset flag */
                                                  /* Bit 24: Reserved */
#define RCC_CSR_OBLRSTF                 (1 << 25) /* Bit 25: Options bytes loading reset flag */
#define RCC_CSR_PINRSTF                 (1 << 26) /* Bit 26: PIN reset flag */
#define RCC_CSR_PORRSTF                 (1 << 27) /* Bit 27: POR/PDR reset flag */
#define RCC_CSR_SFTRSTF                 (1 << 28) /* Bit 28: software reset flag */
#define RCC_CSR_IWDGRSTF                (1 << 29) /* Bit 29: IWDG reset flag */
#define RCC_CSR_WWDGRSTF                (1 << 30) /* Bit 30: WWDG reset flag */
#define RCC_CSR_LPWRRSTF                (1 << 31) /* Bit 31: Low-power reset flag */

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_RCC_H */
