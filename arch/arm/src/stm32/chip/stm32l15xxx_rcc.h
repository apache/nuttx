/************************************************************************************
 * arch/arm/src/stm32/chip/stm32l15xx_rcc.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32L15XXX_RCC_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32L15XXX_RCC_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_RCC_CR_OFFSET         0x0000  /* Clock control register */
#define STM32_RCC_ICSCR_OFFSET      0x0004  /* Internal clock sources calibration register */
#define STM32_RCC_CFGR_OFFSET       0x0008  /* Clock configuration register */
#define STM32_RCC_CIR_OFFSET        0x000c  /* Clock interrupt register */
#define STM32_RCC_AHBRSTR_OFFSET    0x0010  /* AHB Peripheral reset register */
#define STM32_RCC_APB2RSTR_OFFSET   0x0014  /* APB2 Peripheral reset register */
#define STM32_RCC_APB1RSTR_OFFSET   0x0018  /* APB1 Peripheral reset register */
#define STM32_RCC_AHBENR_OFFSET     0x001c  /* AHB Peripheral Clock enable register */
#define STM32_RCC_APB2ENR_OFFSET    0x0020  /* APB2 Peripheral Clock enable register */
#define STM32_RCC_APB1ENR_OFFSET    0x0024  /* APB1 Peripheral Clock enable register */
#define STM32_RCC_AHBLPENR_OFFSET   0x0028  /* AHB peripheral clock enable in low power mode register */
#define STM32_RCC_APB2LPENR_OFFSET  0x002c  /* APB2 peripheral clock enable in low power mode register */
#define STM32_RCC_APB1LPENR_OFFSET  0x0030  /* APB1 peripheral clock enable in low power mode register */
#define STM32_RCC_CSR_OFFSET        0x0034  /* Control/status register */

/* Register Addresses ***************************************************************/

#define STM32_RCC_CR                (STM32_RCC_BASE+STM32_RCC_CR_OFFSET)
#define STM32_RCC_ICSCR             (STM32_RCC_BASE+STM32_RCC_ICSCR_OFFSET)
#define STM32_RCC_CFGR              (STM32_RCC_BASE+STM32_RCC_CFGR_OFFSET)
#define STM32_RCC_CIR               (STM32_RCC_BASE+STM32_RCC_CIR_OFFSET)
#define STM32_RCC_AHBRSTR           (STM32_RCC_BASE+STM32_RCC_AHBRSTR_OFFSET)
#define STM32_RCC_APB2RSTR          (STM32_RCC_BASE+STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_APB1RSTR          (STM32_RCC_BASE+STM32_RCC_APB1RSTR_OFFSET)
#define STM32_RCC_AHBENR            (STM32_RCC_BASE+STM32_RCC_AHBENR_OFFSET)
#define STM32_RCC_APB2ENR           (STM32_RCC_BASE+STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_APB1ENR           (STM32_RCC_BASE+STM32_RCC_APB1ENR_OFFSET)
#define STM32_RCC_AHBLPENR          (STM32_RCC_BASE+STM32_RCC_AHBLPENR_OFFSET)
#define STM32_RCC_APB2LPENR         (STM32_RCC_BASE+STM32_RCC_APB2LPENR_OFFSET)
#define STM32_RCC_APB1LPENR         (STM32_RCC_BASE+STM32_RCC_APB1LPENR_OFFSET)
#define STM32_RCC_CSR               (STM32_RCC_BASE+STM32_RCC_CSR_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* Clock control register */

#define RCC_CR_HSION                (1 << 0)  /* Bit 0: Internal high speed clock enable */
#define RCC_CR_HSIRDY               (1 << 1)  /* Bit 1: Internal high speed clock ready flag */
                                              /* Bits 2-7: Reserved */
#define RCC_CR_MSION                (1 << 8)  /* Bit 8: MSI clock enable */
#define RCC_CR_MSIRDY               (1 << 9)  /* Bit 9: MSI clock ready flag */
                                              /* Bits 10-15: Reserved */
#define RCC_CR_HSEON                (1 << 16) /* Bit 16: External high speed clock enable */
#define RCC_CR_HSERDY               (1 << 17) /* Bit 17: External high speed clock ready flag */
#define RCC_CR_HSEBYP               (1 << 18) /* Bit 18: External high speed clock bypass */
                                              /* Bits 19-23: Reserved */
#define RCC_CR_PLLON                (1 << 24) /* Bit 24: PLL enable */
#define RCC_CR_PLLRDY               (1 << 25) /* Bit 25: PLL clock ready flag */
                                              /* Bits 26-27: Reserved */
#define RCC_CR_CSSON                (1 << 28) /* Bit 16: Clock security system enable */
#define RCC_CR_RTCPRE_SHIFT         (29)      /* Bits 29-30: RTC/LCD prescaler */
#define RCC_CR_RTCPRE_MASK          (3 << RCC_CR_RTCPRE_SHIFT)
                                              /* Bit 31: Reserved */
#define RCC_CR_RSTVAL               0x00000300

/* Internal clock sources calibration register */

#define RCC_ICSCR_HSICAL_SHIFT      (0)       /* Bits 0-7:  Internal high speed clock calibration */
#define RCC_ICSCR_HSICAL_MASK       (0xff << RCC_ICSCR_HSICAL_SHIFT)
#define RCC_ICSCR_HSITRIM_SHIFT     (8)       /* Bits 8-12:  High speed internal clock trimming */
#define RCC_ICSCR_HSITRIM_MASK      (0x1f << RCC_ICSCR_HSITRIM_SHIFT)
#define RCC_ICSCR_MSIRANGE_SHIFT    (13)      /* Bits 13-15:  MSI clock ranges */
#define RCC_ICSCR_MSIRANGE_MASK     (7 << RCC_ICSCR_MSIRANGE_SHIFT)
#  define RCC_ICSCR_MSIRANGE_0      (0 << RCC_ICSCR_MSIRANGE_SHIFT) /* 000: Range 0 around 65.536 kHz */
#  define RCC_ICSCR_MSIRANGE_1      (1 << RCC_ICSCR_MSIRANGE_SHIFT) /* 001: Range 1 around 131.072 kHz */
#  define RCC_ICSCR_MSIRANGE_2      (2 << RCC_ICSCR_MSIRANGE_SHIFT) /* 010: Range 2 around 262.144 kHz */
#  define RCC_ICSCR_MSIRANGE_3      (3 << RCC_ICSCR_MSIRANGE_SHIFT) /* 011: Range 3 around 524.288 kHz */
#  define RCC_ICSCR_MSIRANGE_4      (4 << RCC_ICSCR_MSIRANGE_SHIFT) /* 100: Range 4 around 1.048 MHz */
#  define RCC_ICSCR_MSIRANGE_5      (5 << RCC_ICSCR_MSIRANGE_SHIFT) /* 101: Range 5 around 2.097 MHz (reset value) */
#  define RCC_ICSCR_MSIRANGE_6      (6 << RCC_ICSCR_MSIRANGE_SHIFT) /* 110: Range 6 around 4.194 MHz */
#define RCC_ICSCR_MSICAL_SHIFT      (16)      /* Bits 16-23:  MSI clock calibration */
#define RCC_ICSCR_MSICAL_MASK       (0xff << RCC_ICSCR_MSICAL_SHIFT)
#define RCC_ICSCR_MSITRIM_SHIFT     (24)      /* Bits 24-31:  MSI clock trimming */
#define RCC_ICSCR_MSITRIM_MASK      (0xff << RCC_ICSCR_MSITRIM_SHIFT)

#define RCC_ICSR_RSTVAL             0x0000b000

/* Clock configuration register */

#define RCC_CFGR_SW_SHIFT           (0)       /* Bits 0-1: System clock Switch */
#define RCC_CFGR_SW_MASK            (3 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_MSI           (0 << RCC_CFGR_SW_SHIFT) /* 00: MSI selected as system clock */
#  define RCC_CFGR_SW_HSI           (1 << RCC_CFGR_SW_SHIFT) /* 01: HSI selected as system clock */
#  define RCC_CFGR_SW_HSE           (2 << RCC_CFGR_SW_SHIFT) /* 10: HSE selected as system clock */
#  define RCC_CFGR_SW_PLL           (3 << RCC_CFGR_SW_SHIFT) /* 11: PLL selected as system clock */
#define RCC_CFGR_SWS_SHIFT          (2)       /* Bits 2-3: System Clock Switch Status */
#define RCC_CFGR_SWS_MASK           (3 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_MSI          (0 << RCC_CFGR_SWS_SHIFT) /* 00: MSI oscillator used as system clock */
#  define RCC_CFGR_SWS_HSI          (1 << RCC_CFGR_SWS_SHIFT) /* 01: HSI oscillator used as system clock */
#  define RCC_CFGR_SWS_HSE          (2 << RCC_CFGR_SWS_SHIFT) /* 10: HSE oscillator used as system clock */
#  define RCC_CFGR_SWS_PLL          (3 << RCC_CFGR_SWS_SHIFT) /* 11: PLL used as system clock */
#define RCC_CFGR_HPRE_SHIFT         (4)       /* Bits 4-7: AHB prescaler */
#define RCC_CFGR_HPRE_MASK          (0x0f << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLK      (0 << RCC_CFGR_HPRE_SHIFT) /* 0xxx: SYSCLK not divided */
#  define RCC_CFGR_HPRE_SYSCLKd2    (8 << RCC_CFGR_HPRE_SHIFT) /* 1000: SYSCLK divided by 2 */
#  define RCC_CFGR_HPRE_SYSCLKd4    (9 << RCC_CFGR_HPRE_SHIFT) /* 1001: SYSCLK divided by 4 */
#  define RCC_CFGR_HPRE_SYSCLKd8    (10 << RCC_CFGR_HPRE_SHIFT) /* 1010: SYSCLK divided by 8 */
#  define RCC_CFGR_HPRE_SYSCLKd16   (11 << RCC_CFGR_HPRE_SHIFT) /* 1011: SYSCLK divided by 16 */
#  define RCC_CFGR_HPRE_SYSCLKd64   (12 << RCC_CFGR_HPRE_SHIFT) /* 1100: SYSCLK divided by 64 */
#  define RCC_CFGR_HPRE_SYSCLKd128  (13 << RCC_CFGR_HPRE_SHIFT) /* 1101: SYSCLK divided by 128 */
#  define RCC_CFGR_HPRE_SYSCLKd256  (14 << RCC_CFGR_HPRE_SHIFT) /* 1110: SYSCLK divided by 256 */
#  define RCC_CFGR_HPRE_SYSCLKd512  (15 << RCC_CFGR_HPRE_SHIFT) /* 1111: SYSCLK divided by 512 */
#define RCC_CFGR_PPRE1_SHIFT        (8)       /* Bits 18-10: APB Low speed prescaler (APB1) */
#define RCC_CFGR_PPRE1_MASK         (7 << RCC_CFGR_PPRE1_SHIFT)
#  define RCC_CFGR_PPRE1_HCLK       (0 << RCC_CFGR_PPRE1_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE1_HCLKd2     (4 << RCC_CFGR_PPRE1_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE1_HCLKd4     (5 << RCC_CFGR_PPRE1_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE1_HCLKd8     (6 << RCC_CFGR_PPRE1_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE1_HCLKd16    (7 << RCC_CFGR_PPRE1_SHIFT) /* 111: HCLK divided by 16 */
#define RCC_CFGR_PPRE2_SHIFT        (11)      /* Bits 11-13: APB High speed prescaler (APB2) */
#define RCC_CFGR_PPRE2_MASK         (7 << RCC_CFGR_PPRE2_SHIFT)
#  define RCC_CFGR_PPRE2_HCLK       (0 << RCC_CFGR_PPRE2_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE2_HCLKd2     (4 << RCC_CFGR_PPRE2_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE2_HCLKd4     (5 << RCC_CFGR_PPRE2_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE2_HCLKd8     (6 << RCC_CFGR_PPRE2_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE2_HCLKd16    (7 << RCC_CFGR_PPRE2_SHIFT) /* 111: HCLK divided by 16 */
                                              /* Bits 14-15: Reserved */
#define RCC_CFGR_PLLSRC             (1 << 16) /* Bit 16: PLL entry clock source */
                                              /* Bit 17: Reserved */
#define RCC_CFGR_PLLMUL_SHIFT       (18)      /* Bits 18-21: PLL Multiplication Factor */
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
#define RCC_CFGR_PLLDIV_SHIFT       (22)      /* Bits 22-23: PLL output division */
#define RCC_CFGR_PLLDIV_MASK        (3 << RCC_CFGR_PLLDIV_SHIFT)
#  define RCC_CFGR_PLLDIV_2         (1 << RCC_CFGR_PLLDIV_SHIFT) /* 01: PLL clock output = PLLVCO / 2 */
#  define RCC_CFGR_PLLDIV_3         (2 << RCC_CFGR_PLLDIV_SHIFT) /* 10: PLL clock output = PLLVCO / 3 */
#  define RCC_CFGR_PLLDIV_4         (3 << RCC_CFGR_PLLDIV_SHIFT) /* 11: PLL clock output = PLLVCO / 4 */
#define RCC_CFGR_MCOSEL_SHIFT       (24)      /* Bits 24-26: Microcontroller clock output selection */
#define RCC_CFGR_MCOSEL_MASK        (7 << RCC_CFGR_MCOSEL_SHIFT)
#  define RCC_CFGR_MCOSEL_DISABLED  (0 << RCC_CFGR_MCOSEL_SHIFT)  /* 000: MCO output disabled, no clock on MCO */
#  define RCC_CFGR_MCOSEL_SYSCLK    (1 << RCC_CFGR_MCOSEL_SHIFT)  /* 001: SYSCLK clock selected */
#  define RCC_CFGR_MCOSEL_HSICLK    (2 << RCC_CFGR_MCOSEL_SHIFT)  /* 010: HSI oscillator clock selected */
#  define RCC_CFGR_MCOSEL_MSICLK    (3 << RCC_CFGR_MCOSEL_SHIFT)  /* 011: MSI oscillator clock selected */
#  define RCC_CFGR_MCOSEL_HSECLK    (4 << RCC_CFGR_MCOSEL_SHIFT)  /* 100: HSE oscillator clock selected */
#  define RCC_CFGR_MCOSEL_PLLCLK    (5 << RCC_CFGR_MCOSEL_SHIFT)  /* 101: PLL clock selected */
#  define RCC_CFGR_MCOSEL_LSICLK    (6 << RCC_CFGR_MCOSEL_SHIFT)  /* 110: LSI oscillator clock selected */
#  define RCC_CFGR_MCOSEL_LSECLK    (7 << RCC_CFGR_MCOSEL_SHIFT)  /* 111: LSE oscillator clock selected */
                                              /* Bit 27: Reserved */
#define RCC_CFGR_MCOPRE_SHIFT       (28)      /* Bits 28-30: Microcontroller clock output selection */
#define RCC_CFGR_MCOPRE_MASK        (7 << RCC_CFGR_MCOPRE_SHIFT)
#  define RCC_CFGR_MCOPRE_DIV1      (0 << RCC_CFGR_MCOPRE_SHIFT) /* 000: MCO is divided by 1 */
#  define RCC_CFGR_MCOPRE_DIV2      (1 << RCC_CFGR_MCOPRE_SHIFT) /* 001: MCO is divided by 2 */
#  define RCC_CFGR_MCOPRE_DIV4      (2 << RCC_CFGR_MCOPRE_SHIFT) /* 010: MCO is divided by 4 */
#  define RCC_CFGR_MCOPRE_DIV8      (3 << RCC_CFGR_MCOPRE_SHIFT) /* 011: MCO is divided by 8 */
#  define RCC_CFGR_MCOPRE_DIV16     (4 << RCC_CFGR_MCOPRE_SHIFT) /* 100: MCO is divided by 16 */
                                              /* Bit 31: Reserved */
#define RCC_CFGR_RESET              0x00000000

/* Clock interrupt register */

#define RCC_CIR_LSIRDYF             (1 << 0)  /* Bit 0: LSI ready interrupt flag */
#define RCC_CIR_LSERDYF             (1 << 1)  /* Bit 1: LSE ready interrupt flag */
#define RCC_CIR_HSIRDYF             (1 << 2)  /* Bit 2: HSI ready interrupt flag */
#define RCC_CIR_HSERDYF             (1 << 3)  /* Bit 3: HSE ready interrupt flag */
#define RCC_CIR_PLLRDYF             (1 << 4)  /* Bit 4: PLL ready interrupt flag */
#define RCC_CIR_MSIRDYF             (1 << 5)  /* Bit 5: MSI ready interrupt flag */
#define RCC_CIR_LSECSSF             (1 << 6)  /* Bit 6: LSE CSS Interrupt flag */
#define RCC_CIR_CSSF                (1 << 7)  /* Bit 7: Clock security system interrupt flag */
#define RCC_CIR_LSIRDYIE            (1 << 8)  /* Bit 8: LSI ready interrupt enable */
#define RCC_CIR_LSERDYIE            (1 << 9)  /* Bit 9: LSE ready interrupt enable */
#define RCC_CIR_HSIRDYIE            (1 << 10) /* Bit 10: HSI ready interrupt enable */
#define RCC_CIR_HSERDYIE            (1 << 11) /* Bit 11: HSE ready interrupt enable */
#define RCC_CIR_PLLRDYIE            (1 << 12) /* Bit 12: PLL ready interrupt enable */
#define RCC_CIR_MSIRDYIE            (1 << 13) /* Bit 13: MSI ready interrupt enable */
#define RCC_CIR_LSECSSIE            (1 << 14) /* Bit 14: LSE CSS interrupt enable */
                                              /* Bit 15: Reserved */
#define RCC_CIR_LSIRDYC             (1 << 16) /* Bit 16: LSI ready interrupt clear */
#define RCC_CIR_LSERDYC             (1 << 17) /* Bit 17: LSE ready interrupt clear */
#define RCC_CIR_HSIRDYC             (1 << 18) /* Bit 18: HSI ready interrupt clear */
#define RCC_CIR_HSERDYC             (1 << 19) /* Bit 19: HSE ready interrupt clear */
#define RCC_CIR_PLLRDYC             (1 << 20) /* Bit 20: PLL ready interrupt clear */
#define RCC_CIR_MSIRDYC             (1 << 21) /* Bit 21: MSI ready interrupt clear */
#define RCC_CIR_LSECSSC             (1 << 22) /* Bit 22: LSE CSS interrupt clear */
#define RCC_CIR_CSSC                (1 << 23) /* Bit 23: Clock Security system interrupt clear */
                                              /* Bits 24-31: Reserved */

/* AHB peripheral clock reset register (RCC_AHBRSTR) */

#define RCC_AHBRSTR_GPIOPARST       (1 << 0)  /* Bit 0:  I/O port A reset */
#define RCC_AHBRSTR_GPIOPBRST       (1 << 1)  /* Bit 1:  I/O port B reset */
#define RCC_AHBRSTR_GPIOPCRST       (1 << 2)  /* Bit 2:  I/O port C reset */
#define RCC_AHBRSTR_GPIOPDRST       (1 << 3)  /* Bit 3:  I/O port D reset */
#define RCC_AHBRSTR_GPIOPERST       (1 << 4)  /* Bit 4:  I/O port E reset */
#define RCC_AHBRSTR_GPIOPHRST       (1 << 5)  /* Bit 5:  I/O port H reset */
#define RCC_AHBRSTR_GPIOPFRST       (1 << 6)  /* Bit 6:  I/O port F reset */
#define RCC_AHBRSTR_GPIOPGRST       (1 << 7)  /* Bit 7:  I/O port G reset */
                                              /* Bits 8-11: Reserved */
#define RCC_AHBRSTR_CRCRST          (1 << 12) /* Bit 12: CRC reset */
                                              /* Bits 13-14: Reserved */
#define RCC_AHBRSTR_FLITFRST        (1 << 15) /* Bit 15: FLITF reset */
                                              /* Bits 16-23: Reserved */
#define RCC_AHBRSTR_DMA1RST         (1 << 24) /* Bit 24: DMA1 reset */
#define RCC_AHBRSTR_DMA2RST         (1 << 25) /* Bit 25: DMA2 reset */
                                              /* Bit 26: Reserved */
#define RCC_AHBRSTR_AESRST          (1 << 27) /* Bit 27: AES reset */
                                              /* Bits 28-29: Reserved */
#define RCC_AHBRSTR_FSMCRST         (1 << 30) /* Bit 30: FSMC reset */
                                              /* Bit 31: Reserved */

/* APB2 Peripheral reset register */

#define RCC_APB2RSTR_SYSCFGRST      (1 << 0)  /* Bit 0:  System configuration controller reset */
                                              /* Bit 1:  Reserved */
#define RCC_APB2RSTR_TIM9RST        (1 << 2)  /* Bit 2:  TIM9 timer reset */
#define RCC_APB2RSTR_TIM10RST       (1 << 3)  /* Bit 3:  TIM10 timer reset */
#define RCC_APB2RSTR_TIM11RST       (1 << 4)  /* Bit 4:  TIM11 timer reset */
                                              /* Bits 5-8: Reserved */
#define RCC_APB2RSTR_ADC1RST        (1 << 9)  /* Bit 9:  ADC1 interface reset */
                                              /* Bit 10: Reserved */
#define RCC_APB2RSTR_SDIORST        (1 << 11) /* Bit 11: SDIO reset */
#define RCC_APB2RSTR_SPI1RST        (1 << 12) /* Bit 12: SPI 1 reset */
                                              /* Bit 13: Reserved */
#define RCC_APB2RSTR_USART1RST      (1 << 14) /* Bit 14: USART1 reset */
                                              /* Bits 15-31: Reserved */

/* APB1 Peripheral reset register */

#define RCC_APB1RSTR_TIM2RST        (1 << 0)  /* Bit 0:  Timer 2 reset */
#define RCC_APB1RSTR_TIM3RST        (1 << 1)  /* Bit 1:  Timer 3 reset */
#define RCC_APB1RSTR_TIM4RST        (1 << 2)  /* Bit 2:  Timer 4 reset */
#define RCC_APB1RSTR_TIM5RST        (1 << 3)  /* Bit 3:  Timer 5 reset */
#define RCC_APB1RSTR_TIM6RST        (1 << 4)  /* Bit 4:  Timer 6 reset */
#define RCC_APB1RSTR_TIM7RST        (1 << 5)  /* Bit 5:  Timer 7 reset */
                                              /* Bits 6-8: Reserved */
#define RCC_APB1RSTR_LCDRST         (1 << 9)  /* Bit 9:  LCD reset */
                                              /* Bit 10: Reserved */
#define RCC_APB1RSTR_WWDGRST        (1 << 11) /* Bit 11: Window Watchdog reset */
                                              /* Bits 12-13: Reserved */
#define RCC_APB1RSTR_SPI2RST        (1 << 14) /* Bit 14: SPI 2 reset */
#define RCC_APB1RSTR_SPI3RST        (1 << 15) /* Bit 15: SPI 3 reset */
                                              /* Bit 16: Reserved */
#define RCC_APB1RSTR_USART2RST      (1 << 17) /* Bit 17: USART 2 reset */
#define RCC_APB1RSTR_USART3RST      (1 << 18) /* Bit 18: USART 3 reset */
#define RCC_APB1RSTR_UART4RST       (1 << 19) /* Bit 19: UART 4 reset */
#define RCC_APB1RSTR_UART5RST       (1 << 20) /* Bit 20: UART 5 reset */
#define RCC_APB1RSTR_I2C1RST        (1 << 21) /* Bit 21: I2C 1 reset */
#define RCC_APB1RSTR_I2C2RST        (1 << 22) /* Bit 22: I2C 2 reset */
#define RCC_APB1RSTR_USBRST         (1 << 23) /* Bit 23: USB reset */
                                              /* Bits 24-27: Reserved */
#define RCC_APB1RSTR_PWRRST         (1 << 28) /* Bit 28: Power interface reset */
#define RCC_APB1RSTR_DACRST         (1 << 29) /* Bit 29: DAC interface reset */
                                              /* Bit 30: Reserved */
#define RCC_APB1RSTR_COMPRST        (1 << 31) /* Bit 31: COMP interface reset */

/* AHB Peripheral Clock enable register */

#define RCC_AHBENR_GPIOEN(n)        (1 << (n))
#define RCC_AHBENR_GPIOPAEN         (1 << 0)  /* Bit 0:  I/O port A clock enable */
#define RCC_AHBENR_GPIOPBEN         (1 << 1)  /* Bit 1:  I/O port B clock enable */
#define RCC_AHBENR_GPIOPCEN         (1 << 2)  /* Bit 2:  I/O port C clock enable */
#define RCC_AHBENR_GPIOPDEN         (1 << 3)  /* Bit 3:  I/O port D clock enable */
#define RCC_AHBENR_GPIOPEEN         (1 << 4)  /* Bit 4:  I/O port E clock enable */
#define RCC_AHBENR_GPIOPHEN         (1 << 5)  /* Bit 5:  I/O port H clock enable */
#define RCC_AHBENR_GPIOPFEN         (1 << 6)  /* Bit 6:  I/O port F clock enable */
#define RCC_AHBENR_GPIOPGEN         (1 << 7)  /* Bit 7:  I/O port G clock enable */
                                              /* Bits 8-11: Reserved */
#define RCC_AHBENR_CRCEN            (1 << 12) /* Bit 12: CRC clock enable */
                                              /* Bits 13-14: Reserved */
#define RCC_AHBENR_FLITFEN          (1 << 15) /* Bit 15: FLITF clock enable */
                                              /* Bits 16-23: Reserved */
#define RCC_AHBENR_DMA1EN           (1 << 24) /* Bit 24: DMA1 clock enable */
#define RCC_AHBENR_DMA2EN           (1 << 25) /* Bit 25: DMA2 clock enable */
                                              /* Bit 26: Reserved */
#define RCC_AHBENR_AESEN            (1 << 27) /* Bit 27: AES clock enable */
                                              /* Bits 28-29: Reserved */
#define RCC_AHBENR_FSMCEN           (1 << 30) /* Bit 30: FSMC clock enable */
                                              /* Bit 31: Reserved */

/* APB2 Peripheral Clock enable register */

#define RCC_APB2ENR_SYSCFGEN        (1 << 0)  /* Bit 0:  System configuration controller clock enable */
                                              /* Bit 1:  Reserved */
#define RCC_APB2ENR_TIM9EN          (1 << 2)  /* Bit 2:  TIM9 timer clock enable */
#define RCC_APB2ENR_TIM10EN         (1 << 3)  /* Bit 3:  TIM10 timer clock enable */
#define RCC_APB2ENR_TIM11EN         (1 << 4)  /* Bit 4:  TIM11 timer clock enable */
                                              /* Bits 5-8: Reserved */
#define RCC_APB2ENR_ADC1EN          (1 << 9)  /* Bit 9:  ADC1 interface clock enable */
                                              /* Bit 10: Reserved */
#define RCC_APB2ENR_SDIOEN          (1 << 11) /* Bit 11: SDIO clock enable */
#define RCC_APB2ENR_SPI1EN          (1 << 12) /* Bit 12: SPI 1 clock enable */
                                              /* Bit 13: Reserved */
#define RCC_APB2ENR_USART1EN        (1 << 14) /* Bit 14: USART1 clock enable */
                                              /* Bits 15-31: Reserved */

/* APB1 Peripheral Clock enable register */

#define RCC_APB1ENR_TIM2EN          (1 << 0)  /* Bit 0:  Timer 2 clock enable */
#define RCC_APB1ENR_TIM3EN          (1 << 1)  /* Bit 1:  Timer 3 clock enable */
#define RCC_APB1ENR_TIM4EN          (1 << 2)  /* Bit 2:  Timer 4 clock enable */
#define RCC_APB1ENR_TIM5EN          (1 << 3)  /* Bit 3:  Timer 5 clock enable */
#define RCC_APB1ENR_TIM6EN          (1 << 4)  /* Bit 4:  Timer 6 clock enable */
#define RCC_APB1ENR_TIM7EN          (1 << 5)  /* Bit 5:  Timer 7 clock enable */
                                              /* Bits 6-8: Reserved */
#define RCC_APB1ENR_LCDEN           (1 << 9)  /* Bit 9:  LCD clock enable */
                                              /* Bit 10: Reserved */
#define RCC_APB1ENR_WWDGEN          (1 << 11) /* Bit 11: Window Watchdog clock enable */
                                              /* Bits 12-13: Reserved */
#define RCC_APB1ENR_SPI2EN          (1 << 14) /* Bit 14: SPI 2 clock enable */
#define RCC_APB1ENR_SPI3EN          (1 << 15) /* Bit 15: SPI 3 clock enable */
                                              /* Bit 16: Reserved */
#define RCC_APB1ENR_USART2EN        (1 << 17) /* Bit 17: USART 2 clock enable */
#define RCC_APB1ENR_USART3EN        (1 << 18) /* Bit 18: USART 3 clock enable */
#define RCC_APB1ENR_UART4EN         (1 << 19) /* Bit 19: UART 4 clock enable */
#define RCC_APB1ENR_UART5EN         (1 << 20) /* Bit 20: UART 5 clock enable */
#define RCC_APB1ENR_I2C1EN          (1 << 21) /* Bit 21: I2C 1 clock enable */
#define RCC_APB1ENR_I2C2EN          (1 << 22) /* Bit 22: I2C 2 clock enable */
#define RCC_APB1ENR_USBEN           (1 << 23) /* Bit 23: USB clock enable */
                                              /* Bits 24-27: Reserved */
#define RCC_APB1ENR_PWREN           (1 << 28) /* Bit 28: Power interface clock enable */
#define RCC_APB1ENR_DACEN           (1 << 29) /* Bit 29: DAC interface clock enable */
                                              /* Bit 30: Reserved */
#define RCC_APB1ENR_COMPEN          (1 << 31) /* Bit 31: COMP interface clock enable */

/* AHB peripheral clock enable in low power mode register */

#define RCC_AHBLPENR_GPIOPALPEN     (1 << 0)  /* Bit 0:  I/O port A clock enable during sleep mode */
#define RCC_AHBLPENR_GPIOPBLPEN     (1 << 1)  /* Bit 1:  I/O port B clock enable during sleep mode */
#define RCC_AHBLPENR_GPIOPCLPEN     (1 << 2)  /* Bit 2:  I/O port C clock enable during sleep mode */
#define RCC_AHBLPENR_GPIOPDLPEN     (1 << 3)  /* Bit 3:  I/O port D clock enable during sleep mode */
#define RCC_AHBLPENR_GPIOPELPEN     (1 << 4)  /* Bit 4:  I/O port E clock enable during sleep mode */
#define RCC_AHBLPENR_GPIOPHLPEN     (1 << 5)  /* Bit 5:  I/O port H clock enable during sleep mode */
#define RCC_AHBLPENR_GPIOPFLPEN     (1 << 6)  /* Bit 6:  I/O port F clock enable during sleep mode */
#define RCC_AHBLPENR_GPIOPGLPEN     (1 << 7)  /* Bit 7:  I/O port G clock enable during sleep mode */
                                              /* Bits 8-11: Reserved */
#define RCC_AHBLPENR_CRCLPEN        (1 << 12) /* Bit 12: CRC clock enable during sleep mode */
                                              /* Bits 13-14: Reserved */
#define RCC_AHBLPENR_FLITFLPEN      (1 << 15) /* Bit 15: FLITF clock enable during sleep mode */
#define RCC_AHBLPENR_SRAMLPEN       (1 << 16) /* Bit 16: SRAM clock enable during sleep mode */
                                              /* Bits 17-23: Reserved */
#define RCC_AHBLPENR_DMA1LPEN       (1 << 24) /* Bit 24: DMA1 clock enable during sleep mode */
#define RCC_AHBLPENR_DMA2LPEN       (1 << 25) /* Bit 25: DMA2 clock enable during sleep mode */
                                              /* Bit 26: Reserved */
#define RCC_AHBLPENR_AESLPEN        (1 << 27) /* Bit 27: AES clock enable during sleep mode */
                                              /* Bits 28-29: Reserved */
#define RCC_AHBLPENR_FSMCLPEN       (1 << 30) /* Bit 30: FSMC clock enable during sleep mode */
                                              /* Bit 31: Reserved */

/* APB2 peripheral clock enable in low power mode register */

#define RCC_APB2LPENR_SYSCFGLPEN    (1 << 0)  /* Bit 0:  System configuration controller clock enable during sleep mode */
                                              /* Bit 1:  Reserved */
#define RCC_APB2LPENR_TIM9LPEN      (1 << 2)  /* Bit 2:  TIM9 timer clock enable during sleep mode */
#define RCC_APB2LPENR_TIM10LPEN     (1 << 3)  /* Bit 3:  TIM10 timer clock enable during sleep mode */
#define RCC_APB2LPENR_TIM11LPEN     (1 << 4)  /* Bit 4:  TIM11 timer clock enable during sleep mode */
                                              /* Bits 5-8: Reserved */
#define RCC_APB2LPENR_ADC1LPEN      (1 << 9)  /* Bit 9:  ADC1 interface clock enable during sleep mode */
                                              /* Bit 10: Reserved */
#define RCC_APB2LPENR_SDIOLPEN      (1 << 11) /* Bit 11: SDIO clock enable during sleep mode */
#define RCC_APB2LPENR_SPI1LPEN      (1 << 12) /* Bit 12: SPI 1 clock enable during sleep mode */
                                              /* Bit 13: Reserved */
#define RCC_APB2LPENR_USART1LPEN    (1 << 14) /* Bit 14: USART1 clock enable during sleep mode */
                                              /* Bits 15-31: Reserved */

/* APB1 peripheral clock enable in low power mode register */

#define RCC_APB1LPENR_TIM2LPEN      (1 << 0)  /* Bit 0:  Timer 2 clock enable during sleep mode */
#define RCC_APB1LPENR_TIM3LPEN      (1 << 1)  /* Bit 1:  Timer 3 clock enable during sleep mode */
#define RCC_APB1LPENR_TIM4LPEN      (1 << 2)  /* Bit 2:  Timer 4 clock enable during sleep mode */
#define RCC_APB1LPENR_TIM5LPEN      (1 << 3)  /* Bit 3:  Timer 5 clock enable during sleep mode */
#define RCC_APB1LPENR_TIM6LPEN      (1 << 4)  /* Bit 4:  Timer 6 clock enable during sleep mode */
#define RCC_APB1LPENR_TIM7LPEN      (1 << 5)  /* Bit 5:  Timer 7 clock enable during sleep mode */
                                              /* Bits 6-8: Reserved */
#define RCC_APB1LPENR_LCDLPEN       (1 << 9)  /* Bit 9:  LCD clock enable during sleep mode */
                                              /* Bit 10: Reserved */
#define RCC_APB1LPENR_WWDGLPEN      (1 << 11) /* Bit 11: Window Watchdog clock enable during sleep mode */
                                              /* Bits 12-13: Reserved */
#define RCC_APB1LPENR_SPI2LPEN      (1 << 14) /* Bit 14: SPI 2 clock enable during sleep mode */
#define RCC_APB1LPENR_SPI3LPEN      (1 << 15) /* Bit 15: SPI 3 clock enable during sleep mode */
                                              /* Bit 16: Reserved */
#define RCC_APB1LPENR_USART2LPEN    (1 << 17) /* Bit 17: USART 2 clock enable during sleep mode */
#define RCC_APB1LPENR_USART3LPEN    (1 << 18) /* Bit 18: USART 3 clock enable during sleep mode */
#define RCC_APB1LPENR_UART4LPEN     (1 << 19) /* Bit 19: UART 4 clock enable during sleep mode */
#define RCC_APB1LPENR_UART5LPEN     (1 << 20) /* Bit 20: UART 5 clock enable during sleep mode */
#define RCC_APB1LPENR_I2C1LPEN      (1 << 21) /* Bit 21: I2C 1 clock enable during sleep mode */
#define RCC_APB1LPENR_I2C2LPEN      (1 << 22) /* Bit 22: I2C 2 clock enable during sleep mode */
#define RCC_APB1LPENR_USBLPEN       (1 << 23) /* Bit 23: USB clock enable during sleep mode */
                                              /* Bits 24-27: Reserved */
#define RCC_APB1LPENR_PWRLPEN       (1 << 28) /* Bit 28: Power interface clock enable during sleep mode */
#define RCC_APB1LPENR_DACLPEN       (1 << 29) /* Bit 29: DAC interface clock enable during sleep mode */
                                              /* Bit 30: Reserved */
#define RCC_APB1LPENR_COMPLPEN      (1 << 31) /* Bit 31: COMP interface clock enable during sleep mode */

/* Control/status register */

#define RCC_CSR_LSION               (1 << 0)  /* Bit 0:  Internal Low Speed oscillator enable */
#define RCC_CSR_LSIRDY              (1 << 1)  /* Bit 1:  Internal Low Speed oscillator Ready */
                                              /* Bits 2-7: Reserved */
#define RCC_CSR_LSEON               (1 << 8)  /* Bit 8:  External Low Speed oscillator enable */
#define RCC_CSR_LSERDY              (1 << 9)  /* Bit 9:  External Low Speed oscillator Ready */
#define RCC_CSR_LSEBYP              (1 << 10) /* Bit 10: External low-speed oscillator bypass */
#define RCC_CSR_LSECSSON            (1 << 11) /* Bit 11: CSS on LSE enable */
#define RCC_CSR_LSECSSD             (1 << 12) /* Bit 12: CSS on LSE failure Detection */
                                              /* Bits 13-15: Reserved */
#define RCC_CSR_RTCSEL_SHIFT        (16)      /* Bits 16-17: RTC and LCD clock source selection */
#define RCC_CSR_RTCSEL_MASK         (3 << RCC_CSR_RTCSEL_SHIFT)
#  define RCC_CSR_RTCSEL_NONE       (0 << RCC_CSR_RTCSEL_SHIFT) /* 00: No clock */
#  define RCC_CSR_RTCSEL_LSE        (1 << RCC_CSR_RTCSEL_SHIFT) /* 01: LSE oscillator clock is RTC/LCD clock */
#  define RCC_CSR_RTCSEL_LSI        (2 << RCC_CSR_RTCSEL_SHIFT) /* 10: LSI oscillator clock is RTC/LCD clock */
#  define RCC_CSR_RTCSEL_HSE        (3 << RCC_CSR_RTCSEL_SHIFT) /* 11: Divided HSE oscillator clock is RTC/LCD clock */
#define RCC_CSR_RTCEN               (1 << 22) /* Bit 22: RTC clock enable */
#define RCC_CSR_RTCRST              (1 << 23) /* Bit 23: RTC software reset */
#define RCC_CSR_RMVF                (1 << 24) /* Bit 24: Remove reset flag */
#define RCC_CSR_OBLRSTF             (1 << 25) /* Bit 25: Option byte loader reset flag */
#define RCC_CSR_PINRSTF             (1 << 26) /* Bit 26: PIN reset flag */
#define RCC_CSR_PORRSTF             (1 << 27) /* Bit 27: POR/PDR reset flag */
#define RCC_CSR_SFTRSTF             (1 << 28) /* Bit 28: Software Reset flag */
#define RCC_CSR_IWDGRSTF            (1 << 29) /* Bit 29: Independent Watchdog reset flag */
#define RCC_CSR_WWDGRSTF            (1 << 30) /* Bit 30: Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF            (1 << 31) /* Bit 31: Low-Power reset flag */

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32L15XXX_RCC_H */

