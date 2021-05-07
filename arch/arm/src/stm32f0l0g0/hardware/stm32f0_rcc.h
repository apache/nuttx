/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32f0_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32F0_RCC_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32F0_RCC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_RCC_CR_OFFSET         0x0000  /* Clock control register */
#define STM32_RCC_CFGR_OFFSET       0x0004  /* Clock configuration register */
#define STM32_RCC_CIR_OFFSET        0x0008  /* Clock interrupt register */
#define STM32_RCC_APB2RSTR_OFFSET   0x000c  /* APB2 Peripheral reset register */
#define STM32_RCC_APB1RSTR_OFFSET   0x0010  /* APB1 Peripheral reset register */
#define STM32_RCC_AHBENR_OFFSET     0x0014  /* AHB Peripheral Clock enable register */
#define STM32_RCC_APB2ENR_OFFSET    0x0018  /* APB2 Peripheral Clock enable register */
#define STM32_RCC_APB1ENR_OFFSET    0x001c  /* APB1 Peripheral Clock enable register */
#define STM32_RCC_BDCR_OFFSET       0x0020  /* Backup domain control register */
#define STM32_RCC_CSR_OFFSET        0x0024  /* Control/status register */
#define STM32_RCC_AHBRSTR_OFFSET    0x0028  /* AHB Reset register */
#define STM32_RCC_CFGR2_OFFSET      0x002c  /* Clock configuration register 2 */
#define STM32_RCC_CFGR3_OFFSET      0x0030  /* Clock configuration register 3 */
#define STM32_RCC_CR2_OFFSET        0x0034  /* Clock control register 2 */

/* Register Addresses *******************************************************/

#define STM32_RCC_CR                (STM32_RCC_BASE+STM32_RCC_CR_OFFSET)
#define STM32_RCC_CFGR              (STM32_RCC_BASE+STM32_RCC_CFGR_OFFSET)
#define STM32_RCC_CIR               (STM32_RCC_BASE+STM32_RCC_CIR_OFFSET)
#define STM32_RCC_APB2RSTR          (STM32_RCC_BASE+STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_APB1RSTR          (STM32_RCC_BASE+STM32_RCC_APB1RSTR_OFFSET)
#define STM32_RCC_AHBENR            (STM32_RCC_BASE+STM32_RCC_AHBENR_OFFSET)
#define STM32_RCC_APB2ENR           (STM32_RCC_BASE+STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_APB1ENR           (STM32_RCC_BASE+STM32_RCC_APB1ENR_OFFSET)
#define STM32_RCC_BDCR              (STM32_RCC_BASE+STM32_RCC_BDCR_OFFSET)
#define STM32_RCC_CSR               (STM32_RCC_BASE+STM32_RCC_CSR_OFFSET)
#define STM32_RCC_AHBRSTR           (STM32_RCC_BASE+STM32_RCC_AHBRSTR_OFFSET)
#define STM32_RCC_CFGR2             (STM32_RCC_BASE+STM32_RCC_CFGR2_OFFSET)
#define STM32_RCC_CFGR3             (STM32_RCC_BASE+STM32_RCC_CFGR3_OFFSET)
#define STM32_RCC_CR2               (STM32_RCC_BASE+STM32_RCC_CR2_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Clock control register */

#define RCC_CR_HSION                (1 << 0)  /* Bit 0: Internal High Speed clock enable */
#define RCC_CR_HSIRDY               (1 << 1)  /* Bit 1: Internal High Speed clock ready flag */
#define RCC_CR_HSITRIM_SHIFT        (3)       /* Bits 7-3: Internal High Speed clock trimming */
#define RCC_CR_HSITRIM_MASK         (0x1f << RCC_CR_HSITRIM_SHIFT)
#define RCC_CR_HSICAL_SHIFT         (8)       /* Bits 15-8: Internal High Speed clock Calibration */
#define RCC_CR_HSICAL_MASK          (0xff << RCC_CR_HSICAL_SHIFT)
#define RCC_CR_HSEON                (1 << 16) /* Bit 16: External High Speed clock enable */
#define RCC_CR_HSERDY               (1 << 17) /* Bit 17: External High Speed clock ready flag */
#define RCC_CR_HSEBYP               (1 << 18) /* Bit 18: External High Speed clock Bypass */
#define RCC_CR_CSSON                (1 << 19) /* Bit 19: Clock Security System enable */
#define RCC_CR_PLLON                (1 << 24) /* Bit 24: PLL enable */
#define RCC_CR_PLLRDY               (1 << 25) /* Bit 25: PLL clock ready flag */

/* Clock configuration register */

#define RCC_CFGR_SW_SHIFT           (0)       /* Bits 1-0: System clock Switch */
#define RCC_CFGR_SW_MASK            (3 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_HSI           (0 << RCC_CFGR_SW_SHIFT) /* 00: HSI selected as system clock */
#  define RCC_CFGR_SW_HSE           (1 << RCC_CFGR_SW_SHIFT) /* 01: HSE selected as system clock */
#  define RCC_CFGR_SW_PLL           (2 << RCC_CFGR_SW_SHIFT) /* 10: PLL selected as system clock */
#  define RCC_CFGR_SW_HSI48         (3 << RCC_CFGR_SW_SHIFT) /* 11: HSI48 selected as system clock */

#define RCC_CFGR_SWS_SHIFT          (2)       /* Bits 3-2: System Clock Switch Status */
#define RCC_CFGR_SWS_MASK           (3 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_HSI          (0 << RCC_CFGR_SWS_SHIFT) /* 00: HSI oscillator used as system clock */
#  define RCC_CFGR_SWS_HSE          (1 << RCC_CFGR_SWS_SHIFT) /* 01: HSE oscillator used as system clock */
#  define RCC_CFGR_SWS_PLL          (2 << RCC_CFGR_SWS_SHIFT) /* 10: PLL used as system clock */
#  define RCC_CFGR_SWS_HSI48        (3 << RCC_CFGR_SWS_SHIFT) /* 11: HSI48 used as system clock */

#define RCC_CFGR_HPRE_SHIFT         (4)       /* Bits 7-4: AHB prescaler */
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

#define RCC_CFGR_PPRE1_SHIFT        (8)       /* Bits 10-8: APB Low speed prescaler (APB1) */
#define RCC_CFGR_PPRE1_MASK         (7 << RCC_CFGR_PPRE1_SHIFT)
#  define RCC_CFGR_PPRE1_HCLK       (0 << RCC_CFGR_PPRE1_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE1_HCLKd2     (4 << RCC_CFGR_PPRE1_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE1_HCLKd4     (5 << RCC_CFGR_PPRE1_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE1_HCLKd8     (6 << RCC_CFGR_PPRE1_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE1_HCLKd16    (7 << RCC_CFGR_PPRE1_SHIFT) /* 111: HCLK divided by 16 */

                                              /* Bits 13-11: Reserve.
                                               * Keep the reset value
                                               */

#define RCC_CFGR_ADCPRE             (1 << 14) /* Bit 14: ADC prescaler, Obsolete use ADC_CFGR2 */
#define RCC_CFGR_PLLSRC_SHIFT           (15)  /* Bit 15: PLL input clock source */
#define RCC_CFGR_PLLSRC_MASK            (3 << RCC_CFGR_PLLSRC_SHIFT)
#  define RCC_CFGR_PLLSRC_HSId2         (0 << RCC_CFGR_PLLSRC_SHIFT) /* 00: HSI/2 as PLL input clock */
#  define RCC_CFGR_PLLSRC_HS1_PREDIV    (1 << RCC_CFGR_PLLSRC_SHIFT) /* 01: HSE/PREDIV as PLL input clock */
#  define RCC_CFGR_PLLSRC_HSE_PREDIV    (2 << RCC_CFGR_PLLSRC_SHIFT) /* 10: HSE/PREDIV as PLL input clock */
#  define RCC_CFGR_PLLSRC_HSI48_PREDIV  (3 << RCC_CFGR_PLLSRC_SHIFT) /* 11: HSI48/PREDIV as PLL input clock */

#define RCC_CFGR_PLLXTPRE_MASK      (1 << 17) /* Bit 17: HSE divider for PLL entry */
#  define RCC_CFGR_PLLXTPRE_DIV1    (0 << 17) /*         0=No divistion */
#  define RCC_CFGR_PLLXTPRE_DIV2    (1 << 17) /*         1=Divide by two */
#define RCC_CFGR_PLLMUL_SHIFT       (18)      /* Bits 21-18: PLL Multiplication Factor */
#define RCC_CFGR_PLLMUL_MASK        (0x0f << RCC_CFGR_PLLMUL_SHIFT)
#  define RCC_CFGR_PLLMUL_CLKx2     (0 << RCC_CFGR_PLLMUL_SHIFT)  /* 0000: PLL input clock x 2 */
#  define RCC_CFGR_PLLMUL_CLKx3     (1 << RCC_CFGR_PLLMUL_SHIFT)  /* 0001: PLL input clock x 3 */
#  define RCC_CFGR_PLLMUL_CLKx4     (2 << RCC_CFGR_PLLMUL_SHIFT)  /* 0010: PLL input clock x 4 */
#  define RCC_CFGR_PLLMUL_CLKx5     (3 << RCC_CFGR_PLLMUL_SHIFT)  /* 0011: PLL input clock x 5 */
#  define RCC_CFGR_PLLMUL_CLKx6     (4 << RCC_CFGR_PLLMUL_SHIFT)  /* 0100: PLL input clock x 6 */
#  define RCC_CFGR_PLLMUL_CLKx7     (5 << RCC_CFGR_PLLMUL_SHIFT)  /* 0101: PLL input clock x 7 */
#  define RCC_CFGR_PLLMUL_CLKx8     (6 << RCC_CFGR_PLLMUL_SHIFT)  /* 0110: PLL input clock x 8 */
#  define RCC_CFGR_PLLMUL_CLKx9     (7 << RCC_CFGR_PLLMUL_SHIFT)  /* 0111: PLL input clock x 9 */
#  define RCC_CFGR_PLLMUL_CLKx10    (8 << RCC_CFGR_PLLMUL_SHIFT)  /* 1000: PLL input clock x 10 */
#  define RCC_CFGR_PLLMUL_CLKx11    (9 << RCC_CFGR_PLLMUL_SHIFT)  /* 1001: PLL input clock x 11 */
#  define RCC_CFGR_PLLMUL_CLKx12    (10 << RCC_CFGR_PLLMUL_SHIFT) /* 1010: PLL input clock x 12 */
#  define RCC_CFGR_PLLMUL_CLKx13    (11 << RCC_CFGR_PLLMUL_SHIFT) /* 1011: PLL input clock x 13 */
#  define RCC_CFGR_PLLMUL_CLKx14    (12 << RCC_CFGR_PLLMUL_SHIFT) /* 1100: PLL input clock x 14 */
#  define RCC_CFGR_PLLMUL_CLKx15    (13 << RCC_CFGR_PLLMUL_SHIFT) /* 1101: PLL input clock x 15 */
#  define RCC_CFGR_PLLMUL_CLKx16    (14 << RCC_CFGR_PLLMUL_SHIFT) /* 111x: PLL input clock x 16 */

                                    /* Bit 22-23: Reserved */

#define RCC_CFGR_MCO_SHIFT          (24)      /* Bits 27-24: Microcontroller Clock Output */
#define RCC_CFGR_MCO_MASK           (15 << RCC_CFGR_MCO_SHIFT)
#  define RCC_CFGR_NOCLK            (0 << RCC_CFGR_MCO_SHIFT)    /* 0000: No clock */
#  define RCC_CFGR_HSI14            (1 << RCC_CFGR_MCO_SHIFT)    /* 0001: Internal RC 14MHz oscillator */
#  define RCC_CFGR_LSI              (2 << RCC_CFGR_MCO_SHIFT)    /* 0010: Internal Low Speed (LSI) oscillator */
#  define RCC_CFGR_LSE              (2 << RCC_CFGR_MCO_SHIFT)    /* 0011: External Low Speed (LSE) oscillator */
#  define RCC_CFGR_SYSCLK           (4 << RCC_CFGR_MCO_SHIFT)    /* 0100: System clock selected */
#  define RCC_CFGR_INTCLK           (5 << RCC_CFGR_MCO_SHIFT)    /* 0101: Internal 8 MHz RC oscillator clock selected */
#  define RCC_CFGR_EXTCLK           (6 << RCC_CFGR_MCO_SHIFT)    /* 0110: External 4-32 MHz oscillator clock selected */
#  define RCC_CFGR_PLLCLKd2         (7 << RCC_CFGR_MCO_SHIFT)    /* 0111: PLL clock selected (divided by 1 or 2
                                                                  * depending on PLLNODIV) */
#  define RCC_CFGR_PLL2CLK          (8 << RCC_CFGR_MCO_SHIFT)    /* 1000: Internal RC 48MHz (HSI48) oscillator */
#define RCC_CFGR_MCOPRE_SHIFT       (28)                         /* Bits 28-30: Microcontroller Clock Output Prescaler,
                                                                  * not available on STM32F05x */
#define RCC_CFGR_MCOPRE_MASK        (3 << RCC_CFGR_MCOPRE_SHIFT)
#  define RCC_CFGR_MCOPRE_DIV1      (0 << RCC_CFGR_MCOPRE_SHIFT) /* 000: MCO is divided by 1 */
#  define RCC_CFGR_MCOPRE_DIV2      (1 << RCC_CFGR_MCOPRE_SHIFT) /* 001: MCO is divided by 2 */
#  define RCC_CFGR_MCOPRE_DIV4      (2 << RCC_CFGR_MCOPRE_SHIFT) /* 010: MCO is divided by 4 */
#  define RCC_CFGR_MCOPRE_DIV8      (3 << RCC_CFGR_MCOPRE_SHIFT) /* 011: MCO is divided by 8 */
#  define RCC_CFGR_MCOPRE_DIV16     (4 << RCC_CFGR_MCOPRE_SHIFT) /* 100: MCO is divided by 16 */
#  define RCC_CFGR_MCOPRE_DIV32     (5 << RCC_CFGR_MCOPRE_SHIFT) /* 101: MCO is divided by 32 */
#  define RCC_CFGR_MCOPRE_DIV64     (6 << RCC_CFGR_MCOPRE_SHIFT) /* 110: MCO is divided by 64 */
#  define RCC_CFGR_MCOPRE_DIV128    (7 << RCC_CFGR_MCOPRE_SHIFT) /* 111: MCO is divided by 128 */

/* Clock interrupt register */

#define RCC_CIR_LSIRDYF             (1 << 0)  /* Bit 0: LSI Ready Interrupt flag */
#define RCC_CIR_LSERDYF             (1 << 1)  /* Bit 1: LSE Ready Interrupt flag */
#define RCC_CIR_HSIRDYF             (1 << 2)  /* Bit 2: HSI Ready Interrupt flag */
#define RCC_CIR_HSERDYF             (1 << 3)  /* Bit 3: HSE Ready Interrupt flag */
#define RCC_CIR_PLLRDYF             (1 << 4)  /* Bit 4: PLL Ready Interrupt flag */
#define RCC_CIR_HSI14RDYF           (1 << 5)  /* Bit 5: HSI14 Ready Interrupt flag */
#define RCC_CIR_HSI48RDYF           (1 << 6)  /* Bit 6: HSI48 Ready Interrupt flag */
#define RCC_CIR_CSSF                (1 << 7)  /* Bit 7: Clock Security System Interrupt flag */
#define RCC_CIR_LSIRDYIE            (1 << 8)  /* Bit 8: LSI Ready Interrupt Enable */
#define RCC_CIR_LSERDYIE            (1 << 9)  /* Bit 9: LSE Ready Interrupt Enable */
#define RCC_CIR_HSIRDYIE            (1 << 10) /* Bit 10: HSI Ready Interrupt Enable */
#define RCC_CIR_HSERDYIE            (1 << 11) /* Bit 11: HSE Ready Interrupt Enable */
#define RCC_CIR_PLLRDYIE            (1 << 12) /* Bit 12: PLL Ready Interrupt Enable */
#define RCC_CIR_HSI14RDYIE          (1 << 13) /* Bit 13: HSI14 Ready Interrupt Enable */
#define RCC_CIR_HSI48RDYIE          (1 << 14) /* Bit 14: HSI48 Ready Interrupt Enable */
#define RCC_CIR_LSIRDYC             (1 << 16) /* Bit 16: LSI Ready Interrupt Clear */
#define RCC_CIR_LSERDYC             (1 << 17) /* Bit 17: LSE Ready Interrupt Clear */
#define RCC_CIR_HSIRDYC             (1 << 18) /* Bit 18: HSI Ready Interrupt Clear */
#define RCC_CIR_HSERDYC             (1 << 19) /* Bit 19: HSE Ready Interrupt Clear */
#define RCC_CIR_PLLRDYC             (1 << 20) /* Bit 20: PLL Ready Interrupt Clear */
#define RCC_CIR_HSI14RDYC           (1 << 21) /* Bit 21: HSI14 Ready Interrupt Clear */
#define RCC_CIR_HSI48RDYC           (1 << 22) /* Bit 22: HSI48 Ready Interrupt Clear */
#define RCC_CIR_CSSC                (1 << 23) /* Bit 23: Clock Security System Interrupt Clear */

/* APB2 Peripheral reset register */

#define RCC_APB2RSTR_SYSCFGRST      (1 << 0)  /* Bit 0: SYSCFG reset */
#define RCC_APB2RSTR_USART6RST      (1 << 5)  /* Bit 5: USART6 reset */
#define RCC_APB2RSTR_USART7RST      (1 << 6)  /* Bit 6: USART7 reset */
#define RCC_APB2RSTR_USART8RST      (1 << 7)  /* Bit 7: USART8 reset */
#define RCC_APB2RSTR_ADC1RST        (1 << 9)  /* Bit 9: ADC1 interface reset */
#define RCC_APB2RSTR_TIM1RST        (1 << 11) /* Bit 11: TIM1 Timer reset */
#define RCC_APB2RSTR_SPI1RST        (1 << 12) /* Bit 12: SPI 1 reset */
#define RCC_APB2RSTR_USART1RST      (1 << 14) /* Bit 14: USART1 reset */
#define RCC_APB2RSTR_TIM15RST       (1 << 16) /* Bit 16: TIM15 reset */
#define RCC_APB2RSTR_TIM16RST       (1 << 17) /* Bit 17: TIM16 reset */
#define RCC_APB2RSTR_TIM17RST       (1 << 18) /* Bit 18: TIM17 reset */
#define RCC_APB2RSTR_DBGMCURST      (1 << 22) /* Bit 22: Debug MCU reset */

/* APB1 Peripheral reset register */

#define RCC_APB1RSTR_TIM2RST        (1 << 0)  /* Bit 0: Timer 2 reset */
#define RCC_APB1RSTR_TIM3RST        (1 << 1)  /* Bit 1: Timer 3 reset */
#define RCC_APB1RSTR_TIM6RST        (1 << 4)  /* Bit 4: Timer 6 reset */
#define RCC_APB1RSTR_TIM7RST        (1 << 5)  /* Bit 5: Timer 7 reset */
#define RCC_APB1RSTR_TIM14RST       (1 << 8)  /* Bit 8: TIM14 reset */
#define RCC_APB1RSTR_WWDGRST        (1 << 11) /* Bit 11: Window Watchdog reset */
#define RCC_APB1RSTR_SPI2RST        (1 << 14) /* Bit 14: SPI 2 reset */
#define RCC_APB1RSTR_USART2RST      (1 << 17) /* Bit 17: USART 2 reset */
#define RCC_APB1RSTR_USART3RST      (1 << 18) /* Bit 18: USART 3 reset */
#define RCC_APB1RSTR_USART4RST      (1 << 19) /* Bit 19: USART 4 reset */
#define RCC_APB1RSTR_USART5RST      (1 << 20) /* Bit 20: USART 5 reset */
#define RCC_APB1RSTR_I2C1RST        (1 << 21) /* Bit 21: I2C 1 reset */
#define RCC_APB1RSTR_I2C2RST        (1 << 22) /* Bit 22: I2C 2 reset */
#define RCC_APB1RSTR_USBRST         (1 << 23) /* Bit 23: USB reset */
#define RCC_APB1RSTR_CAN1RST        (1 << 25) /* Bit 25: CAN1 reset */
#define RCC_APB1RSTR_CRSRST         (1 << 27) /* Bit 27: CRS / Backup interface reset */
#define RCC_APB1RSTR_PWRRST         (1 << 28) /* Bit 28: Power interface reset */
#define RCC_APB1RSTR_DAC1RST        (1 << 29) /* Bit 29: DAC1 interface reset */
#define RCC_APB1RSTR_CECRST         (1 << 30) /* Bit 30: CEC reset */

/* AHB Peripheral Clock enable register */

#define RCC_AHBENR_DMA1EN           (1 << 0)  /* Bit 0: DMA1 clock enable */
#define RCC_AHBENR_DMA2EN           (1 << 1)  /* Bit 1: DMA2 clock enable */
#define RCC_AHBENR_SRAMEN           (1 << 2)  /* Bit 2: SRAM interface clock enable */
#define RCC_AHBENR_FLITFEN          (1 << 4)  /* Bit 4: FLITF clock enable */
#define RCC_AHBENR_CRCEN            (1 << 6)  /* Bit 6: CRC clock enable */
#define RCC_AHBENR_IOPAEN           (1 << 17) /* Bit 17: I/O port A clock enable  */
#define RCC_AHBENR_IOPBEN           (1 << 18) /* Bit 18: I/O port B clock enable  */
#define RCC_AHBENR_IOPCEN           (1 << 19) /* Bit 19: I/O port C clock enable  */
#define RCC_AHBENR_IOPDEN           (1 << 20) /* Bit 20: I/O port D clock enable  */
#define RCC_AHBENR_IOPEEN           (1 << 21) /* Bit 21: I/O port E clock enable  */
#define RCC_AHBENR_IOPFEN           (1 << 22) /* Bit 22: I/O port F clock enable  */
#define RCC_AHBENR_TSCEN            (1 << 24) /* Bit 24: Touch sensing controller clock enable */

/* APB2 Peripheral Clock enable register */

#define RCC_APB2ENR_SYSCFGCOMPEN    (1 << 0)  /* Bit 0: SYSCFG & COMP clock enable */
#define RCC_APB2ENR_USART6EN        (1 << 5)  /* Bit 5: USART6 clock enable */
#define RCC_APB2ENR_USART7EN        (1 << 6)  /* Bit 6: USART7 clock enable */
#define RCC_APB2ENR_USART8EN        (1 << 7)  /* Bit 7: USART8 & COMP clock enable */
#define RCC_APB2ENR_ADC1EN          (1 << 9)  /* Bit 10: ADC1 interface clock enable */
#define RCC_APB2ENR_TIM1EN          (1 << 11) /* Bit 11: TIM1 Timer clock enable */
#define RCC_APB2ENR_SPI1EN          (1 << 12) /* Bit 12: SPI 1 clock enable */
#define RCC_APB2ENR_USART1EN        (1 << 14) /* Bit 14: USART1 clock enable */
#define RCC_APB2ENR_TIM15EN         (1 << 16) /* Bit 16: TIM15 clock enable */
#define RCC_APB2ENR_TIM16EN         (1 << 17) /* Bit 17: TIM16 clock enable */
#define RCC_APB2ENR_TIM17EN         (1 << 18) /* Bit 18: TIM17 clock enable */
#define RCC_APB2ENR_DBGMCUEN        (1 << 22) /* Bit 18: Debug MCU clock enable */

/* APB1 Peripheral Clock enable register */

#define RCC_APB1ENR_TIM2EN          (1 << 0)  /* Bit 0: Timer 2 clock enable */
#define RCC_APB1ENR_TIM3EN          (1 << 1)  /* Bit 1: Timer 3 clock enable */
#define RCC_APB1ENR_TIM4EN          (1 << 2)  /* Bit 2: Timer 4 clock enable */
#define RCC_APB1ENR_TIM6EN          (1 << 4)  /* Bit 4: Timer 6 clock enable */
#define RCC_APB1ENR_TIM7EN          (1 << 5)  /* Bit 5: Timer 7 clock enable */
#define RCC_APB1ENR_TIM14EN         (1 << 8)  /* Bit 8: Timer 14 clock enable */
#define RCC_APB1ENR_WWDGEN          (1 << 11) /* Bit 11: Window Watchdog clock enable */
#define RCC_APB1ENR_SPI2EN          (1 << 14) /* Bit 14: SPI 2 clock enable */
#define RCC_APB1ENR_USART2EN        (1 << 17) /* Bit 17: USART 2 clock enable */
#define RCC_APB1ENR_USART3EN        (1 << 18) /* Bit 18: USART 3 clock enable */
#define RCC_APB1ENR_USART4EN        (1 << 19) /* Bit 19: USART 4 clock enable */
#define RCC_APB1ENR_USART5EN        (1 << 20) /* Bit 20: USART 5 clock enable */
#define RCC_APB1ENR_I2C1EN          (1 << 21) /* Bit 21: I2C 1 clock enable */
#define RCC_APB1ENR_I2C2EN          (1 << 22) /* Bit 22: I2C 2 clock enable */
#define RCC_APB1ENR_USBEN           (1 << 23) /* Bit 23: USB clock enable */
#define RCC_APB1ENR_CAN1EN          (1 << 25) /* Bit 25: CAN 1 clock enable */
#define RCC_APB1ENR_CRSEN           (1 << 27) /* Bit 27: CRS / Backup interface clock enable */
#define RCC_APB1ENR_PWREN           (1 << 28) /* Bit 28: Power interface clock enable */
#define RCC_APB1ENR_DAC1EN          (1 << 29) /* Bit 29: DAC1 interface clock enable */
#define RCC_APB1ENR_CECEN           (1 << 30) /* Bit 30: CEC clock enable */

/* RTC domain control register */

#define RCC_BDCR_LSEON              (1 << 0)  /* Bit 0: External Low Speed oscillator enable */
#define RCC_BDCR_LSERDY             (1 << 1)  /* Bit 1: External Low Speed oscillator Ready */
#define RCC_BDCR_LSEBYP             (1 << 2)  /* Bit 2: External Low Speed oscillator Bypass */
#define RCC_BDCR_RTCSEL_SHIFT       (8)       /* Bits 9:8: RTC clock source selection */
#define RCC_BDCR_RTCSEL_MASK        (3 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_NOCLK     (0 << RCC_BDCR_RTCSEL_SHIFT) /* 00: No clock */
#  define RCC_BDCR_RTCSEL_LSE       (1 << RCC_BDCR_RTCSEL_SHIFT) /* 01: LSE oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_LSI       (2 << RCC_BDCR_RTCSEL_SHIFT) /* 10: LSI oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_HSE       (3 << RCC_BDCR_RTCSEL_SHIFT) /* 11: HSE oscillator clock divided by 128 used as RTC clock */

#define RCC_BDCR_RTCEN              (1 << 15) /* Bit 15: RTC clock enable */
#define RCC_BDCR_BDRST              (1 << 16) /* Bit 16: Backup domain software reset */

/* Control/status register */

#define RCC_CSR_LSION               (1 << 0)  /* Bit 0: Internal Low Speed oscillator enable */
#define RCC_CSR_LSIRDY              (1 << 1)  /* Bit 1: Internal Low Speed oscillator Ready */
#define RCC_CSR_V18PWRRSTF          (1 << 23) /* Bit 23: Reset flag of the 1.8V domain */
#define RCC_CSR_RMVF                (1 << 24) /* Bit 24: Remove reset flag */
#define RCC_CSR_OBLRSTF             (1 << 25) /* Bit 24: Option byte loader reset flag */
#define RCC_CSR_PINRSTF             (1 << 26) /* Bit 26: PIN reset flag */
#define RCC_CSR_PORRSTF             (1 << 27) /* Bit 27: POR/PDR reset flag */
#define RCC_CSR_SFTRSTF             (1 << 28) /* Bit 28: Software Reset flag */
#define RCC_CSR_IWDGRSTF            (1 << 29) /* Bit 29: Independent Watchdog reset flag */
#define RCC_CSR_WWDGRSTF            (1 << 30) /* Bit 30: Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF            (1 << 31) /* Bit 31: Low-Power reset flag */

/* AHB peripheral reset register */

#define RCC_AHBRSTR_IOPARST         (1 << 17) /* Bit 17: I/O port A reset */
#define RCC_AHBRSTR_IOPBRST         (1 << 18) /* Bit 18: I/O port B reset */
#define RCC_AHBRSTR_IOPCRST         (1 << 19) /* Bit 19: I/O port C reset */
#define RCC_AHBRSTR_IOPDRST         (1 << 20) /* Bit 20: I/O port D reset */
#define RCC_AHBRSTR_IOPERST         (1 << 21) /* Bit 21: I/O port E reset */
#define RCC_AHBRSTR_IOPFRST         (1 << 22) /* Bit 22: I/O port F reset */
#define RCC_AHBRSTR_TSCRST          (1 << 24) /* Bit 24: Touch sensing reset */

/* Clock configuration register 2 */

#define RCC_CFGR2_PREDIV1_SHIFT     (0)
#define RCC_CFGR2_PREDIV1_MASK      (0x0f << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d1       (0 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d2       (1 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d3       (2 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d4       (3 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d5       (4 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d6       (5 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d7       (6 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d8       (7 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d9       (8 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d10      (9 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d11      (10 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d12      (11 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d13      (12 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d14      (13 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d15      (14 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d16      (15 << RCC_CFGR2_PREDIV1_SHIFT)

/* Clock configuration register 3 */

#define RCC_CFGR3_USART1SW_SHIFT    (0)       /* Bits 0-1: USART1 clock source selection */
#define RCC_CFGR3_USART1SW_MASK     (3 << RCC_CFGR3_USART1SW_SHIFT)
#  define RCC_CFGR3_USART1SW_PCLK   (0 << RCC_CFGR3_USART1SW_SHIFT) /* PCLK is USART1 clock source */
#  define RCC_CFGR3_USART1SW_SYSCLK (1 << RCC_CFGR3_USART1SW_SHIFT) /* SYSCLK is USART1 clock */
#  define RCC_CFGR3_USART1SW_LSE    (2 << RCC_CFGR3_USART1SW_SHIFT) /* LSE is USART1 clock */
#  define RCC_CFGR3_USART1SW_HSI    (3 << RCC_CFGR3_USART1SW_SHIFT) /* HSI is USART1 clock */

#define RCC_CFGR3_CECSW             (1 << 6)  /* Bit 6: HDMI CEC clock source selection */
#define RCC_CFGR3_USBSW             (1 << 7)  /* Bit 7: USB clock source selection */
#define RCC_CFGR3_CLK48_HSI48       0
#define RCC_CFGR3_CLK48_PLL         (1 << 7)
#define RCC_CFGR3_ADCSW             (1 << 8)  /* Bit 8: ADC clock source selection */
#define RCC_CFGR3_USART2SW_SHIFT    (16)      /* Bits 16-17: USART2 clock source selection */
#define RCC_CFGR3_USART2SW_MASK     (3 << RCC_CFGR3_USART2SW_SHIFT)
#  define RCC_CFGR3_USART2SW_PCLK   (0 << RCC_CFGR3_USART2SW_SHIFT) /* PCLK is USART2 clock source */
#  define RCC_CFGR3_USART2SW_SYSCLK (1 << RCC_CFGR3_USART2SW_SHIFT) /* SYSCLK is USART2 clock */
#  define RCC_CFGR3_USART2SW_LSE    (2 << RCC_CFGR3_USART2SW_SHIFT) /* LSE is USART2 clock */
#  define RCC_CFGR3_USART2SW_HSI    (3 << RCC_CFGR3_USART2SW_SHIFT) /* HSI is USART2 clock */

#define RCC_CFGR3_USART3SW_SHIFT    (18)      /* Bits 18-19: USART3 clock source selection */
#define RCC_CFGR3_USART3SW_MASK     (3 << RCC_CFGR3_USART3SW_SHIFT)
#  define RCC_CFGR3_USART3SW_PCLK   (0 << RCC_CFGR3_USART3SW_SHIFT) /* PCLK is USART3 clock source */
#  define RCC_CFGR3_USART3SW_SYSCLK (1 << RCC_CFGR3_USART3SW_SHIFT) /* SYSCLK is USART3 clock */
#  define RCC_CFGR3_USART3SW_LSE    (2 << RCC_CFGR3_USART3SW_SHIFT) /* LSE is USART3 clock */
#  define RCC_CFGR3_USART3SW_HSI    (3 << RCC_CFGR3_USART3SW_SHIFT) /* HSI is USART3 clock */

/* Clock control register 2 */

#define RCC_CR2_HSI14ON             (1 << 0)  /* Bit 0: HSI14 clock enable */
#define RCC_CR2_HSI14RDY            (1 << 1)  /* Bit 1: HSI14 clock ready flag */
#define RCC_CR2_HSI14DIS            (1 << 2)  /* Bit 2: HSI14 clock request from ADC disable */
#define RCC_CR2_HSI14TRIM_SHIFT     (3)       /* Bits 3-7: HSI14 clock trimming */
#define RCC_CR2_HSI14TRIM_MASK      (31 << RCC_CR2_HSI14TRIM_SHIFT)
#define RCC_CR2_HSI14CAL_SHIFT      (8)       /* Bits 8-15: HSI14 clock calibration */
#define RCC_CR2_HSI14CAL_MASK       (0xff << RCC_CR2_HSI14CAL_SHIFT)
#define RCC_CR2_HSI48ON             (1 << 16) /* Bit 16: HSI48 clock enable */
#define RCC_CR2_HSI48RDY            (1 << 17) /* Bit 17: HSI48 clock ready flag */
#define RCC_CR2_HSI48CAL_SHIFT      (24)      /* Bits 24-31: HSI48 factory clock calibration */
#define RCC_CR2_HSI48CAL_MASK       (0xff << RCC_CR2_HSI48CAL_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32F0_RCC_H */
