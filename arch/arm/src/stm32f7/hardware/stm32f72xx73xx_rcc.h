/****************************************************************************
 * arch/arm/src/stm32f7/hardware/stm32f72xx73xx_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32F72XX73XX_RCC_H
#define __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32F72XX73XX_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_STM32F7_STM32F72XX) || defined(CONFIG_STM32F7_STM32F73XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_RCC_CR_OFFSET          0x0000  /* Clock control register */
#define STM32_RCC_PLLCFG_OFFSET      0x0004  /* PLL configuration register */
#define STM32_RCC_CFGR_OFFSET        0x0008  /* Clock configuration register */
#define STM32_RCC_CIR_OFFSET         0x000c  /* Clock interrupt register */
#define STM32_RCC_AHB1RSTR_OFFSET    0x0010  /* AHB1 peripheral reset register */
#define STM32_RCC_AHB2RSTR_OFFSET    0x0014  /* AHB2 peripheral reset register */
#define STM32_RCC_AHB3RSTR_OFFSET    0x0018  /* AHB3 peripheral reset register */
#define STM32_RCC_APB1RSTR_OFFSET    0x0020  /* APB1 Peripheral reset register */
#define STM32_RCC_APB2RSTR_OFFSET    0x0024  /* APB2 Peripheral reset register */
#define STM32_RCC_AHB1ENR_OFFSET     0x0030  /* AHB1 Peripheral Clock enable register */
#define STM32_RCC_AHB2ENR_OFFSET     0x0034  /* AHB2 Peripheral Clock enable register */
#define STM32_RCC_AHB3ENR_OFFSET     0x0038  /* AHB3 Peripheral Clock enable register */
#define STM32_RCC_APB1ENR_OFFSET     0x0040  /* APB1 Peripheral Clock enable register */
#define STM32_RCC_APB2ENR_OFFSET     0x0044  /* APB2 Peripheral Clock enable register */
#define STM32_RCC_AHB1LPENR_OFFSET   0x0050  /* RCC AHB1 low power mode peripheral clock enable register */
#define STM32_RCC_AHB2LPENR_OFFSET   0x0054  /* RCC AHB2 low power mode peripheral clock enable register */
#define STM32_RCC_AHB3LPENR_OFFSET   0x0058  /* RCC AHB3 low power mode peripheral clock enable register */
#define STM32_RCC_APB1LPENR_OFFSET   0x0060  /* RCC APB1 low power mode peripheral clock enable register */
#define STM32_RCC_APB2LPENR_OFFSET   0x0064  /* RCC APB2 low power mode peripheral clock enable register */
#define STM32_RCC_BDCR_OFFSET        0x0070  /* Backup domain control register */
#define STM32_RCC_CSR_OFFSET         0x0074  /* Control/status register */
#define STM32_RCC_SSCGR_OFFSET       0x0080  /* Spread spectrum clock generation register */
#define STM32_RCC_PLLI2SCFGR_OFFSET  0x0084  /* PLLI2S configuration register */
#define STM32_RCC_PLLSAICFGR_OFFSET  0x0088  /* PLLSAI configuration register */
#define STM32_RCC_DCKCFGR1_OFFSET    0x008c  /* Dedicated clocks configuration register 1 */
#define STM32_RCC_DCKCFGR2_OFFSET    0x0090  /* Dedicated clocks configuration register 2 */

/* Register Addresses *******************************************************/

#define STM32_RCC_CR                (STM32_RCC_BASE+STM32_RCC_CR_OFFSET)
#define STM32_RCC_PLLCFG            (STM32_RCC_BASE+STM32_RCC_PLLCFG_OFFSET)
#define STM32_RCC_CFGR              (STM32_RCC_BASE+STM32_RCC_CFGR_OFFSET)
#define STM32_RCC_CIR               (STM32_RCC_BASE+STM32_RCC_CIR_OFFSET)
#define STM32_RCC_AHB1RSTR          (STM32_RCC_BASE+STM32_RCC_AHB1RSTR_OFFSET)
#define STM32_RCC_AHB2RSTR          (STM32_RCC_BASE+STM32_RCC_AHB2RSTR_OFFSET)
#define STM32_RCC_AHB3RSTR          (STM32_RCC_BASE+STM32_RCC_AHB3RSTR_OFFSET)
#define STM32_RCC_APB1RSTR          (STM32_RCC_BASE+STM32_RCC_APB1RSTR_OFFSET)
#define STM32_RCC_APB2RSTR          (STM32_RCC_BASE+STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_AHB1ENR           (STM32_RCC_BASE+STM32_RCC_AHB1ENR_OFFSET)
#define STM32_RCC_AHB2ENR           (STM32_RCC_BASE+STM32_RCC_AHB2ENR_OFFSET)
#define STM32_RCC_AHB3ENR           (STM32_RCC_BASE+STM32_RCC_AHB3ENR_OFFSET)
#define STM32_RCC_APB1ENR           (STM32_RCC_BASE+STM32_RCC_APB1ENR_OFFSET)
#define STM32_RCC_APB2ENR           (STM32_RCC_BASE+STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_AHB1LPENR         (STM32_RCC_BASE+STM32_RCC_AHB1LPENR_OFFSET)
#define STM32_RCC_AHB2LPENR         (STM32_RCC_BASE+STM32_RCC_AHB2LPENR_OFFSET)
#define STM32_RCC_AHB3LPENR         (STM32_RCC_BASE+STM32_RCC_AHB3LPENR_OFFSET)
#define STM32_RCC_APB1LPENR         (STM32_RCC_BASE+STM32_RCC_APB1LPENR_OFFSET)
#define STM32_RCC_APB2LPENR         (STM32_RCC_BASE+STM32_RCC_APB2LPENR_OFFSET)
#define STM32_RCC_BDCR              (STM32_RCC_BASE+STM32_RCC_BDCR_OFFSET)
#define STM32_RCC_CSR               (STM32_RCC_BASE+STM32_RCC_CSR_OFFSET)
#define STM32_RCC_SSCGR             (STM32_RCC_BASE+STM32_RCC_SSCGR_OFFSET)
#define STM32_RCC_PLLI2SCFGR        (STM32_RCC_BASE+STM32_RCC_PLLI2SCFGR_OFFSET)
#define STM32_RCC_PLLSAICFGR        (STM32_RCC_BASE+STM32_RCC_PLLSAICFGR_OFFSET)
#define STM32_RCC_DCKCFGR1          (STM32_RCC_BASE+STM32_RCC_DCKCFGR1_OFFSET)
#define STM32_RCC_DCKCFGR2          (STM32_RCC_BASE+STM32_RCC_DCKCFGR2_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Clock control register */

#define RCC_CR_HSION                 (1 << 0)  /* Bit 0: Internal High Speed clock enable */
#define RCC_CR_HSIRDY                (1 << 1)  /* Bit 1: Internal High Speed clock ready flag */
#define RCC_CR_HSITRIM_SHIFT         (3)       /* Bits 7-3: Internal High Speed clock trimming */
#define RCC_CR_HSITRIM_MASK          (0x1f << RCC_CR_HSITRIM_SHIFT)
#define RCC_CR_HSICAL_SHIFT          (8)       /* Bits 15-8: Internal High Speed clock Calibration */
#define RCC_CR_HSICAL_MASK           (0xff << RCC_CR_HSICAL_SHIFT)
#define RCC_CR_HSEON                 (1 << 16) /* Bit 16: External High Speed clock enable */
#define RCC_CR_HSERDY                (1 << 17) /* Bit 17: External High Speed clock ready flag */
#define RCC_CR_HSEBYP                (1 << 18) /* Bit 18: External High Speed clock Bypass */
#define RCC_CR_CSSON                 (1 << 19) /* Bit 19: Clock Security System enable */
#define RCC_CR_PLLON                 (1 << 24) /* Bit 24: PLL enable */
#define RCC_CR_PLLRDY                (1 << 25) /* Bit 25: PLL clock ready flag */
#define RCC_CR_PLLI2SON              (1 << 26) /* Bit 26: PLLI2S enable */
#define RCC_CR_PLLI2SRDY             (1 << 27) /* Bit 27: PLLI2S clock ready flag */
#define RCC_CR_PLLSAION              (1 << 28) /* Bit 28: PLLSAI enable */
#define RCC_CR_PLLSAIRDY             (1 << 29) /* Bit 29: PLLSAI clock ready flag */

/* PLL configuration register */

#define RCC_PLLCFG_PLLM_SHIFT        (0)      /* Bits 0-5: Main PLL (PLL) and audio PLL (PLLI2S)
                                               * input clock divider */
#define RCC_PLLCFG_PLLM_MASK         (0x3f << RCC_PLLCFG_PLLM_SHIFT)
#  define RCC_PLLCFG_PLLM(n)         ((n) << RCC_PLLCFG_PLLM_SHIFT) /* n = 2..63 */

#define RCC_PLLCFG_PLLN_SHIFT        (6)      /* Bits 6-14: Main PLL (PLL) VCO multiplier */
#define RCC_PLLCFG_PLLN_MASK         (0x1ff << RCC_PLLCFG_PLLN_SHIFT)
#  define RCC_PLLCFG_PLLN(n)         ((n) << RCC_PLLCFG_PLLN_SHIFT) /* n = 2..432 */

#define RCC_PLLCFG_PLLP_SHIFT        (16)      /* Bits 16-17: Main PLL (PLL) main system clock divider */
#define RCC_PLLCFG_PLLP_MASK         (3 << RCC_PLLCFG_PLLP_SHIFT)
#  define RCC_PLLCFG_PLLP(n)         ((((n)>>1)-1)<< RCC_PLLCFG_PLLP_SHIFT) /* n=2,4,6,8 */

#  define RCC_PLLCFG_PLLP_2          (0 << RCC_PLLCFG_PLLP_SHIFT) /* 00: PLLP = 2 */
#  define RCC_PLLCFG_PLLP_4          (1 << RCC_PLLCFG_PLLP_SHIFT) /* 01: PLLP = 4 */
#  define RCC_PLLCFG_PLLP_6          (2 << RCC_PLLCFG_PLLP_SHIFT) /* 10: PLLP = 6 */
#  define RCC_PLLCFG_PLLP_8          (3 << RCC_PLLCFG_PLLP_SHIFT) /* 11: PLLP = 8 */

#define RCC_PLLCFG_PLLSRC            (1 << 22) /* Bit 22: Main PLL(PLL) and audio PLL (PLLI2S)
                                                * entry clock source */
#  define RCC_PLLCFG_PLLSRC_HSI      (0)
#  define RCC_PLLCFG_PLLSRC_HSE      RCC_PLLCFG_PLLSRC
#define RCC_PLLCFG_PLLQ_SHIFT        (24)      /* Bits 24-27: Main PLL (PLL) divider
                                                * (USB OTG FS, SDIO and RNG clocks) */
#define RCC_PLLCFG_PLLQ_MASK         (15 << RCC_PLLCFG_PLLQ_SHIFT)
#  define RCC_PLLCFG_PLLQ(n)         ((n) << RCC_PLLCFG_PLLQ_SHIFT) /* n=2..15 */

#define RCC_PLLCFG_RESET             (0x24003010) /* PLLCFG reset value */

/* Clock configuration register */

#define RCC_CFGR_SW_SHIFT            (0)       /* Bits 0-1: System clock Switch */
#define RCC_CFGR_SW_MASK             (3 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_HSI            (0 << RCC_CFGR_SW_SHIFT) /* 00: HSI selected as system clock */
#  define RCC_CFGR_SW_HSE            (1 << RCC_CFGR_SW_SHIFT) /* 01: HSE selected as system clock */
#  define RCC_CFGR_SW_PLL            (2 << RCC_CFGR_SW_SHIFT) /* 10: PLL selected as system clock */

#define RCC_CFGR_SWS_SHIFT           (2)       /* Bits 2-3: System Clock Switch Status */
#define RCC_CFGR_SWS_MASK            (3 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_HSI           (0 << RCC_CFGR_SWS_SHIFT) /* 00: HSI oscillator used as system clock */
#  define RCC_CFGR_SWS_HSE           (1 << RCC_CFGR_SWS_SHIFT) /* 01: HSE oscillator used as system clock */
#  define RCC_CFGR_SWS_PLL           (2 << RCC_CFGR_SWS_SHIFT) /* 10: PLL used as system clock */

#define RCC_CFGR_HPRE_SHIFT          (4)       /* Bits 4-7: AHB prescaler */
#define RCC_CFGR_HPRE_MASK           (0x0f << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLK       (0 << RCC_CFGR_HPRE_SHIFT)  /* 0xxx: SYSCLK not divided */
#  define RCC_CFGR_HPRE_SYSCLKd2     (8 << RCC_CFGR_HPRE_SHIFT)  /* 1000: SYSCLK divided by 2 */
#  define RCC_CFGR_HPRE_SYSCLKd4     (9 << RCC_CFGR_HPRE_SHIFT)  /* 1001: SYSCLK divided by 4 */
#  define RCC_CFGR_HPRE_SYSCLKd8     (10 << RCC_CFGR_HPRE_SHIFT) /* 1010: SYSCLK divided by 8 */
#  define RCC_CFGR_HPRE_SYSCLKd16    (11 << RCC_CFGR_HPRE_SHIFT) /* 1011: SYSCLK divided by 16 */
#  define RCC_CFGR_HPRE_SYSCLKd64    (12 << RCC_CFGR_HPRE_SHIFT) /* 1100: SYSCLK divided by 64 */
#  define RCC_CFGR_HPRE_SYSCLKd128   (13 << RCC_CFGR_HPRE_SHIFT) /* 1101: SYSCLK divided by 128 */
#  define RCC_CFGR_HPRE_SYSCLKd256   (14 << RCC_CFGR_HPRE_SHIFT) /* 1110: SYSCLK divided by 256 */
#  define RCC_CFGR_HPRE_SYSCLKd512   (15 << RCC_CFGR_HPRE_SHIFT) /* 1111: SYSCLK divided by 512 */

#define RCC_CFGR_PPRE1_SHIFT         (10)      /* Bits 10-12: APB Low speed prescaler  (APB1) */
#define RCC_CFGR_PPRE1_MASK          (7 << RCC_CFGR_PPRE1_SHIFT)
#  define RCC_CFGR_PPRE1_HCLK        (0 << RCC_CFGR_PPRE1_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE1_HCLKd2      (4 << RCC_CFGR_PPRE1_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE1_HCLKd4      (5 << RCC_CFGR_PPRE1_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE1_HCLKd8      (6 << RCC_CFGR_PPRE1_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE1_HCLKd16     (7 << RCC_CFGR_PPRE1_SHIFT) /* 111: HCLK divided by 16 */

#define RCC_CFGR_PPRE2_SHIFT         (13)      /* Bits 13-15: APB High speed prescaler (APB2) */
#define RCC_CFGR_PPRE2_MASK          (7 << RCC_CFGR_PPRE2_SHIFT)
#  define RCC_CFGR_PPRE2_HCLK        (0 << RCC_CFGR_PPRE2_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE2_HCLKd2      (4 << RCC_CFGR_PPRE2_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE2_HCLKd4      (5 << RCC_CFGR_PPRE2_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE2_HCLKd8      (6 << RCC_CFGR_PPRE2_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE2_HCLKd16     (7 << RCC_CFGR_PPRE2_SHIFT) /* 111: HCLK divided by 16 */

#define RCC_CFGR_RTCPRE_SHIFT        (16)      /* Bits 16-20: APB High speed prescaler (APB2) */
#define RCC_CFGR_RTCPRE_MASK         (31 << RCC_CFGR_RTCPRE_SHIFT)
#  define RCC_CFGR_RTCPRE(n)         ((n) << RCC_CFGR_RTCPRE_SHIFT) /* HSE/n, n=1..31 */

#define RCC_CFGR_MCO1_SHIFT          (21)      /* Bits 21-22: Microcontroller Clock Output */
#define RCC_CFGR_MCO1_MASK           (3 << RCC_CFGR_MCO1_SHIFT)
#  define RCC_CFGR_MCO1_HSI          (0 << RCC_CFGR_MCO1_SHIFT) /* 00: HSI clock selected */
#  define RCC_CFGR_MCO1_LSE          (1 << RCC_CFGR_MCO1_SHIFT) /* 01: LSE oscillator selected */
#  define RCC_CFGR_MCO1_HSE          (2 << RCC_CFGR_MCO1_SHIFT) /* 10: HSE oscillator clock selected */
#  define RCC_CFGR_MCO1_PLL          (3 << RCC_CFGR_MCO1_SHIFT) /* 11: PLL clock selected */

#define RCC_CFGR_I2SSRC              (1 << 23) /* Bit 23: I2S clock selection */
#define RCC_CFGR_MCO1PRE_SHIFT       (24)      /* Bits 24-26: MCO1 prescaler */
#define RCC_CFGR_MCO1PRE_MASK        (7 << RCC_CFGR_MCO1PRE_SHIFT)
#  define RCC_CFGR_MCO1PRE_NONE      (0 << RCC_CFGR_MCO1PRE_SHIFT) /* 0xx: no division */
#  define RCC_CFGR_MCO1PRE_DIV2      (4 << RCC_CFGR_MCO1PRE_SHIFT) /* 100: division by 2 */
#  define RCC_CFGR_MCO1PRE_DIV3      (5 << RCC_CFGR_MCO1PRE_SHIFT) /* 101: division by 3 */
#  define RCC_CFGR_MCO1PRE_DIV4      (6 << RCC_CFGR_MCO1PRE_SHIFT) /* 110: division by 4 */
#  define RCC_CFGR_MCO1PRE_DIV5      (7 << RCC_CFGR_MCO1PRE_SHIFT) /* 111: division by 5 */

#define RCC_CFGR_MCO2PRE_SHIFT       (27)      /* Bits 27-29: MCO2 prescaler */
#define RCC_CFGR_MCO2PRE_MASK        (7 << RCC_CFGR_MCO2PRE_SHIFT)
#  define RCC_CFGR_MCO2PRE_NONE      (0 << RCC_CFGR_MCO2PRE_SHIFT) /* 0xx: no division */
#  define RCC_CFGR_MCO2PRE_DIV2      (4 << RCC_CFGR_MCO2PRE_SHIFT) /* 100: division by 2 */
#  define RCC_CFGR_MCO2PRE_DIV3      (5 << RCC_CFGR_MCO2PRE_SHIFT) /* 101: division by 3 */
#  define RCC_CFGR_MCO2PRE_DIV4      (6 << RCC_CFGR_MCO2PRE_SHIFT) /* 110: division by 4 */
#  define RCC_CFGR_MCO2PRE_DIV5      (7 << RCC_CFGR_MCO2PRE_SHIFT) /* 111: division by 5 */

#define RCC_CFGR_MCO2_SHIFT          (30)      /* Bits 30-31: Microcontroller clock output 2 */
#define RCC_CFGR_MCO2_MASK           (3 << RCC_CFGR_MCO2_SHIFT)
#  define RCC_CFGR_MCO2_SYSCLK       (0 << RCC_CFGR_MCO2_SHIFT) /* 00: System clock  (SYSCLK) selected */
#  define RCC_CFGR_MCO2_PLLI2S       (1 << RCC_CFGR_MCO2_SHIFT) /* 01: PLLI2S clock selected */
#  define RCC_CFGR_MCO2_HSE          (2 << RCC_CFGR_MCO2_SHIFT) /* 10: HSE oscillator clock selected */
#  define RCC_CFGR_MCO2_PLL          (3 << RCC_CFGR_MCO2_SHIFT) /* 11: PLL clock selected */

/* Clock interrupt register */

#define RCC_CIR_LSIRDYF              (1 << 0)  /* Bit 0: LSI Ready Interrupt flag */
#define RCC_CIR_LSERDYF              (1 << 1)  /* Bit 1: LSE Ready Interrupt flag */
#define RCC_CIR_HSIRDYF              (1 << 2)  /* Bit 2: HSI Ready Interrupt flag */
#define RCC_CIR_HSERDYF              (1 << 3)  /* Bit 3: HSE Ready Interrupt flag */
#define RCC_CIR_PLLRDYF              (1 << 4)  /* Bit 4: PLL Ready Interrupt flag */
#define RCC_CIR_PLLI2SRDYF           (1 << 5)  /* Bit 5: PLLI2S Ready Interrupt flag */
#define RCC_CIR_PLLSAIRDYF           (1 << 6)  /* Bit 6: PLLSAI Ready Interrupt flag */
#define RCC_CIR_CSSF                 (1 << 7)  /* Bit 7: Clock Security System Interrupt flag */
#define RCC_CIR_LSIRDYIE             (1 << 8)  /* Bit 8: LSI Ready Interrupt Enable */
#define RCC_CIR_LSERDYIE             (1 << 9)  /* Bit 9: LSE Ready Interrupt Enable */
#define RCC_CIR_HSIRDYIE             (1 << 10) /* Bit 10: HSI Ready Interrupt Enable */
#define RCC_CIR_HSERDYIE             (1 << 11) /* Bit 11: HSE Ready Interrupt Enable */
#define RCC_CIR_PLLRDYIE             (1 << 12) /* Bit 12: PLL Ready Interrupt Enable */
#define RCC_CIR_PLLI2SRDYIE          (1 << 13) /* Bit 13: PLLI2S Ready Interrupt enable */
#define RCC_CIR_PLLSAIRDYIE          (1 << 14) /* Bit 14: PLLSAI Ready Interrupt enable */
#define RCC_CIR_LSIRDYC              (1 << 16) /* Bit 16: LSI Ready Interrupt Clear */
#define RCC_CIR_LSERDYC              (1 << 17) /* Bit 17: LSE Ready Interrupt Clear */
#define RCC_CIR_HSIRDYC              (1 << 18) /* Bit 18: HSI Ready Interrupt Clear */
#define RCC_CIR_HSERDYC              (1 << 19) /* Bit 19: HSE Ready Interrupt Clear */
#define RCC_CIR_PLLRDYC              (1 << 20) /* Bit 20: PLL Ready Interrupt Clear */
#define RCC_CIR_PLLI2SRDYC           (1 << 21) /* Bit 21: PLLI2S Ready Interrupt clear */
#define RCC_CIR_PLLSAIRDYC           (1 << 22) /* Bit 22: PLLSAI Ready Interrupt clear */
#define RCC_CIR_CSSC                 (1 << 23) /* Bit 23: Clock Security System Interrupt Clear */

/* AHB1 peripheral reset register */

#define RCC_AHB1RSTR_GPIOARST        (1 << 0)  /* Bit 0:  IO port A reset */
#define RCC_AHB1RSTR_GPIOBRST        (1 << 1)  /* Bit 1:  IO port B reset */
#define RCC_AHB1RSTR_GPIOCRST        (1 << 2)  /* Bit 2:  IO port C reset */
#define RCC_AHB1RSTR_GPIODRST        (1 << 3)  /* Bit 3:  IO port D reset */
#define RCC_AHB1RSTR_GPIOERST        (1 << 4)  /* Bit 4:  IO port E reset */
#define RCC_AHB1RSTR_GPIOFRST        (1 << 5)  /* Bit 5:  IO port F reset */
#define RCC_AHB1RSTR_GPIOGRST        (1 << 6)  /* Bit 6:  IO port G reset */
#define RCC_AHB1RSTR_GPIOHRST        (1 << 7)  /* Bit 7:  IO port H reset */
#define RCC_AHB1RSTR_GPIOIRST        (1 << 8)  /* Bit 8:  IO port I reset */
#define RCC_AHB1RSTR_CRCRST          (1 << 12) /* Bit 12  CRC reset */
#define RCC_AHB1RSTR_DMA1RST         (1 << 21) /* Bit 21: DMA1 reset */
#define RCC_AHB1RSTR_DMA2RST         (1 << 22) /* Bit 22: DMA2 reset */
#define RCC_AHB1RSTR_DMA2DRST        (1 << 23) /* Bit 23: DMA2D reset */
#define RCC_AHB1RSTR_OTGHSRST        (1 << 29) /* Bit 29: USB OTG HS module reset */

/* AHB2 peripheral reset register */

#define RCC_AHB2RSTR_CRYPRST         (1 << 4)  /* Bit 4: Cryptographic module reset */
#define RCC_AHB2RSTR_RNGRST          (1 << 6)  /* Bit 6: Random number generator module reset */
#define RCC_AHB2RSTR_OTGFSRST        (1 << 7)  /* Bit 7: USB OTG FS module reset */

/* AHB3 peripheral reset register */

#define RCC_AHB3RSTR_FMCRST          (1 << 0)  /* Bit 0: Flexible static memory controller module reset */
#define RCC_AHB3RSTR_QSPIRST         (1 << 1)  /* Bit 1: Quad SPI memory controller reset */

/* APB1 Peripheral reset register */

#define RCC_APB1RSTR_TIM2RST         (1 << 0)  /* Bit 0:  TIM2 reset */
#define RCC_APB1RSTR_TIM3RST         (1 << 1)  /* Bit 1:  TIM3 reset */
#define RCC_APB1RSTR_TIM4RST         (1 << 2)  /* Bit 2:  TIM4 reset */
#define RCC_APB1RSTR_TIM5RST         (1 << 3)  /* Bit 3:  TIM5 reset */
#define RCC_APB1RSTR_TIM6RST         (1 << 4)  /* Bit 4:  TIM6 reset */
#define RCC_APB1RSTR_TIM7RST         (1 << 5)  /* Bit 5:  TIM7 reset */
#define RCC_APB1RSTR_TIM12RST        (1 << 6)  /* Bit 6:  TIM12 reset */
#define RCC_APB1RSTR_TIM13RST        (1 << 7)  /* Bit 7:  TIM13 reset */
#define RCC_APB1RSTR_TIM14RST        (1 << 8)  /* Bit 8:  TIM14 reset */
#define RCC_APB1RSTR_LPTIM1RST       (1 << 9)  /* Bit 9:  LPTIM1 reset */
#define RCC_APB1RSTR_WWDGRST         (1 << 11) /* Bit 11: Window watchdog reset */
#define RCC_APB1RSTR_SPI2RST         (1 << 14) /* Bit 14: SPI 2 reset */
#define RCC_APB1RSTR_SPI3RST         (1 << 15) /* Bit 15: SPI 3 reset */
#define RCC_APB1RSTR_USART2RST       (1 << 17) /* Bit 17: USART 2 reset */
#define RCC_APB1RSTR_USART3RST       (1 << 18) /* Bit 18: USART 3 reset */
#define RCC_APB1RSTR_UART4RST        (1 << 19) /* Bit 19: UART 4 reset */
#define RCC_APB1RSTR_UART5RST        (1 << 20) /* Bit 20: UART 5 reset */
#define RCC_APB1RSTR_I2C1RST         (1 << 21) /* Bit 21: I2C 1 reset */
#define RCC_APB1RSTR_I2C2RST         (1 << 22) /* Bit 22: I2C 2 reset */
#define RCC_APB1RSTR_I2C3RST         (1 << 23) /* Bit 23: I2C 3 reset */
#define RCC_APB1RSTR_CAN1RST         (1 << 25) /* Bit 25: CAN1 reset */
#define RCC_APB1RSTR_PWRRST          (1 << 28) /* Bit 28: Power interface reset */
#define RCC_APB1RSTR_DACRST          (1 << 29) /* Bit 29: DAC reset */
#define RCC_APB1RSTR_UART7RST        (1 << 30) /* Bit 30: UART 7 reset */
#define RCC_APB1RSTR_UART8RST        (1 << 31) /* Bit 31: UART 8 reset */

/* APB2 Peripheral reset register */

#define RCC_APB2RSTR_TIM1RST         (1 << 0)  /* Bit 0:  TIM1 reset */
#define RCC_APB2RSTR_TIM8RST         (1 << 1)  /* Bit 1:  TIM8 reset */
#define RCC_APB2RSTR_USART1RST       (1 << 4)  /* Bit 4:  USART1 reset */
#define RCC_APB2RSTR_USART6RST       (1 << 5)  /* Bit 5:  USART6 reset */
#define RCC_APB2RSTR_SDMMC2RST       (1 << 7)  /* Bit 7:  SDMMC2 reset */
#define RCC_APB2RSTR_ADCRST          (1 << 8)  /* Bit 8:  ADC interface reset (common to all ADCs) */
#define RCC_APB2RSTR_SDMMC1RST       (1 << 11) /* Bit 11: SDMMC1 reset */
#define RCC_APB2RSTR_SPI1RST         (1 << 12) /* Bit 12: SPI1 reset */
#define RCC_APB2RSTR_SPI4RST         (1 << 13) /* Bit 13: SPI4 reset */
#define RCC_APB2RSTR_SYSCFGRST       (1 << 14) /* Bit 14: System configuration controller reset */
#define RCC_APB2RSTR_TIM9RST         (1 << 16) /* Bit 16: TIM9 reset */
#define RCC_APB2RSTR_TIM10RST        (1 << 17) /* Bit 17: TIM10 reset */
#define RCC_APB2RSTR_TIM11RST        (1 << 18) /* Bit 18: TIM11 reset */
#define RCC_APB2RSTR_SPI5RST         (1 << 20) /* Bit 20: SPI 5 reset */
#define RCC_APB2RSTR_SAI1RST         (1 << 22) /* Bit 22: SAI 1 reset */
#define RCC_APB2RSTR_SAI2RST         (1 << 23) /* Bit 23: SAI 2 reset */
#define RCC_APB2RSTR_OTGPHYCRST      (1 << 31) /* Bit 31: OTGPHYC reset */

/* AHB1 Peripheral Clock enable register */

#define RCC_AHB1ENR_GPIOEN(n)        (1 << (n))
#define RCC_AHB1ENR_GPIOAEN          (1 << 0)  /* Bit 0:  IO port A clock enable */
#define RCC_AHB1ENR_GPIOBEN          (1 << 1)  /* Bit 1:  IO port B clock enable */
#define RCC_AHB1ENR_GPIOCEN          (1 << 2)  /* Bit 2:  IO port C clock enable */
#define RCC_AHB1ENR_GPIODEN          (1 << 3)  /* Bit 3:  IO port D clock enable */
#define RCC_AHB1ENR_GPIOEEN          (1 << 4)  /* Bit 4:  IO port E clock enable */
#define RCC_AHB1ENR_GPIOFEN          (1 << 5)  /* Bit 5:  IO port F clock enable */
#define RCC_AHB1ENR_GPIOGEN          (1 << 6)  /* Bit 6:  IO port G clock enable */
#define RCC_AHB1ENR_GPIOHEN          (1 << 7)  /* Bit 7:  IO port H clock enable */
#define RCC_AHB1ENR_GPIOIEN          (1 << 8)  /* Bit 8:  IO port I clock enable */
#define RCC_AHB1ENR_CRCEN            (1 << 12) /* Bit 12: CRC clock enable */
#define RCC_AHB1ENR_BKPSRAMEN        (1 << 18) /* Bit 18: Backup SRAM interface clock enable */
#define RCC_AHB1ENR_DTCMRAMEN        (1 << 20) /* Bit 20: DTCM RAM clock enable */
#define RCC_AHB1ENR_DMA1EN           (1 << 21) /* Bit 21: DMA1 clock enable */
#define RCC_AHB1ENR_DMA2EN           (1 << 22) /* Bit 22: DMA2 clock enable */
#define RCC_AHB1ENR_OTGHSEN          (1 << 29) /* Bit 29: USB OTG HS clock enable */
#define RCC_AHB1ENR_OTGHSULPIEN      (1 << 30) /* Bit 30: USB OTG HSULPI clock enable */

/* AHB2 Peripheral Clock enable register */

#define RCC_AHB2ENR_CRYPEN           (1 << 4)  /* Bit 4: Cryptographic modules clock enable */
#define RCC_AHB2ENR_RNGEN            (1 << 6)  /* Bit 6: Random number generator clock enable */
#define RCC_AHB2ENR_OTGFSEN          (1 << 7)  /* Bit 7: USB OTG FS clock enable */

/* AHB3 Peripheral Clock enable register */

#define RCC_AHB3ENR_FMCEN            (1 << 0) /* Bit 0: Flexible static memory controller module clock enable */
#define RCC_AHB3ENR_QSPIEN           (1 << 1) /* Bit 1: Quad SPI memory controller clock enable */

/* APB1 Peripheral Clock enable register */

#define RCC_APB1ENR_TIM2EN           (1 << 0)  /* Bit 0:  TIM 2 clock enable */
#define RCC_APB1ENR_TIM3EN           (1 << 1)  /* Bit 1:  TIM 3 clock enable */
#define RCC_APB1ENR_TIM4EN           (1 << 2)  /* Bit 2:  TIM 4 clock enable */
#define RCC_APB1ENR_TIM5EN           (1 << 3)  /* Bit 3:  TIM 5 clock enable */
#define RCC_APB1ENR_TIM6EN           (1 << 4)  /* Bit 4:  TIM 6 clock enable */
#define RCC_APB1ENR_TIM7EN           (1 << 5)  /* Bit 5:  TIM 7 clock enable */
#define RCC_APB1ENR_TIM12EN          (1 << 6)  /* Bit 6:  TIM 12 clock enable */
#define RCC_APB1ENR_TIM13EN          (1 << 7)  /* Bit 7:  TIM 13 clock enable */
#define RCC_APB1ENR_TIM14EN          (1 << 8)  /* Bit 8:  TIM 14 clock enable */
#define RCC_APB1ENR_LPTIM1EN         (1 << 9)  /* Bit 9:  LPTIM 1 clock enable */
#define RCC_APB1ENR_RTCAPBEN         (1 << 10) /* Bit 10:  RTCAPB clock enable */
#define RCC_APB1ENR_WWDGEN           (1 << 11) /* Bit 11: Window watchdog clock enable */
#define RCC_APB1ENR_SPI2EN           (1 << 14) /* Bit 14: SPI 2 clock enable */
#define RCC_APB1ENR_SPI3EN           (1 << 15) /* Bit 15: SPI 3 clock enable */
#define RCC_APB1ENR_USART2EN         (1 << 17) /* Bit 17: USART 2 clock enable */
#define RCC_APB1ENR_USART3EN         (1 << 18) /* Bit 18: USART 3 clock enable */
#define RCC_APB1ENR_UART4EN          (1 << 19) /* Bit 19: UART 4 clock enable */
#define RCC_APB1ENR_UART5EN          (1 << 20) /* Bit 20: UART 5 clock enable */
#define RCC_APB1ENR_I2C1EN           (1 << 21) /* Bit 21: I2C 1 clock enable */
#define RCC_APB1ENR_I2C2EN           (1 << 22) /* Bit 22: I2C 2 clock enable */
#define RCC_APB1ENR_I2C3EN           (1 << 23) /* Bit 23: I2C 3 clock enable */
#define RCC_APB1ENR_CAN1EN           (1 << 25) /* Bit 25: CAN 1 clock enable */
#define RCC_APB1ENR_PWREN            (1 << 28) /* Bit 28: Power interface clock enable */
#define RCC_APB1ENR_DACEN            (1 << 29) /* Bit 29: DAC interface clock enable */
#define RCC_APB1ENR_UART7EN          (1 << 30) /* Bit 30: UART7 clock enable */
#define RCC_APB1ENR_UART8EN          (1 << 31) /* Bit 31: UART8 clock enable */

/* APB2 Peripheral Clock enable register */

#define RCC_APB2ENR_TIM1EN           (1 << 0)  /* Bit 0:  TIM 1 clock enable */
#define RCC_APB2ENR_TIM8EN           (1 << 1)  /* Bit 1:  TIM 8 clock enable */
#define RCC_APB2ENR_USART1EN         (1 << 4)  /* Bit 4:  USART 1 clock enable */
#define RCC_APB2ENR_USART6EN         (1 << 5)  /* Bit 5:  USART 6 clock enable */
#define RCC_APB2ENR_SDMMC2EN         (1 << 7)  /* Bit 7:  SDMMC 2 clock enable */
#define RCC_APB2ENR_ADC1EN           (1 << 8)  /* Bit 8:  ADC 1 clock enable */
#define RCC_APB2ENR_ADC2EN           (1 << 9)  /* Bit 9:  ADC 2 clock enable */
#define RCC_APB2ENR_ADC3EN           (1 << 10) /* Bit 10: ADC 3 clock enable */
#define RCC_APB2ENR_SDMMC1EN         (1 << 11) /* Bit 11: SDMMC 1 clock enable */
#define RCC_APB2ENR_SPI1EN           (1 << 12) /* Bit 12: SPI 1 clock enable */
#define RCC_APB2ENR_SPI4EN           (1 << 13) /* Bit 13: SPI 4 clock enable */
#define RCC_APB2ENR_SYSCFGEN         (1 << 14) /* Bit 14: System configuration controller clock enable */
#define RCC_APB2ENR_TIM9EN           (1 << 16) /* Bit 16: TIM 9 clock enable */
#define RCC_APB2ENR_TIM10EN          (1 << 17) /* Bit 17: TIM1 0 clock enable */
#define RCC_APB2ENR_TIM11EN          (1 << 18) /* Bit 18: TIM  11 clock enable */
#define RCC_APB2ENR_SPI5EN           (1 << 20) /* Bit 20: SPI 5 clock enable */
#define RCC_APB2ENR_SAI1EN           (1 << 22) /* Bit 22: SAI 1 clock enable */
#define RCC_APB2ENR_SAI2EN           (1 << 23) /* Bit 23: SAI 2 clock enable */
#define RCC_APB2ENR_OTGPHYCEN        (1 << 31) /* Bit 31: OTGPHYC enable */

/* RCC AHB1 low power mode peripheral clock enable register */

#define RCC_AHB1LPENR_GPIOLPEN(n)    (1 << (n))
#define RCC_AHB1LPENR_GPIOALPEN      (1 << 0)  /* Bit 0:  IO port A clock enable during Sleep mode */
#define RCC_AHB1LPENR_GPIOBLPEN      (1 << 1)  /* Bit 1:  IO port B clock enable during Sleep mode */
#define RCC_AHB1LPENR_GPIOCLPEN      (1 << 2)  /* Bit 2:  IO port C clock enable during Sleep mode */
#define RCC_AHB1LPENR_GPIODLPEN      (1 << 3)  /* Bit 3:  IO port D clock enable during Sleep mode */
#define RCC_AHB1LPENR_GPIOELPEN      (1 << 4)  /* Bit 4:  IO port E clock enable during Sleep mode */
#define RCC_AHB1LPENR_GPIOFLPEN      (1 << 5)  /* Bit 5:  IO port F clock enable during Sleep mode */
#define RCC_AHB1LPENR_GPIOGLPEN      (1 << 6)  /* Bit 6:  IO port G clock enable during Sleep mode */
#define RCC_AHB1LPENR_GPIOHLPEN      (1 << 7)  /* Bit 7:  IO port H clock enable during Sleep mode */
#define RCC_AHB1LPENR_GPIOILPEN      (1 << 8)  /* Bit 8:  IO port I clock enable during Sleep mode */
#define RCC_AHB1LPENR_CRCLPEN        (1 << 12) /* Bit 12: CRC clock enable during Sleep mode */
#define RCC_AHB1LPENR_AXILPEN        (1 << 13) /* Bit 12: AXI to AHB bridge clock enable during Sleep mode */
#define RCC_AHB1LPENR_FLITFLPEN      (1 << 15) /* Bit 15: Flash interface clock enable during Sleep mode */
#define RCC_AHB1LPENR_SRAM1LPEN      (1 << 16) /* Bit 16: SRAM 1 interface clock enable during Sleep mode */
#define RCC_AHB1LPENR_SRAM2LPEN      (1 << 17) /* Bit 17: SRAM 2 interface clock enable during Sleep mode */
#define RCC_AHB1LPENR_BKPSRAMLPEN    (1 << 18) /* Bit 18: Backup SRAM interface clock enable during Sleep mode */
#define RCC_AHB1LPENR_DTCMLPEN       (1 << 20) /* Bit 20: DTCM RAM clock enable during Sleep mode */
#define RCC_AHB1LPENR_DMA1LPEN       (1 << 21) /* Bit 21: DMA1 clock enable during Sleep mode */
#define RCC_AHB1LPENR_DMA2LPEN       (1 << 22) /* Bit 22: DMA2 clock enable during Sleep mode */
#define RCC_AHB1LPENR_OTGHSLPEN      (1 << 29) /* Bit 29: USB OTG HS clock enable during Sleep mode */
#define RCC_AHB1LPENR_OTGHSULPILPEN  (1 << 30) /* Bit 30: USB OTG HSULPI clock enable during Sleep mode */

/* RCC AHB2 low power mode peripheral clock enable register */

#define RCC_AHB2LPENR_CRYPLPEN       (1 << 4)  /* Bit 4: Cryptographic modules clock enable during Sleep mode */
#define RCC_AHB2LPENR_RNGLPEN        (1 << 6)  /* Bit 6: Random number generator clock enable during Sleep mode */
#define RCC_AHB2LPENR_OTGFLPSEN      (1 << 7)  /* Bit 7: USB OTG FS clock enable during Sleep mode */

/* RCC AHB3 low power mode peripheral clock enable register */

#define RCC_AHB3LPENR_FSMLPEN        (1 << 0) /* Bit 0: Flexible static memory controller module clock
                                               *        enable during Sleep mode */
#define RCC_AHB3LPENR_QSPILPEN       (1 << 1) /* Bit 1: Quad SPI memory controller clock
                                               *        enable during Sleep mode */

/* RCC APB1 low power mode peripheral clock enable register */

#define RCC_APB1LPENR_TIM2LPEN       (1 << 0)  /* Bit 0:  TIM 2 clock enable during Sleep mode */
#define RCC_APB1LPENR_TIM3LPEN       (1 << 1)  /* Bit 1:  TIM 3 clock enable during Sleep mode */
#define RCC_APB1LPENR_TIM4LPEN       (1 << 2)  /* Bit 2:  TIM 4 clock enable during Sleep mode */
#define RCC_APB1LPENR_TIM5LPEN       (1 << 3)  /* Bit 3:  TIM 5 clock enable during Sleep mode */
#define RCC_APB1LPENR_TIM6LPEN       (1 << 4)  /* Bit 4:  TIM 6 clock enable during Sleep mode */
#define RCC_APB1LPENR_TIM7LPEN       (1 << 5)  /* Bit 5:  TIM 7 clock enable during Sleep mode */
#define RCC_APB1LPENR_TIM12LPEN      (1 << 6)  /* Bit 6:  TIM 12 clock enable during Sleep mode */
#define RCC_APB1LPENR_TIM13LPEN      (1 << 7)  /* Bit 7:  TIM 13 clock enable during Sleep mode */
#define RCC_APB1LPENR_TIM14LPEN      (1 << 8)  /* Bit 8:  TIM 14 clock enable during Sleep mode */
#define RCC_APB1LPENR_LPTIM1LPEN     (1 << 9)  /* Bit 9:  LPTIM 1 clock enable during Sleep mode */
#define RCC_APB1LPENR_RTCAPBEN       (1 << 10) /* Bit 10: RTCAPB clock enable during Sleep mode */
#define RCC_APB1LPENR_WWDGLPEN       (1 << 11) /* Bit 11: Window watchdog clock enable during Sleep mode */
#define RCC_APB1LPENR_SPI2LPEN       (1 << 14) /* Bit 14: SPI 2 clock enable during Sleep mode */
#define RCC_APB1LPENR_SPI3LPEN       (1 << 15) /* Bit 15: SPI 3 clock enable during Sleep mode */
#define RCC_APB1LPENR_USART2LPEN     (1 << 17) /* Bit 17: USART 2 clock enable during Sleep mode */
#define RCC_APB1LPENR_USART3LPEN     (1 << 18) /* Bit 18: USART 3 clock enable during Sleep mode */
#define RCC_APB1LPENR_UART4LPEN      (1 << 19) /* Bit 19: UART 4 clock enable during Sleep mode */
#define RCC_APB1LPENR_UART5LPEN      (1 << 20) /* Bit 20: UART 5 clock enable during Sleep mode */
#define RCC_APB1LPENR_I2C1LPEN       (1 << 21) /* Bit 21: I2C 1 clock enable during Sleep mode */
#define RCC_APB1LPENR_I2C2LPEN       (1 << 22) /* Bit 22: I2C 2 clock enable during Sleep mode */
#define RCC_APB1LPENR_I2C3LPEN       (1 << 23) /* Bit 23: I2C 3 clock enable during Sleep mode */
#define RCC_APB1LPENR_CAN1LPEN       (1 << 25) /* Bit 25: CAN 1 clock enable during Sleep mode */
#define RCC_APB1LPENR_PWRLPEN        (1 << 28) /* Bit 28: Power interface clock enable during Sleep mode */
#define RCC_APB1LPENR_DACLPEN        (1 << 29) /* Bit 29: DAC interface clock enable during Sleep mode */
#define RCC_APB1LPENR_UART7LPEN      (1 << 30) /* Bit 30: UART 7 clock enable during Sleep mode */
#define RCC_APB1LPENR_UART8LPEN      (1 << 31) /* Bit 31: UART 8 clock enable during Sleep mode */

/* RCC APB2 low power mode peripheral clock enable register */

#define RCC_APB2LPENR_TIM1LPEN       (1 << 0)  /* Bit 0:  TIM 1 clock enable during Sleep mode */
#define RCC_APB2LPENR_TIM8LPEN       (1 << 1)  /* Bit 1:  TIM 8 clock enable during Sleep mode */
#define RCC_APB2LPENR_USART1LPEN     (1 << 4)  /* Bit 4:  USART 1 clock enable during Sleep mode */
#define RCC_APB2LPENR_USART6LPEN     (1 << 5)  /* Bit 5:  USART 6 clock enable during Sleep mode */
#define RCC_APB2LPENR_SDMMC2LPEN     (1 << 7)  /* Bit 7:  SDMMC 2 clock enable during Sleep mode */
#define RCC_APB2LPENR_ADC1LPEN       (1 << 8)  /* Bit 8:  ADC 1 clock enable during Sleep mode */
#define RCC_APB2LPENR_ADC2LPEN       (1 << 9)  /* Bit 9:  ADC 2 clock enable during Sleep mode */
#define RCC_APB2LPENR_ADC3LPEN       (1 << 10) /* Bit 10: ADC 3 clock enable during Sleep mode */
#define RCC_APB2LPENR_SDMMC1LPEN     (1 << 11) /* Bit 11: SDMMC 1 clock enable during Sleep mode */
#define RCC_APB2LPENR_SPI1LPEN       (1 << 12) /* Bit 12: SPI 1 clock enable during Sleep mode */
#define RCC_APB2LPENR_SPI4LPEN       (1 << 13) /* Bit 13: SPI 4 clock enable during Sleep mode */
#define RCC_APB2LPENR_SYSCFGLPEN     (1 << 14) /* Bit 14: System configuration controller clock enable during Sleep mode */
#define RCC_APB2LPENR_TIM9LPEN       (1 << 16) /* Bit 16: TIM 9 clock enable during Sleep mode */
#define RCC_APB2LPENR_TIM10LPEN      (1 << 17) /* Bit 17: TIM 10 clock enable during Sleep mode */
#define RCC_APB2LPENR_TIM11LPEN      (1 << 18) /* Bit 18: TIM 11 clock enable during Sleep mode */
#define RCC_APB2LPENR_SPI5LPEN       (1 << 20) /* Bit 20: SPI 5 clock enable during Sleep mode */
#define RCC_APB2LPENR_SAI1LPEN       (1 << 22) /* Bit 22: SAI 1 clock enable during Sleep mode */
#define RCC_APB2LPENR_SAI2LPEN       (1 << 23) /* Bit 23: SAI 2 clock enable during Sleep mode */

/* Backup domain control register */

#define RCC_BDCR_LSEON               (1 << 0)  /* Bit 0: External Low Speed oscillator enable */
#define RCC_BDCR_LSERDY              (1 << 1)  /* Bit 1: External Low Speed oscillator Ready */
#define RCC_BDCR_LSEBYP              (1 << 2)  /* Bit 2: External Low Speed oscillator Bypass */
#define RCC_BDCR_LSEDRV_SHIFT        (3)       /* Bits 4:3: LSE oscillator Drive selection */
#define RCC_BDCR_LSEDRV_MASK         (3 << RCC_BDCR_LSEDRV_SHIFT)
#  define RCC_BDCR_LSEDRV_LOW        (0 << RCC_BDCR_LSEDRV_SHIFT) /* 00: Low driving capability */
#  define RCC_BDCR_LSEDRV_MEDHI      (1 << RCC_BDCR_LSEDRV_SHIFT) /* 01: Medium high driving capability */
#  define RCC_BDCR_LSEDRV_MEDLO      (2 << RCC_BDCR_LSEDRV_SHIFT) /* 10: Medium low driving capability */
#  define RCC_BDCR_LSEDRV_HIGH       (3 << RCC_BDCR_LSEDRV_SHIFT) /* 11: High driving capability */

#define RCC_BDCR_RTCSEL_SHIFT        (8)       /* Bits 9:8: RTC clock source selection */
#define RCC_BDCR_RTCSEL_MASK         (3 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_NOCLK      (0 << RCC_BDCR_RTCSEL_SHIFT) /* 00: No clock */
#  define RCC_BDCR_RTCSEL_LSE        (1 << RCC_BDCR_RTCSEL_SHIFT) /* 01: LSE oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_LSI        (2 << RCC_BDCR_RTCSEL_SHIFT) /* 10: LSI oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_HSE        (3 << RCC_BDCR_RTCSEL_SHIFT) /* 11: HSE oscillator clock divided by 128 used as RTC clock */

#define RCC_BDCR_RTCEN               (1 << 15) /* Bit 15: RTC clock enable */
#define RCC_BDCR_BDRST               (1 << 16) /* Bit 16: Backup domain software reset */

/* Control/status register */

#define RCC_CSR_LSION                (1 << 0)  /* Bit 0: Internal Low Speed oscillator enable */
#define RCC_CSR_LSIRDY               (1 << 1)  /* Bit 1: Internal Low Speed oscillator Ready */
#define RCC_CSR_RMVF                 (1 << 24) /* Bit 24: Remove reset flag */
#define RCC_CSR_BORRSTF              (1 << 25) /* Bit 25: BOR reset flag */
#define RCC_CSR_PINRSTF              (1 << 26) /* Bit 26: PIN reset flag */
#define RCC_CSR_PORRSTF              (1 << 27) /* Bit 27: POR/PDR reset flag */
#define RCC_CSR_SFTRSTF              (1 << 28) /* Bit 28: Software Reset flag */
#define RCC_CSR_IWDGRSTF             (1 << 29) /* Bit 29: Independent Watchdog reset flag */
#define RCC_CSR_WWDGRSTF             (1 << 30) /* Bit 30: Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF             (1 << 31) /* Bit 31: Low-Power reset flag */

/* Spread spectrum clock generation register */

#define RCC_SSCGR_MODPER_SHIFT       (0)       /* Bit 0-12: Modulation period */
#define RCC_SSCGR_MODPER_MASK        (0x1fff << RCC_SSCGR_MODPER_SHIFT)
#  define RCC_SSCGR_MODPER(n)        ((n) << RCC_SSCGR_MODPER_SHIFT)
#define RCC_SSCGR_INCSTEP_SHIFT      (13)      /* Bit 13-27: Incrementation step */
#define RCC_SSCGR_INCSTEP_MASK       (0x7fff << RCC_SSCGR_INCSTEP_SHIFT)
#  define RCC_SSCGR_INCSTEP(n)       ((n) << RCC_SSCGR_INCSTEP_SHIFT)
#define RCC_SSCGR_SPREADSEL          (1 << 30) /* Bit 30: Spread Select */
#define RCC_SSCGR_SSCGEN             (1 << 31) /* Bit 31: Spread spectrum modulation enable */

/* PLLI2S configuration register */

#define RCC_PLLI2SCFGR_PLLI2SN_SHIFT (6)       /* Bits 6-14: PLLI2S multiplication factor for VCO */
#define RCC_PLLI2SCFGR_PLLI2SN_MASK  (0x1ff << RCC_PLLI2SCFGR_PLLI2SN_SHIFT)
#  define RCC_PLLI2SCFGR_PLLI2SN(n)  ((uint32_t)(n) << RCC_PLLI2SCFGR_PLLI2SN_SHIFT)
#define RCC_PLLI2SCFGR_PLLI2SQ_SHIFT (24)      /* Bits 24-27: PLLI2S division factor for SAIs clock */
#define RCC_PLLI2SCFGR_PLLI2SQ_MASK  (15 << RCC_PLLI2SCFGR_PLLI2SQ_SHIFT)
#  define RCC_PLLI2SCFGR_PLLI2SQ(n)  ((uint32_t)(n) << RCC_PLLI2SCFGR_PLLI2SQ_SHIFT)
#define RCC_PLLI2SCFGR_PLLI2SR_SHIFT (28)      /* Bits 28-30: PLLI2S division factor for I2S clocks */
#define RCC_PLLI2SCFGR_PLLI2SR_MASK  (7 << RCC_PLLI2SCFGR_PLLI2SR_SHIFT)
#  define RCC_PLLI2SCFGR_PLLI2SR(n)  ((uint32_t)(n) << RCC_PLLI2SCFGR_PLLI2SR_SHIFT)

/* PLLSAI configuration register */

#define RCC_PLLSAICFGR_PLLSAIN_SHIFT (6)       /* Bits 6-14: PLLSAI divider (N) for VCO */
#define RCC_PLLSAICFGR_PLLSAIN_MASK  (0x1ff << RCC_PLLSAICFGR_PLLSAIN_SHIFT)
#  define RCC_PLLSAICFGR_PLLSAIN(n)  ((n) << RCC_PLLSAICFGR_PLLSAIN_SHIFT)
#define RCC_PLLSAICFGR_PLLSAIP_SHIFT (16)      /* Bits 16-17: PLLSAI division factor for 48MHz clock */
#define RCC_PLLSAICFGR_PLLSAIP_MASK  (3 << RCC_PLLSAICFGR_PLLSAIP_SHIFT)
#  define RCC_PLLSAICFGR_PLLSAIP(n)  ((((n)>>1)-1) << RCC_PLLSAICFGR_PLLSAIP_SHIFT)
#define RCC_PLLSAICFGR_PLLSAIQ_SHIFT (24)      /* Bits 24-27: PLLSAI division factor for SAI clock */
#define RCC_PLLSAICFGR_PLLSAIQ_MASK  (0x0F << RCC_PLLSAICFGR_PLLSAIQ_SHIFT)
#  define RCC_PLLSAICFGR_PLLSAIQ(n)  ((n) << RCC_PLLSAICFGR_PLLSAIQ_SHIFT)

/* Dedicated clocks configuration register 1 */

#define RCC_DCKCFGR1_PLLI2SDIVQ_SHIFT (0)      /* Bits 0-4: PLLI2S division factor for I2S clock */
#define RCC_DCKCFGR1_PLLI2SDIVQ_MASK  (0x1F << RCC_DCKCFGR1_PLLI2SDIVQ_SHIFT)
#  define RCC_DCKCFGR1_PLLI2SDIVQ(n)  ((n) << RCC_DCKCFGR1_PLLI2SDIVQ_SHIFT)
#define RCC_DCKCFGR1_PLLSAIDIVQ_SHIFT (8)      /* Bits 8-12: PLLSAI division factor for SAI clock */
#define RCC_DCKCFGR1_PLLSAIDIVQ_MASK  (0x1F << RCC_DCKCFGR1_PLLSAIDIVQ_SHIFT)
#  define RCC_DCKCFGR1_PLLSAIDIVQ(n)  ((n) << RCC_DCKCFGR1_PLLSAIDIVQ_SHIFT)
#define RCC_DCKCFGR1_SAI1SEL_SHIFT   (20)      /* Bits 20-21: SAI 1 clock source selection */
#define RCC_DCKCFGR1_SAI1SEL_MASK    (0x3 << RCC_DCKCFGR1_SAI1SEL_SHIFT)
#  define RCC_DCKCFGR1_SAI1SEL(n)    ((n) << RCC_DCKCFGR1_SAI1SEL_SHIFT)
#define RCC_DCKCFGR1_SAI2SEL_SHIFT   (22)      /* Bits 22-23: SAI 2 clock source selection */
#define RCC_DCKCFGR1_SAI2SEL_MASK    (0x3 << RCC_DCKCFGR1_SAI2SEL_SHIFT)
#  define RCC_DCKCFGR1_SAI2SEL(n)    ((n) << RCC_DCKCFGR1_SAI2SEL_SHIFT)
#define RCC_DCKCFGR1_TIMPRESEL       (1 << 24) /* Bit 24: Timer clock prescaler selection */

/* Dedicated clocks configuration register 2 */

#define RCC_DCKCFGR2_USART1SEL_SHIFT    (0)    /* Bits 0-1: USART 1 clock source selection */
#define RCC_DCKCFGR2_USART1SEL_MASK     (3 << RCC_DCKCFGR2_USART1SEL_SHIFT)
#  define RCC_DCKCFGR2_USART1SEL_APB    (0 << RCC_DCKCFGR2_USART1SEL_SHIFT) /* APB2 clock (PCLK2) is selected as USART 1 clock */
#  define RCC_DCKCFGR2_USART1SEL_SYSCLK (1 << RCC_DCKCFGR2_USART1SEL_SHIFT) /* System clock is selected as USART 1 clock */
#  define RCC_DCKCFGR2_USART1SEL_HSI    (2 << RCC_DCKCFGR2_USART1SEL_SHIFT) /* HSI clock is selected as USART 1 clock */
#  define RCC_DCKCFGR2_USART1SEL_LSE    (3 << RCC_DCKCFGR2_USART1SEL_SHIFT) /* LSE clock is selected as USART 1 clock */

#define RCC_DCKCFGR2_USART2SEL_SHIFT    (2)    /* Bits 2-3: USART 2 clock source selection */
#define RCC_DCKCFGR2_USART2SEL_MASK     (3 << RCC_DCKCFGR2_USART2SEL_SHIFT)
#  define RCC_DCKCFGR2_USART2SEL_APB    (0 << RCC_DCKCFGR2_USART2SEL_SHIFT) /* APB1 clock (PCLK1) is selected as USART 2 clock */
#  define RCC_DCKCFGR2_USART2SEL_SYSCLK (1 << RCC_DCKCFGR2_USART2SEL_SHIFT) /* System clock is selected as USART 2 clock */
#  define RCC_DCKCFGR2_USART2SEL_HSI    (2 << RCC_DCKCFGR2_USART2SEL_SHIFT) /* HSI clock is selected as USART 2 clock */
#  define RCC_DCKCFGR2_USART2SEL_LSE    (3 << RCC_DCKCFGR2_USART2SEL_SHIFT) /* LSE clock is selected as USART 2 clock */

#define RCC_DCKCFGR2_USART3SEL_SHIFT    (4)    /* Bits 4-5: USART 3 clock source selection */
#define RCC_DCKCFGR2_USART3SEL_MASK     (3 << RCC_DCKCFGR2_USART3SEL_SHIFT)
#  define RCC_DCKCFGR2_USART3SEL_APB    (0 << RCC_DCKCFGR2_USART3SEL_SHIFT) /* APB1 clock (PCLK1) is selected as USART 3 clock */
#  define RCC_DCKCFGR2_USART3SEL_SYSCLK (1 << RCC_DCKCFGR2_USART3SEL_SHIFT) /* System clock is selected as USART 3 clock */
#  define RCC_DCKCFGR2_USART3SEL_HSI    (2 << RCC_DCKCFGR2_USART3SEL_SHIFT) /* HSI clock is selected as USART 3 clock */
#  define RCC_DCKCFGR2_USART3SEL_LSE    (3 << RCC_DCKCFGR2_USART3SEL_SHIFT) /* LSE clock is selected as USART 3 clock */

#define RCC_DCKCFGR2_UART4SEL_SHIFT     (6)    /* Bits 6-7: UART 4 clock source selection */
#define RCC_DCKCFGR2_UART4SEL_MASK      (3 << RCC_DCKCFGR2_UART4SEL_SHIFT)
#  define RCC_DCKCFGR2_UART4SEL_APB     (0 << RCC_DCKCFGR2_UART4SEL_SHIFT)  /* APB1 clock (PCLK1) is selected as UART 4 clock */
#  define RCC_DCKCFGR2_UART4SEL_SYSCLK  (1 << RCC_DCKCFGR2_UART4SEL_SHIFT)  /* System clock is selected as UART 4 clock */
#  define RCC_DCKCFGR2_UART4SEL_HSI     (2 << RCC_DCKCFGR2_UART4SEL_SHIFT)  /* HSI clock is selected as UART 4 clock */
#  define RCC_DCKCFGR2_UART4SEL_LSE     (3 << RCC_DCKCFGR2_UART4SEL_SHIFT)  /* LSE clock is selected as UART 4 clock */

#define RCC_DCKCFGR2_UART5SEL_SHIFT     (8)    /* Bits 8-9:  UART 5 clock source selection */
#define RCC_DCKCFGR2_UART5SEL_MASK      (3 << RCC_DCKCFGR2_UART5SEL_SHIFT)
#  define RCC_DCKCFGR2_UART5SEL_APB     (0 << RCC_DCKCFGR2_UART5SEL_SHIFT)  /* APB1 clock (PCLK1) is selected as UART 5 clock */
#  define RCC_DCKCFGR2_UART5SEL_SYSCLK  (1 << RCC_DCKCFGR2_UART5SEL_SHIFT)  /* System clock is selected as UART 5 clock */
#  define RCC_DCKCFGR2_UART5SEL_HSI     (2 << RCC_DCKCFGR2_UART5SEL_SHIFT)  /* HSI clock is selected as UART 5 clock */
#  define RCC_DCKCFGR2_UART5SEL_LSE     (3 << RCC_DCKCFGR2_UART5SEL_SHIFT)  /* LSE clock is selected as UART 5 clock */

#define RCC_DCKCFGR2_USART6SEL_SHIFT    (10)   /* Bits 10-11:  USART 6 clock source selection */
#define RCC_DCKCFGR2_USART6SEL_MASK     (3 << RCC_DCKCFGR2_USART6SEL_SHIFT)
#  define RCC_DCKCFGR2_USART6SEL_APB    (0 << RCC_DCKCFGR2_USART6SEL_SHIFT) /* APB2 clock (PCLK2) is selected as USART 6 clock */
#  define RCC_DCKCFGR2_USART6SEL_SYSCLK (1 << RCC_DCKCFGR2_USART6SEL_SHIFT) /* System clock is selected as USART 6 clock */
#  define RCC_DCKCFGR2_USART6SEL_HSI    (2 << RCC_DCKCFGR2_USART6SEL_SHIFT) /* HSI clock is selected as USART 6 clock */
#  define RCC_DCKCFGR2_USART6SEL_LSE    (3 << RCC_DCKCFGR2_USART6SEL_SHIFT) /* LSE clock is selected as USART 6 clock */

#define RCC_DCKCFGR2_UART7SEL_SHIFT     (12)   /* Bits 12-13:  UART 7 clock source selection */
#define RCC_DCKCFGR2_UART7SEL_MASK      (3 << RCC_DCKCFGR2_UART7SEL_SHIFT)
#  define RCC_DCKCFGR2_UART7SEL_APB     (0 << RCC_DCKCFGR2_UART7SEL_SHIFT)  /* APB1 clock (PCLK1) is selected as UART 7 clock */
#  define RCC_DCKCFGR2_UART7SEL_SYSCLK  (1 << RCC_DCKCFGR2_UART7SEL_SHIFT)  /* System clock is selected as UART 7 clock */
#  define RCC_DCKCFGR2_UART7SEL_HSI     (2 << RCC_DCKCFGR2_UART7SEL_SHIFT)  /* HSI clock is selected as UART 7 clock */
#  define RCC_DCKCFGR2_UART7SEL_LSE     (3 << RCC_DCKCFGR2_UART7SEL_SHIFT)  /* LSE clock is selected as UART 7 clock */

#define RCC_DCKCFGR2_UART8SEL_SHIFT     (14)   /* Bits 14-15: UART 8 clock source selection */
#define RCC_DCKCFGR2_UART8SEL_MASK      (3 << RCC_DCKCFGR2_UART8SEL_SHIFT)
#  define RCC_DCKCFGR2_UART8SEL_APB     (0 << RCC_DCKCFGR2_UART8SEL_SHIFT)  /* APB1 clock (PCLK1) is selected as UART 8 clock */
#  define RCC_DCKCFGR2_UART8SEL_SYSCLK  (1 << RCC_DCKCFGR2_UART8SEL_SHIFT)  /* System clock is selected as UART 8 clock */
#  define RCC_DCKCFGR2_UART8SEL_HSI     (2 << RCC_DCKCFGR2_UART8SEL_SHIFT)  /* HSI clock is selected as UART 8 clock */
#  define RCC_DCKCFGR2_UART8SEL_LSE     (3 << RCC_DCKCFGR2_UART8SEL_SHIFT)  /* LSE clock is selected as UART 8 clock */

#define RCC_DCKCFGR2_I2C1SEL_SHIFT      (16)   /* Bits 16-17: I2C1 clock source selection */
#define RCC_DCKCFGR2_I2C1SEL_MASK       (3 << RCC_DCKCFGR2_I2C1SEL_SHIFT)
#  define RCC_DCKCFGR2_I2C1SEL_APB      (0 << RCC_DCKCFGR2_I2C1SEL_SHIFT)   /* APB1 clock (PCLK1) is selected as I2C 1 clock */
#  define RCC_DCKCFGR2_I2C1SEL_SYSCLK   (1 << RCC_DCKCFGR2_I2C1SEL_SHIFT)   /* System clock is selected as I2C 1 clock */
#  define RCC_DCKCFGR2_I2C1SEL_HSI      (2 << RCC_DCKCFGR2_I2C1SEL_SHIFT)   /* HSI clock is selected as I2C 1 clock */

#define RCC_DCKCFGR2_I2C2SEL_SHIFT      (18)   /* Bits 18-19: I2C2 clock source selection */
#define RCC_DCKCFGR2_I2C2SEL_MASK       (3 << RCC_DCKCFGR2_I2C2SEL_SHIFT)
#  define RCC_DCKCFGR2_I2C2SEL_APB      (0 << RCC_DCKCFGR2_I2C2SEL_SHIFT)   /* APB1 clock (PCLK1) is selected as I2C 2 clock */
#  define RCC_DCKCFGR2_I2C2SEL_SYSCLK   (1 << RCC_DCKCFGR2_I2C2SEL_SHIFT)   /* System clock is selected as I2C 2 clock */
#  define RCC_DCKCFGR2_I2C2SEL_HSI      (2 << RCC_DCKCFGR2_I2C2SEL_SHIFT)   /* HSI clock is selected as I2C 2 clock */

#define RCC_DCKCFGR2_I2C3SEL_SHIFT      (20)   /* Bits 20-21: I2C3 clock source selection */
#define RCC_DCKCFGR2_I2C3SEL_MASK       (3 << RCC_DCKCFGR2_I2C3SEL_SHIFT)
#  define RCC_DCKCFGR2_I2C3SEL_APB      (0 << RCC_DCKCFGR2_I2C3SEL_SHIFT)   /* APB1 clock (PCLK1) is selected as I2C 3 clock */
#  define RCC_DCKCFGR2_I2C3SEL_SYSCLK   (1 << RCC_DCKCFGR2_I2C3SEL_SHIFT)   /* System clock is selected as I2C 3 clock */
#  define RCC_DCKCFGR2_I2C3SEL_HSI      (2 << RCC_DCKCFGR2_I2C3SEL_SHIFT)   /* HSI clock is selected as I2C 3 clock */

#define RCC_DCKCFGR2_LPTIM1SEL_SHIFT    (24)   /* Bits 24-25: Low power timer 1 clock source selection */
#define RCC_DCKCFGR2_LPTIM1SEL_MASK     (3 << RCC_DCKCFGR2_LPTIM1SEL_SHIFT)
#  define RCC_DCKCFGR2_LPTIM1SEL_APB    (0 << RCC_DCKCFGR2_LPTIM1SEL_SHIFT) /* APB1 clock (PCLK1) is selected as LPTIM 1 clock */
#  define RCC_DCKCFGR2_LPTIM1SEL_SYSCLK (1 << RCC_DCKCFGR2_LPTIM1SEL_SHIFT) /* System clock is selected as LPTIM 1 clock */
#  define RCC_DCKCFGR2_LPTIM1SEL_HSI    (2 << RCC_DCKCFGR2_LPTIM1SEL_SHIFT) /* HSI clock is selected as LPTIM 1 clock */
#  define RCC_DCKCFGR2_LPTIM1SEL_LSE    (3 << RCC_DCKCFGR2_LPTIM1SEL_SHIFT) /* LSE clock is selected as LPTIM 1 clock */

#define RCC_DCKCFGR2_CK48MSEL_SHIFT     (27)   /* Bit 27: 48MHz clock source selection */
#define RCC_DCKCFGR2_CK48MSEL_MASK      (1 << RCC_DCKCFGR2_CK48MSEL_SHIFT)
#  define RCC_DCKCFGR2_CK48MSEL_PLL     (0 << RCC_DCKCFGR2_CK48MSEL_SHIFT)  /* 48MHz clock from PLL is selected */
#  define RCC_DCKCFGR2_CK48MSEL_PLLSAI  (1 << RCC_DCKCFGR2_CK48MSEL_SHIFT)  /* 48MHz clock from PLLSAI is selected */

#define RCC_DCKCFGR2_SDMMCSEL_SHIFT     (28)   /* Bit 28: SDMMC1 clock source selection */
#define RCC_DCKCFGR2_SDMMCSEL_MASK      (1 << RCC_DCKCFGR2_SDMMCSEL_SHIFT)
#  define RCC_DCKCFGR2_SDMMCSEL_48MHZ   (0 << RCC_DCKCFGR2_SDMMCSEL_SHIFT)  /* 48 MHz clock is selected as SDMMC clock */
#  define RCC_DCKCFGR2_SDMMCSEL_SYSCLK  (1 << RCC_DCKCFGR2_SDMMCSEL_SHIFT)  /* System clock is selected as SDMMC clock */

#define RCC_DCKCFGR2_SDMMC2SEL_SHIFT     (29)   /* Bit 29: SDMMC2 clock source selection */
#define RCC_DCKCFGR2_SDMMC2SEL_MASK      (1 << RCC_DCKCFGR2_SDMMC2SEL_SHIFT)
#  define RCC_DCKCFGR2_SDMMC2SEL_48MHZ   (0 << RCC_DCKCFGR2_SDMMC2SEL_SHIFT)  /* 48 MHz clock is selected as SDMMC clock */
#  define RCC_DCKCFGR2_SDMMC2SEL_SYSCLK  (1 << RCC_DCKCFGR2_SDMMC2SEL_SHIFT)  /* System clock is selected as SDMMC clock */

#endif /* CONFIG_STM32F7_STM32F72XX || CONFIG_STM32F7_STM32F73XX */
#endif /* __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32F74XX75XX_RCC_H */
