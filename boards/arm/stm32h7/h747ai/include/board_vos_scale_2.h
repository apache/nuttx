/****************************************************************************
 * boards/arm/stm32h7/h747ai/include/board_vos_scale_2.h
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

#ifndef __BOARDS_ARM_STM32H7_H747AI_INCLUDE_BOARD_VOS_SCALE_2_H
#define __BOARDS_ARM_STM32H7_H747AI_INCLUDE_BOARD_VOS_SCALE_2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/* Do not include STM32 H7 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PLL1, wide 4 - 8 MHz input, enable DIVP, DIVQ, DIVR
 *
 *   PLL1_VCO = (48,000,000 / 3) * 37.5 = 600 MHz
 *
 *   PLL1P = PLL1_VCO/2  = 600 MHz / 2   = 300 MHz
 *   PLL1Q = PLL1_VCO/4  = 600 MHz / 4   = 150 MHz
 *   PLL1R = PLL1_VCO/2  = 600 MHz / 2   = 300 MHz
 */

#define STM32_PLLCFG_PLL1CFG (RCC_PLLCFGR_PLL1VCOSEL_WIDE | \
                              RCC_PLLCFGR_PLL1RGE_8_16_MHZ | \
                              RCC_PLLCFGR_DIVP1EN | \
                              RCC_PLLCFGR_DIVQ1EN | \
                              RCC_PLLCFGR_DIVR1EN | \
                              RCC_PLLCFGR_PLL1FRACEN)
#define STM32_PLLCFG_PLL1M       RCC_PLLCKSELR_DIVM1(3)
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(37)
#define STM32_PLLCFG_PLL1FRACN   RCC_PLL1FRACR_FRACN1(4096)
#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR_P1(2)
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(4)
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR_R1(2)

#define STM32_VCO1_FREQUENCY     ((STM32_HSE_FREQUENCY / 3) * (37 + (4096 / (1 << 13))))
#define STM32_PLL1P_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)
#define STM32_PLL1Q_FREQUENCY    (STM32_VCO1_FREQUENCY / 4)
#define STM32_PLL1R_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)

/* PLL2 */

#define STM32_PLLCFG_PLL2CFG (RCC_PLLCFGR_PLL1VCOSEL_MEDIUM | \
                              RCC_PLLCFGR_PLL1RGE_8_16_MHZ | \
                              RCC_PLLCFGR_DIVP2EN | \
                              RCC_PLLCFGR_DIVQ2EN | \
                              RCC_PLLCFGR_DIVR2EN | \
                              RCC_PLLCFGR_PLL2FRACEN)
#define STM32_PLLCFG_PLL2M       RCC_PLLCKSELR_DIVM2(3)
#define STM32_PLLCFG_PLL2N       RCC_PLL2DIVR_N2(9)
#define STM32_PLLCFG_PLL2FRACN   RCC_PLL2FRACR_FRACN2(3072)
#define STM32_PLLCFG_PLL2P       RCC_PLL2DIVR_P2(2)
#define STM32_PLLCFG_PLL2Q       RCC_PLL2DIVR_Q2(2)
#define STM32_PLLCFG_PLL2R       RCC_PLL2DIVR_R2(4)

#define STM32_VCO2_FREQUENCY     ((STM32_HSE_FREQUENCY / 3) * (9 + (3072 / (1 << 13))))
#define STM32_PLL2P_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)
#define STM32_PLL2Q_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)
#define STM32_PLL2R_FREQUENCY    (STM32_VCO2_FREQUENCY / 4)

/* PLL3 */

#define STM32_PLLCFG_PLL3CFG 0
#define STM32_PLLCFG_PLL3M   0
#define STM32_PLLCFG_PLL3N   0
#define STM32_PLLCFG_PLL3P   0
#define STM32_PLLCFG_PLL3Q   0
#define STM32_PLLCFG_PLL3R   0

#define STM32_VCO3_FREQUENCY
#define STM32_PLL3P_FREQUENCY
#define STM32_PLL3Q_FREQUENCY
#define STM32_PLL3R_FREQUENCY

/* SYSCLK = PLL1P = 300 MHz
 * M7 CPUCLK = SYSCLK / 1 = 300 MHz
 * M4 CPUCLK = HCLK / 2 = 150 MHz
 */

#define STM32_RCC_D1CFGR_D1CPRE  (RCC_D1CFGR_D1CPRE_SYSCLK)
#define STM32_SYSCLK_FREQUENCY   (STM32_PLL1P_FREQUENCY)
#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
#  define STM32_CPUCLK_FREQUENCY (STM32_SYSCLK_FREQUENCY / 1)
#else
#  define STM32_CPUCLK_FREQUENCY (STM32_HCLK_FREQUENCY / 2)
#endif

/* Configure Clock Assignments */

/* AHB clock (HCLK) is SYSCLK/2 (240 MHz max)
 * HCLK1 = HCLK2 = HCLK3 = HCLK4
 */

#define STM32_RCC_D1CFGR_HPRE   RCC_D1CFGR_HPRE_SYSCLKd2        /* HCLK  = SYSCLK / 2 */
#define STM32_ACLK_FREQUENCY    (STM32_SYSCLK_FREQUENCY / 2)    /* ACLK in D1, HCLK3 in D1 */
#define STM32_HCLK_FREQUENCY    (STM32_SYSCLK_FREQUENCY / 2)    /* HCLK in D2, HCLK4 in D3 */

/* APB1 clock (PCLK1) is HCLK/2 (150 MHz) */

#define STM32_RCC_D2CFGR_D2PPRE1  RCC_D2CFGR_D2PPRE1_HCLKd2     /* PCLK1 = HCLK / 2 */
#define STM32_PCLK1_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB2 clock (PCLK2) is HCLK/2 (150 MHz) */

#define STM32_RCC_D2CFGR_D2PPRE2  RCC_D2CFGR_D2PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB3 clock (PCLK3) is HCLK/2 (150 MHz) */

#define STM32_RCC_D1CFGR_D1PPRE   RCC_D1CFGR_D1PPRE_HCLKd2      /* PCLK3 = HCLK / 2 */
#define STM32_PCLK3_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB4 clock (PCLK4) is HCLK/2 (150 MHz) */

#define STM32_RCC_D3CFGR_D3PPRE   RCC_D3CFGR_D3PPRE_HCLKd2      /* PCLK4 = HCLK / 2 */
#define STM32_PCLK4_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32H7_H747AI_INCLUDE_BOARD_VOS_SCALE_2_H */
