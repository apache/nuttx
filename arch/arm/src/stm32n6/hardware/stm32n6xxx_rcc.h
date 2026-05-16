/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_RCC_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/stm32n6xxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_RCC_CR_OFFSET           0x0000  /* Clock control register */
#define STM32_RCC_SR_OFFSET           0x0004  /* Clock status register */
#define STM32_RCC_CFGR1_OFFSET        0x0020  /* Clock configuration register 1 */
#define STM32_RCC_CFGR2_OFFSET        0x0024  /* Clock configuration register 2 */
#define STM32_RCC_PLL1CFGR1_OFFSET    0x0080  /* PLL1 configuration register 1 */
#define STM32_RCC_PLL1CFGR3_OFFSET    0x0088  /* PLL1 configuration register 3 */
#define STM32_RCC_IC1CFGR_OFFSET      0x00c4  /* IC1 configuration register */
#define STM32_RCC_IC2CFGR_OFFSET      0x00c8  /* IC2 configuration register */
#define STM32_RCC_IC3CFGR_OFFSET      0x00cc  /* IC3 configuration register */
#define STM32_RCC_IC6CFGR_OFFSET      0x00d8  /* IC6 configuration register */
#define STM32_RCC_IC11CFGR_OFFSET     0x00ec  /* IC11 configuration register */
#define STM32_RCC_CCIPR13_OFFSET      0x0174  /* Peripheral kernel clock select register 13 */

/* Peripheral clock enable / set / clear register offsets.  Each enable
 * register (xxxENR) has a paired set register (xxxENSR) that performs an
 * atomic OR and a clear register (xxxENCR) that performs an atomic AND
 * (Reference: RM0486 14.5).  The same triplet pattern applies to the LPENR
 * (sleep mode) registers.
 */

#define STM32_RCC_MEMENR_OFFSET       0x024c  /* AXI/AHB SRAM clock enable register */
#define STM32_RCC_AHB4ENR_OFFSET      0x025c  /* AHB4 peripheral clock enable register */
#define STM32_RCC_APB1LENR_OFFSET     0x0264  /* APB1 peripheral clock enable register 1 */
#define STM32_RCC_APB2ENR_OFFSET      0x026c  /* APB2 peripheral clock enable register */
#define STM32_RCC_APB4HENR_OFFSET     0x0278  /* APB4 peripheral clock enable register 2 */
#define STM32_RCC_BUSLPENR_OFFSET     0x0284  /* Bus clocks enable in Sleep mode */
#define STM32_RCC_MEMLPENR_OFFSET     0x028c  /* SRAM clocks enable in Sleep mode */
#define STM32_RCC_APB1LLPENR_OFFSET   0x02a4  /* APB1 LP clock enable register 1 */
#define STM32_RCC_APB2LPENR_OFFSET    0x02ac  /* APB2 LP clock enable register */

#define STM32_RCC_DIVENR_OFFSET       0x0240  /* IC divider enable register */
#define STM32_RCC_DIVENSR_OFFSET      0x0a40  /* IC divider enable set register */

#define STM32_RCC_MEMENSR_OFFSET      0x0a4c  /* SRAM clock enable set register */
#define STM32_RCC_AHB4ENSR_OFFSET     0x0a5c  /* AHB4 clock enable set register */
#define STM32_RCC_APB1LENSR_OFFSET    0x0a64  /* APB1 clock enable set register 1 */
#define STM32_RCC_APB2ENSR_OFFSET     0x0a6c  /* APB2 clock enable set register */
#define STM32_RCC_APB4HENSR_OFFSET    0x0a78  /* APB4 clock enable set register 2 */
#define STM32_RCC_BUSLPENSR_OFFSET    0x0a84  /* Bus LP clock enable set register */
#define STM32_RCC_MEMLPENSR_OFFSET    0x0a8c  /* SRAM LP clock enable set register */
#define STM32_RCC_APB1LLPENSR_OFFSET  0x0aa4  /* APB1 LP clock enable set register 1 */
#define STM32_RCC_APB2LPENSR_OFFSET   0x0aac  /* APB2 LP clock enable set register */

#define STM32_RCC_CCR_OFFSET          0x1000  /* Clock control clear register */
#define STM32_RCC_APB2ENCR_OFFSET     0x126c  /* APB2 clock enable clear register */

#define STM32_RCC_CSR_OFFSET          0x0800  /* Clock status (set) register */

/* Register Addresses *******************************************************/

#define STM32_RCC_CR                  (STM32_RCC_BASE + STM32_RCC_CR_OFFSET)
#define STM32_RCC_SR                  (STM32_RCC_BASE + STM32_RCC_SR_OFFSET)
#define STM32_RCC_CFGR1               (STM32_RCC_BASE + STM32_RCC_CFGR1_OFFSET)
#define STM32_RCC_CFGR2               (STM32_RCC_BASE + STM32_RCC_CFGR2_OFFSET)
#define STM32_RCC_PLL1CFGR1           (STM32_RCC_BASE + STM32_RCC_PLL1CFGR1_OFFSET)
#define STM32_RCC_PLL1CFGR3           (STM32_RCC_BASE + STM32_RCC_PLL1CFGR3_OFFSET)
#define STM32_RCC_IC1CFGR             (STM32_RCC_BASE + STM32_RCC_IC1CFGR_OFFSET)
#define STM32_RCC_IC2CFGR             (STM32_RCC_BASE + STM32_RCC_IC2CFGR_OFFSET)
#define STM32_RCC_IC3CFGR             (STM32_RCC_BASE + STM32_RCC_IC3CFGR_OFFSET)
#define STM32_RCC_IC6CFGR             (STM32_RCC_BASE + STM32_RCC_IC6CFGR_OFFSET)
#define STM32_RCC_IC11CFGR            (STM32_RCC_BASE + STM32_RCC_IC11CFGR_OFFSET)
#define STM32_RCC_CCIPR13             (STM32_RCC_BASE + STM32_RCC_CCIPR13_OFFSET)

#define STM32_RCC_DIVENR              (STM32_RCC_BASE + STM32_RCC_DIVENR_OFFSET)
#define STM32_RCC_DIVENSR             (STM32_RCC_BASE + STM32_RCC_DIVENSR_OFFSET)

#define STM32_RCC_MEMENR              (STM32_RCC_BASE + STM32_RCC_MEMENR_OFFSET)
#define STM32_RCC_AHB4ENR             (STM32_RCC_BASE + STM32_RCC_AHB4ENR_OFFSET)
#define STM32_RCC_APB1LENR            (STM32_RCC_BASE + STM32_RCC_APB1LENR_OFFSET)
#define STM32_RCC_APB2ENR             (STM32_RCC_BASE + STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_APB4HENR            (STM32_RCC_BASE + STM32_RCC_APB4HENR_OFFSET)
#define STM32_RCC_BUSLPENR            (STM32_RCC_BASE + STM32_RCC_BUSLPENR_OFFSET)
#define STM32_RCC_MEMLPENR            (STM32_RCC_BASE + STM32_RCC_MEMLPENR_OFFSET)
#define STM32_RCC_APB1LLPENR          (STM32_RCC_BASE + STM32_RCC_APB1LLPENR_OFFSET)
#define STM32_RCC_APB2LPENR           (STM32_RCC_BASE + STM32_RCC_APB2LPENR_OFFSET)

#define STM32_RCC_MEMENSR             (STM32_RCC_BASE + STM32_RCC_MEMENSR_OFFSET)
#define STM32_RCC_AHB4ENSR            (STM32_RCC_BASE + STM32_RCC_AHB4ENSR_OFFSET)
#define STM32_RCC_APB1LENSR           (STM32_RCC_BASE + STM32_RCC_APB1LENSR_OFFSET)
#define STM32_RCC_APB2ENSR            (STM32_RCC_BASE + STM32_RCC_APB2ENSR_OFFSET)
#define STM32_RCC_APB4HENSR           (STM32_RCC_BASE + STM32_RCC_APB4HENSR_OFFSET)
#define STM32_RCC_BUSLPENSR           (STM32_RCC_BASE + STM32_RCC_BUSLPENSR_OFFSET)
#define STM32_RCC_MEMLPENSR           (STM32_RCC_BASE + STM32_RCC_MEMLPENSR_OFFSET)
#define STM32_RCC_APB1LLPENSR         (STM32_RCC_BASE + STM32_RCC_APB1LLPENSR_OFFSET)
#define STM32_RCC_APB2LPENSR          (STM32_RCC_BASE + STM32_RCC_APB2LPENSR_OFFSET)

#define STM32_RCC_CCR                 (STM32_RCC_BASE + STM32_RCC_CCR_OFFSET)
#define STM32_RCC_APB2ENCR            (STM32_RCC_BASE + STM32_RCC_APB2ENCR_OFFSET)

#define STM32_RCC_CSR                 (STM32_RCC_BASE + STM32_RCC_CSR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Clock control register */

#define RCC_CR_PLL1ON                 (1 << 8)  /* Bit 8:  PLL1 enable */
#define RCC_CR_HSION                  (1 << 3)  /* Bit 3:  HSI enable */

/* Clock status register */

#define RCC_SR_PLL1RDY                (1 << 8)  /* Bit 8:  PLL1 clock ready */
#define RCC_SR_HSIRDY                 (1 << 3)  /* Bit 3:  HSI clock ready */

/* Clock configuration register 1.  SYSSW = 0b11 selects three IC dividers
 * (IC2 for SYSCLK, IC6 for AHB, IC11 for APB) -- the SVD names this state
 * after the first IC only.
 */

#define RCC_CFGR1_SYSSWS_SHIFT        (28)
#define RCC_CFGR1_SYSSWS_MASK         (0x3 << RCC_CFGR1_SYSSWS_SHIFT)
#define RCC_CFGR1_SYSSWS_IC2_IC6_IC11 (3 << RCC_CFGR1_SYSSWS_SHIFT)
#define RCC_CFGR1_SYSSW_SHIFT         (24)
#define RCC_CFGR1_SYSSW_MASK          (0x3 << RCC_CFGR1_SYSSW_SHIFT)
#define RCC_CFGR1_SYSSW_IC2_IC6_IC11  (3 << RCC_CFGR1_SYSSW_SHIFT)
#define RCC_CFGR1_CPUSWS_SHIFT        (20)
#define RCC_CFGR1_CPUSWS_MASK         (0x3 << RCC_CFGR1_CPUSWS_SHIFT)
#define RCC_CFGR1_CPUSWS_IC1          (3 << RCC_CFGR1_CPUSWS_SHIFT)
#define RCC_CFGR1_CPUSW_SHIFT         (16)
#define RCC_CFGR1_CPUSW_MASK          (0x3 << RCC_CFGR1_CPUSW_SHIFT)
#define RCC_CFGR1_CPUSW_IC1           (3 << RCC_CFGR1_CPUSW_SHIFT)

/* Clock configuration register 2.  HPRE divides the SYSCLK fed to AHB
 * before it reaches the CPU/peripheral bus matrix.
 */

#define RCC_CFGR2_HPRE_SHIFT          (20)
#define RCC_CFGR2_HPRE_MASK           (0x7 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLK         (0 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd2       (1 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd4       (2 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd8       (3 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd16      (4 << RCC_CFGR2_HPRE_SHIFT)

#define RCC_CFGR2                     STM32_RCC_CFGR2

/* PLL1 configuration register 1 */

#define RCC_PLL1CFGR1_SEL_SHIFT       (28)
#define RCC_PLL1CFGR1_SEL_MASK        (0x7 << RCC_PLL1CFGR1_SEL_SHIFT)
#define RCC_PLL1CFGR1_SEL_HSI         (0 << RCC_PLL1CFGR1_SEL_SHIFT)
#define RCC_PLL1CFGR1_DIVM_SHIFT      (20)       /* Bits 25-20: Reference divider */
#define RCC_PLL1CFGR1_DIVM_MASK       (0x3f << RCC_PLL1CFGR1_DIVM_SHIFT)
#define RCC_PLL1CFGR1_DIVN_SHIFT      (8)        /* Bits 19-8: Feedback divider */
#define RCC_PLL1CFGR1_DIVN_MASK       (0xfff << RCC_PLL1CFGR1_DIVN_SHIFT)

/* PLL1 configuration register 3 */

#define RCC_PLL1CFGR3_PDIVEN          (1 << 30)  /* Bit 30: Post-divider and PLL output enable */
#define RCC_PLL1CFGR3_PDIV1_SHIFT     (27)       /* Bits 29-27: Post-divider 1 */
#define RCC_PLL1CFGR3_PDIV1_MASK      (0x7 << RCC_PLL1CFGR3_PDIV1_SHIFT)
#define RCC_PLL1CFGR3_PDIV2_SHIFT     (24)       /* Bits 26-24: Post-divider 2 */
#define RCC_PLL1CFGR3_PDIV2_MASK      (0x7 << RCC_PLL1CFGR3_PDIV2_SHIFT)
#define RCC_PLL1CFGR3_MODSSDIS        (1 << 2)   /* Bit 2:  Modulation spread spectrum disable */

/* IC1..IC20 configuration registers -- all share the same layout.  Field
 * SEL selects the PLL source (PLL1..PLL4); field INT is an 8-bit integer
 * divider where INT[7:0] = N-1 yielding a divide ratio of N.
 */

#define RCC_ICCFGR_SEL_SHIFT          (28)
#define RCC_ICCFGR_SEL_MASK           (0x3 << RCC_ICCFGR_SEL_SHIFT)
#define RCC_ICCFGR_SEL_PLL1           (0 << RCC_ICCFGR_SEL_SHIFT)
#define RCC_ICCFGR_INT_SHIFT          (16)
#define RCC_ICCFGR_INT_MASK           (0xff << RCC_ICCFGR_INT_SHIFT)

/* IC divider enable register */

#define RCC_DIVENR_IC11EN             (1 << 10)  /* Bit 10: IC11 enable */
#define RCC_DIVENR_IC6EN              (1 << 5)   /* Bit 5:  IC6 enable */
#define RCC_DIVENR_IC3EN              (1 << 2)   /* Bit 2:  IC3 enable */
#define RCC_DIVENR_IC2EN              (1 << 1)   /* Bit 1:  IC2 enable */
#define RCC_DIVENR_IC1EN              (1 << 0)   /* Bit 0:  IC1 enable */

/* SRAM clock enable register */

#define RCC_MEMENR_CACHEAXIRAMEN      (1 << 10)  /* Bit 10: CACHEAXIRAM enable */
#define RCC_MEMENR_AXISRAM2EN         (1 << 8)   /* Bit 8:  AXISRAM2 enable */
#define RCC_MEMENR_AXISRAM1EN         (1 << 7)   /* Bit 7:  AXISRAM1 enable */
#define RCC_MEMENR_AXISRAM6EN         (1 << 3)   /* Bit 3:  AXISRAM6 enable */
#define RCC_MEMENR_AXISRAM5EN         (1 << 2)   /* Bit 2:  AXISRAM5 enable */
#define RCC_MEMENR_AXISRAM4EN         (1 << 1)   /* Bit 1:  AXISRAM4 enable */
#define RCC_MEMENR_AXISRAM3EN         (1 << 0)   /* Bit 0:  AXISRAM3 enable */

#define RCC_MEMENR_ALLAXISRAM         (RCC_MEMENR_AXISRAM1EN | RCC_MEMENR_AXISRAM2EN | \
                                       RCC_MEMENR_AXISRAM3EN | RCC_MEMENR_AXISRAM4EN | \
                                       RCC_MEMENR_AXISRAM5EN | RCC_MEMENR_AXISRAM6EN)

/* AHB4 peripheral clock enable register */

#define RCC_AHB4ENR_PWREN             (1 << 18)  /* Bit 18: PWR enable */
#define RCC_AHB4ENR_GPIOQEN           (1 << 16)  /* Bit 16: GPIOQ enable */
#define RCC_AHB4ENR_GPIOPEN           (1 << 15)  /* Bit 15: GPIOP enable */
#define RCC_AHB4ENR_GPIOOEN           (1 << 14)  /* Bit 14: GPIOO enable */
#define RCC_AHB4ENR_GPIONEN           (1 << 13)  /* Bit 13: GPION enable */
#define RCC_AHB4ENR_GPIOHEN           (1 << 7)   /* Bit 7:  GPIOH enable */
#define RCC_AHB4ENR_GPIOGEN           (1 << 6)   /* Bit 6:  GPIOG enable */
#define RCC_AHB4ENR_GPIOFEN           (1 << 5)   /* Bit 5:  GPIOF enable */
#define RCC_AHB4ENR_GPIOEEN           (1 << 4)   /* Bit 4:  GPIOE enable */
#define RCC_AHB4ENR_GPIODEN           (1 << 3)   /* Bit 3:  GPIOD enable */
#define RCC_AHB4ENR_GPIOCEN           (1 << 2)   /* Bit 2:  GPIOC enable */
#define RCC_AHB4ENR_GPIOBEN           (1 << 1)   /* Bit 1:  GPIOB enable */
#define RCC_AHB4ENR_GPIOAEN           (1 << 0)   /* Bit 0:  GPIOA enable */

/* APB1 peripheral clock enable register 1 */

#define RCC_APB1LENR_TIM2EN           (1 << 0)   /* Bit 0:  TIM2 enable */

/* APB2 peripheral clock enable register */

#define RCC_APB2ENR_USART1EN          (1 << 4)   /* Bit 4:  USART1 enable */

/* APB4 peripheral clock enable register 2 */

#define RCC_APB4HENR_BSECEN           (1 << 1)   /* Bit 1:  BSEC enable */
#define RCC_APB4HENR_SYSCFGEN         (1 << 0)   /* Bit 0:  SYSCFG enable */

/* Bus clock enable in Sleep mode */

#define RCC_BUSLPENR_ACLKNCLPEN       (1 << 1)   /* Bit 1:  ACLKNC clock enable in CSLEEP */
#define RCC_BUSLPENR_ACLKNLPEN        (1 << 0)   /* Bit 0:  ACLKN clock enable in CSLEEP */

/* SRAM clock enable in Sleep mode */

#define RCC_MEMLPENR_CACHEAXIRAMLPEN  (1 << 10)  /* Bit 10: CACHEAXIRAM enable in CSLEEP */
#define RCC_MEMLPENR_AXISRAM2LPEN     (1 << 8)
#define RCC_MEMLPENR_AXISRAM1LPEN     (1 << 7)
#define RCC_MEMLPENR_AXISRAM6LPEN     (1 << 3)
#define RCC_MEMLPENR_AXISRAM5LPEN     (1 << 2)
#define RCC_MEMLPENR_AXISRAM4LPEN     (1 << 1)
#define RCC_MEMLPENR_AXISRAM3LPEN     (1 << 0)

#define RCC_MEMLPENR_ALLAXISRAM       (RCC_MEMLPENR_AXISRAM1LPEN | RCC_MEMLPENR_AXISRAM2LPEN | \
                                       RCC_MEMLPENR_AXISRAM3LPEN | RCC_MEMLPENR_AXISRAM4LPEN | \
                                       RCC_MEMLPENR_AXISRAM5LPEN | RCC_MEMLPENR_AXISRAM6LPEN)

/* APB1 peripheral clock enable in Sleep mode (register 1) */

#define RCC_APB1LLPENR_TIM2LPEN       (1 << 0)   /* Bit 0:  TIM2 enable in CSLEEP */

/* APB2 peripheral clock enable in Sleep mode */

#define RCC_APB2LPENR_USART1LPEN      (1 << 4)   /* Bit 4:  USART1 enable in CSLEEP */

/* Peripheral kernel clock select register 13 */

#define RCC_CCIPR13_USART1SEL_SHIFT   (0)
#define RCC_CCIPR13_USART1SEL_MASK    (0x7 << RCC_CCIPR13_USART1SEL_SHIFT)
#define RCC_CCIPR13_USART1SEL_HSI     (6 << RCC_CCIPR13_USART1SEL_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_RCC_H */
