/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32g4xxxx_syscfg.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_SYSCFG_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_SYSCFG_MEMRMP_OFFSET       0x0000                    /* SYSCFG memory remap register */
#define STM32_SYSCFG_CFGR1_OFFSET        0x0004                    /* SYSCFG configuration register 1 */
#define STM32_SYSCFG_EXTICR_OFFSET(p)    (0x0008 + ((p) & 0x000c)) /* Registers are displaced by 4! */
#  define STM32_SYSCFG_EXTICR1_OFFSET    0x0008                    /* SYSCFG external interrupt configuration register 1 */
#  define STM32_SYSCFG_EXTICR2_OFFSET    0x000c                    /* SYSCFG external interrupt configuration register 2 */
#  define STM32_SYSCFG_EXTICR3_OFFSET    0x0010                    /* SYSCFG external interrupt configuration register 3 */
#  define STM32_SYSCFG_EXTICR4_OFFSET    0x0014                    /* SYSCFG external interrupt configuration register 4 */
#define STM32_SYSCFG_SCSR_OFFSET         0x0018                    /* SYSCFG CCMSRAM control and status register */
#define STM32_SYSCFG_CFGR2_OFFSET        0x001c                    /* SYSCFG configuration register 2 */
#define STM32_SYSCFG_SWPR_OFFSET         0x0020                    /* SYSCFG CCMSRAM write protection register */
#define STM32_SYSCFG_SKR_OFFSET          0x0024                    /* SYSCFG CCMSRAM key register */

/* Register Addresses *******************************************************/

#define STM32_SYSCFG_MEMRMP              (STM32_SYSCFG_BASE + STM32_SYSCFG_MEMRMP_OFFSET)
#define STM32_SYSCFG_CFGR1               (STM32_SYSCFG_BASE + STM32_SYSCFG_CFGR1_OFFSET)
#define STM32_SYSCFG_EXTICR(p)           (STM32_SYSCFG_BASE + STM32_SYSCFG_EXTICR_OFFSET(p))
#  define STM32_SYSCFG_EXTICR1           (STM32_SYSCFG_BASE + STM32_SYSCFG_EXTICR1_OFFSET)
#  define STM32_SYSCFG_EXTICR2           (STM32_SYSCFG_BASE + STM32_SYSCFG_EXTICR2_OFFSET)
#  define STM32_SYSCFG_EXTICR3           (STM32_SYSCFG_BASE + STM32_SYSCFG_EXTICR3_OFFSET)
#  define STM32_SYSCFG_EXTICR4           (STM32_SYSCFG_BASE + STM32_SYSCFG_EXTICR4_OFFSET)
#define STM32_SYSCFG_SCSR                (STM32_SYSCFG_BASE + STM32_SYSCFG_SCSR_OFFSET)
#define STM32_SYSCFG_CFGR2               (STM32_SYSCFG_BASE + STM32_SYSCFG_CFGR2_OFFSET)
#define STM32_SYSCFG_SWPR                (STM32_SYSCFG_BASE + STM32_SYSCFG_SWPR_OFFSET)
#define STM32_SYSCFG_SKR                 (STM32_SYSCFG_BASE + STM32_SYSCFG_SKR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Memory remap register (MEMRMP) */

#define SYSCFG_MEMRMP_MEM_MODE_SHIFT     (0)
#define SYSCFG_MEMRMP_MEM_MODE_MASK      (0x7 << SYSCFG_MEMRMP_MEM_MODE_SHIFT)
#  define SYSCFG_MEMRMP_MEM_MODE_FLASH   (0x0 << SYSCFG_MEMRMP_MEM_MODE_SHIFT) /* Main Flash memory mapped at 0x00000000 */
#  define SYSCFG_MEMRMP_MEM_MODE_SYSTEM  (0x1 << SYSCFG_MEMRMP_MEM_MODE_SHIFT) /* System Flash memory mapped at 0x00000000 */
#  define SYSCFG_MEMRMP_MEM_MODE_FSMC    (0x2 << SYSCFG_MEMRMP_MEM_MODE_SHIFT) /* FSMC memory */
#  define SYSCFG_MEMRMP_MEM_MODE_SRAM1   (0x3 << SYSCFG_MEMRMP_MEM_MODE_SHIFT) /* SRAM1 mapped at 0x00000000 */
#  define SYSCFG_MEMRMP_MEM_MODE_QSPI    (0x4 << SYSCFG_MEMRMP_MEM_MODE_SHIFT) /* QUADSPI memory mapped at 0x00000000 */
#define SYSCFG_MEMRMP_FB_MODE_MASK       (1 << 8)                              /* User Flash Bank mode selection */

/* Configuration register 1 (CFGR1) */

#define SYSCFG_CFGR1_BOOSTEN_MASK        (1 << 8)     /* I/O analog switch voltage booster enable */
#define SYSCFG_CFGR1_ANASWVDD_MASK       (1 << 9)     /* GPIO analog switch control voltage selection */
#define SYSCFG_CFGR1_I2C_PB6_FMP_MASK    (1 << 16)    /* I2C PB6 fast mode plus */
#define SYSCFG_CFGR1_I2C_PB7_FMP_MASK    (1 << 17)    /* I2C PB7 fast mode plus */
#define SYSCFG_CFGR1_I2C_PB8_FMP_MASK    (1 << 18)    /* I2C PB8 fast mode plus */
#define SYSCFG_CFGR1_I2C_PB9_FMP_MASK    (1 << 19)    /* I2C PB9 fast mode plus */
#define SYSCFG_CFGR1_I2C1_FMP_MASK       (1 << 20)    /* I2C1 fast mode plus */
#define SYSCFG_CFGR1_I2C2_FMP_MASK       (1 << 21)    /* I2C2 fast mode plus */
#define SYSCFG_CFGR1_I2C3_FMP_MASK       (1 << 22)    /* I2C3 fast mode plus */
#define SYSCFG_CFGR1_I2C4_FMP_MASK       (1 << 23)    /* I2C4 fast mode plus */
#define SYSCFG_CFGR1_FPUIE_INVALIDOP     (1 << 26)    /* Invalid operation interrupt enable */
#define SYSCFG_CFGR1_FPUIE_DIVZERO       (1 << 27)    /* Divide by zero interrupt enable */
#define SYSCFG_CFGR1_FPUIE_UNDERFLOW     (1 << 28)    /* Underflow interrupt enable */
#define SYSCFG_CFGR1_FPUIE_OVERFLOW      (1 << 29)    /* Overflow interrupt enable */
#define SYSCFG_CFGR1_FPUIE_DENORMAL      (1 << 30)    /* Input denormal interrupt enable */
#define SYSCFG_CFGR1_FPUIE_INEXACT       (1 << 31)    /* Inexact interrupt enable */

/* External interrupt (EXTI) configuration registers 1-4 (EXTICR1..4) */

#define SYSCFG_EXTICR_PORTA              (0)          /* 0000: PA[x] pin */
#define SYSCFG_EXTICR_PORTB              (1)          /* 0001: PB[x] pin */
#define SYSCFG_EXTICR_PORTC              (2)          /* 0010: PC[x] pin */
#define SYSCFG_EXTICR_PORTD              (3)          /* 0011: PD[x] pin */
#define SYSCFG_EXTICR_PORTE              (4)          /* 0100: PE[x] pin */
#define SYSCFG_EXTICR_PORTF              (5)          /* 0101: PF[x] pin */
#define SYSCFG_EXTICR_PORTG              (6)          /* 0110: PG[x] pin */

#define SYSCFG_EXTICR_PORT_MASK          (15)
#define SYSCFG_EXTICR_EXTI_SHIFT(g)      (((g) & 3) << 2)
#define SYSCFG_EXTICR_EXTI_MASK(g)       (SYSCFG_EXTICR_PORT_MASK << (SYSCFG_EXTICR_EXTI_SHIFT(g)))

#define SYSCFG_EXTICR1_EXTI0_SHIFT       (0)          /* Bits 0-3: EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI0_MASK        (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI0_SHIFT)
#define SYSCFG_EXTICR1_EXTI1_SHIFT       (4)          /* Bits 4-7: EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI1_MASK        (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI1_SHIFT)
#define SYSCFG_EXTICR1_EXTI2_SHIFT       (8)          /* Bits 8-11: EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI2_MASK        (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI2_SHIFT)
#define SYSCFG_EXTICR1_EXTI3_SHIFT       (12)         /* Bits 12-15: EXTI 3 configuration */
#define SYSCFG_EXTICR1_EXTI3_MASK        (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI3_SHIFT)

#define SYSCFG_EXTICR2_EXTI4_SHIFT       (0)          /* Bits 0-3: EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI4_MASK        (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI4_SHIFT)
#define SYSCFG_EXTICR2_EXTI5_SHIFT       (4)          /* Bits 4-7: EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI5_MASK        (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI5_SHIFT)
#define SYSCFG_EXTICR2_EXTI6_SHIFT       (8)          /* Bits 8-11: EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI6_MASK        (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI6_SHIFT)
#define SYSCFG_EXTICR2_EXTI7_SHIFT       (12)         /* Bits 12-15: EXTI 7 configuration */
#define SYSCFG_EXTICR2_EXTI7_MASK        (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI7_SHIFT)

#define SYSCFG_EXTICR3_EXTI8_SHIFT       (0)          /* Bits 0-3: EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI8_MASK        (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI8_SHIFT)
#define SYSCFG_EXTICR3_EXTI9_SHIFT       (4)          /* Bits 4-7: EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI9_MASK        (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI9_SHIFT)
#define SYSCFG_EXTICR3_EXTI10_SHIFT      (8)          /* Bits 8-11: EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI10_MASK       (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI10_SHIFT)
#define SYSCFG_EXTICR3_EXTI11_SHIFT      (12)         /* Bits 12-15: EXTI 11 configuration */
#define SYSCFG_EXTICR3_EXTI11_MASK       (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI11_SHIFT)

#define SYSCFG_EXTICR4_EXTI12_SHIFT      (0)          /* Bits 0-3: EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI12_MASK       (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI12_SHIFT)
#define SYSCFG_EXTICR4_EXTI13_SHIFT      (4)          /* Bits 4-7: EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI13_MASK       (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI13_SHIFT)
#define SYSCFG_EXTICR4_EXTI14_SHIFT      (8)          /* Bits 8-11: EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI14_MASK       (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI14_SHIFT)
#define SYSCFG_EXTICR4_EXTI15_SHIFT      (12)         /* Bits 12-15: EXTI 15 configuration */
#define SYSCFG_EXTICR4_EXTI15_MASK       (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI15_SHIFT)

/* CCM SRAM control and status register (SCSR) */

#define SYSCFG_SCSR_CCMER                (1 << 0)     /* CCMSRAM Erase Request */
#define SYSCFG_SCSR_CCMBSY               (1 << 1)     /* CCMSRAM Erase In Progress */

/* Configuration register 2 (CFGR2) */

#define SYSCFG_CFGR2_CLL                 (1 << 0)     /* Cortex M4 lockup (HARDFAULT) output enable bit */
#define SYSCFG_CFGR2_SPL                 (1 << 1)     /* SRAM1 and CCM SRAM parity lock */
#define SYSCFG_CFGR2_PVDL                (1 << 2)     /* PVD lock enable bit */
#define SYSCFG_CFGR2_ECCL                (1 << 3)     /* ECC lock */
#define SYSCFG_CFGR2_SPF                 (1 << 8)     /* SRAM1 and CCM STAM parity error flag */

/* CCM SRAM write protection register (SWPR) */

#define SYSCFG_SWPR_PWP(n)               (1 << (n))   /* CCMSRAM  Write protection for page n=0..31 */

/* CCM SRAM key register (SKR) */

#define SYSCFG_SKR_KEY_SHIFT             (0)          /* CCMSRAM write protection key for software erase */
#define SYSCFG_SKR_KEY_MASK              (0xff << SYSCFG_SKR_KEY_SHIFT)
#  define SYSCFG_SKR_KEY(n)              (((n) << SYSCFG_SKR_KEY_SHIFT) & SYSCFG_SKR_KEY_MASK)

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_SYSCFG_H */
