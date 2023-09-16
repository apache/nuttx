/****************************************************************************
 * arch/arm/src/at32/hardware/at32f43xxx_syscfg.h
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

#ifndef __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_SYSCFG_H
#define __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define AT32_SCFG_CFG1_OFFSET               (0x00)
#define AT32_SCFG_CFG2_OFFSET               (0x04)
#define AT32_SCFG_EXTICR_OFFSET(p)          (0x0008 + ((p) & 0x000c)) /* Registers are displaced by 4! */
#define AT32_SCFG_EXINTC1_OFFSET            (0x08)
#define AT32_SCFG_EXINTC2_OFFSET            (0x0C)
#define AT32_SCFG_EXINTC3_OFFSET            (0x10)
#define AT32_SCFG_EXINTC4_OFFSET            (0x14)
#define AT32_SCFG_UHDRV_OFFSET              (0x2C)

/* Register Addresses *******************************************************/

#define AT32_SCFG_CFG1                      (AT32_SCFG_BASE+AT32_SCFG_CFG1_OFFSET)
#define AT32_SCFG_CFG2                      (AT32_SCFG_BASE+AT32_SCFG_CFG2_OFFSET)
#define AT32_SCFG_EXTICR(p)                 (AT32_SCFG_BASE+AT32_SCFG_EXTICR_OFFSET(p))
#define AT32_SCFG_EXINTC1                   (AT32_SCFG_BASE+AT32_SCFG_EXINTC1_OFFSET)
#define AT32_SCFG_EXINTC2                   (AT32_SCFG_BASE+AT32_SCFG_EXINTC2_OFFSET)
#define AT32_SCFG_EXINTC3                   (AT32_SCFG_BASE+AT32_SCFG_EXINTC3_OFFSET)
#define AT32_SCFG_EXINTC4                   (AT32_SCFG_BASE+AT32_SCFG_EXINTC4_OFFSET)
#define AT32_SCFG_UHDRV                     (AT32_SCFG_BASE+AT32_SCFG_UHDRV_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* SCFG config1 register */

#define SCFG_CFG1_MEM_MAP_SEL_SHIFT         (0) /* memory address mapping selection */
#define SCFG_CFG1_MEM_MAP_SEL_MASK          (3 << SCFG_CFG1_MEM_MAP_SEL_SHIFT)
#  define SCFG_CFG1_MEM_MAP_SEL_FLASH       (0 << SCFG_CFG1_MEM_MAP_SEL_SHIFT) /* 000: Main Flash memory mapped at 0x0000 0000 */
#  define SCFG_CFG1_MEM_MAP_SEL_SYSTEM      (1 << SCFG_CFG1_MEM_MAP_SEL_SHIFT) /* 001: System Flash memory mapped at 0x0000 0000 */
#  define SCFG_CFG1_MEM_MAP_SEL_XMC         (2 << SCFG_CFG1_MEM_MAP_SEL_SHIFT) /* 010: XMC Bank1 (NOR/PSRAM 1 and 2) mapped at 0x0000 0000 */
#  define SCFG_CFG1_MEM_MAP_SEL_SRAM        (3 << SCFG_CFG1_MEM_MAP_SEL_SHIFT) /* 011: Embedded SRAM (112kB) mapped at 0x0000 0000 */
#  define SCFG_CFG1_MEM_MAP_SEL_SDRAM       (4 << SCFG_CFG1_MEM_MAP_SEL_SHIFT) /* 100: XMC SDRAM Bank1 mapped at 0x0000 0000 */

#define SCFG_CFG1_IR_POL                    (1 << 5) /* Infrared output polarity selection */

#define SCFG_CFG1_IR_SRC_SEL_SHIFT          (6) /* Infrared modulation envelope signal source selection */
#define SCFG_CFG1_IR_SRC_SEL_MASK           (3 << SCFG_CFG1_IR_SRC_SEL_SHIFT) 
#  define SCFG_CFG1_IR_SRC_SEL_TMR10        (0 << SCFG_CFG1_IR_SRC_SEL_SHIFT) /* Source use TRM10 */
#  define SCFG_CFG1_IR_SRC_SEL_USART1       (1 << SCFG_CFG1_IR_SRC_SEL_SHIFT) /* Source use USART1 */
#  define SCFG_CFG1_IR_SRC_SEL_USART2       (2 << SCFG_CFG1_IR_SRC_SEL_SHIFT) /* Source use USART2 */

#define SCFG_CFG1_SWAP_XMC_SHIFT            (6) /* Infrared modulation envelope signal source selection */
#define SCFG_CFG1_SWAP_XMC_MASK             (3 << SCFG_CFG1_SWAP_XMC_SHIFT) 
#  define SCFG_CFG1_SWAP_XMC_NONE           (0 << SCFG_CFG1_SWAP_XMC_SHIFT) /* No swap */
#  define SCFG_CFG1_SWAP_XMC_SDRAM1         (1 << SCFG_CFG1_SWAP_XMC_SHIFT) /* SDRAM swap1 */
#  define SCFG_CFG1_SWAP_XMC_QSPI2          (2 << SCFG_CFG1_SWAP_XMC_SHIFT) /* QSPI2 swap */
#  define SCFG_CFG1_SWAP_XMC_SDRAM2         (3 << SCFG_CFG1_SWAP_XMC_SHIFT) /* SDRAM swap2 */

/* SCFG config2 register */

#define SCFG_CFG2_MII_RMII_SEL              (1 << 23) /* MII or RMII selection */

#define SCFG_EXINTCR_PORT_MASK              (15)
#define SCFG_EXINTCR_EXTI_SHIFT(g)          (((g) & 3) << 2)
#define SCFG_EXINTCR_EXTI_MASK(g)           (SCFG_EXINTCR_PORT_MASK << (SCFG_EXINTCR_EXTI_SHIFT(g)))

/* SCFG EXINTC1 register */

#define SCFG_EXINTC1_EXINT0_SHIFT           (0) /* configure EXINT0 source */
#define SCFG_EXINTC1_EXINT0_MASK            (15 << SCFG_EXINTC1_EXINT0_SHIFT)
#  define SCFG_EXINTC1_EXINT0_GPIOA         (0 << SCFG_EXINTC1_EXINT0_SHIFT)
#  define SCFG_EXINTC1_EXINT0_GPIOB         (1 << SCFG_EXINTC1_EXINT0_SHIFT)
#  define SCFG_EXINTC1_EXINT0_GPIOC         (2 << SCFG_EXINTC1_EXINT0_SHIFT)
#  define SCFG_EXINTC1_EXINT0_GPIOD         (3 << SCFG_EXINTC1_EXINT0_SHIFT)
#  define SCFG_EXINTC1_EXINT0_GPIOE         (4 << SCFG_EXINTC1_EXINT0_SHIFT)
#  define SCFG_EXINTC1_EXINT0_GPIOF         (5 << SCFG_EXINTC1_EXINT0_SHIFT)
#  define SCFG_EXINTC1_EXINT0_GPIOG         (6 << SCFG_EXINTC1_EXINT0_SHIFT)
#  define SCFG_EXINTC1_EXINT0_GPIOH         (7 << SCFG_EXINTC1_EXINT0_SHIFT)

#define SCFG_EXINTC1_EXINT1_SHIFT           (4) /* configure EXINT1 source */
#define SCFG_EXINTC1_EXINT1_MASK            (15 << SCFG_EXINTC1_EXINT1_SHIFT)
#  define SCFG_EXINTC1_EXINT1_GPIOA         (0 << SCFG_EXINTC1_EXINT1_SHIFT)
#  define SCFG_EXINTC1_EXINT1_GPIOB         (1 << SCFG_EXINTC1_EXINT1_SHIFT)
#  define SCFG_EXINTC1_EXINT1_GPIOC         (2 << SCFG_EXINTC1_EXINT1_SHIFT)
#  define SCFG_EXINTC1_EXINT1_GPIOD         (3 << SCFG_EXINTC1_EXINT1_SHIFT)
#  define SCFG_EXINTC1_EXINT1_GPIOE         (4 << SCFG_EXINTC1_EXINT1_SHIFT)
#  define SCFG_EXINTC1_EXINT1_GPIOF         (5 << SCFG_EXINTC1_EXINT1_SHIFT)
#  define SCFG_EXINTC1_EXINT1_GPIOG         (6 << SCFG_EXINTC1_EXINT1_SHIFT)
#  define SCFG_EXINTC1_EXINT1_GPIOH         (7 << SCFG_EXINTC1_EXINT1_SHIFT)

#define SCFG_EXINTC1_EXINT2_SHIFT           (8) /* configure EXINT2 source */
#define SCFG_EXINTC1_EXINT2_MASK            (15 << SCFG_EXINTC1_EXINT2_SHIFT)
#  define SCFG_EXINTC1_EXINT2_GPIOA         (0 << SCFG_EXINTC1_EXINT2SHIFT)
#  define SCFG_EXINTC1_EXINT2_GPIOB         (1 << SCFG_EXINTC1_EXINT2_SHIFT)
#  define SCFG_EXINTC1_EXINT2_GPIOC         (2 << SCFG_EXINTC1_EXINT2_SHIFT)
#  define SCFG_EXINTC1_EXINT2_GPIOD         (3 << SCFG_EXINTC1_EXINT2_SHIFT)
#  define SCFG_EXINTC1_EXINT2_GPIOE         (4 << SCFG_EXINTC1_EXINT2_SHIFT)
#  define SCFG_EXINTC1_EXINT2_GPIOF         (5 << SCFG_EXINTC1_EXINT2_SHIFT)
#  define SCFG_EXINTC1_EXINT2_GPIOG         (6 << SCFG_EXINTC1_EXINT2_SHIFT)
#  define SCFG_EXINTC1_EXINT2_GPIOH         (7 << SCFG_EXINTC1_EXINT2_SHIFT)

#define SCFG_EXINTC1_EXINT3_SHIFT           (12) /* configure EXINT3 source */
#define SCFG_EXINTC1_EXINT3_MASK            (15 << SCFG_EXINTC1_EXINT3_SHIFT)
#  define SCFG_EXINTC1_EXINT3_GPIOA         (0 << SCFG_EXINTC1_EXINT3_SHIFT)
#  define SCFG_EXINTC1_EXINT3_GPIOB         (1 << SCFG_EXINTC1_EXINT3_SHIFT)
#  define SCFG_EXINTC1_EXINT3_GPIOC         (2 << SCFG_EXINTC1_EXINT3_SHIFT)
#  define SCFG_EXINTC1_EXINT3_GPIOD         (3 << SCFG_EXINTC1_EXINT3_SHIFT)
#  define SCFG_EXINTC1_EXINT3_GPIOE         (4 << SCFG_EXINTC1_EXINT3_SHIFT)
#  define SCFG_EXINTC1_EXINT3_GPIOF         (5 << SCFG_EXINTC1_EXINT3_SHIFT)
#  define SCFG_EXINTC1_EXINT3_GPIOG         (6 << SCFG_EXINTC1_EXINT3_SHIFT)
#  define SCFG_EXINTC1_EXINT3_GPIOH         (7 << SCFG_EXINTC1_EXINT3_SHIFT)

/* SCFG EXINTC2 register */

#define SCFG_EXINTC2_EXINT4_SHIFT           (0) /* configure EXINT4 source */
#define SCFG_EXINTC2_EXINT4_MASK            (15 << SCFG_EXINTC2_EXINT4_SHIFT)
#  define SCFG_EXINTC2_EXINT4_GPIOA         (0 << SCFG_EXINTC2_EXINT4_SHIFT)
#  define SCFG_EXINTC2_EXINT4_GPIOB         (1 << SCFG_EXINTC2_EXINT4_SHIFT)
#  define SCFG_EXINTC2_EXINT4_GPIOC         (2 << SCFG_EXINTC2_EXINT4_SHIFT)
#  define SCFG_EXINTC2_EXINT4_GPIOD         (3 << SCFG_EXINTC2_EXINT4_SHIFT)
#  define SCFG_EXINTC2_EXINT4_GPIOE         (4 << SCFG_EXINTC2_EXINT4_SHIFT)
#  define SCFG_EXINTC2_EXINT4_GPIOF         (5 << SCFG_EXINTC2_EXINT4_SHIFT)
#  define SCFG_EXINTC2_EXINT4_GPIOG         (6 << SCFG_EXINTC2_EXINT4_SHIFT)
#  define SCFG_EXINTC2_EXINT4_GPIOH         (7 << SCFG_EXINTC2_EXINT4_SHIFT)

#define SCFG_EXINTC2_EXINT5_SHIFT           (4) /* configure EXINT5 source */
#define SCFG_EXINTC2_EXINT5_MASK            (15 << SCFG_EXINTC2_EXINT5_SHIFT)
#  define SCFG_EXINTC2_EXINT5_GPIOA         (0 << SCFG_EXINTC2_EXINT5_SHIFT)
#  define SCFG_EXINTC2_EXINT5_GPIOB         (1 << SCFG_EXINTC2_EXINT5_SHIFT)
#  define SCFG_EXINTC2_EXINT5_GPIOC         (2 << SCFG_EXINTC2_EXINT5_SHIFT)
#  define SCFG_EXINTC2_EXINT5_GPIOD         (3 << SCFG_EXINTC2_EXINT5_SHIFT)
#  define SCFG_EXINTC2_EXINT5_GPIOE         (4 << SCFG_EXINTC2_EXINT5_SHIFT)
#  define SCFG_EXINTC2_EXINT5_GPIOF         (5 << SCFG_EXINTC2_EXINT5_SHIFT)
#  define SCFG_EXINTC2_EXINT5_GPIOG         (6 << SCFG_EXINTC2_EXINT5_SHIFT)
#  define SCFG_EXINTC2_EXINT5_GPIOH         (7 << SCFG_EXINTC2_EXINT5_SHIFT)

#define SCFG_EXINTC2_EXINT6_SHIFT           (8) /* configure EXINT6 source */
#define SCFG_EXINTC2_EXINT6_MASK            (15 << SCFG_EXINTC2_EXINT6_SHIFT)
#  define SCFG_EXINTC2_EXINT6_GPIOA         (0 << SCFG_EXINTC2_EXINT6SHIFT)
#  define SCFG_EXINTC2_EXINT6_GPIOB         (1 << SCFG_EXINTC2_EXINT6_SHIFT)
#  define SCFG_EXINTC2_EXINT6_GPIOC         (2 << SCFG_EXINTC2_EXINT6_SHIFT)
#  define SCFG_EXINTC2_EXINT6_GPIOD         (3 << SCFG_EXINTC2_EXINT6_SHIFT)
#  define SCFG_EXINTC2_EXINT6_GPIOE         (4 << SCFG_EXINTC2_EXINT6_SHIFT)
#  define SCFG_EXINTC2_EXINT6_GPIOF         (5 << SCFG_EXINTC2_EXINT6_SHIFT)
#  define SCFG_EXINTC2_EXINT6_GPIOG         (6 << SCFG_EXINTC2_EXINT6_SHIFT)
#  define SCFG_EXINTC2_EXINT6_GPIOH         (7 << SCFG_EXINTC2_EXINT6_SHIFT)

#define SCFG_EXINTC2_EXINT7_SHIFT           (12) /* configure EXINT7 source */
#define SCFG_EXINTC2_EXINT7_MASK            (15 << SCFG_EXINTC2_EXINT7_SHIFT)
#  define SCFG_EXINTC2_EXINT7_GPIOA         (0 << SCFG_EXINTC2_EXINT7_SHIFT)
#  define SCFG_EXINTC2_EXINT7_GPIOB         (1 << SCFG_EXINTC2_EXINT7_SHIFT)
#  define SCFG_EXINTC2_EXINT7_GPIOC         (2 << SCFG_EXINTC2_EXINT7_SHIFT)
#  define SCFG_EXINTC2_EXINT7_GPIOD         (3 << SCFG_EXINTC2_EXINT7_SHIFT)
#  define SCFG_EXINTC2_EXINT7_GPIOE         (4 << SCFG_EXINTC2_EXINT7_SHIFT)
#  define SCFG_EXINTC2_EXINT7_GPIOF         (5 << SCFG_EXINTC2_EXINT7_SHIFT)
#  define SCFG_EXINTC2_EXINT7_GPIOG         (6 << SCFG_EXINTC2_EXINT7_SHIFT)
#  define SCFG_EXINTC2_EXINT7_GPIOH         (7 << SCFG_EXINTC2_EXINT7_SHIFT)

/* SCFG EXINTC3 register */

#define SCFG_EXINTC3_EXINT8_SHIFT           (0) /* configure EXINT8 source */
#define SCFG_EXINTC3_EXINT8_MASK            (15 << SCFG_EXINTC3_EXINT8_SHIFT)
#  define SCFG_EXINTC3_EXINT8_GPIOA         (0 << SCFG_EXINTC3_EXINT8_SHIFT)
#  define SCFG_EXINTC3_EXINT8_GPIOB         (1 << SCFG_EXINTC3_EXINT8_SHIFT)
#  define SCFG_EXINTC3_EXINT8_GPIOC         (2 << SCFG_EXINTC3_EXINT8_SHIFT)
#  define SCFG_EXINTC3_EXINT8_GPIOD         (3 << SCFG_EXINTC3_EXINT8_SHIFT)
#  define SCFG_EXINTC3_EXINT8_GPIOE         (4 << SCFG_EXINTC3_EXINT8_SHIFT)
#  define SCFG_EXINTC3_EXINT8_GPIOF         (5 << SCFG_EXINTC3_EXINT8_SHIFT)
#  define SCFG_EXINTC3_EXINT8_GPIOG         (6 << SCFG_EXINTC3_EXINT8_SHIFT)
#  define SCFG_EXINTC3_EXINT8_GPIOH         (7 << SCFG_EXINTC3_EXINT8_SHIFT)

#define SCFG_EXINTC3_EXINT9_SHIFT           (4) /* configure EXINT9 source */
#define SCFG_EXINTC3_EXINT9_MASK            (15 << SCFG_EXINTC3_EXINT9_SHIFT)
#  define SCFG_EXINTC3_EXINT9_GPIOA         (0 << SCFG_EXINTC3_EXINT9_SHIFT)
#  define SCFG_EXINTC3_EXINT9_GPIOB         (1 << SCFG_EXINTC3_EXINT9_SHIFT)
#  define SCFG_EXINTC3_EXINT9_GPIOC         (2 << SCFG_EXINTC3_EXINT9_SHIFT)
#  define SCFG_EXINTC3_EXINT9_GPIOD         (3 << SCFG_EXINTC3_EXINT9_SHIFT)
#  define SCFG_EXINTC3_EXINT9_GPIOE         (4 << SCFG_EXINTC3_EXINT9_SHIFT)
#  define SCFG_EXINTC3_EXINT9_GPIOF         (5 << SCFG_EXINTC3_EXINT9_SHIFT)
#  define SCFG_EXINTC3_EXINT9_GPIOG         (6 << SCFG_EXINTC3_EXINT9_SHIFT)
#  define SCFG_EXINTC3_EXINT9_GPIOH         (7 << SCFG_EXINTC3_EXINT9_SHIFT)

#define SCFG_EXINTC3_EXINT10_SHIFT          (8) /* configure EXINT10 source */
#define SCFG_EXINTC3_EXINT10_MASK           (15 << SCFG_EXINTC3_EXINT10_SHIFT)
#  define SCFG_EXINTC3_EXINT10_GPIOA        (0 << SCFG_EXINTC3_EXINT10SHIFT)
#  define SCFG_EXINTC3_EXINT10_GPIOB        (1 << SCFG_EXINTC3_EXINT10_SHIFT)
#  define SCFG_EXINTC3_EXINT10_GPIOC        (2 << SCFG_EXINTC3_EXINT10_SHIFT)
#  define SCFG_EXINTC3_EXINT10_GPIOD        (3 << SCFG_EXINTC3_EXINT10_SHIFT)
#  define SCFG_EXINTC3_EXINT10_GPIOE        (4 << SCFG_EXINTC3_EXINT10_SHIFT)
#  define SCFG_EXINTC3_EXINT10_GPIOF        (5 << SCFG_EXINTC3_EXINT10_SHIFT)
#  define SCFG_EXINTC3_EXINT10_GPIOG        (6 << SCFG_EXINTC3_EXINT10_SHIFT)
#  define SCFG_EXINTC3_EXINT10_GPIOH        (7 << SCFG_EXINTC3_EXINT10_SHIFT)

#define SCFG_EXINTC3_EXINT11_SHIFT          (12) /* configure EXINT11 source */
#define SCFG_EXINTC3_EXINT11_MASK           (15 << SCFG_EXINTC3_EXINT11_SHIFT)
#  define SCFG_EXINTC3_EXINT11_GPIOA        (0 << SCFG_EXINTC3_EXINT11_SHIFT)
#  define SCFG_EXINTC3_EXINT11_GPIOB        (1 << SCFG_EXINTC3_EXINT11_SHIFT)
#  define SCFG_EXINTC3_EXINT11_GPIOC        (2 << SCFG_EXINTC3_EXINT11_SHIFT)
#  define SCFG_EXINTC3_EXINT11_GPIOD        (3 << SCFG_EXINTC3_EXINT11_SHIFT)
#  define SCFG_EXINTC3_EXINT11_GPIOE        (4 << SCFG_EXINTC3_EXINT11_SHIFT)
#  define SCFG_EXINTC3_EXINT11_GPIOF        (5 << SCFG_EXINTC3_EXINT11_SHIFT)
#  define SCFG_EXINTC3_EXINT11_GPIOG        (6 << SCFG_EXINTC3_EXINT11_SHIFT)
#  define SCFG_EXINTC3_EXINT11_GPIOH        (7 << SCFG_EXINTC3_EXINT11_SHIFT)

/* SCFG EXINTC4 register */

#define SCFG_EXINTC4_EXINT12_SHIFT          (0) /* configure EXINT12 source */
#define SCFG_EXINTC4_EXINT12_MASK           (15 << SCFG_EXINTC4_EXINT12_SHIFT)
#  define SCFG_EXINTC4_EXINT12_GPIOA        (0 << SCFG_EXINTC4_EXINT12_SHIFT)
#  define SCFG_EXINTC4_EXINT12_GPIOB        (1 << SCFG_EXINTC4_EXINT12_SHIFT)
#  define SCFG_EXINTC4_EXINT12_GPIOC        (2 << SCFG_EXINTC4_EXINT12_SHIFT)
#  define SCFG_EXINTC4_EXINT12_GPIOD        (3 << SCFG_EXINTC4_EXINT12_SHIFT)
#  define SCFG_EXINTC4_EXINT12_GPIOE        (4 << SCFG_EXINTC4_EXINT12_SHIFT)
#  define SCFG_EXINTC4_EXINT12_GPIOF        (5 << SCFG_EXINTC4_EXINT12_SHIFT)
#  define SCFG_EXINTC4_EXINT12_GPIOG        (6 << SCFG_EXINTC4_EXINT12_SHIFT)
#  define SCFG_EXINTC4_EXINT12_GPIOH        (7 << SCFG_EXINTC4_EXINT12_SHIFT)

#define SCFG_EXINTC4_EXINT13_SHIFT          (4) /* configure EXINT13 source */
#define SCFG_EXINTC4_EXINT13_MASK           (15 << SCFG_EXINTC4_EXINT13_SHIFT)
#  define SCFG_EXINTC4_EXINT13_GPIOA        (0 << SCFG_EXINTC4_EXINT13_SHIFT)
#  define SCFG_EXINTC4_EXINT13_GPIOB        (1 << SCFG_EXINTC4_EXINT13_SHIFT)
#  define SCFG_EXINTC4_EXINT13_GPIOC        (2 << SCFG_EXINTC4_EXINT13_SHIFT)
#  define SCFG_EXINTC4_EXINT13_GPIOD        (3 << SCFG_EXINTC4_EXINT13_SHIFT)
#  define SCFG_EXINTC4_EXINT13_GPIOE        (4 << SCFG_EXINTC4_EXINT13_SHIFT)
#  define SCFG_EXINTC4_EXINT13_GPIOF        (5 << SCFG_EXINTC4_EXINT13_SHIFT)
#  define SCFG_EXINTC4_EXINT13_GPIOG        (6 << SCFG_EXINTC4_EXINT13_SHIFT)
#  define SCFG_EXINTC4_EXINT13_GPIOH        (7 << SCFG_EXINTC4_EXINT13_SHIFT)

#define SCFG_EXINTC4_EXINT14_SHIFT          (8) /* configure EXINT14 source */
#define SCFG_EXINTC4_EXINT14_MASK           (15 << SCFG_EXINTC4_EXINT14_SHIFT)
#  define SCFG_EXINTC4_EXINT14_GPIOA        (0 << SCFG_EXINTC4_EXINT14SHIFT)
#  define SCFG_EXINTC4_EXINT14_GPIOB        (1 << SCFG_EXINTC4_EXINT14_SHIFT)
#  define SCFG_EXINTC4_EXINT14_GPIOC        (2 << SCFG_EXINTC4_EXINT14_SHIFT)
#  define SCFG_EXINTC4_EXINT14_GPIOD        (3 << SCFG_EXINTC4_EXINT14_SHIFT)
#  define SCFG_EXINTC4_EXINT14_GPIOE        (4 << SCFG_EXINTC4_EXINT14_SHIFT)
#  define SCFG_EXINTC4_EXINT14_GPIOF        (5 << SCFG_EXINTC4_EXINT14_SHIFT)
#  define SCFG_EXINTC4_EXINT14_GPIOG        (6 << SCFG_EXINTC4_EXINT14_SHIFT)
#  define SCFG_EXINTC4_EXINT14_GPIOH        (7 << SCFG_EXINTC4_EXINT14_SHIFT)

#define SCFG_EXINTC4_EXINT15_SHIFT          (12) /* configure EXINT15 source */
#define SCFG_EXINTC4_EXINT15_MASK           (15 << SCFG_EXINTC4_EXINT15_SHIFT)
#  define SCFG_EXINTC4_EXINT15_GPIOA        (0 << SCFG_EXINTC4_EXINT15_SHIFT)
#  define SCFG_EXINTC4_EXINT15_GPIOB        (1 << SCFG_EXINTC4_EXINT15_SHIFT)
#  define SCFG_EXINTC4_EXINT15_GPIOC        (2 << SCFG_EXINTC4_EXINT15_SHIFT)
#  define SCFG_EXINTC4_EXINT15_GPIOD        (3 << SCFG_EXINTC4_EXINT15_SHIFT)
#  define SCFG_EXINTC4_EXINT15_GPIOE        (4 << SCFG_EXINTC4_EXINT15_SHIFT)
#  define SCFG_EXINTC4_EXINT15_GPIOF        (5 << SCFG_EXINTC4_EXINT15_SHIFT)
#  define SCFG_EXINTC4_EXINT15_GPIOG        (6 << SCFG_EXINTC4_EXINT15_SHIFT)
#  define SCFG_EXINTC4_EXINT15_GPIOH        (7 << SCFG_EXINTC4_EXINT15_SHIFT)

/* SCFG UHDRV register */

#define SCFG_UHDRV_PB3_UH                   (1 << 0) /* PB3 Ultra high sourcing/sinking strength */
#define SCFG_UHDRV_PB9_UH                   (1 << 1) /* PB9 Ultra high sourcing/sinking strength */
#define SCFG_UHDRV_PB10_UH                  (1 << 2) /* PB10 Ultra high sourcing/sinking strength */
#define SCFG_UHDRV_PD12_UH                  (1 << 5) /* PD12 Ultra high sourcing/sinking strength */
#define SCFG_UHDRV_PD13_UH                  (1 << 6) /* PD13 Ultra high sourcing/sinking strength */
#define SCFG_UHDRV_PD14_UH                  (1 << 7) /* PD14 Ultra high sourcing/sinking strength */
#define SCFG_UHDRV_PD15_UH                  (1 << 8) /* PD15 Ultra high sourcing/sinking strength */
#define SCFG_UHDRV_PF14_UH                  (1 << 9) /* PF14 Ultra high sourcing/sinking strength */
#define SCFG_UHDRV_PF15_UH                  (1 << 10)/* PF15 Ultra high sourcing/sinking strength */

#endif /* __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_SYSCFG_H */
