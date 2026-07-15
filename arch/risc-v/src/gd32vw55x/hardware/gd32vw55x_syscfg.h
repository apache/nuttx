/****************************************************************************
 * arch/risc-v/src/gd32vw55x/hardware/gd32vw55x_syscfg.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_SYSCFG_H
#define __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gd32vw55x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define GD32VW55X_SYSCFG_CFG0_OFFSET      0x0000  /* Configuration 0 */
#define GD32VW55X_SYSCFG_EXTISS0_OFFSET   0x0008  /* EXTI source sel 0 */
#define GD32VW55X_SYSCFG_EXTISS1_OFFSET   0x000c  /* EXTI source sel 1 */
#define GD32VW55X_SYSCFG_EXTISS2_OFFSET   0x0010  /* EXTI source sel 2 */
#define GD32VW55X_SYSCFG_EXTISS3_OFFSET   0x0014  /* EXTI source sel 3 */
#define GD32VW55X_SYSCFG_CPSCTL_OFFSET    0x0020  /* I/O compensation ctrl */
#define GD32VW55X_SYSCFG_CFG1_OFFSET      0x0054  /* Configuration 1 */
#define GD32VW55X_SYSCFG_SCFG_OFFSET      0x0068  /* Shared SRAM config */
#define GD32VW55X_SYSCFG_TIMER0CFG_OFFSET 0x0100  /* TIMER0 slave config */
#define GD32VW55X_SYSCFG_TIMER1CFG_OFFSET 0x0104  /* TIMER1 slave config */
#define GD32VW55X_SYSCFG_TIMER2CFG_OFFSET 0x0108  /* TIMER2 slave config */

/* Register addresses *******************************************************/

#define GD32VW55X_SYSCFG_CFG0 \
  (GD32VW55X_SYSCFG_BASE + GD32VW55X_SYSCFG_CFG0_OFFSET)
#define GD32VW55X_SYSCFG_EXTISS0 \
  (GD32VW55X_SYSCFG_BASE + GD32VW55X_SYSCFG_EXTISS0_OFFSET)
#define GD32VW55X_SYSCFG_EXTISS1 \
  (GD32VW55X_SYSCFG_BASE + GD32VW55X_SYSCFG_EXTISS1_OFFSET)
#define GD32VW55X_SYSCFG_EXTISS2 \
  (GD32VW55X_SYSCFG_BASE + GD32VW55X_SYSCFG_EXTISS2_OFFSET)
#define GD32VW55X_SYSCFG_EXTISS3 \
  (GD32VW55X_SYSCFG_BASE + GD32VW55X_SYSCFG_EXTISS3_OFFSET)
#define GD32VW55X_SYSCFG_CPSCTL \
  (GD32VW55X_SYSCFG_BASE + GD32VW55X_SYSCFG_CPSCTL_OFFSET)
#define GD32VW55X_SYSCFG_CFG1 \
  (GD32VW55X_SYSCFG_BASE + GD32VW55X_SYSCFG_CFG1_OFFSET)
#define GD32VW55X_SYSCFG_SCFG \
  (GD32VW55X_SYSCFG_BASE + GD32VW55X_SYSCFG_SCFG_OFFSET)

/* There are four EXTI source selection registers, each one holding the
 * source of four EXTI lines in a 4-bit field.
 */

#define GD32VW55X_SYSCFG_EXTISS(pin) \
  (GD32VW55X_SYSCFG_EXTISS0 + (((pin) >> 2) << 2))

/* Register bitfield definitions ********************************************/

/* SYSCFG configuration register 0 (CFG0) */

#define SYSCFG_CFG0_BOOT_MODE_SHIFT       0
#define SYSCFG_CFG0_BOOT_MODE_MASK        (3 << SYSCFG_CFG0_BOOT_MODE_SHIFT)
#  define SYSCFG_CFG0_BOOT_MODE_FLASH     (0 << SYSCFG_CFG0_BOOT_MODE_SHIFT)
#  define SYSCFG_CFG0_BOOT_MODE_SRAM      (1 << SYSCFG_CFG0_BOOT_MODE_SHIFT)
#  define SYSCFG_CFG0_BOOT_MODE_SYSTEM    (2 << SYSCFG_CFG0_BOOT_MODE_SHIFT)

/* SYSCFG EXTI source selection registers (EXTISS0-3) */

#define SYSCFG_EXTISS_SHIFT(pin)          (((pin) & 3) << 2)
#define SYSCFG_EXTISS_MASK(pin)           (15 << SYSCFG_EXTISS_SHIFT(pin))

#define SYSCFG_EXTI_SOURCE_GPIOA          0
#define SYSCFG_EXTI_SOURCE_GPIOB          1
#define SYSCFG_EXTI_SOURCE_GPIOC          2

/* SYSCFG I/O compensation control register (CPSCTL) */

#define SYSCFG_CPSCTL_CPS_EN              (1 << 0)  /* Compensation cell en */
#define SYSCFG_CPSCTL_CPS_RDY             (1 << 8)  /* Compensation ready */

/* SYSCFG configuration register 1 (CFG1) */

#define SYSCFG_CFG1_LVD_LOCK              (1 << 2)  /* LVD lock enable */

/* SYSCFG shared SRAM configuration register (SCFG) */

#define SYSCFG_SCFG_SOWNSEL               (1 << 0)  /* 1=core owns SRAM */

#endif /* __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_SYSCFG_H */
