/****************************************************************************
 * arch/arm/src/tlsr82/hardware/tlsr82_timer.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_TIMER_H
#define __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/tlsr82/chip.h>

#include "hardware/tlsr82_register.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Watchdog Register definitions */

#define WDOG_STATUS_REG               REG_ADDR8(0x72)

#define WDOG_CTRL_REG                 REG_ADDR32(0x620)

/* Bit definitions */

#define WDOG_STATUS_RESET_SHIFT       0
#define WDOG_STATUS_RESET_MASK        (0x1 << WDOG_STATUS_RESET_SHIFT)
#define WDOG_STATUS_RESET_YES         (0x1 << WDOG_STATUS_RESET_SHIFT)
#define WDOG_STATUS_RESET_NO          (0x0 << WDOG_STATUS_RESET_SHIFT)

#define WDOG_CTRL_CAPT_SHIFT          9
#define WDOG_CTRL_CAPT_MASK           (0x3fff << WDOG_CTRL_CAPT_SHIFT)

#define WDOG_CTRL_ENABLE_SHIFT        23
#define WDOG_CTRL_ENABLE_MASK         (0x1 << WDOG_CTRL_ENABLE_SHIFT)
#define WDOG_CTRL_ENABLE_ON           (0x1 << WDOG_CTRL_ENABLE_SHIFT)
#define WDOG_CTRL_ENABLE_OFF          (0x0 << WDOG_CTRL_ENABLE_SHIFT)

/* Timer Regisger definitions */

#define TIMER_CTRL_REG                REG_ADDR32(0x620)

#define TIMER_STATUS_REG              REG_ADDR8(0x623)

#define TIMER_CAPT_REG(n)             REG_ADDR32(0x624 + ((n) << 2))
#define TIMER_CAPT0_REG               REG_ADDR32(0x624)
#define TIMER_CAPT1_REG               REG_ADDR32(0x628)
#define TIMER_CAPT2_REG               REG_ADDR32(0x62c)

#define TIMER_TICK_REG(n)             REG_ADDR32(0x630 + ((n) << 2))
#define TIMER_TICK0_REG               REG_ADDR32(0x630)
#define TIMER_TICK1_REG               REG_ADDR32(0x634)
#define TIMER_TICK2_REG               REG_ADDR32(0x638)

#define SYSTIMER_TICK_REG             REG_ADDR32(0x740)
#define SYSTIMER_CAPT_REG             REG_ADDR32(0x744)
#define SYSTIMER_CALI_REG             REG_ADDR8(0x749)
#define SYSTIMER_CTRL_REG             REG_ADDR8(0x74a)
#define SYSTIMER_STATUS_REG           REG_ADDR8(0x74b)
#define SYSTIMER_IRQ_MASK_REG         REG_ADDR8(0x748)

/* Bit definition */

#define TIMER_CTRL_T0_ENABLE_SHIFT    0
#define TIMER_CTRL_T0_MODE_SHIFT      1
#define TIMER_CTRL_T1_ENABLE_SHIFT    3
#define TIMER_CTRL_T1_MODE_SHIFT      4
#define TIMER_CTRL_T2_ENABLE_SHIFT    6
#define TIMER_CTRL_T2_MODE_SHIFT      7
#define TIMER_CTRL_WDOG_CAPT_SHIFT    9

#define TIMER_CTRL_T0_ENABLE          BIT(0)
#define TIMER_CTRL_T0_MODE            BIT_RNG(1, 2)
#define TIMER_CTRL_T1_ENABLE          BIT(3)
#define TIMER_CTRL_T1_MODE            BIT_RNG(4, 5)
#define TIMER_CTRL_T2_ENABLE          BIT(6)
#define TIMER_CTRL_T2_MODE            BIT_RNG(7, 8)
#define TIMER_CTRL_WDOG_CAPT          BIT(9, 22)

#define TIMER_STATUS_T0_CLR           BIT(0)
#define TIMER_STATUS_T1_CLR           BIT(1)
#define TIMER_STATUS_T2_CLR           BIT(2)
#define TIMER_STATUS_WDOG_CLR         BIT(3)

#define SYSTIMER_CTRL_TIMER_EN        BIT(1)
#define SYSTIMER_CTRL_CALI_EN         BIT(3)

#define SYSTIMER_IRQ_MASK_EN          BIT(2)

#endif /* __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_TIMER_H */
