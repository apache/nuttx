/****************************************************************************
 * arch/risc-v/src/bl808/hardware/bl808_timer.h
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

#ifndef __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_TIMER_H
#define __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl808_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BL808_TIMER_BASE(n) (((n) == 0) ? BL808_TIMER0_BASE \
                             : BL808_TIMER1_BASE)

/* Register offsets *********************************************************/

#define BL808_TIMER_TCCR_OFFSET        0x00
#define BL808_TIMER_CH0_COMP0_OFFSET   0x10
#define BL808_TIMER_CH1_COMP0_OFFSET   0x1c
#define BL808_TIMER_CH0_COUNTER_OFFSET 0x2c
#define BL808_TIMER_CH1_COUNTER_OFFSET 0x30
#define BL808_TIMER_CH0_IE_OFFSET      0x44
#define BL808_TIMER_CH1_IE_OFFSET      0x48
#define BL808_WDT_MODE_OFFSET          0x64
#define BL808_WDT_COMP_OFFSET          0x68
#define BL808_WDT_COUNTER_OFFSET       0x6C
#define BL808_TIMER_CH0_ICLR_OFFSET    0x78
#define BL808_TIMER_CH1_ICLR_OFFSET    0x7c
#define BL808_WDT_ICLR_OFFSET          0x80
#define BL808_TIMER_EN_CLR_OFFSET      0x84
#define BL808_TIMER_MODE_OFFSET        0x88
#define BL808_WDT_COUNT_CLEAR_OFFSET   0x98
#define BL808_WDT_KEY1_OFFSET          0x9C
#define BL808_WDT_KEY2_OFFSET          0xA0
#define BL808_TIMER_DIV_OFFSET         0xBC

/* Register definitions *****************************************************/

#define BL808_TIMER_TCCR(n) (BL808_TIMER_BASE(n) + BL808_TIMER_TCCR_OFFSET)
#define BL808_TIMER_CH0_COMP0(n) (BL808_TIMER_BASE(n) + BL808_TIMER_CH0_COMP0_OFFSET)
#define BL808_TIMER_CH1_COMP0(n) (BL808_TIMER_BASE(n) + BL808_TIMER_CH1_COMP0_OFFSET)
#define BL808_TIMER_CH0_COUNTER(n) (BL808_TIMER_BASE(n) + BL808_TIMER_CH0_COUNTER_OFFSET)
#define BL808_TIMER_CH1_COUNTER(n) (BL808_TIMER_BASE(n) + BL808_TIMER_CH1_COUNTER_OFFSET)
#define BL808_TIMER_CH0_IE(n) (BL808_TIMER_BASE(n) + BL808_TIMER_CH0_IE_OFFSET)
#define BL808_TIMER_CH1_IE(n) (BL808_TIMER_BASE(n) + BL808_TIMER_CH1_IE_OFFSET)
#define BL808_WDT_MODE(n) (BL808_TIMER_BASE(n) + BL808_WDT_MODE_OFFSET)
#define BL808_WDT_COMP(n) (BL808_TIMER_BASE(n) + BL808_WDT_COMP_OFFSET)
#define BL808_WDT_COUNTER(n) (BL808_TIMER_BASE(n) + BL808_WDT_COUNTER_OFFSET)
#define BL808_TIMER_CH0_ICLR(n) (BL808_TIMER_BASE(n) + BL808_TIMER_CH0_ICLR_OFFSET)
#define BL808_TIMER_CH1_ICLR(n) (BL808_TIMER_BASE(n) + BL808_TIMER_CH1_ICLR_OFFSET)
#define BL808_WDT_ICLR(n) (BL808_TIMER_BASE(n) + BL808_WDT_ICLR_OFFSET)
#define BL808_TIMER_EN_CLR(n) (BL808_TIMER_BASE(n) + BL808_TIMER_EN_CLR_OFFSET)
#define BL808_TIMER_MODE(n) (BL808_TIMER_BASE(n) + BL808_TIMER_MODE_OFFSET)
#define BL808_WDT_COUNT_CLEAR(n) (BL808_TIMER_BASE(n) + BL808_WDT_COUNT_CLEAR_OFFSET)
#define BL808_WDT_KEY1(n) (BL808_TIMER_BASE(n) + BL808_WDT_KEY1_OFFSET)
#define BL808_WDT_KEY2(n) (BL808_TIMER_BASE(n) + BL808_WDT_KEY2_OFFSET)
#define BL808_TIMER_DIV(n) (BL808_TIMER_BASE(n) + BL808_TIMER_DIV_OFFSET)

/* Register bit definitions *************************************************/

/* TIMER_TCCR */

#define TIMER_CH0_CLKSEL_SHIFT   (0)
#define TIMER_CH0_CLKSEL_MASK    (0xf << TIMER_CH0_CLKSEL_SHIFT)
#define TIMER_CH1_CLKSEL_SHIFT   (4)
#define TIMER_CH1_CLKSEL_MASK    (0xf << TIMER_CH1_CLKSEL_SHIFT)
#define WDT_CLKSEL_SHIFT         (8)
#define WDT_CLKSEL_MASK          (0xf << WDT_CLKSEL_SHIFT)

/* TIMER_CH(0/1)_I(E/CLR) */

#define TIMER_COMP0_INT          (1 << 0)

/* WDT_MODE */

#define WDT_EN                   (1 << 0)
#define WDT_RESET_EN             (1 << 1)

/* WDT_COMP */

#define WDT_COMP_MASK            (0xffff)

/* WDT_ICLR */

#define WDT_CLEAR_IRQ            (1 << 0)

/* TIMER_EN_CLR */

#define TIMER_CH0_EN             (1 << 1)
#define TIMER_CH1_EN             (1 << 2)
#define TIMER_CH0_CLR            (1 << 5)
#define TIMER_CH1_CLR            (1 << 6)

/* TIMER_MODE */

#define TIMER_CH0_MODE           (1 << 1)
#define TIMER_CH1_MODE           (1 << 2)

/* WDT_COUNT_CLEAR */

#define WDT_CLEAR_COUNT          (1 << 0)

/* TIMER_DIV */

#define TIMER_CH0_DIV_SHIFT      (8)
#define TIMER_CH0_DIV_MASK       (0xff << TIMER_CH0_DIV_SHIFT)
#define TIMER_CH1_DIV_SHIFT      (16)
#define TIMER_CH1_DIV_MASK       (0xff << TIMER_CH1_DIV_SHIFT)

#endif /* __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_TIMER_H */
