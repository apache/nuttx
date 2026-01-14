/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_ticks.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_TICKS_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_TICKS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_TICKS_PROC0_CTRL_OFFSET        0x00000000
#define RP23XX_TICKS_PROC0_CYCLES_OFFSET      0x00000004
#define RP23XX_TICKS_PROC0_COUNT_OFFSET       0x00000008

#define RP23XX_TICKS_PROC1_CTRL_OFFSET        0x0000000c
#define RP23XX_TICKS_PROC1_CYCLES_OFFSET      0x00000010
#define RP23XX_TICKS_PROC1_COUNT_OFFSET       0x00000014

#define RP23XX_TICKS_TIMER0_CTRL_OFFSET       0x00000018
#define RP23XX_TICKS_TIMER0_CYCLES_OFFSET     0x0000001c
#define RP23XX_TICKS_TIMER0_COUNT_OFFSET      0x00000020

#define RP23XX_TICKS_TIMER1_CTRL_OFFSET       0x00000024
#define RP23XX_TICKS_TIMER1_CYCLES_OFFSET     0x00000028
#define RP23XX_TICKS_TIMER1_COUNT_OFFSET      0x0000002c

#define RP23XX_TICKS_WATCHDOG_CTRL_OFFSET     0x00000030
#define RP23XX_TICKS_WATCHDOG_CYCLES_OFFSET   0x00000034
#define RP23XX_TICKS_WATCHDOG_COUNT_OFFSET    0x00000038

#define RP23XX_TICKS_CTRL_OFFSET(n)           ((n) * 12 + RP23XX_TICKS_PROC0_CTRL_OFFSET)
#define RP23XX_TICKS_CYCLES_OFFSET(n)         ((n) * 12 + RP23XX_TICKS_PROC0_CYCLES_OFFSET)
#define RP23XX_TICKS_COUNT_OFFSET(n)          ((n) * 12 + RP23XX_TICKS_PROC0_COUNT_OFFSET)

/* Register definitions *****************************************************/

#define RP23XX_TICKS_CTRL(n)           (RP23XX_TICKS_BASE + RP23XX_TICKS_CTRL_OFFSET(n))
#define RP23XX_TICKS_CYCLES(n)         (RP23XX_TICKS_BASE + RP23XX_TICKS_CYCLES_OFFSET(n))
#define RP23XX_TICKS_COUNT(n)          (RP23XX_TICKS_BASE + RP23XX_TICKS_COUNT_OFFSET(n))

#define RP23XX_TICKS_PROC0_CTRL        (RP23XX_TICKS_BASE + RP23XX_TICKS_PROC0_CTRL_OFFSET)
#define RP23XX_TICKS_PROC0_CYCLES      (RP23XX_TICKS_BASE + RP23XX_TICKS_PROC0_CYCLES_OFFSET)
#define RP23XX_TICKS_PROC0_COUNT       (RP23XX_TICKS_BASE + RP23XX_TICKS_PROC0_COUNT_OFFSET)

#define RP23XX_TICKS_PROC1_CTRL        (RP23XX_TICKS_BASE + RP23XX_TICKS_PROC1_CTRL_OFFSET)
#define RP23XX_TICKS_PROC1_CYCLES      (RP23XX_TICKS_BASE + RP23XX_TICKS_PROC1_CYCLES_OFFSET)
#define RP23XX_TICKS_PROC1_COUNT       (RP23XX_TICKS_BASE + RP23XX_TICKS_PROC1_COUNT_OFFSET)

#define RP23XX_TICKS_TIMER0_CTRL       (RP23XX_TICKS_BASE + RP23XX_TICKS_TIMER0_CTRL_OFFSET)
#define RP23XX_TICKS_TIMER0_CYCLES     (RP23XX_TICKS_BASE + RP23XX_TICKS_TIMER0_CYCLES_OFFSET)
#define RP23XX_TICKS_TIMER0_COUNT      (RP23XX_TICKS_BASE + RP23XX_TICKS_TIMER0_COUNT_OFFSET)

#define RP23XX_TICKS_TIMER1_CTRL       (RP23XX_TICKS_BASE + RP23XX_TICKS_TIMER1_CTRL_OFFSET)
#define RP23XX_TICKS_TIMER1_CYCLES     (RP23XX_TICKS_BASE + RP23XX_TICKS_TIMER1_CYCLES_OFFSET)
#define RP23XX_TICKS_TIMER1_COUNT      (RP23XX_TICKS_BASE + RP23XX_TICKS_TIMER1_COUNT_OFFSET)

#define RP23XX_TICKS_WATCHDOG_CTRL     (RP23XX_TICKS_BASE + RP23XX_TICKS_WATCHDOG_CTRL_OFFSET)
#define RP23XX_TICKS_WATCHDOG_CYCLES   (RP23XX_TICKS_BASE + RP23XX_TICKS_WATCHDOG_CYCLES_OFFSET)
#define RP23XX_TICKS_WATCHDOG_COUNT    (RP23XX_TICKS_BASE + RP23XX_TICKS_WATCHDOG_COUNT_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_TICKS_WATCHDOG_CTRL_EN             (1 << 0)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_TICKS_H */
