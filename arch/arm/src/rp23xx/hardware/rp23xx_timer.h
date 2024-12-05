/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_timer.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_TIMER_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_TIMER_TIMEHW_OFFSET   0x00000000
#define RP23XX_TIMER_TIMELW_OFFSET   0x00000004
#define RP23XX_TIMER_TIMEHR_OFFSET   0x00000008
#define RP23XX_TIMER_TIMELR_OFFSET   0x0000000c
#define RP23XX_TIMER_ALARM0_OFFSET   0x00000010
#define RP23XX_TIMER_ALARM1_OFFSET   0x00000014
#define RP23XX_TIMER_ALARM2_OFFSET   0x00000018
#define RP23XX_TIMER_ALARM3_OFFSET   0x0000001c
#define RP23XX_TIMER_ARMED_OFFSET    0x00000020
#define RP23XX_TIMER_TIMERAWH_OFFSET 0x00000024
#define RP23XX_TIMER_TIMERAWL_OFFSET 0x00000028
#define RP23XX_TIMER_DBGPAUSE_OFFSET 0x0000002c
#define RP23XX_TIMER_PAUSE_OFFSET    0x00000030
#define RP23XX_TIMER_LOCKED_OFFSET   0x00000034
#define RP23XX_TIMER_SOURCE_OFFSET   0x00000038
#define RP23XX_TIMER_INTR_OFFSET     0x0000003c
#define RP23XX_TIMER_INTE_OFFSET     0x00000040
#define RP23XX_TIMER_INTF_OFFSET     0x00000044
#define RP23XX_TIMER_INTS_OFFSET     0x00000048

/* Register definitions *****************************************************/

#define RP23XX_TIMER_TIMEHW   (RP23XX_TIMER_BASE + RP23XX_TIMER_TIMEHW_OFFSET)
#define RP23XX_TIMER_TIMELW   (RP23XX_TIMER_BASE + RP23XX_TIMER_TIMELW_OFFSET)
#define RP23XX_TIMER_TIMEHR   (RP23XX_TIMER_BASE + RP23XX_TIMER_TIMEHR_OFFSET)
#define RP23XX_TIMER_TIMELR   (RP23XX_TIMER_BASE + RP23XX_TIMER_TIMELR_OFFSET)
#define RP23XX_TIMER_ALARM0   (RP23XX_TIMER_BASE + RP23XX_TIMER_ALARM0_OFFSET)
#define RP23XX_TIMER_ALARM1   (RP23XX_TIMER_BASE + RP23XX_TIMER_ALARM1_OFFSET)
#define RP23XX_TIMER_ALARM2   (RP23XX_TIMER_BASE + RP23XX_TIMER_ALARM2_OFFSET)
#define RP23XX_TIMER_ALARM3   (RP23XX_TIMER_BASE + RP23XX_TIMER_ALARM3_OFFSET)
#define RP23XX_TIMER_ARMED    (RP23XX_TIMER_BASE + RP23XX_TIMER_ARMED_OFFSET)
#define RP23XX_TIMER_TIMERAWH (RP23XX_TIMER_BASE + RP23XX_TIMER_TIMERAWH_OFFSET)
#define RP23XX_TIMER_TIMERAWL (RP23XX_TIMER_BASE + RP23XX_TIMER_TIMERAWL_OFFSET)
#define RP23XX_TIMER_DBGPAUSE (RP23XX_TIMER_BASE + RP23XX_TIMER_DBGPAUSE_OFFSET)
#define RP23XX_TIMER_PAUSE    (RP23XX_TIMER_BASE + RP23XX_TIMER_PAUSE_OFFSET)
#define RP23XX_TIMER_LOCKED   (RP23XX_TIMER_BASE + RP23XX_TIMER_LOCKED_OFFSET)
#define RP23XX_TIMER_SOURCE   (RP23XX_TIMER_BASE + RP23XX_TIMER_SOURCE_OFFSET)
#define RP23XX_TIMER_INTR     (RP23XX_TIMER_BASE + RP23XX_TIMER_INTR_OFFSET)
#define RP23XX_TIMER_INTE     (RP23XX_TIMER_BASE + RP23XX_TIMER_INTE_OFFSET)
#define RP23XX_TIMER_INTF     (RP23XX_TIMER_BASE + RP23XX_TIMER_INTF_OFFSET)
#define RP23XX_TIMER_INTS     (RP23XX_TIMER_BASE + RP23XX_TIMER_INTS_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_TIMER_DBGPAUSE_DBG1_MASK  (1 << 2)
#define RP23XX_TIMER_DBGPAUSE_DBG0_MASK  (1 << 1)
#define RP23XX_TIMER_PAUSE_MASK          (1 << 0)
#define RP23XX_TIMER_LOCKED_MASK         (1 << 0)
#define RP23XX_TIMER_SOURCE_MASK         (1 << 0)
#define RP23XX_TIMER_SOURCE_CLK_SYS_MASK (1 << 0)
#define RP23XX_TIMER_INTR_ALARM_3_MASK   (1 << 3)
#define RP23XX_TIMER_INTR_ALARM_2_MASK   (1 << 2)
#define RP23XX_TIMER_INTR_ALARM_1_MASK   (1 << 1)
#define RP23XX_TIMER_INTR_ALARM_0_MASK   (1 << 0)
#define RP23XX_TIMER_INTE_ALARM_3_MASK   (1 << 3)
#define RP23XX_TIMER_INTE_ALARM_2_MASK   (1 << 2)
#define RP23XX_TIMER_INTE_ALARM_1_MASK   (1 << 1)
#define RP23XX_TIMER_INTE_ALARM_0_MASK   (1 << 0)
#define RP23XX_TIMER_INTF_ALARM_3_MASK   (1 << 3)
#define RP23XX_TIMER_INTF_ALARM_2_MASK   (1 << 2)
#define RP23XX_TIMER_INTF_ALARM_1_MASK   (1 << 1)
#define RP23XX_TIMER_INTF_ALARM_0_MASK   (1 << 0)
#define RP23XX_TIMER_INTS_ALARM_3_MASK   (1 << 3)
#define RP23XX_TIMER_INTS_ALARM_2_MASK   (1 << 2)
#define RP23XX_TIMER_INTS_ALARM_1_MASK   (1 << 1)
#define RP23XX_TIMER_INTS_ALARM_0_MASK   (1 << 0)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_TIMER_H */
