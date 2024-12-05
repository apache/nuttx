/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_watchdog.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_WATCHDOG_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_WATCHDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_WATCHDOG_CTRL_OFFSET         0x000000
#define RP23XX_WATCHDOG_LOAD_OFFSET         0x000004
#define RP23XX_WATCHDOG_REASON_OFFSET       0x000008
#define RP23XX_WATCHDOG_SCRATCH_OFFSET(n)   (0x00000c + (n) * 4)

/* Register definitions *****************************************************/

#define RP23XX_WATCHDOG_CTRL        (RP23XX_WATCHDOG_BASE + RP23XX_WATCHDOG_CTRL_OFFSET)
#define RP23XX_WATCHDOG_LOAD        (RP23XX_WATCHDOG_BASE + RP23XX_WATCHDOG_LOAD_OFFSET)
#define RP23XX_WATCHDOG_REASON      (RP23XX_WATCHDOG_BASE + RP23XX_WATCHDOG_REASON_OFFSET)
#define RP23XX_WATCHDOG_SCRATCH(n)  (RP23XX_WATCHDOG_BASE + RP23XX_WATCHDOG_SCRATCH_OFFSET(n))

/* Register bit definitions *************************************************/

#define RP23XX_WATCHDOG_CTRL_TRIGGER        (1 << 31)
#define RP23XX_WATCHDOG_CTRL_ENABLE         (1 << 30)
#define RP23XX_WATCHDOG_CTRL_PAUSE_DBG1     (1 << 26)
#define RP23XX_WATCHDOG_CTRL_PAUSE_DBG0     (1 << 25)
#define RP23XX_WATCHDOG_CTRL_PAUSE_JTAG     (1 << 24)
#define RP23XX_WATCHDOG_CTRL_TIME_MASK      (0xffffff)
#define RP23XX_WATCHDOG_LOAD_MASK           (0xffffff)
#define RP23XX_WATCHDOG_REASON_FORCE        (1 << 1)
#define RP23XX_WATCHDOG_REASON_TIMER        (1 << 0)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_WATCHDOG_H */
