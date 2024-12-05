/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_bootram.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_BOOTRAM_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_BOOTRAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_BOOTRAM_WRITE_ONCE_OFFSET(n)     ((n) * 4 + 0x000800)
#define RP23XX_BOOTRAM_BOOTLOCK_STAT_OFFSET     0x00000808
#define RP23XX_BOOTRAM_BOOTLOCK_OFFSET(n)       ((n) * 4 + 0x00080c)

/* Register definitions *****************************************************/

#define RP23XX_BOOTRAM_WRITE_ONCE(n)   (RP23XX_BOOTRAM_BASE + RP23XX_BOOTRAM_WRITE_ONCE_OFFSET(n))
#define RP23XX_BOOTRAM_BOOTLOCK_STAT   (RP23XX_BOOTRAM_BASE + RP23XX_BOOTRAM_BOOTLOCK_STAT_OFFSET)
#define RP23XX_BOOTRAM_BOOTLOCK(n)   (RP23XX_BOOTRAM_BASE + RP23XX_BOOTRAM_BOOTLOCK_OFFSET(n))

/* Register bit definitions *************************************************/

#define RP23XX_BOOTRAM_WRITE_ONCE_MASK      0xffffffff
#define RP23XX_BOOTRAM_BOOTLOCK_STAT_MASK   0x000000ff
#define RP23XX_BOOTRAM_BOOTLOCK_MASK        0xffffffff

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_BOOTRAM_H */
