/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_xip.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_XIP_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_XIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_XIP_CTRL_OFFSET          0x00000000
#define RP23XX_XIP_STAT_OFFSET          0x00000008
#define RP23XX_XIP_CTR_HIT_OFFSET       0x0000000c
#define RP23XX_XIP_CTR_ACC_OFFSET       0x00000010
#define RP23XX_XIP_STREAM_ADDR_OFFSET   0x00000014
#define RP23XX_XIP_STREAM_CTR_OFFSET    0x00000018
#define RP23XX_XIP_STREAM_FIFO_OFFSET   0x0000001c

/* Register definitions *****************************************************/

#define RP23XX_XIP_CTRL         (RP23XX_XIP_BASE + RP23XX_XIP_CTRL_OFFSET)
#define RP23XX_XIP_STAT         (RP23XX_XIP_BASE + RP23XX_XIP_STAT_OFFSET)
#define RP23XX_XIP_CTR_HIT      (RP23XX_XIP_BASE + RP23XX_XIP_CTR_HIT_OFFSET)
#define RP23XX_XIP_CTR_ACC      (RP23XX_XIP_BASE + RP23XX_XIP_CTR_ACC_OFFSET)
#define RP23XX_XIP_STREAM_ADDR  (RP23XX_XIP_BASE + RP23XX_XIP_STREAM_ADDR_OFFSET)
#define RP23XX_XIP_STREAM_CTR   (RP23XX_XIP_BASE + RP23XX_XIP_STREAM_CTR_OFFSET)
#define RP23XX_XIP_STREAM_FIFO  (RP23XX_XIP_BASE + RP23XX_XIP_STREAM_FIFO_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_XIP_CTRL_WRITABLE_M1             (1 << 11)
#define RP23XX_XIP_CTRL_WRITABLE_M0             (1 << 10)
#define RP23XX_XIP_CTRL_SPLIT_WAYS              (1 << 9)
#define RP23XX_XIP_CTRL_MAINT_NONSEC            (1 << 8)
#define RP23XX_XIP_CTRL_NO_UNTRANSLATED_NONSEC  (1 << 7)
#define RP23XX_XIP_CTRL_NO_UNTRANSLATED_SEC     (1 << 6)
#define RP23XX_XIP_CTRL_NO_UNCACHED_NONSEC      (1 << 5)
#define RP23XX_XIP_CTRL_NO_UNCACHED_SEC         (1 << 4)
#define RP23XX_XIP_CTRL_POWER_DOWN              (1 << 3)
#define RP23XX_XIP_CTRL_EN_NONSECURE            (1 << 1)
#define RP23XX_XIP_CTRL_EN_SECURE               (1 << 0)

#define RP23XX_XIP_STAT_FIFO_FULL               (1 << 2)
#define RP23XX_XIP_STAT_FIFO_EMPTY              (1 << 1)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_XIP_H */
