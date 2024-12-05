/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_psm.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_PSM_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_PSM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_PSM_FRCE_ON_OFFSET   0x000000  /* Force block out of reset (i.e. power it on) */
#define RP23XX_PSM_FRCE_OFF_OFFSET  0x000004  /* Force into reset (i.e. power it off) */
#define RP23XX_PSM_WDSEL_OFFSET     0x000008  /* Set to 1 if this peripheral should be reset when the watchdog fires. */
#define RP23XX_PSM_DONE_OFFSET      0x00000c  /* Indicates the peripheral's registers are ready to access. */

/* Register definitions *****************************************************/

#define RP23XX_PSM_FRCE_ON   (RP23XX_PSM_BASE + RP23XX_PSM_FRCE_ON_OFFSET)
#define RP23XX_PSM_FRCE_OFF  (RP23XX_PSM_BASE + RP23XX_PSM_FRCE_OFF_OFFSET)
#define RP23XX_PSM_WDSEL     (RP23XX_PSM_BASE + RP23XX_PSM_WDSEL_OFFSET)
#define RP23XX_PSM_DONE      (RP23XX_PSM_BASE + RP23XX_PSM_DONE_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_PSM_PROC1                 (1 << 24)
#define RP23XX_PSM_PROC0                 (1 << 23)
#define RP23XX_PSM_ACCESSCTRL            (1 << 22)
#define RP23XX_PSM_SIO                   (1 << 21)
#define RP23XX_PSM_XIP                   (1 << 20)
#define RP23XX_PSM_SRAM9                 (1 << 19)
#define RP23XX_PSM_SRAM8                 (1 << 18)
#define RP23XX_PSM_SRAM7                 (1 << 17)
#define RP23XX_PSM_SRAM6                 (1 << 16)
#define RP23XX_PSM_SRAM5                 (1 << 15)
#define RP23XX_PSM_SRAM4                 (1 << 14)
#define RP23XX_PSM_SRAM3                 (1 << 13)
#define RP23XX_PSM_SRAM2                 (1 << 12)
#define RP23XX_PSM_SRAM1                 (1 << 11)
#define RP23XX_PSM_SRAM0                 (1 << 10)
#define RP23XX_PSM_BOOTRAM               (1 << 9)
#define RP23XX_PSM_ROM                   (1 << 8)
#define RP23XX_PSM_BUSFABRIC             (1 << 7)
#define RP23XX_PSM_PSM_READY             (1 << 6)
#define RP23XX_PSM_CLOCKS                (1 << 5)
#define RP23XX_PSM_RESETS                (1 << 4)
#define RP23XX_PSM_XOSC                  (1 << 3)
#define RP23XX_PSM_ROSC                  (1 << 2)
#define RP23XX_PSM_OTP                   (1 << 1)
#define RP23XX_PSM_PROC_COLD             (1 << 0)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_PSM_H */
