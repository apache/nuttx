/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_xosc.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_XOSC_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_XOSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_XOSC_CTRL_OFFSET     0x00000000  /* Crystal Oscillator Control */
#define RP23XX_XOSC_STATUS_OFFSET   0x00000004  /* Crystal Oscillator Status */
#define RP23XX_XOSC_DORMANT_OFFSET  0x00000008  /* Crystal Oscillator pause control This is used to save power by pausing the XOSC On power-up this field is initialised to WAKE An invalid write will also select WAKE WARNING: stop the PLLs before selecting dormant mode WARNING: setup the irq before selecting dormant mode */
#define RP23XX_XOSC_STARTUP_OFFSET  0x0000000c  /* Controls the startup delay */
#define RP23XX_XOSC_COUNT_OFFSET    0x00000010  /* A down counter running at the xosc frequency which counts to zero and stops. To start the counter write a non-zero value. Can be used for short software pauses when setting up time sensitive hardware. */

/* Register definitions *****************************************************/

#define RP23XX_XOSC_CTRL     (RP23XX_XOSC_BASE + RP23XX_XOSC_CTRL_OFFSET)
#define RP23XX_XOSC_STATUS   (RP23XX_XOSC_BASE + RP23XX_XOSC_STATUS_OFFSET)
#define RP23XX_XOSC_DORMANT  (RP23XX_XOSC_BASE + RP23XX_XOSC_DORMANT_OFFSET)
#define RP23XX_XOSC_STARTUP  (RP23XX_XOSC_BASE + RP23XX_XOSC_STARTUP_OFFSET)
#define RP23XX_XOSC_COUNT    (RP23XX_XOSC_BASE + RP23XX_XOSC_COUNT_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_XOSC_CTRL_ENABLE_SHIFT             (12)  /* On power-up this field is initialised to DISABLE and the chip runs from the ROSC. If the chip has subsequently been programmed to run from the XOSC then setting this field to DISABLE may lock-up the chip. If this is a concern then run the clk_ref from the ROSC and enable the clk_sys RESUS feature. The 12-bit code is intended to give some protection against accidental writes. An invalid setting will enable the oscillator. */
#define RP23XX_XOSC_CTRL_ENABLE_MASK              (0xfff << RP23XX_XOSC_CTRL_ENABLE_SHIFT)
#define RP23XX_XOSC_CTRL_ENABLE_DISABLE           (0xd1e << RP23XX_XOSC_CTRL_ENABLE_SHIFT)
#define RP23XX_XOSC_CTRL_ENABLE_ENABLE            (0xfab << RP23XX_XOSC_CTRL_ENABLE_SHIFT)
#define RP23XX_XOSC_CTRL_FREQ_RANGE_MASK          (0xfff)
#define RP23XX_XOSC_CTRL_FREQ_RANGE_1_15MHZ       (0xaa0)
#define RP23XX_XOSC_CTRL_FREQ_RANGE_10_30MHZ      (0xaa1)
#define RP23XX_XOSC_CTRL_FREQ_RANGE_25_60MHZ      (0xaa2)
#define RP23XX_XOSC_CTRL_FREQ_RANGE_40_100MHZ     (0xaa3)

#define RP23XX_XOSC_STATUS_STABLE                 (1 << 31)  /* Oscillator is running and stable */
#define RP23XX_XOSC_STATUS_BADWRITE               (1 << 24)  /* An invalid value has been written to CTRL_ENABLE or CTRL_FREQ_RANGE or DORMANT */
#define RP23XX_XOSC_STATUS_ENABLED                (1 << 12)  /* Oscillator is enabled but not necessarily running and stable, resets to 0 */
#define RP23XX_XOSC_STATUS_FREQ_RANGE_MASK        (0x03)
#define RP23XX_XOSC_STATUS_FREQ_RANGE_1_15MHZ     (0x0)
#define RP23XX_XOSC_STATUS_FREQ_RANGE_10_30MHZ    (0x1)
#define RP23XX_XOSC_STATUS_FREQ_RANGE_25_60MHZ    (0x2)
#define RP23XX_XOSC_STATUS_FREQ_RANGE_40_100MHZ   (0x3)

#define RP23XX_XOSC_DORMANT_DORMANT               (0x636f6d61)
#define RP23XX_XOSC_DORMANT_WAKE                  (0x77616b65)

#define RP23XX_XOSC_STARTUP_X4                    (1 << 20)  /* Multiplies the startup_delay by 4. This is of little value to the user given that the delay can be programmed directly */
#define RP23XX_XOSC_STARTUP_DELAY_MASK            (0x3fff)   /* in multiples of 256*xtal_period */

#define RP23XX_XOSC_COUNT_MASK                    (0xffff)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_XOSC_H */
