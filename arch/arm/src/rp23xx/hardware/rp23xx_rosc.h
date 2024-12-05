/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_rosc.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_ROSC_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_ROSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_ROSC_CTRL_OFFSET       0x00000000
#define RP23XX_ROSC_FREQA_OFFSET      0x00000004
#define RP23XX_ROSC_FREQB_OFFSET      0x00000008
#define RP23XX_ROSC_RANDOM_OFFSET     0x0000000c
#define RP23XX_ROSC_DORMANT_OFFSET    0x00000010
#define RP23XX_ROSC_DIV_OFFSET        0x00000014
#define RP23XX_ROSC_PHASE_OFFSET      0x00000018
#define RP23XX_ROSC_STATUS_OFFSET     0x0000001c
#define RP23XX_ROSC_RANDOMBIT_OFFSET  0x00000020
#define RP23XX_ROSC_COUNT_OFFSET      0x00000024

/* Register definitions *****************************************************/

#define RP23XX_ROSC_CTRL        (RP23XX_ROSC_BASE + RP23XX_ROSC_CTRL_OFFSET)
#define RP23XX_ROSC_FREQA       (RP23XX_ROSC_BASE + RP23XX_ROSC_FREQA_OFFSET)
#define RP23XX_ROSC_FREQB       (RP23XX_ROSC_BASE + RP23XX_ROSC_FREQB_OFFSET)
#define RP23XX_ROSC_RANDOM      (RP23XX_ROSC_BASE + RP23XX_ROSC_RANDOM_OFFSET)
#define RP23XX_ROSC_DORMANT     (RP23XX_ROSC_BASE + RP23XX_ROSC_DORMANT_OFFSET)
#define RP23XX_ROSC_DIV         (RP23XX_ROSC_BASE + RP23XX_ROSC_DIV_OFFSET)
#define RP23XX_ROSC_PHASE       (RP23XX_ROSC_BASE + RP23XX_ROSC_PHASE_OFFSET)
#define RP23XX_ROSC_STATUS      (RP23XX_ROSC_BASE + RP23XX_ROSC_STATUS_OFFSET)
#define RP23XX_ROSC_RANDOMBIT   (RP23XX_ROSC_BASE + RP23XX_ROSC_RANDOMBIT_OFFSET)
#define RP23XX_ROSC_COUNT       (RP23XX_ROSC_BASE + RP23XX_ROSC_COUNT_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_ROSC_CTRL_ENABLE_SHIFT        (12)  /* On power-up this field is initialised to ENABLE The system clock must be switched to another source before setting this field to DISABLE otherwise the chip will lock up The 12-bit code is intended to give some protection against accidental writes. An invalid setting will enable the oscillator. */
#define RP23XX_ROSC_CTRL_ENABLE_MASK         (0xfff << RP23XX_ROSC_CTRL_ENABLE_SHIFT)
#define RP23XX_ROSC_CTRL_ENABLE_DISABLE      (0xd1e << RP23XX_ROSC_CTRL_ENABLE_SHIFT)
#define RP23XX_ROSC_CTRL_ENABLE_ENABLE       (0xfab << RP23XX_ROSC_CTRL_ENABLE_SHIFT)
#define RP23XX_ROSC_CTRL_FREQ_RANGE_MASK     (0xfff)
#define RP23XX_ROSC_CTRL_FREQ_RANGE_LOW      (0xfa4)
#define RP23XX_ROSC_CTRL_FREQ_RANGE_MEDIUM   (0xfa5)
#define RP23XX_ROSC_CTRL_FREQ_RANGE_HIGH     (0xfa7)
#define RP23XX_ROSC_CTRL_FREQ_RANGE_TOOHIGH  (0xfa6)

#define RP23XX_ROSC_FREQA_PASSWD_SHIFT       (16)    /* Set to 0x9696 to apply the settings Any other value in this field will set all drive strengths to 0 */
#define RP23XX_ROSC_FREQA_PASSWD_MASK        (0xffff << RP23XX_ROSC_FREQA_PASSWD_SHIFT)
#define RP23XX_ROSC_FREQA_PASSWD_PASS        (0x9696 << RP23XX_ROSC_FREQA_PASSWD_SHIFT)
#define RP23XX_ROSC_FREQA_DS3_SHIFT          (12)    /* Stage 3 drive strength */
#define RP23XX_ROSC_FREQA_DS3_MASK           (0x07 << RP23XX_ROSC_FREQA_DS3_SHIFT)
#define RP23XX_ROSC_FREQA_DS2_SHIFT          (8)     /* Stage 2 drive strength */
#define RP23XX_ROSC_FREQA_DS2_MASK           (0x07 << RP23XX_ROSC_FREQA_DS2_SHIFT)
#define RP23XX_ROSC_FREQA_DS1_SHIFT          (4)     /* Stage 1 drive strength */
#define RP23XX_ROSC_FREQA_DS1_MASK           (0x07 << RP23XX_ROSC_FREQA_DS1_SHIFT)
#define RP23XX_ROSC_FREQA_DS0_MASK           (0x07)  /* Stage 0 drive strength */

#define RP23XX_ROSC_FREQB_PASSWD_SHIFT       (16)    /* Set to 0x9696 to apply the settings Any other value in this field will set all drive strengths to 0 */
#define RP23XX_ROSC_FREQB_PASSWD_MASK        (0xffff << RP23XX_ROSC_FREQB_PASSWD_SHIFT)
#define RP23XX_ROSC_FREQB_PASSWD_PASS        (0x9696 << RP23XX_ROSC_FREQB_PASSWD_SHIFT)
#define RP23XX_ROSC_FREQB_DS7_SHIFT          (12)    /* Stage 7 drive strength */
#define RP23XX_ROSC_FREQB_DS7_MASK           (0x07 << RP23XX_ROSC_FREQB_DS7_SHIFT)
#define RP23XX_ROSC_FREQB_DS6_SHIFT          (8)     /* Stage 6 drive strength */
#define RP23XX_ROSC_FREQB_DS6_MASK           (0x07 << RP23XX_ROSC_FREQB_DS6_SHIFT)
#define RP23XX_ROSC_FREQB_DS5_SHIFT          (4)     /* Stage 5 drive strength */
#define RP23XX_ROSC_FREQB_DS5_MASK           (0x07 << RP23XX_ROSC_FREQB_DS5_SHIFT)
#define RP23XX_ROSC_FREQB_DS4_MASK           (0x07)  /* Stage 4 drive strength */

#define RP23XX_ROSC_DORMANT_DORMANT          (0x636f6d61)
#define RP23XX_ROSC_DORMANT_WAKE             (0x77616b65)

#define RP23XX_ROSC_DIV_MASK                 (0xffff)
#define RP23XX_ROSC_DIV_PASS                 (0xaa00)

#define RP23XX_ROSC_PHASE_PASSWD_SHIFT       (4)       /* set to 0xaa0 any other value enables the output with shift=0 */
#define RP23XX_ROSC_PHASE_PASSWD_MASK        (0xff << RP23XX_ROSC_PHASE_PASSWD_SHIFT)
#define RP23XX_ROSC_PHASE_ENABLE             (1 << 3)  /* enable the phase-shifted output this can be changed on-the-fly */
#define RP23XX_ROSC_PHASE_FLIP               (1 << 2)  /* invert the phase-shifted output this is ignored when div=1 */
#define RP23XX_ROSC_PHASE_SHIFT_MASK         (0x03)    /* phase shift the phase-shifted output by SHIFT input clocks this can be changed on-the-fly must be set to 0 before setting div=1 */

#define RP23XX_ROSC_STATUS_STABLE            (1 << 31) /* Oscillator is running and stable */
#define RP23XX_ROSC_STATUS_BADWRITE          (1 << 24) /* An invalid value has been written to CTRL_ENABLE or CTRL_FREQ_RANGE or FRFEQA or FREQB or DORMANT */
#define RP23XX_ROSC_STATUS_DIV_RUNNING       (1 << 16) /* post-divider is running this resets to 0 but transitions to 1 during chip startup */
#define RP23XX_ROSC_STATUS_ENABLED           (1 << 12) /* Oscillator is enabled but not necessarily running and stable this resets to 0 but transitions to 1 during chip startup */

#define RP23XX_ROSC_RANDOMBIT_MASK           (1 << 0)

#define RP23XX_ROSC_COUNT_MASK               (0xffff)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_ROSC_H */
