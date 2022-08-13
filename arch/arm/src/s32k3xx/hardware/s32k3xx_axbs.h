/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_axbs.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_AXBS_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_AXBS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AXBS Register Offsets ****************************************************/

#define S32K3XX_AXBS_PRS0_OFFSET (0x0000) /* Priority Slave Register 0 (PRS0) */
#define S32K3XX_AXBS_CRS0_OFFSET (0x0010) /* Control Register 0 (CRS0) */
#define S32K3XX_AXBS_PRS1_OFFSET (0x0100) /* Priority Slave Register 1 (PRS1) */
#define S32K3XX_AXBS_CRS1_OFFSET (0x0110) /* Control Register 1 (CRS1) */
#define S32K3XX_AXBS_PRS2_OFFSET (0x0200) /* Priority Slave Register 2 (PRS2) */
#define S32K3XX_AXBS_CRS2_OFFSET (0x0210) /* Control Register 2 (CRS2) */
#define S32K3XX_AXBS_PRS3_OFFSET (0x0300) /* Priority Slave Register 3 (PRS3) */
#define S32K3XX_AXBS_CRS3_OFFSET (0x0310) /* Control Register 3 (CRS3) */
#define S32K3XX_AXBS_PRS4_OFFSET (0x0400) /* Priority Slave Register 4 (PRS4) */
#define S32K3XX_AXBS_CRS4_OFFSET (0x0410) /* Control Register 4 (CRS4) */
#define S32K3XX_AXBS_PRS5_OFFSET (0x0500) /* Priority Slave Register 5 (PRS5) */
#define S32K3XX_AXBS_CRS5_OFFSET (0x0510) /* Control Register 5 (CRS5) */
#define S32K3XX_AXBS_PRS6_OFFSET (0x0600) /* Priority Slave Register 6 (PRS6) */
#define S32K3XX_AXBS_CRS6_OFFSET (0x0610) /* Control Register 6 (CRS6) */

/* AXBS Register Addresses **************************************************/

#define S32K3XX_AXBS_PRS0        (S32K3XX_AXBS_BASE + S32K3XX_AXBS_PRS0_OFFSET)
#define S32K3XX_AXBS_CRS0        (S32K3XX_AXBS_BASE + S32K3XX_AXBS_CRS0_OFFSET)
#define S32K3XX_AXBS_PRS1        (S32K3XX_AXBS_BASE + S32K3XX_AXBS_PRS1_OFFSET)
#define S32K3XX_AXBS_CRS1        (S32K3XX_AXBS_BASE + S32K3XX_AXBS_CRS1_OFFSET)
#define S32K3XX_AXBS_PRS2        (S32K3XX_AXBS_BASE + S32K3XX_AXBS_PRS2_OFFSET)
#define S32K3XX_AXBS_CRS2        (S32K3XX_AXBS_BASE + S32K3XX_AXBS_CRS2_OFFSET)
#define S32K3XX_AXBS_PRS3        (S32K3XX_AXBS_BASE + S32K3XX_AXBS_PRS3_OFFSET)
#define S32K3XX_AXBS_CRS3        (S32K3XX_AXBS_BASE + S32K3XX_AXBS_CRS3_OFFSET)
#define S32K3XX_AXBS_PRS4        (S32K3XX_AXBS_BASE + S32K3XX_AXBS_PRS4_OFFSET)
#define S32K3XX_AXBS_CRS4        (S32K3XX_AXBS_BASE + S32K3XX_AXBS_CRS4_OFFSET)
#define S32K3XX_AXBS_PRS5        (S32K3XX_AXBS_BASE + S32K3XX_AXBS_PRS5_OFFSET)
#define S32K3XX_AXBS_CRS5        (S32K3XX_AXBS_BASE + S32K3XX_AXBS_CRS5_OFFSET)
#define S32K3XX_AXBS_PRS6        (S32K3XX_AXBS_BASE + S32K3XX_AXBS_PRS6_OFFSET)
#define S32K3XX_AXBS_CRS6        (S32K3XX_AXBS_BASE + S32K3XX_AXBS_CRS6_OFFSET)

/* AXBS Register Bitfield Definitions ***************************************/

/* Priority Slave Register n (PRSn) */

#define AXBS_PRS_M0_SHIFT        (0)       /* Bit 0-2: Master 0 Priority (M0) */
#define AXBS_PRS_M0_MASK         (0x07 << AXBS_PRS_M0_SHIFT)
                                           /* Bit 3: Reserved */
#define AXBS_PRS_M1_SHIFT        (4)       /* Bit 4-6: Master 1 Priority (M1) */
#define AXBS_PRS_M1_MASK         (0x07 << AXBS_PRS_M1_SHIFT)
                                           /* Bit 7: Reserved */
#define AXBS_PRS_M2_SHIFT        (8)       /* Bit 8-10: Master 2 Priority (M2) */
#define AXBS_PRS_M2_MASK         (0x07 << AXBS_PRS_M2_SHIFT)
                                           /* Bit 11: Reserved */
#define AXBS_PRS_M3_SHIFT        (12)      /* Bit 12-14: Master 3 Priority (M3) */
#define AXBS_PRS_M3_MASK         (0x07 << AXBS_PRS_M3_SHIFT)
                                           /* Bit 15: Reserved */
#define AXBS_PRS_M4_SHIFT        (16)      /* Bit 16-18: Master 4 Priority (M4) */
#define AXBS_PRS_M4_MASK         (0x07 << AXBS_PRS_M4_SHIFT)
                                           /* Bits 19-31: Reserved */

/* Control Register n (CRSn) */

#define AXBS_CRS_PARK_SHIFT      (0)       /* Bits 0-2: Determines which master port the slave parks on when (PARK) */
#define AXBS_CRS_PARK_MASK       (0x07 << AXBS_CRS_PARK_SHIFT)
                                           /* Bit 3: Reserved */
#define AXBS_CRS_PCTL_SHIFT      (4)       /* Bits 4-5: Parking Control (PCTL) */
#define AXBS_CRS_PCTL_MASK       (0x03 << AXBS_CRS_PCTL_SHIFT)
#  define AXBS_CRS_PCTL_PARK     (0x00 << AXBS_CRS_PCTL_SHIFT) /* Slave port parks on the master port defined by the PARK bit field */
#  define AXBS_CRS_PCTL_LAST     (0x01 << AXBS_CRS_PCTL_SHIFT) /* Slave port parks on the last master port in control */
#  define AXBS_CRS_PCTL_LPOW     (0x02 << AXBS_CRS_PCTL_SHIFT) /* Low-power park */

                                           /* Bits 7-6: Reserved */
#define AXBS_CRS_ARB_SHIFT       (8)       /* Bits 8-9: Arbitration Mode (ARB) */
#define AXBS_CRS_ARB_MASK        (0x03 << AXBS_CRS_ARB_SHIFT)
#  define AXBS_CRS_ARB_FIX       (0x00 << AXBS_CRS_ARB_SHIFT) /* Fixed priority */
#  define AXBS_CRS_ARB_RR        (0x01 << AXBS_CRS_ARB_SHIFT) /* Round-robin (rotating) priority */

                                           /* Bits 10-15: Reserved */
#define AXBS_CRS_HPE0            (1 << 16) /* Bit 16: High Priority Elevation 0 (HPE0) */
#define AXBS_CRS_HPE1            (1 << 17) /* Bit 17: High Priority Elevation 1 (HPE1) */
#define AXBS_CRS_HPE2            (1 << 18) /* Bit 18: High Priority Elevation 2 (HPE2) */
#define AXBS_CRS_HPE3            (1 << 19) /* Bit 19: High Priority Elevation 3 (HPE3) */
#define AXBS_CRS_HPE4            (1 << 20) /* Bit 20: High Priority Elevation 4 (HPE4) */
                                           /* Bits 21-29: Reserved */
#define AXBS_CRS_HLP             (1 << 30) /* Bit 30: Halt Low Priority (HLP) */
#define AXBS_CRS_RO              (1 << 31) /* Bit 31: Read Only (RO) */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_AXBS_H */
