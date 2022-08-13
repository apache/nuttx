/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_xbic.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_XBIC_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_XBIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* XBIC Register Offsets ****************************************************/

#define S32K3XX_XBIC_MCR_OFFSET (0x00) /* XBIC Module Control Register (MCR) */
#define S32K3XX_XBIC_EIR_OFFSET (0x04) /* XBIC Error Injection Register (EIR) */
#define S32K3XX_XBIC_ESR_OFFSET (0x08) /* XBIC Error Status Register (ESR) */
#define S32K3XX_XBIC_EAR_OFFSET (0x0c) /* XBIC Error Address (EAR) */

/* XBIC Register Addresses **************************************************/

#define S32K3XX_XBIC0_MCR       (S32K3XX_XBIC0_BASE + S32K3XX_XBIC_MCR_OFFSET)
#define S32K3XX_XBIC0_EIR       (S32K3XX_XBIC0_BASE + S32K3XX_XBIC_EIR_OFFSET)
#define S32K3XX_XBIC0_ESR       (S32K3XX_XBIC0_BASE + S32K3XX_XBIC_ESR_OFFSET)
#define S32K3XX_XBIC0_EAR       (S32K3XX_XBIC0_BASE + S32K3XX_XBIC_EAR_OFFSET)

#define S32K3XX_XBIC1_MCR       (S32K3XX_XBIC1_BASE + S32K3XX_XBIC_MCR_OFFSET)
#define S32K3XX_XBIC1_EIR       (S32K3XX_XBIC1_BASE + S32K3XX_XBIC_EIR_OFFSET)
#define S32K3XX_XBIC1_ESR       (S32K3XX_XBIC1_BASE + S32K3XX_XBIC_ESR_OFFSET)
#define S32K3XX_XBIC1_EAR       (S32K3XX_XBIC1_BASE + S32K3XX_XBIC_EAR_OFFSET)

#define S32K3XX_XBIC2_MCR       (S32K3XX_XBIC2_BASE + S32K3XX_XBIC_MCR_OFFSET)
#define S32K3XX_XBIC2_EIR       (S32K3XX_XBIC2_BASE + S32K3XX_XBIC_EIR_OFFSET)
#define S32K3XX_XBIC2_ESR       (S32K3XX_XBIC2_BASE + S32K3XX_XBIC_ESR_OFFSET)
#define S32K3XX_XBIC2_EAR       (S32K3XX_XBIC2_BASE + S32K3XX_XBIC_EAR_OFFSET)

#define S32K3XX_XBIC3_MCR       (S32K3XX_XBIC3_BASE + S32K3XX_XBIC_MCR_OFFSET)
#define S32K3XX_XBIC3_EIR       (S32K3XX_XBIC3_BASE + S32K3XX_XBIC_EIR_OFFSET)
#define S32K3XX_XBIC3_ESR       (S32K3XX_XBIC3_BASE + S32K3XX_XBIC_ESR_OFFSET)
#define S32K3XX_XBIC3_EAR       (S32K3XX_XBIC3_BASE + S32K3XX_XBIC_EAR_OFFSET)

/* XBIC Register Bitfield Definitions ***************************************/

/* XBIC Module Control Register (MCR) */

                                          /* Bits 0-15: Reserved */
#define XBIC_MCR_ME7            (1 << 16) /* Bit 16: Master Port 7 Enable for Feedback Integrity Check (ME7) */
#define XBIC_MCR_ME6            (1 << 17) /* Bit 17: Master Port 6 Enable for Feedback Integrity Check (ME6) */
#define XBIC_MCR_ME5            (1 << 18) /* Bit 18: Master Port 5 Enable for Feedback Integrity Check (ME5) */
#define XBIC_MCR_ME4            (1 << 19) /* Bit 19: Master Port 4 Enable for Feedback Integrity Check (ME4) */
#define XBIC_MCR_ME3            (1 << 20) /* Bit 20: Master Port 3 Enable for Feedback Integrity Check (ME3) */
#define XBIC_MCR_ME2            (1 << 21) /* Bit 21: Master Port 2 Enable for Feedback Integrity Check (ME2) */
#define XBIC_MCR_ME1            (1 << 22) /* Bit 22: Master Port 1 Enable for Feedback Integrity Check (ME1) */
#define XBIC_MCR_ME0            (1 << 23) /* Bit 23: Master Port 0 Enable for Feedback Integrity Check (ME0) */
#define XBIC_MCR_SE7            (1 << 24) /* Bit 24: Slave Port 7 EDC Error Detection Enable (SE7) */
#define XBIC_MCR_SE6            (1 << 25) /* Bit 25: Slave Port 6 EDC Error Detection Enable (SE6) */
#define XBIC_MCR_SE5            (1 << 26) /* Bit 26: Slave Port 5 EDC Error Detection Enable (SE5) */
#define XBIC_MCR_SE4            (1 << 27) /* Bit 27: Slave Port 4 EDC Error Detection Enable (SE4) */
#define XBIC_MCR_SE3            (1 << 28) /* Bit 28: Slave Port 3 EDC Error Detection Enable (SE3) */
#define XBIC_MCR_SE2            (1 << 29) /* Bit 29: Slave Port 2 EDC Error Detection Enable (SE2) */
#define XBIC_MCR_SE1            (1 << 30) /* Bit 30: Slave Port 1 EDC Error Detection Enable (SE1) */
#define XBIC_MCR_SE0            (1 << 31) /* Bit 31: Slave Port 0 EDC Error Detection Enable (SE0) */

/* XBIC Error Injection Register (EIR) */

#define XBIC_EIR_SYN_SHIFT      (0)       /* Bits 0-7: Syndrome (SYN) */
#define XBIC_EIR_SYN_MASK       (0xff << XBIC_EIR_SYN_SHIFT)
#define XBIC_EIR_MST_SHIFT      (8)       /* Bits 8-11: Target Master ID (MST) */
#define XBIC_EIR_MST_MASK       (0x0f << XBIC_EIR_MST_SHIFT)
#define XBIC_EIR_SLV_SHIFT      (12)      /* Bits 12-14: Target Slave Port (SLV) */
#define XBIC_EIR_SLV_MASK       (0x07 << XBIC_EIR_SLV_SHIFT)
                                          /* Bits 15-30: Reserved */
#define XBIC_EIR_EIE            (1 << 31) /* Bit 31: Errir Injection Enable (EIE) */

/* XBIC Error Status Register (ESR) */

#define XBIC_ESR_SYN_SHIFT      (0)       /* Bits 0-7: Syndrome (SYN) */
#define XBIC_ESR_SYN_MASK       (0xff << XBIC_ESR_SYN_SHIFT)
#define XBIC_ESR_MST_SHIFT      (8)       /* Bits 8-11: Master ID (MST) */
#define XBIC_ESR_MST_MASK       (0x0f << XBIC_ESR_MST_SHIFT)
#define XBIC_ESR_SLV_SHIFT      (12)      /* Bits 12-14: Slave Port (SLV) */
#define XBIC_ESR_SLV_MASK       (0x07 << XBIC_ESR_SLV_SHIFT)
#define XBIC_ESR_DPME7          (1 << 15) /* Bit 15: Data Phase Master Port 7 Error (DPME7) */
#define XBIC_ESR_DPME6          (1 << 16) /* Bit 16: Data Phase Master Port 6 Error (DPME6) */
#define XBIC_ESR_DPME5          (1 << 17) /* Bit 17: Data Phase Master Port 5 Error (DPME5) */
#define XBIC_ESR_DPME4          (1 << 18) /* Bit 18: Data Phase Master Port 4 Error (DPME4) */
#define XBIC_ESR_DPME3          (1 << 19) /* Bit 19: Data Phase Master Port 3 Error (DPME3) */
#define XBIC_ESR_DPME2          (1 << 20) /* Bit 20: Data Phase Master Port 2 Error (DPME2) */
#define XBIC_ESR_DPME1          (1 << 21) /* Bit 21: Data Phase Master Port 1 Error (DPME1) */
#define XBIC_ESR_DPME0          (1 << 22) /* Bit 22: Data Phase Master Port 0 Error (DPME0) */
#define XBIC_ESR_DPSE7          (1 << 23) /* Bit 23: Data Phase Slave Port 7 Error (DPSE7) */
#define XBIC_ESR_DPSE6          (1 << 24) /* Bit 24: Data Phase Slave Port 6 Error (DPSE6) */
#define XBIC_ESR_DPSE5          (1 << 25) /* Bit 25: Data Phase Slave Port 5 Error (DPSE5) */
#define XBIC_ESR_DPSE4          (1 << 26) /* Bit 26: Data Phase Slave Port 4 Error (DPSE4) */
#define XBIC_ESR_DPSE3          (1 << 27) /* Bit 27: Data Phase Slave Port 3 Error (DPSE3) */
#define XBIC_ESR_DPSE2          (1 << 28) /* Bit 28: Data Phase Slave Port 2 Error (DPSE2) */
#define XBIC_ESR_DPSE1          (1 << 29) /* Bit 29: Data Phase Slave Port 1 Error (DPSE1) */
#define XBIC_ESR_DPSE0          (1 << 30) /* Bit 30: Data Phase Slave Port 0 Error (DPSE0) */
#define XBIC_ESR_VLD            (1 << 31) /* Bit 31: Error Status Valid (VLD) */

/* XBIC Error Address (EAR) */

#define XBIC_EAR_ADDR_SHIFT     (0)       /* Bits 0-31: Error Address (ADDR) */
#define XBIC_EAR_ADDR_MASK      (0xffffffff << XBIC_EAR_ADDR_SHIFT)

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_XBIC_H */
