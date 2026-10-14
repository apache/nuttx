/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_iomm.h
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

/* References:
 * RM57Lx 16/32-Bit RISC Flash Microcontroller, Technical Reference Manual,
 * Texas Instruments, Literature Number: SPNU562
 *
 * Register offsets were cross-checked against TI's HALCoGen-generated
 * HL_reg_pinmux.h for RM57L843.  RM57L843 has considerably more pin
 * multiplexing control registers than the TMS570LS devices, and they are
 * split into three separate, non-contiguous regions (see below).
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_IOMM_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_IOMM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RM57_IOMM_REVISION_OFFSET     0x0000 /* Revision Register */
#define RM57_IOMM_BOOT_OFFSET         0x0020 /* Boot Mode Register */
#define RM57_IOMM_KICK0_OFFSET        0x0038 /* Kicker Register 0 */
#define RM57_IOMM_KICK1_OFFSET        0x003c /* Kicker Register 1 */
#define RM57_IOMM_ERRRAWSTATUS_OFFSET 0x00e0 /* Error Raw Status / Set Register */
#define RM57_IOMM_ERRSTATUS_OFFSET    0x00e4 /* Error Enabled Status / Clear Register */
#define RM57_IOMM_ERRENABLE_OFFSET    0x00e8 /* Error Signaling Enable Register */
#define RM57_IOMM_ERRENABLECLR_OFFSET 0x00ec /* Error Signaling Enable Clear Register */
#define RM57_IOMM_FAULTADDRESS_OFFSET 0x00f4 /* Fault Address Register */
#define RM57_IOMM_FAULTSTATUS_OFFSET  0x00f8 /* Fault Status Register */
#define RM57_IOMM_FAULTCLR_OFFSET     0x00fc /* Fault Clear Register */

/* Pin Multiplexing Control Registers.
 *
 * The PINMMR registers are indexed as a single array starting at offset
 * 0x0110, but only three sub-ranges of that array are implemented:
 *
 *   INDEX     OFFSET RANGE   CONTENTS
 *   0   - 37  0x0110-0x01a4  Output (ball) pin multiplexing control
 *   38  - 79  0x01a8-0x024c  Reserved
 *   80  - 99  0x0250-0x029c  Input pin multiplexing control
 *   100 - 159 0x02a0-0x038c  Reserved
 *   160 - 179 0x0390-0x03dc  Special functionality control
 *
 * Reading or writing an index in a reserved range has no effect.  The
 * chip-specific pinmux header (rm57l843_pinmux.h) defines the index, shift
 * and value of every pin function, so the individual register indices do
 * not need to be spelled out here.
 */

#define RM57_IOMM_PINMMR_OFFSET(n)    (0x0110 + ((unsigned int)(n) << 2))

#define RM57_IOMM_PINMMR_OUT_FIRST    0    /* First output mux index */
#define RM57_IOMM_PINMMR_OUT_LAST     37   /* Last output mux index */
#define RM57_IOMM_PINMMR_IN_FIRST     80   /* First input mux index */
#define RM57_IOMM_PINMMR_IN_LAST      99   /* Last input mux index */
#define RM57_IOMM_PINMMR_SPCL_FIRST   160  /* First special function index */
#define RM57_IOMM_PINMMR_SPCL_LAST    179  /* Last special function index */

/* Register Addresses *******************************************************/

#define RM57_IOMM_REVISION            (RM57_IOMM_BASE+RM57_IOMM_REVISION_OFFSET)
#define RM57_IOMM_BOOT                (RM57_IOMM_BASE+RM57_IOMM_BOOT_OFFSET)
#define RM57_IOMM_KICK0               (RM57_IOMM_BASE+RM57_IOMM_KICK0_OFFSET)
#define RM57_IOMM_KICK1               (RM57_IOMM_BASE+RM57_IOMM_KICK1_OFFSET)
#define RM57_IOMM_ERRRAWSTATUS        (RM57_IOMM_BASE+RM57_IOMM_ERRRAWSTATUS_OFFSET)
#define RM57_IOMM_ERRSTATUS           (RM57_IOMM_BASE+RM57_IOMM_ERRSTATUS_OFFSET)
#define RM57_IOMM_ERRENABLE           (RM57_IOMM_BASE+RM57_IOMM_ERRENABLE_OFFSET)
#define RM57_IOMM_ERRENABLECLR        (RM57_IOMM_BASE+RM57_IOMM_ERRENABLECLR_OFFSET)
#define RM57_IOMM_FAULTADDRESS        (RM57_IOMM_BASE+RM57_IOMM_FAULTADDRESS_OFFSET)
#define RM57_IOMM_FAULTSTATUS         (RM57_IOMM_BASE+RM57_IOMM_FAULTSTATUS_OFFSET)
#define RM57_IOMM_FAULTCLR            (RM57_IOMM_BASE+RM57_IOMM_FAULTCLR_OFFSET)

#define RM57_IOMM_PINMMR(n)           (RM57_IOMM_BASE+RM57_IOMM_PINMMR_OFFSET(n))

/* Register Bit-Field Definitions *******************************************/

/* Kicker Register 0 and 1.
 *
 * The PINMMR registers are write-protected until the two kicker registers
 * are written with the unlock values below, in order.  Writing any other
 * value to either register locks them again.
 */

#define IOMM_KICK0_UNLOCK             0x83e70b13 /* Unlock value */
#define IOMM_KICK0_LOCK               0x00000000 /* Any other value locks */

#define IOMM_KICK1_UNLOCK             0x95a4f1e0 /* Unlock value */
#define IOMM_KICK1_LOCK               0x00000000 /* Any other value locks */

/* Error Raw Status/Set, Error Enabled Status/Clear, Error Signaling Enable,
 * Error Signaling Enable Clear, Fault Address, Fault Status and Fault Clear
 * Registers.
 *
 * These registers report accesses that violate the kicker write protection
 * or the privilege mode of the accessing master.  Their bit assignments are
 * not used by this port; refer to SPNU562 if error signaling is needed.
 */

/* Pin Multiplexing Control Register n.
 *
 * Each PINMMR register holds four 8-bit fields, one per multiplexed ball
 * (or per input signal / special function).  Exactly one bit of a field
 * should be set to select the corresponding function.
 */

#define IOMM_PINMMR_PINSHIFT(n)       (((n) & 3) << 3)
#define IOMM_PINMMR_PINMASK(n)        (0xff << IOMM_PINMMR_PINSHIFT(n))
#  define IOMM_PINMMR_PINVALUE(n,v)   ((uint32_t)(v) << IOMM_PINMMR_PINSHIFT(n))

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_IOMM_H */
