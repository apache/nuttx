/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_l2ramw.h
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

/* Level 2 RAM wrapper (L2RAMW), SPNU562A chapter 8.
 *
 * Besides ECC and parity control this module carries its own diagnostic
 * stimulus registers: a data/ECC vector is written to DIAG_DATA_VECTOR_x /
 * DIAG_ECC and pushed through both SECDED blocks and the redundant address
 * decode compare with one RAMTEST trigger.  The outcome appears in
 * RAMERRSTATUS and on ESM group2 channel 7 - no RAM cell is corrupted and
 * no data abort is taken, unlike the TCRAM scheme on Cortex-R4 parts.
 *
 * Offsets follow TRM Table 8-2 and match HALCoGen's HL_reg_l2ramw.h.
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_L2RAMW_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_L2RAMW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RM57_L2RAMW_RAMCTRL_OFFSET        0x0000 /* Module Control Register */
#define RM57_L2RAMW_RAMERRSTATUS_OFFSET   0x0010 /* Error Status Register */

/* Diagnostic Data Vector High */
#define RM57_L2RAMW_DIAGDATAH_OFFSET      0x0024

/* Diagnostic Data Vector Low */
#define RM57_L2RAMW_DIAGDATAL_OFFSET      0x0028
#define RM57_L2RAMW_DIAGECC_OFFSET        0x002c /* Diagnostic ECC Vector */
#define RM57_L2RAMW_RAMTEST_OFFSET        0x0030 /* RAM Test Register */

/* Address Decode Vector Test */
#define RM57_L2RAMW_RAMADDRDECVECT_OFFSET 0x0038
#define RM57_L2RAMW_MEMINITDOMAIN_OFFSET  0x003c /* Memory Init Domain */

/* Bank to Domain Mapping 0 */
#define RM57_L2RAMW_BANKDOMAINMAP0_OFFSET 0x0044

/* Bank to Domain Mapping 1 */
#define RM57_L2RAMW_BANKDOMAINMAP1_OFFSET 0x0048

/* Register Addresses *******************************************************/

#define RM57_L2RAMW_RAMCTRL \
  (RM57_L2RAMW_BASE + RM57_L2RAMW_RAMCTRL_OFFSET)
#define RM57_L2RAMW_RAMERRSTATUS \
  (RM57_L2RAMW_BASE + RM57_L2RAMW_RAMERRSTATUS_OFFSET)
#define RM57_L2RAMW_DIAGDATAH \
  (RM57_L2RAMW_BASE + RM57_L2RAMW_DIAGDATAH_OFFSET)
#define RM57_L2RAMW_DIAGDATAL \
  (RM57_L2RAMW_BASE + RM57_L2RAMW_DIAGDATAL_OFFSET)
#define RM57_L2RAMW_DIAGECC \
  (RM57_L2RAMW_BASE + RM57_L2RAMW_DIAGECC_OFFSET)
#define RM57_L2RAMW_RAMTEST \
  (RM57_L2RAMW_BASE + RM57_L2RAMW_RAMTEST_OFFSET)
#define RM57_L2RAMW_RAMADDRDECVECT \
  (RM57_L2RAMW_BASE + RM57_L2RAMW_RAMADDRDECVECT_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Module Control Register */

/* Bits 0-3: ECC detect */
#define L2RAMW_RAMCTRL_ECC_DETECT_MASK    (0x0f << 0)
#  define L2RAMW_RAMCTRL_ECC_DETECT_DIS   (0x05 << 0) /* Only 5h disables */
#  define L2RAMW_RAMCTRL_ECC_DETECT_ENA   (0x0a << 0)

/* Bit 4: Capture write SERR */
#define L2RAMW_RAMCTRL_CPUWSC             (1 << 4)

/* Bit 8: ECC memory writable */
#define L2RAMW_RAMCTRL_ECC_WR_EN          (1 << 8)

/* Bit 12: ESM on scrub error */
#define L2RAMW_RAMCTRL_EEMMS              (1 << 12)

/* Bits 16-19: Ah disables */
#define L2RAMW_RAMCTRL_ADDR_PAR_DIS_MASK  (0x0f << 16)

/* Bit 20: Memory scrubbing */
#define L2RAMW_RAMCTRL_MSE                (1 << 20)
#define L2RAMW_RAMCTRL_ADDR_PAR_OVR_MASK  (0x0f << 24) /* Bits 24-27 */
#define L2RAMW_RAMCTRL_EMU_TRACE_DIS      (1 << 30)

/* Error Status Register (all flags W1C) */

#define L2RAMW_ERR_CPUWE            (1 << 0)  /* CPU write ECC single error */
#define L2RAMW_ERR_REME             (1 << 1)  /* Read SECDED malfunction */
#define L2RAMW_ERR_ADE              (1 << 2)  /* Redundant address decode */
#define L2RAMW_ERR_WEME             (1 << 3)  /* Write SECDED malfunction */
#define L2RAMW_ERR_ADDE             (1 << 4)  /* Address decode, diagnostic */
#define L2RAMW_ERR_CPUWDE           (1 << 5)  /* CPU write ECC double error */

/* Read-modify-write double error */
#define L2RAMW_ERR_RMWDE            (1 << 7)
#define L2RAMW_ERR_PACE             (1 << 8)  /* Address/control parity */
#define L2RAMW_ERR_MME              (1 << 9)  /* Write data merged mux */

/* Read SECDED malf., diagnostic */
#define L2RAMW_ERR_REMDE            (1 << 10)

/* Write SECDED malf., diagnostic */
#define L2RAMW_ERR_WEMDE            (1 << 11)
#define L2RAMW_ERR_MMDE             (1 << 12) /* Merged mux, diagnostic */
#define L2RAMW_ERR_MIE              (1 << 13) /* Memory initialization */
#define L2RAMW_ERR_CPEOI            (1 << 15) /* Command parity on idle */
#define L2RAMW_ERR_MSACP            (1 << 16) /* Scrubbing addr/ctrl parity */
#define L2RAMW_ERR_MSRA             (1 << 17) /* Scrubbing address decode */

/* Scrubbing SECDED malfunction */
#define L2RAMW_ERR_MSSM             (1 << 18)

/* Diagnostic write single-bit */
#define L2RAMW_ERR_DWSE             (1 << 19)

/* Diagnostic write double-bit */
#define L2RAMW_ERR_DWDE             (1 << 20)
#define L2RAMW_ERR_DRSE             (1 << 21) /* Diagnostic read single-bit */
#define L2RAMW_ERR_DRDE             (1 << 22) /* Diagnostic read double-bit */

/* The diagnostic flags that must all be clear before RAMTEST will accept
 * a trigger (TRM 8.2.6): bits 22, 21, 20, 19, 12, 11, 10 and 4.
 */

#define L2RAMW_ERR_DIAG_ALL         (L2RAMW_ERR_DRDE | L2RAMW_ERR_DRSE | \
                                     L2RAMW_ERR_DWDE | L2RAMW_ERR_DWSE | \
                                     L2RAMW_ERR_MMDE | L2RAMW_ERR_WEMDE | \
                                     L2RAMW_ERR_REMDE | L2RAMW_ERR_ADDE)

/* RAM Test Register */

/* Bits 0-3: Test enable key */
#define L2RAMW_RAMTEST_ENABLE_MASK  (0x0f << 0)
#  define L2RAMW_RAMTEST_ENABLE     (0x0a << 0)
#  define L2RAMW_RAMTEST_DISABLE    (0x05 << 0)
#define L2RAMW_RAMTEST_MODE_MASK    (3 << 6)  /* Bits 6-7: Test mode */
#  define L2RAMW_RAMTEST_MODE_INEQ  (1 << 6)  /* Inequality check */
#  define L2RAMW_RAMTEST_MODE_EQ    (2 << 6)  /* Equality check */

/* Bit 8: Trigger, self-clearing */
#define L2RAMW_RAMTEST_TRIGGER      (1 << 8)

/* The ECC of an all-zero 64-bit word is 0x0c in every byte lane (TRM
 * 8.2.3), so these are the DIAG_ECC values that present a clean word and
 * a word with one or two flipped ECC bits.
 */

#define L2RAMW_DIAGECC_ZERO         (0x0c)

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_L2RAMW_H */
