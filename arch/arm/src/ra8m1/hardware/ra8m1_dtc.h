/****************************************************************************
 * arch/arm/src/ra8m1/hardware/ra8m1_dtc.h
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

#ifndef __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_DTC_H
#define __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_DTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/ra8m1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define R_DTC_DTCCR_OFFSET                  0x0000  /* DTC Control Register (8-bits) */
#define R_DTC_DTCCR_NS_OFFSET               0x0000  /* DTC Control Register for Non Secure Region (8-bits) */
#define R_DTC_DTCVBR_OFFSET                 0x0004  /* DTC Vector Base Register (32-bits) */
#define R_DTC_DTCVBR_NS_OFFSET              0x0004  /* DTC Vector Base Register for Non Secure Region (32-bits) */
#define R_DTC_DTCST_OFFSET                  0x000c  /* DTC Module Start Register (8-bits) */
#define R_DTC_DTCSTS_OFFSET                 0x000e  /* DTC Status Register (16-bits) */
#define R_DTC_DTCCR_S_OFFSET                0x0010  /* DTC Control Register for Secure Region (8-bits) */
#define R_DTC_DTCCR_SEC_OFFSET              0x0010  /* DTC Control Register for Secure Region (8-bits) */
#define R_DTC_DTCVBR_S_OFFSET               0x0014  /* DTC Vector Base Register for secure Region (32-bits) */
#define R_DTC_DTCVBR_SEC_OFFSET             0x0014  /* DTC Vector Base Register for secure Region (32-bits) */
#define R_DTC_DTEVR_OFFSET                  0x0020  /* DTC Error Vector Register (32-bits) */

/* Register Addresses *******************************************************/

/* DTC Registers */

#define R_DTC_DTCCR                        (R_DTC_BASE + R_DTC_DTCCR_OFFSET)
#define R_DTC_DTCCR_NS                     (R_DTC_BASE + R_DTC_DTCCR_NS_OFFSET)
#define R_DTC_DTCVBR                       (R_DTC_BASE + R_DTC_DTCVBR_OFFSET)
#define R_DTC_DTCVBR_NS                    (R_DTC_BASE + R_DTC_DTCVBR_NS_OFFSET)
#define R_DTC_DTCST                        (R_DTC_BASE + R_DTC_DTCST_OFFSET)
#define R_DTC_DTCSTS                       (R_DTC_BASE + R_DTC_DTCSTS_OFFSET)
#define R_DTC_DTCCR_S                      (R_DTC_BASE + R_DTC_DTCCR_S_OFFSET)
#define R_DTC_DTCCR_SEC                    (R_DTC_BASE + R_DTC_DTCCR_SEC_OFFSET)
#define R_DTC_DTCVBR_S                     (R_DTC_BASE + R_DTC_DTCVBR_S_OFFSET)
#define R_DTC_DTCVBR_SEC                   (R_DTC_BASE + R_DTC_DTCVBR_SEC_OFFSET)
#define R_DTC_DTEVR                        (R_DTC_BASE + R_DTC_DTEVR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* DTC Control Register for Non Secure Region (8-bits) **********************/

#define R_DTC_DTCCR_NS_RRS (1 <<  4)  /* 10: DTC Transfer Information Read Skip Enable. */

/* DTC Vector Base Register for Non Secure Region (32-bits) *****************/

#define R_DTC_DTCVBR_NS_DTCVBR_SHIFT (10)
#define R_DTC_DTCVBR_NS_DTCVBR_MASK (0x3fffff)

/* DTC Module Start Register (8-bits) ***************************************/

#define R_DTC_DTCST_DTCST (1 <<  0)  /* 01: DTC Module Start */

/* DTC Status Register (16-bits) ********************************************/

#define R_DTC_DTCSTS_VECN_SHIFT (0)
#define R_DTC_DTCSTS_VECN_MASK (0xff)
#define R_DTC_DTCSTS_ACT (1 << 15)  /* 8000: DTC Active Flag */

/* DTC Control Register for Secure Region (8-bits) **************************/

#define R_DTC_DTCCR_S_RRSS (1 <<  4)  /* 10: DTC Transfer Information Read Skip Enable for secure */

/* DTC Vector Base Register for secure Region (32-bits) *********************/

#define R_DTC_DTCVBR_S_DTCVBRS_SHIFT (10)
#define R_DTC_DTCVBR_S_DTCVBRS_MASK (0x3fffff)

/* DTC Error Vector Register (32-bits) **************************************/

#define R_DTC_DTEVR_DTEV_SHIFT (0)
#define R_DTC_DTEVR_DTEV_MASK (0xff)
#define R_DTC_DTEVR_DTEVSAM (1 <<  8)  /* 100: DTC Error Vector Number SA Monitor */
#define R_DTC_DTEVR_DTESTA (1 << 16)   /* 10000: DTC Error Status Flag */

/* DTC Vector Table Entry (32-bits) *****************************************
 *
 * One entry per IELSR vector number, at DTCVBR(_SEC) + 0x4 * vector.  Not a
 * peripheral register: it is a plain 32-bit word the DTC reads out of SRAM
 * (manual figure 17.3).  Bits 31:2 are the transfer information's start
 * address (so it must be word-aligned); bit 1 is reserved; bit 0 selects
 * privileged/unprivileged access for that vector's transfer.
 */

#define R_DTC_VECTBL_ADDR_MASK        (0xfffffffc)
#define R_DTC_VECTBL_UNPRIVILEGED     (1 << 0)  /* 0: privileged, 1: unprivileged */

/* The vector table must start on a 1024-byte boundary (manual 17.3.1); this
 * is also the alignment DTCVBR(_SEC) itself is shifted for
 * (R_DTC_DTCVBR_S_DTCVBRS_SHIFT above).
 */

#define R_DTC_VECTBL_ALIGN            (1024)

/* DTC Transfer Information (16 bytes) **************************************
 *
 * Not peripheral registers either: MRA, MRB, SAR, DAR, CRA and CRB are the
 * byte/halfword/word layout of one 16-byte block in SRAM that a vector
 * table entry above points to (manual figure 17.4, and the "Base address:
 * DTCVBR(_SEC) / Offset address: 0x03 + 0x4 x Vector number (Inaccessible
 * directly from the CPU...)" wording under each of section 17.2.2-17.2.7 is
 * this same block, described per-vector).  The CPU accesses these directly
 * as normal SRAM; the DTC copies them to/from its internal registers.
 */

#define R_DTC_TRANSFER_INFO_MRB_OFFSET  0x02  /* DTC Mode Register B (8-bits) */
#define R_DTC_TRANSFER_INFO_MRA_OFFSET  0x03  /* DTC Mode Register A (8-bits) */
#define R_DTC_TRANSFER_INFO_SAR_OFFSET  0x04  /* Transfer Source Register (32-bits) */
#define R_DTC_TRANSFER_INFO_DAR_OFFSET  0x08  /* Transfer Destination Register (32-bits) */
#define R_DTC_TRANSFER_INFO_CRB_OFFSET  0x0c  /* Transfer Count Register B (16-bits) */
#define R_DTC_TRANSFER_INFO_CRA_OFFSET  0x0e  /* Transfer Count Register A (16-bits) */

#define R_DTC_TRANSFER_INFO_SIZE        0x10  /* One block per activation source */

/* MRA (DTC Mode Register A) ************************************************/

#define R_DTC_MRA_SM_SHIFT (2)
#define R_DTC_MRA_SM_MASK (0x3)
#  define R_DTC_MRA_SM_FIXED        (0 << R_DTC_MRA_SM_SHIFT)  /* SAR fixed (write-back skipped) */
#  define R_DTC_MRA_SM_INCREMENT    (2 << R_DTC_MRA_SM_SHIFT)  /* SAR incremented after transfer */
#  define R_DTC_MRA_SM_DECREMENT    (3 << R_DTC_MRA_SM_SHIFT)  /* SAR decremented after transfer */
#define R_DTC_MRA_SZ_SHIFT (4)
#define R_DTC_MRA_SZ_MASK (0x3)
#  define R_DTC_MRA_SZ_BYTE         (0 << R_DTC_MRA_SZ_SHIFT)  /* 8-bit transfer */
#  define R_DTC_MRA_SZ_HALFWORD     (1 << R_DTC_MRA_SZ_SHIFT)  /* 16-bit transfer */
#  define R_DTC_MRA_SZ_WORD         (2 << R_DTC_MRA_SZ_SHIFT)  /* 32-bit transfer */
#define R_DTC_MRA_MD_SHIFT (6)
#define R_DTC_MRA_MD_MASK (0x3)
#  define R_DTC_MRA_MD_NORMAL       (0 << R_DTC_MRA_MD_SHIFT)  /* Normal transfer mode */
#  define R_DTC_MRA_MD_REPEAT       (1 << R_DTC_MRA_MD_SHIFT)  /* Repeat transfer mode */
#  define R_DTC_MRA_MD_BLOCK        (2 << R_DTC_MRA_MD_SHIFT)  /* Block transfer mode */

/* MRB (DTC Mode Register B) ************************************************/

#define R_DTC_MRB_DM_SHIFT (2)
#define R_DTC_MRB_DM_MASK (0x3)
#  define R_DTC_MRB_DM_FIXED        (0 << R_DTC_MRB_DM_SHIFT)  /* DAR fixed (write-back skipped) */
#  define R_DTC_MRB_DM_INCREMENT    (2 << R_DTC_MRB_DM_SHIFT)  /* DAR incremented after transfer */
#  define R_DTC_MRB_DM_DECREMENT    (3 << R_DTC_MRB_DM_SHIFT)  /* DAR decremented after transfer */
#define R_DTC_MRB_DTS   (1 << 4)                               /* 10: 0=destination is repeat/block area, 1=source is */
#define R_DTC_MRB_DISEL (1 << 5)                               /* 20: 0=interrupt after count completes, 1=every transfer */
#define R_DTC_MRB_CHNS  (1 << 6)                               /* 40: Chain transfer select */
#define R_DTC_MRB_CHNE  (1 << 7)                               /* 80: Chain transfer enable */

#endif /* __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_DTC_H */
