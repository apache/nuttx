/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_pbist.h
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

/* Programmable Built-In Self-Test (PBIST) controller, SPNU562A chapter 9.
 *
 * The register frame is only clocked - and so only accessible - once
 * PBIST_PACT has been written, and that in turn needs the controller
 * enabled through MSINENA/MSTGCR in the system module.
 *
 * Offsets follow TRM Table 9-1, expressed from the 0xffffe400 controller
 * base; HALCoGen's HL_reg_pbist.h maps the same registers from 0xffffe560.
 * RAM group numbers and the two algorithm masks are TRM Tables 2-5/2-6.
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_PBIST_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_PBIST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RM57_PBIST_RAMT_OFFSET      0x0160 /* RAM Configuration Register */
#define RM57_PBIST_DLR_OFFSET       0x0164 /* Datalogger Register */

/* Activate/Clock Enable Register */
#define RM57_PBIST_PACT_OFFSET      0x0180
#define RM57_PBIST_ID_OFFSET        0x0184 /* PBIST ID Register */
#define RM57_PBIST_OVER_OFFSET      0x0188 /* Override Register */
#define RM57_PBIST_FSRF0_OFFSET     0x0190 /* Fail Status Fail Register 0 */
#define RM57_PBIST_FSRF1_OFFSET     0x0194 /* Fail Status Fail Register 1 */
#define RM57_PBIST_FSRC0_OFFSET     0x0198 /* Fail Status Count Register 0 */
#define RM57_PBIST_FSRC1_OFFSET     0x019c /* Fail Status Count Register 1 */

/* Fail Status Address Register 0 */
#define RM57_PBIST_FSRA0_OFFSET     0x01a0

/* Fail Status Address Register 1 */
#define RM57_PBIST_FSRA1_OFFSET     0x01a4
#define RM57_PBIST_FSRDL0_OFFSET    0x01a8 /* Fail Status Data Register 0 */
#define RM57_PBIST_FSRDL1_OFFSET    0x01b0 /* Fail Status Data Register 1 */
#define RM57_PBIST_ROM_OFFSET       0x01c0 /* ROM Mask Register */
#define RM57_PBIST_ALGO_OFFSET      0x01c4 /* ROM Algorithm Mask Register */
#define RM57_PBIST_RINFOL_OFFSET    0x01c8 /* RAM Info Mask Lower Register */
#define RM57_PBIST_RINFOU_OFFSET    0x01cc /* RAM Info Mask Upper Register */

/* Register Addresses *******************************************************/

#define RM57_PBIST_RAMT             (RM57_PBIST_BASE + RM57_PBIST_RAMT_OFFSET)
#define RM57_PBIST_DLR              (RM57_PBIST_BASE + RM57_PBIST_DLR_OFFSET)
#define RM57_PBIST_PACT             (RM57_PBIST_BASE + RM57_PBIST_PACT_OFFSET)
#define RM57_PBIST_ID               (RM57_PBIST_BASE + RM57_PBIST_ID_OFFSET)
#define RM57_PBIST_OVER             (RM57_PBIST_BASE + RM57_PBIST_OVER_OFFSET)
#define RM57_PBIST_FSRF0 (RM57_PBIST_BASE + RM57_PBIST_FSRF0_OFFSET)
#define RM57_PBIST_FSRF1 (RM57_PBIST_BASE + RM57_PBIST_FSRF1_OFFSET)
#define RM57_PBIST_FSRC0 (RM57_PBIST_BASE + RM57_PBIST_FSRC0_OFFSET)
#define RM57_PBIST_FSRC1 (RM57_PBIST_BASE + RM57_PBIST_FSRC1_OFFSET)
#define RM57_PBIST_FSRA0 (RM57_PBIST_BASE + RM57_PBIST_FSRA0_OFFSET)
#define RM57_PBIST_FSRA1 (RM57_PBIST_BASE + RM57_PBIST_FSRA1_OFFSET)
#define RM57_PBIST_FSRDL0 (RM57_PBIST_BASE + RM57_PBIST_FSRDL0_OFFSET)
#define RM57_PBIST_FSRDL1 (RM57_PBIST_BASE + RM57_PBIST_FSRDL1_OFFSET)
#define RM57_PBIST_ROM              (RM57_PBIST_BASE + RM57_PBIST_ROM_OFFSET)
#define RM57_PBIST_ALGO             (RM57_PBIST_BASE + RM57_PBIST_ALGO_OFFSET)
#define RM57_PBIST_RINFOL (RM57_PBIST_BASE + RM57_PBIST_RINFOL_OFFSET)
#define RM57_PBIST_RINFOU (RM57_PBIST_BASE + RM57_PBIST_RINFOU_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* RAM Configuration Register (identifies the memory that failed) */

/* Bits 16-23: Return data select */
#define PBIST_RAMT_RDS_SHIFT        (16)
#define PBIST_RAMT_RDS_MASK         (0xff << PBIST_RAMT_RDS_SHIFT)

/* Bits 24-31: RAM group select */
#define PBIST_RAMT_RGS_SHIFT        (24)
#define PBIST_RAMT_RGS_MASK         (0xff << PBIST_RAMT_RGS_SHIFT)

/* Datalogger Register.  Bit 3 is reserved and resets to 1; TI's own code
 * starts a ROM-based run by writing just DLR2|DLR4 (0x14).
 */

#define PBIST_DLR_DLR2              (1 << 2)  /* Bit 2: ROM-based testing */
#define PBIST_DLR_DLR4              (1 << 4)  /* Bit 4: Config access (CPU) */
#define PBIST_DLR_START             (PBIST_DLR_DLR2 | PBIST_DLR_DLR4)

/* Activate/Clock Enable Register.  The TRM documents only bit 0, but both
 * HALCoGen and the SafeTI library write 3; bit 1 gates the ROM clock.
 */

#define PBIST_PACT_PACT0            (1 << 0)  /* Bit 0: Internal clocks */
#define PBIST_PACT_PACT1            (1 << 1)  /* Bit 1: ROM clock */
#define PBIST_PACT_ENABLE           (PBIST_PACT_PACT0 | PBIST_PACT_PACT1)

/* Override Register */

/* Bit 0: ROM overrides RINFOx */
#define PBIST_OVER_OVER0            (1 << 0)

/* Fail Status Fail Registers 0/1 */

/* Bit 0: Failure on this port */
#define PBIST_FSRF                  (1 << 0)

/* ROM Mask Register */

#define PBIST_ROM_MASK              (3 << 0)
#  define PBIST_ROM_NONE            (0 << 0)
#  define PBIST_ROM_RINFO           (1 << 0)  /* RAM group info from ROM */
#  define PBIST_ROM_ALGO            (2 << 0)  /* Algorithm info from ROM */
#  define PBIST_ROM_BOTH            (3 << 0)  /* Both, for application test */

/* ROM Algorithm Mask Register (TRM Table 2-6) */

#define PBIST_ALGO_TRIPLE_SLOW      (1 << 0)  /* triple_read_slow_read, ROM */
#define PBIST_ALGO_TRIPLE_FAST      (1 << 1)  /* triple_read_fast_read, ROM */
#define PBIST_ALGO_MARCH13N_DP      (1 << 2)  /* March13N, two-port RAM */
#define PBIST_ALGO_MARCH13N_SP      (1 << 3)  /* March13N, single-port RAM */

/* RAM groups (TRM Table 2-5).  Group n is bit (n - 1) of the 64-bit
 * RINFOU:RINFOL pair.
 */

#define PBIST_GROUP(n)              (1ull << ((n) - 1))

#define PBIST_GROUP_PBIST_ROM       PBIST_GROUP(1)
#define PBIST_GROUP_STC1_1_ROM      PBIST_GROUP(2)
#define PBIST_GROUP_STC1_2_ROM      PBIST_GROUP(3)
#define PBIST_GROUP_STC2_ROM        PBIST_GROUP(4)
#define PBIST_GROUP_AWM1            PBIST_GROUP(5)
#define PBIST_GROUP_DCAN1           PBIST_GROUP(6)
#define PBIST_GROUP_DCAN2           PBIST_GROUP(7)
#define PBIST_GROUP_DMA             PBIST_GROUP(8)
#define PBIST_GROUP_HTU1            PBIST_GROUP(9)
#define PBIST_GROUP_MIBSPI1         PBIST_GROUP(10)
#define PBIST_GROUP_MIBSPI2         PBIST_GROUP(11)
#define PBIST_GROUP_MIBSPI3         PBIST_GROUP(12)
#define PBIST_GROUP_N2HET1          PBIST_GROUP(13)
#define PBIST_GROUP_VIM             PBIST_GROUP(14)
#define PBIST_GROUP_RTP             PBIST_GROUP(16)
#define PBIST_GROUP_ATB             PBIST_GROUP(17)
#define PBIST_GROUP_AWM2            PBIST_GROUP(18)
#define PBIST_GROUP_DCAN3           PBIST_GROUP(19)
#define PBIST_GROUP_DCAN4           PBIST_GROUP(20)
#define PBIST_GROUP_HTU2            PBIST_GROUP(21)
#define PBIST_GROUP_MIBSPI4         PBIST_GROUP(22)
#define PBIST_GROUP_MIBSPI5         PBIST_GROUP(23)
#define PBIST_GROUP_N2HET2          PBIST_GROUP(24)
#define PBIST_GROUP_EMAC_STATE      PBIST_GROUP(27)
#define PBIST_GROUP_EMAC_FIFO       PBIST_GROUP(28)
#define PBIST_GROUP_L2RAMW_1        PBIST_GROUP(29)
#define PBIST_GROUP_L2RAMW_2        PBIST_GROUP(30)
#define PBIST_GROUP_ICACHE          PBIST_GROUP(31)
#define PBIST_GROUP_DCACHE          PBIST_GROUP(32)
#define PBIST_GROUP_EMAC_CPPI       PBIST_GROUP(36)
#define PBIST_GROUP_DCACHE_DIRTY    PBIST_GROUP(37)

#define PBIST_GROUPS_ROM            (PBIST_GROUP_PBIST_ROM | \
                                     PBIST_GROUP_STC1_1_ROM | \
                                     PBIST_GROUP_STC1_2_ROM | \
                                     PBIST_GROUP_STC2_ROM)

/* PBIST ROM clock ceiling (SPNS215C 6.13.1) */

#define PBIST_MAX_ROMCLK            (82500000)

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_PBIST_H */
