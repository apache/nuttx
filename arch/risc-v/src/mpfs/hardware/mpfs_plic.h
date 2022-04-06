/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_plic.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_PLIC_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_PLIC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* following 187 uint32_t for irq priority */

#define MPFS_PLIC_PRIORITY   (MPFS_PLIC_BASE + 0x000000)

#define MPFS_PLIC_IP0        (MPFS_PLIC_BASE + 0x001000)
#define MPFS_PLIC_IP1        (MPFS_PLIC_BASE + 0x001004)
#define MPFS_PLIC_IP2        (MPFS_PLIC_BASE + 0x001008)
#define MPFS_PLIC_IP3        (MPFS_PLIC_BASE + 0x00100C)
#define MPFS_PLIC_IP4        (MPFS_PLIC_BASE + 0x001010)
#define MPFS_PLIC_I51        (MPFS_PLIC_BASE + 0x001014)

#define MPFS_HART_MIE_OFFSET (0x100)
#define MPFS_HART_SIE_OFFSET (0x80)

#define MPFS_PLIC_H0_MIE0    (MPFS_PLIC_BASE + 0x002000)
#define MPFS_PLIC_H0_MIE1    (MPFS_PLIC_BASE + 0x002004)
#define MPFS_PLIC_H0_MIE2    (MPFS_PLIC_BASE + 0x002008)
#define MPFS_PLIC_H0_MIE3    (MPFS_PLIC_BASE + 0x00200C)
#define MPFS_PLIC_H0_MIE4    (MPFS_PLIC_BASE + 0x002010)
#define MPFS_PLIC_H0_MIE5    (MPFS_PLIC_BASE + 0x002014)

#define MPFS_PLIC_H1_MIE0    (MPFS_PLIC_BASE + 0x002080)
#define MPFS_PLIC_H1_MIE1    (MPFS_PLIC_BASE + 0x002084)
#define MPFS_PLIC_H1_MIE2    (MPFS_PLIC_BASE + 0x002088)
#define MPFS_PLIC_H1_MIE3    (MPFS_PLIC_BASE + 0x00208C)
#define MPFS_PLIC_H1_MIE4    (MPFS_PLIC_BASE + 0x002090)
#define MPFS_PLIC_H1_MIE5    (MPFS_PLIC_BASE + 0x002094)
#define MPFS_PLIC_H1_SIE0    (MPFS_PLIC_BASE + 0x002100)
#define MPFS_PLIC_H1_SIE1    (MPFS_PLIC_BASE + 0x002104)
#define MPFS_PLIC_H1_SIE2    (MPFS_PLIC_BASE + 0x002108)
#define MPFS_PLIC_H1_SIE3    (MPFS_PLIC_BASE + 0x00210C)
#define MPFS_PLIC_H1_SIE4    (MPFS_PLIC_BASE + 0x002110)
#define MPFS_PLIC_H1_SIE5    (MPFS_PLIC_BASE + 0x002114)

#define MPFS_PLIC_H2_MIE0    (MPFS_PLIC_BASE + 0x002180)
#define MPFS_PLIC_H2_MIE1    (MPFS_PLIC_BASE + 0x002184)
#define MPFS_PLIC_H2_MIE2    (MPFS_PLIC_BASE + 0x002188)
#define MPFS_PLIC_H2_MIE3    (MPFS_PLIC_BASE + 0x00218C)
#define MPFS_PLIC_H2_MIE4    (MPFS_PLIC_BASE + 0x002190)
#define MPFS_PLIC_H2_MIE5    (MPFS_PLIC_BASE + 0x002194)
#define MPFS_PLIC_H2_SIE0    (MPFS_PLIC_BASE + 0x002200)
#define MPFS_PLIC_H2_SIE1    (MPFS_PLIC_BASE + 0x002204)
#define MPFS_PLIC_H2_SIE2    (MPFS_PLIC_BASE + 0x002208)
#define MPFS_PLIC_H2_SIE3    (MPFS_PLIC_BASE + 0x00220C)
#define MPFS_PLIC_H2_SIE4    (MPFS_PLIC_BASE + 0x002210)
#define MPFS_PLIC_H2_SIE5    (MPFS_PLIC_BASE + 0x002214)

#define MPFS_PLIC_H3_MIE0    (MPFS_PLIC_BASE + 0x002280)
#define MPFS_PLIC_H3_MIE1    (MPFS_PLIC_BASE + 0x002284)
#define MPFS_PLIC_H3_MIE2    (MPFS_PLIC_BASE + 0x002288)
#define MPFS_PLIC_H3_MIE3    (MPFS_PLIC_BASE + 0x00228C)
#define MPFS_PLIC_H3_MIE4    (MPFS_PLIC_BASE + 0x002290)
#define MPFS_PLIC_H3_MIE5    (MPFS_PLIC_BASE + 0x002294)
#define MPFS_PLIC_H3_SIE0    (MPFS_PLIC_BASE + 0x002300)
#define MPFS_PLIC_H3_SIE1    (MPFS_PLIC_BASE + 0x002304)
#define MPFS_PLIC_H3_SIE2    (MPFS_PLIC_BASE + 0x002308)
#define MPFS_PLIC_H3_SIE3    (MPFS_PLIC_BASE + 0x00230C)
#define MPFS_PLIC_H3_SIE4    (MPFS_PLIC_BASE + 0x002310)
#define MPFS_PLIC_H3_SIE5    (MPFS_PLIC_BASE + 0x002314)

#define MPFS_PLIC_H4_MIE0    (MPFS_PLIC_BASE + 0x002380)
#define MPFS_PLIC_H4_MIE1    (MPFS_PLIC_BASE + 0x002384)
#define MPFS_PLIC_H4_MIE2    (MPFS_PLIC_BASE + 0x002388)
#define MPFS_PLIC_H4_MIE3    (MPFS_PLIC_BASE + 0x00238C)
#define MPFS_PLIC_H4_MIE4    (MPFS_PLIC_BASE + 0x002390)
#define MPFS_PLIC_H4_MIE5    (MPFS_PLIC_BASE + 0x002394)
#define MPFS_PLIC_H4_SIE0    (MPFS_PLIC_BASE + 0x002400)
#define MPFS_PLIC_H4_SIE1    (MPFS_PLIC_BASE + 0x002404)
#define MPFS_PLIC_H4_SIE2    (MPFS_PLIC_BASE + 0x002408)
#define MPFS_PLIC_H4_SIE3    (MPFS_PLIC_BASE + 0x00240C)
#define MPFS_PLIC_H4_SIE4    (MPFS_PLIC_BASE + 0x002410)
#define MPFS_PLIC_H4_SIE5    (MPFS_PLIC_BASE + 0x002414)

#define MPFS_PLIC_NEXTHART_OFFSET    (0x2000)
#define MPFS_PLIC_MTHRESHOLD_OFFSET  (0x0000)
#define MPFS_PLIC_MCLAIM_OFFSET      (0x0004)
#define MPFS_PLIC_STHRESHOLD_OFFSET  (0x1000)
#define MPFS_PLIC_SCLAIM_OFFSET      (0x1004) /* From hart base */
#define MPFS_PLIC_CLAIM_S_OFFSET     (0x1000) /* From mclaim to sclaim */
#define MPFS_PLIC_THRESHOLD_S_OFFSET (0x1000) /* From mthresh to sthresh */

#define MPFS_PLIC_H0_MTHRESHOLD  (MPFS_PLIC_BASE + 0x200000)
#define MPFS_PLIC_H0_MCLAIM      (MPFS_PLIC_BASE + 0x200004)

#define MPFS_PLIC_H1_MTHRESHOLD  (MPFS_PLIC_BASE + 0x201000)
#define MPFS_PLIC_H1_MCLAIM      (MPFS_PLIC_BASE + 0x201004)
#define MPFS_PLIC_H1_STHRESHOLD  (MPFS_PLIC_BASE + 0x202000)
#define MPFS_PLIC_H1_SCLAIM      (MPFS_PLIC_BASE + 0x202004)

#define MPFS_PLIC_H2_MTHRESHOLD  (MPFS_PLIC_BASE + 0x203000)
#define MPFS_PLIC_H2_MCLAIM      (MPFS_PLIC_BASE + 0x203004)
#define MPFS_PLIC_H2_STHRESHOLD  (MPFS_PLIC_BASE + 0x204000)
#define MPFS_PLIC_H2_SCLAIM      (MPFS_PLIC_BASE + 0x204004)

#define MPFS_PLIC_H3_MTHRESHOLD  (MPFS_PLIC_BASE + 0x205000)
#define MPFS_PLIC_H3_MCLAIM      (MPFS_PLIC_BASE + 0x205004)
#define MPFS_PLIC_H3_STHRESHOLD  (MPFS_PLIC_BASE + 0x206000)
#define MPFS_PLIC_H3_SCLAIM      (MPFS_PLIC_BASE + 0x206004)

#define MPFS_PLIC_H4_MTHRESHOLD  (MPFS_PLIC_BASE + 0x207000)
#define MPFS_PLIC_H4_MCLAIM      (MPFS_PLIC_BASE + 0x207004)
#define MPFS_PLIC_H4_STHRESHOLD  (MPFS_PLIC_BASE + 0x208000)
#define MPFS_PLIC_H4_SCLAIM      (MPFS_PLIC_BASE + 0x208004)

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_PLIC_H */
