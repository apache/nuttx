/****************************************************************************
 * arch/tricore/src/tc4x/hardware/tc4x_cpu.h
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

#ifndef __ARCH_TRICORE_SRC_TC4X_HARDWARE_TC4X_CPU_H
#define __ARCH_TRICORE_SRC_TC4X_HARDWARE_TC4X_CPU_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Start of the other cores: HRA PC and BOOTCON.BHALT of each CPU */

#define TC4X_CPU_HRA_PC(n)         (0xf881fe08 + (n) * 0x40000)
#define TC4X_CPU_HRA_BOOTCON(n)    (0xf881fe60 + (n) * 0x40000)
#define TC4X_BOOTCON_BHALT         (1 << 0)

/* Access protection of the memories and the STM of each CPU.  A PROTxSE
 * register selects the protection range and its state, an ACCENx_WRA/WRB
 * pair has one bit per bus master tag; CPUn owns the tags 2n and 2n + 1.
 */

#define TC4X_CPU_SFR(n)            (0xf8800000 + (n) * 0x40000)
#define TC4X_CPU_PROTSPRSE(n)      (TC4X_CPU_SFR(n) + 0xe008)
#define TC4X_CPU_ACCENSPRCFG(n)    (TC4X_CPU_SFR(n) + 0xe020)
#define TC4X_CPU_ACCENSPR(n, r)    (TC4X_CPU_SFR(n) + 0xe200 + \
                                    (r) * TC4X_ACCEN_RANGE_SIZE)
#define TC4X_CPU_PROTDLMUSE(n)     (TC4X_CPU_SFR(n) + 0xe048)
#define TC4X_CPU_ACCENDLMUCFG(n)   (TC4X_CPU_SFR(n) + 0xe060)
#define TC4X_CPU_ACCENDLMU(n, r)   (TC4X_CPU_SFR(n) + 0xe400 + \
                                    (r) * TC4X_ACCEN_RANGE_SIZE)
#define TC4X_CPU_PROTSFRSE(n)      (TC4X_CPU_SFR(n) + 0xe088)
#define TC4X_CPU_ACCENSFRCFG(n)    (TC4X_CPU_SFR(n) + 0xe0a0)
#define TC4X_CPU_ACCENSFR(n)       (TC4X_CPU_SFR(n) + 0xe600)
#define TC4X_CPU_PROTSTMSE(n)      (TC4X_CPU_SFR(n) + 0xe0d8)
#define TC4X_CPU_ACCENSTMCFG_WRA(n) (TC4X_CPU_SFR(n) + 0xe0e0)

/* A protected memory has TC4X_PROT_RANGES ranges.  Each has an active
 * ACCEN block of TC4X_ACCEN_WORDS registers (WRA, WRB, RDA, RDB, VM, PRS,
 * RGNLA, RGNUA) and is changed through the CFG block of the same layout:
 * select the range, go from Run to Config, write the CFG block and go
 * back to Run, which makes it active.  The SFRs of a CPU, where its STM
 * is, have one range of TC4X_ACCEN_SFR_WORDS registers (no RGNLA, RGNUA).
 */

#define TC4X_PROT_RANGES           16
#define TC4X_PROT_SFR_RANGES       1
#define TC4X_ACCEN_WORDS           8
#define TC4X_ACCEN_SFR_WORDS       6
#define TC4X_ACCEN_RANGE_SIZE      (TC4X_ACCEN_WORDS * sizeof(uint32_t))
#define TC4X_ACCEN_WRA             0

#define TC4X_PROT_RANGE(r)         ((r) << 8)
#define TC4X_PROT_STM_RANGE        1
#define TC4X_PROT_SWEN             (1 << 3)
#define TC4X_PROT_STATE_CONFIG     1
#define TC4X_PROT_STATE_RUN        4
#define TC4X_PROT_SET(r, state)    (TC4X_PROT_RANGE(r) | TC4X_PROT_SWEN | \
                                    (state))
#define TC4X_PROT_STM(state)       TC4X_PROT_SET(TC4X_PROT_STM_RANGE, state)

#define TC4X_CPU_TAGS(n)           (0x3 << ((n) * 2))
#define TC4X_CPU_TAGS_ALL(n)       ((0x1 << ((n) * 2)) - 1)

/* Access protection of the interrupt router for each service provider
 * (TOS): who may route an SRC to it, and who may enable or trigger it,
 * and of the broadcast register INT_SRBn that CPUn writes to send IPIs
 */

#define TC4X_IR_INT_BASE           0xf4430000
#define TC4X_IR_TOS_STRIDE         0x30
#define TC4X_IR_SRB_STRIDE         0x18
#define TC4X_IR_ACCENSRB_WRA(n) \
  (TC4X_IR_INT_BASE + 0x100 + (n) * TC4X_IR_SRB_STRIDE)
#define TC4X_IR_ACCENSCFG_WRA(tos) \
  (TC4X_IR_INT_BASE + 0x300 + (tos) * TC4X_IR_TOS_STRIDE)
#define TC4X_IR_ACCENSCTRL_WRA(tos) \
  (TC4X_IR_INT_BASE + 0x318 + (tos) * TC4X_IR_TOS_STRIDE)

#endif /* __ARCH_TRICORE_SRC_TC4X_HARDWARE_TC4X_CPU_H */
