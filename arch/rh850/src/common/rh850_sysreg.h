/****************************************************************************
 * arch/rh850/src/common/rh850_sysreg.h
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

#ifndef __ARCH_RH850_SRC_COMMON_RH850_SYSREG_H
#define __ARCH_RH850_SRC_COMMON_RH850_SYSREG_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System registers as "regID, selID" for LDSR/STSR */

#define RH850_SR_EIPC       0, 0
#define RH850_SR_EIPSW      1, 0
#define RH850_SR_FEPC       2, 0
#define RH850_SR_FEPSW      3, 0
#define RH850_SR_PSW        5, 0
#define RH850_SR_FPSR       6, 0
#define RH850_SR_FPEPC      7, 0
#define RH850_SR_FPST       8, 0
#define RH850_SR_FPCC       9, 0
#define RH850_SR_FPEC       11, 0
#define RH850_SR_EIIC       13, 0
#define RH850_SR_FEIC       14, 0
#define RH850_SR_CTPC       16, 0
#define RH850_SR_EIWR       28, 0
#define RH850_SR_FEWR       29, 0
#define RH850_SR_EBASE      3, 1
#define RH850_SR_INTBP      4, 1
#define RH850_SR_MEA        6, 2
#define RH850_SR_MEI        8, 2
#define RH850_SR_ISPR       10, 2
#define RH850_SR_RBCR0      15, 2
#define RH850_SR_RBIP       18, 2
#define RH850_SR_ICTAGL     16, 4
#define RH850_SR_ICTAGH     17, 4
#define RH850_SR_ICDATL     18, 4
#define RH850_SR_ICDATH     19, 4
#define RH850_SR_ICERR      28, 4
#define RH850_SR_MCA        8, 5
#define RH850_SR_MCS        9, 5
#define RH850_SR_MCR        11, 5
#define RH850_SR_MCI        12, 5
#define RH850_SR_MPIDX      16, 5
#define RH850_SR_MPLA       20, 5
#define RH850_SR_MPUA       21, 5
#define RH850_SR_MPAT       22, 5

/* PSW bits */

#define RH850_PSW_ID        0x00000020   /* EI level interrupts disabled */
#define RH850_PSW_EP        0x00000040   /* Exception being processed */
#define RH850_PSW_NP        0x00000080   /* FE level interrupts disabled */
#define RH850_PSW_EBV       0x00008000   /* Vectors at EBASE */
#define RH850_PSW_CU0       0x00010000   /* FPU usable */

/* FPSR.FS: subnormal operands and results are flushed, not trapped */

#define RH850_FPSR_FS       0x00020000

/* FPSR.XC: the cause bits E, V, Z, O, U, I */

#define RH850_FPSR_XC_MASK  0x0000fc00

/* FPEC.FPIVD: an imprecise FPU exception (FPI) is reported and pending */

#define RH850_FPEC_FPIVD    0x00000001

/* EIIC of TRAP 0x0 */

#define RH850_EIIC_TRAP0    0x40

/* MPU protection areas on U2C (MPCFG.NMPUE + 1) */

#define RH850_MPU_ENTRIES   24

/* C read of a system register named by its regID, selID pair */

#ifndef __ASSEMBLY__
#  define rh850_stsr_pair(regid, selid)             \
  ({                                                \
    uint32_t __v;                                   \
    __asm__ __volatile__("stsr " #regid ", %0, " #selid \
                         : "=r"(__v));              \
    __v;                                            \
  })
#  define rh850_getsr(sr)   rh850_stsr_pair(sr)
#endif

#endif /* __ARCH_RH850_SRC_COMMON_RH850_SYSREG_H */
