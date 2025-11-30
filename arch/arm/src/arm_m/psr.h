/****************************************************************************
 * arch/arm/src/arm_m/psr.h
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

#ifndef __ARCH_ARM_SRC_ARM_M_PSR_H
#define __ARCH_ARM_SRC_ARM_M_PSR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Application Program Status Register (APSR) */

#define ARM_APSR_GE_SHIFT       16        /* Bits 16-19: Greater than or equal flags */
#define ARM_APSR_GE_MASK        (0xf << ARM_APSR_GE_SHIFT)

#define ARM_APSR_Q              (1 << 27) /* Bit 27: Sticky saturation flag */
#define ARM_APSR_V              (1 << 28) /* Bit 28: Overflow flag */
#define ARM_APSR_C              (1 << 29) /* Bit 29: Carry/borrow flag */
#define ARM_APSR_Z              (1 << 30) /* Bit 30: Zero flag */
#define ARM_APSR_N              (1 << 31) /* Bit 31: Negative, less than flag */

/* Interrupt Program Status Register (IPSR) */

#define ARM_IPSR_ISR_SHIFT      0         /* Bits 8-0: ISR number */
#define ARM_IPSR_ISR_MASK       (0x1ff << ARM_IPSR_ISR_SHIFT)

/* Execution PSR Register (EPSR) */

#define ARM_EPSR_ICIIT1_SHIFT   10        /* Bits 15-10: Interrupt-Continuable-Instruction/If-Then bits */
#define ARM_EPSR_ICIIT1_MASK    (0x3f << ARM_EPSR_ICIIT1_SHIFT)
#define ARM_EPSR_B              (1 << 21) /* Bit 21: Branch target identification active */
#define ARM_EPSR_T              (1 << 24) /* Bit 24: T-bit */
#define ARM_EPSR_ICIIT2_SHIFT   25        /* Bits 26-25: Interrupt-Continuable-Instruction/If-Then bits */
#define ARM_EPSR_ICIIT2_MASK    (3 << ARM_EPSR_ICIIT2_SHIFT)

/* Return PSR Register (RETPSR) */

#define ARM_RETPSR_SPREALIGN    (1 << 9)  /* Bit 9: Stack-pointer re-align */
#define ARM_RETPSR_SFPA         (1 << 20) /* Bit 20: Secure Floating-point active */

/* Save xPSR bits */

#define ARM_XPSR_ISR_SHIFT      ARM_IPSR_ISR_SHIFT
#define ARM_XPSR_ISR_MASK       ARM_IPSR_ISR_MASK
#define ARM_XPSR_SPREALIGN      ARM_RETPSR_SPREALIGN
#define ARM_XPSR_ICIIT1_SHIFT   ARM_EPSR_ICIIT1_SHIFT
#define ARM_XPSR_ICIIT1_MASK    ARM_EPSR_ICIIT1_MASK
#define ARM_XPSR_GE_SHIFT       ARM_APSR_GE_SHIFT
#define ARM_XPSR_GE_MASK        ARM_APSR_GE_MASK
#define ARM_XPSR_SFPA           ARM_RETPSR_SFPA
#define ARM_XPSR_B              ARM_EPSR_B
#define ARM_XPSR_T              ARM_EPSR_T
#define ARM_XPSR_ICIIT2_SHIFT   ARM_EPSR_ICIIT2_SHIFT
#define ARM_XPSR_ICIIT2_MASK    ARM_EPSR_ICIIT2_MASK
#define ARM_XPSR_Q              ARM_APSR_Q
#define ARM_XPSR_V              ARM_APSR_V
#define ARM_XPSR_C              ARM_APSR_C
#define ARM_XPSR_Z              ARM_APSR_Z
#define ARM_XPSR_N              ARM_APSR_N

/* Floating-point Status and Control Register (FPSCR) */

#define ARM_FPSCR_IOC           (1 << 0)  /* Bit 0: Invalid Operation */
#define ARM_FPSCR_DZC           (1 << 1)  /* Bit 1: Divide by Zero */
#define ARM_FPSCR_OFC           (1 << 2)  /* Bit 2: Overflow */
#define ARM_FPSCR_UFC           (1 << 3)  /* Bit 3: Underflow */
#define ARM_FPSCR_IXC           (1 << 4)  /* Bit 4: Inexact */
#define ARM_FPSCR_IDC           (1 << 7)  /* Bit 7: Input Denormal */

#define ARM_FPSCR_LTPSIZE_SHIFT 16        /* Bits 16-18: Vector element size */
#define ARM_FPSCR_LTPSIZE_8BIT  (0x0 << ARM_FPSCR_LTPSIZE_SHIFT)
#define ARM_FPSCR_LTPSIZE_16BIT (0x1 << ARM_FPSCR_LTPSIZE_SHIFT)
#define ARM_FPSCR_LTPSIZE_32BIT (0x2 << ARM_FPSCR_LTPSIZE_SHIFT)
#define ARM_FPSCR_LTPSIZE_64BIT (0x3 << ARM_FPSCR_LTPSIZE_SHIFT)
#define ARM_FPSCR_LTPSIZE_NONE  (0x4 << ARM_FPSCR_LTPSIZE_SHIFT)
#define ARM_FPSCR_LTPSIZE_MASK  (0x7 << ARM_FPSCR_LTPSIZE_SHIFT)

#define ARM_FPSCR_FZ16          (1 << 19) /* Bit 19: Flush-to-zero mode(half-precision) */

#define ARM_FPSCR_RM_SHIFT      22        /* Bits 22-23: Round mode */
#define ARM_FPSCR_RM_NEAR       (0x0 << ARM_FPSCR_RM_SHIFT)
#define ARM_FPSCR_RM_PLUS       (0x1 << ARM_FPSCR_RM_SHIFT)
#define ARM_FPSCR_RM_MINUS      (0x2 << ARM_FPSCR_RM_SHIFT)
#define ARM_FPSCR_RM_ZERO       (0x3 << ARM_FPSCR_RM_SHIFT)
#define ARM_FPSCR_RM_MASK       (0x3 << ARM_FPSCR_RM_SHIFT)

#define ARM_FPSCR_FZ            (1 << 24) /* Bit 24: Flush-to-zero mode */
#define ARM_FPSCR_DN            (1 << 25) /* Bit 25: Default NaN mode */
#define ARM_FPSCR_AHP           (1 << 26) /* Bit 26: Alternative half-precision */
#define ARM_FPSCR_Q             (1 << 27) /* Bit 27: Sticky saturation flag */
#define ARM_FPSCR_V             (1 << 28) /* Bit 28: Overflow flag */
#define ARM_FPSCR_C             (1 << 29) /* Bit 29: Carry/borrow flag */
#define ARM_FPSCR_Z             (1 << 30) /* Bit 30: Zero flag */
#define ARM_FPSCR_N             (1 << 31) /* Bit 31: Negative, less than flag */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_ARM_M_PSR_H */
