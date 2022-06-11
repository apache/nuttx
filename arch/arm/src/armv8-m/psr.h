/****************************************************************************
 * arch/arm/src/armv8-m/psr.h
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

#ifndef __ARCH_ARM_SRC_ARMV8_M_PSR_H
#define __ARCH_ARM_SRC_ARMV8_M_PSR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Application Program Status Register (APSR) */

#define ARMV8M_APSR_GE_SHIFT     16        /* Bits 16-19: Greater than or equal flags */
#define ARMV8M_APSR_GE_MASK      (0xf << ARMV8M_APSR_GE_SHIFT)

#define ARMV8M_APSR_Q            (1 << 27) /* Bit 27: Sticky saturation flag */
#define ARMV8M_APSR_V            (1 << 28) /* Bit 28: Overflow flag */
#define ARMV8M_APSR_C            (1 << 29) /* Bit 29: Carry/borrow flag */
#define ARMV8M_APSR_Z            (1 << 30) /* Bit 30: Zero flag */
#define ARMV8M_APSR_N            (1 << 31) /* Bit 31: Negative, less than flag */

/* Interrupt Program Status Register (IPSR) */

#define ARMV8M_IPSR_ISR_SHIFT    0         /* Bits 8-0: ISR number */
#define ARMV8M_IPSR_ISR_MASK     (0x1ff << ARMV8M_IPSR_ISR_SHIFT)

/* Execution PSR Register (EPSR) */

#define ARMV8M_EPSR_ICIIT1_SHIFT 10        /* Bits 15-10: Interrupt-Continuable-Instruction/If-Then bits */
#define ARMV8M_EPSR_ICIIT1_MASK  (0x3f << ARMV8M_EPSR_ICIIT1_SHIFT)
#define ARMV8M_EPSR_B            (1 << 21) /* Bit 21: Branch target identification active */
#define ARMV8M_EPSR_T            (1 << 24) /* Bit 24: T-bit */
#define ARMV8M_EPSR_ICIIT2_SHIFT 25        /* Bits 26-25: Interrupt-Continuable-Instruction/If-Then bits */
#define ARMV8M_EPSR_ICIIT2_MASK  (3 << ARMV8M_EPSR_ICIIT2_SHIFT)

/* Return PSR Register (RETPSR) */

#define ARMV8M_RETPSR_SPREALIGN  (1 << 9)  /* Bit 9: Stack-pointer re-align */
#define ARMV8M_RETPSR_SFPA       (1 << 20) /* Bit 20: Secure Floating-point active */

/* Save xPSR bits */

#define ARMV8M_XPSR_ISR_SHIFT    ARMV8M_IPSR_ISR_SHIFT
#define ARMV8M_XPSR_ISR_MASK     ARMV8M_IPSR_ISR_MASK
#define ARMV8M_XPSR_SPREALIGN    ARMV8M_RETPSR_SPREALIGN
#define ARMV8M_XPSR_ICIIT1_SHIFT ARMV8M_EPSR_ICIIT1_SHIFT
#define ARMV8M_XPSR_ICIIT1_MASK  ARMV8M_EPSR_ICIIT1_MASK
#define ARMV8M_XPSR_GE_SHIFT     ARMV8M_APSR_GE_SHIFT
#define ARMV8M_XPSR_GE_MASK      ARMV8M_APSR_GE_MASK
#define ARMV8M_XPSR_SFPA         ARMV8M_RETPSR_SFPA
#define ARMV8M_XPSR_B            ARMV8M_EPSR_B
#define ARMV8M_XPSR_T            ARMV8M_EPSR_T
#define ARMV8M_XPSR_ICIIT2_SHIFT ARMV8M_EPSR_ICIIT2_SHIFT
#define ARMV8M_XPSR_ICIIT2_MASK  ARMV8M_EPSR_ICIIT2_MASK
#define ARMV8M_XPSR_Q            ARMV8M_APSR_Q
#define ARMV8M_XPSR_V            ARMV8M_APSR_V
#define ARMV8M_XPSR_C            ARMV8M_APSR_C
#define ARMV8M_XPSR_Z            ARMV8M_APSR_Z
#define ARMV8M_XPSR_N            ARMV8M_APSR_N

/* Floating-point Status and Control Register (FPSCR) */

#define ARMV8M_FPSCR_IOC           (1 << 0)  /* Bit 0: Invalid Operation */
#define ARMV8M_FPSCR_DZC           (1 << 1)  /* Bit 1: Divide by Zero */
#define ARMV8M_FPSCR_OFC           (1 << 2)  /* Bit 2: Overflow */
#define ARMV8M_FPSCR_UFC           (1 << 3)  /* Bit 3: Underflow */
#define ARMV8M_FPSCR_IXC           (1 << 4)  /* Bit 4: Inexact */
#define ARMV8M_FPSCR_IDC           (1 << 7)  /* Bit 7: Input Denormal */

#define ARMV8M_FPSCR_LTPSIZE_SHIFT 16        /* Bits 16-18: Vector element size */
#define ARMV8M_FPSCR_LTPSIZE_8BIT  (0x0 << ARMV8M_FPSCR_LTPSIZE_SHIFT)
#define ARMV8M_FPSCR_LTPSIZE_16BIT (0x1 << ARMV8M_FPSCR_LTPSIZE_SHIFT)
#define ARMV8M_FPSCR_LTPSIZE_32BIT (0x2 << ARMV8M_FPSCR_LTPSIZE_SHIFT)
#define ARMV8M_FPSCR_LTPSIZE_64BIT (0x3 << ARMV8M_FPSCR_LTPSIZE_SHIFT)
#define ARMV8M_FPSCR_LTPSIZE_NONE  (0x4 << ARMV8M_FPSCR_LTPSIZE_SHIFT)
#define ARMV8M_FPSCR_LTPSIZE_MASK  (0x7 << ARMV8M_FPSCR_LTPSIZE_SHIFT)

#define ARMV8M_FPSCR_FZ16          (1 << 19) /* Bit 19: Flush-to-zero mode(half-precision) */

#define ARMV8M_FPSCR_RM_SHIFT      22        /* Bits 22-23: Round mode */
#define ARMV8M_FPSCR_RM_NEAR       (0x0 << ARMV8M_FPSCR_RM_SHIFT)
#define ARMV8M_FPSCR_RM_PLUS       (0x1 << ARMV8M_FPSCR_RM_SHIFT)
#define ARMV8M_FPSCR_RM_MINUS      (0x2 << ARMV8M_FPSCR_RM_SHIFT)
#define ARMV8M_FPSCR_RM_ZERO       (0x3 << ARMV8M_FPSCR_RM_SHIFT)
#define ARMV8M_FPSCR_RM_MASK       (0x3 << ARMV8M_FPSCR_RM_SHIFT)

#define ARMV8M_FPSCR_FZ            (1 << 24) /* Bit 24: Flush-to-zero mode */
#define ARMV8M_FPSCR_DN            (1 << 25) /* Bit 25: Default NaN mode */
#define ARMV8M_FPSCR_AHP           (1 << 26) /* Bit 26: Alternative half-precision */
#define ARMV8M_FPSCR_Q             (1 << 27) /* Bit 27: Sticky saturation flag */
#define ARMV8M_FPSCR_V             (1 << 28) /* Bit 28: Overflow flag */
#define ARMV8M_FPSCR_C             (1 << 29) /* Bit 29: Carry/borrow flag */
#define ARMV8M_FPSCR_Z             (1 << 30) /* Bit 30: Zero flag */
#define ARMV8M_FPSCR_N             (1 << 31) /* Bit 31: Negative, less than flag */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_ARMV8_M_PSR_H */
