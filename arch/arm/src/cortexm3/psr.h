/************************************************************************************
 * arch/arm/src/cortexm3/psr.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_COMMON_CORTEXM_PSR_H
#define __ARCH_ARM_SRC_COMMON_CORTEXM_PSR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Application Program Status Register (APSR) */

#define CORTEXM3_APSR_Q            (1 << 27) /* Bit 27: Sticky saturation flag */
#define CORTEXM3_APSR_V            (1 << 28) /* Bit 28: Overflow flag */
#define CORTEXM3_APSR_C            (1 << 29) /* Bit 29: Carry/borrow flag */
#define CORTEXM3_APSR_Z            (1 << 30) /* Bit 30: Zero flag */
#define CORTEXM3_APSR_N            (1 << 31) /* Bit 31: Negative, less than flag */

/* Interrupt Program Status Register (IPSR) */

#define CORTEXM3_IPSR_ISR_SHIFT    0         /* Bits 8-0: ISR number */
#define CORTEXM3_IPSR_ISR_MASK     (0x1ff << CORTEXM3_IPSR_ISR_SHIFT)

/* Execution PSR Register (EPSR) */

#define CORTEXM3_EPSR_ICIIT1_SHIFT 10        /* Bits 15-10: Interrupt-Continuable-Instruction/If-Then bits */
#define CORTEXM3_EPSR_ICIIT1_MASK  (3 << CORTEXM3_EPSR_ICIIT1_SHIFT)
#define CORTEXM3_EPSR_T            (1 << 24) /* Bit 24: T-bit */
#define CORTEXM3_EPSR_ICIIT2_SHIFT 25        /* Bits 26-25: Interrupt-Continuable-Instruction/If-Then bits */
#define CORTEXM3_EPSR_ICIIT2_MASK  (3 << CORTEXM3_EPSR_ICIIT2_SHIFT)

/* Save xPSR bits */

#define CORTEXM3_XPSR_ISR_SHIFT    CORTEXM3_IPSR_ISR_SHIFT
#define CORTEXM3_XPSR_ISR_MASK     CORTEXM3_IPSR_ISR_MASK
#define CORTEXM3_XPSR_ICIIT1_SHIFT CORTEXM3_EPSR_ICIIT1_SHIFT/
#define CORTEXM3_XPSR_ICIIT1_MASK  CORTEXM3_EPSR_ICIIT1_MASK
#define CORTEXM3_XPSR_T            CORTEXM3_EPSR_T
#define CORTEXM3_XPSR_ICIIT2_SHIFT CORTEXM3_EPSR_ICIIT2_SHIFT
#define CORTEXM3_XPSR_ICIIT2_MASK  CORTEXM3_EPSR_ICIIT2_MASK
#define CORTEXM3_XPSR_Q            CORTEXM3_APSR_Q
#define CORTEXM3_XPSR_V            CORTEXM3_APSR_V
#define CORTEXM3_XPSR_C            CORTEXM3_APSR_C
#define CORTEXM3_XPSR_Z            CORTEXM3_APSR_Z
#define CORTEXM3_XPSR_N            CORTEXM3_APSR_N

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __ARCH_ARM_SRC_COMMON_CORTEXM_PSR_H */
