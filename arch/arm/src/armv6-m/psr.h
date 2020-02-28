/************************************************************************************
 * arch/arm/src/armv6-m/psr.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_COMMON_ARMV6_M_PSR_H
#define __ARCH_ARM_SRC_COMMON_ARMV6_M_PSR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Application Program Status Register (APSR) */

#define ARMV6M_APSR_V            (1 << 28) /* Bit 28: Overflow flag */
#define ARMV6M_APSR_C            (1 << 29) /* Bit 29: Carry/borrow flag */
#define ARMV6M_APSR_Z            (1 << 30) /* Bit 30: Zero flag */
#define ARMV6M_APSR_N            (1 << 31) /* Bit 31: Negative, less than flag */

/* Interrupt Program Status Register (IPSR) */

#define ARMV6M_IPSR_ISR_SHIFT    0         /* Bits 5-0: ISR number */
#define ARMV6M_IPSR_ISR_MASK     (31 << ARMV6M_IPSR_ISR_SHIFT)

/* Execution PSR Register (EPSR) */

#define ARMV6M_EPSR_T            (1 << 24) /* Bit 24: T-bit */

/* Save xPSR bits */

#define ARMV6M_XPSR_ISR_SHIFT    ARMV6M_IPSR_ISR_SHIFT
#define ARMV6M_XPSR_ISR_MASK     ARMV6M_IPSR_ISR_MASK
#define ARMV6M_XPSR_T            ARMV6M_EPSR_T
#define ARMV6M_XPSR_V            ARMV6M_APSR_V
#define ARMV6M_XPSR_C            ARMV6M_APSR_C
#define ARMV6M_XPSR_Z            ARMV6M_APSR_Z
#define ARMV6M_XPSR_N            ARMV6M_APSR_N

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_COMMON_ARMV6_M_PSR_H */
