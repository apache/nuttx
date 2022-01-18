/****************************************************************************
 * arch/arm/src/armv6-m/psr.h
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

#ifndef __ARCH_ARM_SRC_ARMV6_M_PSR_H
#define __ARCH_ARM_SRC_ARMV6_M_PSR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_ARMV6_M_PSR_H */
