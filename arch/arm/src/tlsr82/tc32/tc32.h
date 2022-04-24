/****************************************************************************
 * arch/arm/src/tlsr82/tc32/tc32.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_TC32_TC32_H
#define __ARCH_ARM_SRC_TLSR82_TC32_TC32_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TC32 */

/* PSR bits */

#define PSR_MODE_MASK   0x0000001f /* Bits 0-4: Mode bits */
#define PSR_MODE_IRQ    0x00000012 /*   32-bit IRQ mode */
#define PSR_MODE_SVC    0x00000013 /*   32-bit Supervisor mode */
#define PSR_I_BIT       0x00000080 /* Bit 7: IRQ disable */
                                   /* Bits 8-23: Reserved */
#define PSR_J_BIT       0x01000000 /* Bit 24: Jazelle state bit */
                                   /* Bits 25-26: Reserved */
#define PSR_Q_BIT       0x08000000 /* Bit 27: Sticky overflow */
#define PSR_V_BIT       0x10000000 /* Bit 28: Overflow */
#define PSR_C_BIT       0x20000000 /* Bit 29: Carry/Borrow/Extend */
#define PSR_Z_BIT       0x40000000 /* Bit 30: Zero */
#define PSR_N_BIT       0x80000000 /* Bit 31: Negative/Less than */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_TLSR82_TC32_TC32_H */
