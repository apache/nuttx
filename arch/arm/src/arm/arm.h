/****************************************************************************
 * arch/arm/src/arm/arm.h
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

#ifndef __ARCH_ARM_SRC_ARM_ARM_H
#define __ARCH_ARM_SRC_ARM_ARM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ARM9EJS ******************************************************************/

/* PSR bits */

#define PSR_MODE_MASK   0x0000001f /* Bits 0-4: Mode bits */
#define PSR_MODE_USR26  0x00000000 /*   26-bit User mode */
#define PSR_MODE_FIQ26  0x00000001 /*   26-bit FIQ mode */
#define PSR_MODE_IRQ26  0x00000002 /*   26-bit IRQ mode */
#define PSR_MODE_SVC26  0x00000003 /*   26-bit Supervisor mode */
#define PSR_MODE_MODE32 0x00000010 /*   Bit 4: 32-bit mode */
#define PSR_MODE_USR    0x00000010 /*   32-bit User mode */
#define PSR_MODE_FIQ    0x00000011 /*   32-bit FIQ mode */
#define PSR_MODE_IRQ    0x00000012 /*   32-bit IRQ mode */
#define PSR_MODE_SVC    0x00000013 /*   32-bit Supervisor mode */
#define PSR_MODE_ABT    0x00000017 /*   32-bit Abort mode */
#define PSR_MODE_UND    0x0000001b /*   32-bit Undefined mode */
#define PSR_MODE_SYS    0x0000001f /*   32-bit System mode */
#define PSR_T_BIT       0x00000020 /* Bit 5: Thumb state */
#define PSR_F_BIT       0x00000040 /* Bit 6: FIQ disable */
#define PSR_I_BIT       0x00000080 /* Bit 7: IRQ disable */
                                   /* Bits 8-23: Reserved */
#define PSR_J_BIT       0x01000000 /* Bit 24: Jazelle state bit */
                                   /* Bits 25-26: Reserved */
#define PSR_Q_BIT       0x08000000 /* Bit 27: Sticky overflow */
#define PSR_V_BIT       0x10000000 /* Bit 28: Overflow */
#define PSR_C_BIT       0x20000000 /* Bit 29: Carry/Borrow/Extend */
#define PSR_Z_BIT       0x40000000 /* Bit 30: Zero */
#define PSR_N_BIT       0x80000000 /* Bit 31: Negative/Less than */

/* CR1 bits (CP#15 CR1) */

#define CR_M            0x00000001 /* MMU enable                          */
#define CR_A            0x00000002 /* Alignment abort enable              */
#define CR_C            0x00000004 /* Dcache enable                       */
#define CR_W            0x00000008 /* Write buffer enable                 */
#define CR_P            0x00000010 /* 32-bit exception handler            */
#define CR_D            0x00000020 /* 32-bit data address range           */
#define CR_L            0x00000040 /* Implementation defined              */
#define CR_B            0x00000080 /* Big endian                          */
#define CR_S            0x00000100 /* System MMU protection               */
#define CR_R            0x00000200 /* ROM MMU protection                  */
#define CR_F            0x00000400 /* Implementation defined              */
#define CR_Z            0x00000800 /* Implementation defined              */
#define CR_I            0x00001000 /* Icache enable                       */
#define CR_V            0x00002000 /* Vectors relocated to 0xffff0000     */
#define CR_RR           0x00004000 /* Round Robin cache replacement       */
#define CR_L4           0x00008000 /* LDR pc can set T bit                */
#define CR_DT           0x00010000
#define CR_IT           0x00040000
#define CR_ST           0x00080000
#define CR_FI           0x00200000 /* Fast interrupt (lower latency mode) */
#define CR_U            0x00400000 /* Unaligned access operation          */
#define CR_XP           0x00800000 /* Extended page tables                */
#define CR_VE           0x01000000 /* Vectored interrupts                 */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_ARM_ARM_H */
