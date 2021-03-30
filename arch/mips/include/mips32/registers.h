/****************************************************************************
 * arch/mips/include/mips32/registers.h
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

#ifndef __ARCH_MIPS_INCLUDE_MIPS32_REGISTERS_H
#define __ARCH_MIPS_INCLUDE_MIPS32_REGISTERS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Standard synonmyms for MIPS registers */

#ifdef __ASSEMBLY__
/* Zero register:  Always returns 0 */

#define zero               $0

/* Assembler temporary register:  Reserved for use by the assembler */

#define at_reg             $1

/* Return value registers:  Value returned by function */

#define v0                 $2
#define v1                 $3

/* Argument registers:  First four parameters to a function */

#define a0                 $4
#define a1                 $5
#define a2                 $6
#define a3                 $7

/* Volatile registers: Registers that can be used without saving */

#define t0                 $8
#define t1                 $9
#define t2                 $10
#define t3                 $11
#define t4                 $12
#define t5                 $13
#define t6                 $14
#define t7                 $15
#define t8                 $24
#define t9                 $25

/* Static registers:  Registers that must be saved and restored if used */

#define s0                 $16
#define s1                 $17
#define s2                 $18
#define s3                 $19
#define s4                 $20
#define s5                 $21
#define s6                 $22
#define s7                 $23

/* Reserved for use by interrupt/trap handling logic */

#define k0                 $26
#define k1                 $27

/* Global pointer register */

#define gp                 $28

/* Stack pointer register:  Stack pointer */

#define sp                 $29

/* Register 30 may be either an additional static register or a frame
 * pointer
 */

#define s8                 $30
#define fp                 $30

/* Return address register:  Contains the function return address */

#define ra                 $31
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
#endif

#endif /* __ARCH_MIPS_INCLUDE_MIPS32_REGISTERS_H */
