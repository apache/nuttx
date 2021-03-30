/****************************************************************************
 * arch/mips/src/mips32/mips32-memorymap.h
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

#ifndef __ARCH_MIPS_SRC_MIPS32_MIPS32_MEMORYMAP_H
#define __ARCH_MIPS_SRC_MIPS32_MIPS32_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* MIPS32 address space organization */

#define USEG_BASE  0x00000000
#define USEG_SIZE  0x80000000

#define KSEG0_BASE 0x80000000
#define KSEG0_SIZE 0x20000000

#define KSEG1_BASE 0xa0000000
#define KSEG1_SIZE 0x20000000

#define KSEG2_BASE 0xc0000000
#define KSEG2_SIZE 0x20000000

#define KSEG3_BASE 0xe0000000
#define KSEG3_SIZE 0x20000000

#define DSEG_BASE  0xff200000
#define DSEG_SIZE  0x00200000

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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
#endif /* __ARCH_MIPS_SRC_MIPS32_MIPS32_MEMORYMAP_H */
