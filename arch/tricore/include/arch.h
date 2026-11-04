/****************************************************************************
 * arch/tricore/include/arch.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/arch.h
 */

#ifndef __ARCH_TRICORE_INCLUDE_ARCH_H
#define __ARCH_TRICORE_INCLUDE_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stddef.h>
#endif

#include <arch/barriers.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MTCR and MFCR - move to / from Core Special Function Register */

#define tricore_mtcr(reg, val)                                         \
  ({                                                                   \
    __asm__ volatile ("mtcr %0,%1\n\t"::"i"(reg),"d"(val):"memory");   \
  })

#define tricore_mfcr(reg)                                              \
  ({                                                                   \
    uint32_t __val;                                                    \
    __asm__ volatile ("mfcr %0,%1": "=d" (__val) :"i"(reg): "memory"); \
    __val;                                                             \
  })

/* TriCore core special function register (CSFR) */

#define CPU_PSW             0xFE04  /* Program Status Word */
#define CPU_CORE_ID         0xFE1C  /* CPU Core Identification */
#define CPU_BIV             0xFE20  /* Base Interrupt Vector Pointer */
#define CPU_BTV             0xFE24  /* Base Trap Vector Pointer */
#define CPU_ISP             0xFE28  /* Interrupt Stack Pointer */
#define CPU_FCX             0xFE38  /* Free CSA List Head Pointer */
#define CPU_LCX             0xFE3C  /* Free CSA List Limit Pointer */
#define CPU_VCON0           0xB000  /* CPU virtualization control */

/* Initial PSW values */

#define TRICORE_PSW_IS      0xa80   /* PSW.CDE | PSW.IS | supervisor */
#define TRICORE_PSW_DEFAULT 0x880   /* PSW.CDE | supervisor */

/* PSPR scratch area used as sp/CSA while BIST destroys the ram */

#ifdef CONFIG_ARCH_BIST
#  define PSPR_USTACK       0x7010eb00
#  define PSPR_CSA          0x7010ec00
#  define PSPR_SIZE         0x1000
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_TRICORE_INCLUDE_ARCH_H */
