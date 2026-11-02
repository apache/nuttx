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

/* TriCore core special function register (CSFR) */

#define CPU_PCXI    0xFE00  /* Previous Context Information */
#define CPU_PSW     0xFE04  /* Program Status Word */
#define CPU_PC      0xFE08  /* Program Counter */
#define CPU_SYSCON  0xFE14  /* System Control Register */
#define CPU_CORE_ID 0xFE1C  /* CPU Core Identification */
#define CPU_ICR     0xFE2C  /* Interrupt Control Register */
#define CPU_PPRS    0xFE34  /* Physical Protection Selection */
#define CPU_FCX     0xFE38  /* Free CSA List Head Pointer */
#define CPU_LCX     0xFE3C  /* Free CSA List Limit Pointer */

#define CPU_CCTRL   0xFC00  /* Counter Control */
#define CPU_CCNT    0xFC04  /* CPU Clock Cycle Count */

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

/* Jump back to the return address. */

#ifdef CONFIG_TRICORE_TOOLCHAIN_TASKING
#  define tricore_jump_backto_link()  __asm__ volatile ("ji a11")
#else
#  define tricore_jump_backto_link()  __asm__ volatile ("ji %a11")
#endif

/* Read the return address register. */

#ifdef CONFIG_TRICORE_TOOLCHAIN_TASKING
#  define tricore_get_a11()                                            \
    ({                                                                 \
      uintptr_t __ra;                                                  \
      __asm__ volatile ("mov.aa %0, a11" : "=a"(__ra) : : "a11");      \
      __ra;                                                            \
    })
#else
#  define tricore_get_a11()                                            \
    ({                                                                 \
      uintptr_t __ra;                                                  \
      __asm__ volatile ("mov.aa %0, %%a11" : "=a"(__ra) : : "a11");    \
      __ra;                                                            \
    })
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

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

#endif /* __ARCH_TRICORE_INCLUDE_ARCH_H */
