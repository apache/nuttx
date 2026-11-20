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
#include <arch/csfr.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MTCR and MFCR - move to / from Core Special Function Register */

#define tricore_mtcr(reg, val)                                             \
  ({                                                                       \
    __asm__ __volatile__ ("mtcr %0,%1\n\t"::"i"(reg),"d"(val):"memory");   \
  })

#define tricore_mfcr(reg)                                                  \
  ({                                                                       \
    uint32_t __val;                                                        \
    __asm__ __volatile__ ("mfcr %0,%1": "=d" (__val) :"i"(reg): "memory"); \
    __val;                                                                 \
  })

/* Return from a handler that the trap or interrupt vector entered with
 * jli, to the vector code at A11.
 */

#ifdef CONFIG_TRICORE_TOOLCHAIN_TASKING
#  define tricore_jump_backto_link() \
     __asm__ __volatile__ ("ji a11" : : : "a11")
#else
#  define tricore_jump_backto_link() \
     __asm__ __volatile__ ("ji %a11")
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
/* The task group resources are retained in a single structure, task_group_s
 * that is defined in the header file nuttx/include/nuttx/sched.h. The type
 * arch_addrenv_t must be defined by platform specific logic in
 * nuttx/arch/<architecture>/include/arch.h.
 *
 * These tables would hold the physical address of the level 2 page tables.
 * All would be initially NULL and would not be backed up with physical
 * memory until mappings in the level 2 page table are required.
 */

struct arch_addrenv_s
{
  uintptr_t text;
  size_t    textsize;
  uintptr_t data;
  size_t    datasize;
#ifdef CONFIG_BUILD_PROTECTED
  uintptr_t heap;
  size_t    heapsize;
#endif
};

typedef struct arch_addrenv_s arch_addrenv_t;
#endif

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
