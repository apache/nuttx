/****************************************************************************
 * arch/x86_64/include/arch.h
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

/* This file should never be included directly but, rather, only indirectly
 * through arch/arch.h
 */

#ifndef __ARCH_X86_64_INCLUDE_ARCH_H
#define __ARCH_X86_64_INCLUDE_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

/* Include chip-specific definitions */

#  include <arch/chip/arch.h>

/* Include architecture-specific definitions */

#ifdef CONFIG_ARCH_INTEL64
#  include <arch/intel64/arch.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 4 levels of page table.
 * NOTE: in this implementation:
 *   PTL4 index = 0
 *   PDPT index = 1
 *   PD   index = 2
 *   PT   index = 3
 */

#define ARCH_PGT_MAX_LEVELS 4
#define ARCH_SPGTS          (ARCH_PGT_MAX_LEVELS - 1)

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
#ifndef __ASSEMBLY__

/* The task group resources are retained in a single structure, task_group_s
 * that is defined in the header file nuttx/include/nuttx/sched.h. The type
 * arch_addrenv_t must be defined by platform specific logic in
 * nuttx/arch/<architecture>/include/arch.h.
 */

struct arch_addrenv_s
{
  /* Physical addresses of the static page tables here, these
   * are allocated when a task is created:
   *
   *   spgtables[0] - PML4
   *   spgtables[1] - PDPT
   *   spgtables[2] - PD
   *   PT - dynamic allocation
   */

  uintptr_t spgtables[ARCH_SPGTS];

  /* The text, data, heap bases and heap size here */

  uintptr_t textvbase;
  uintptr_t datavbase;
  uintptr_t heapvbase;
  size_t    heapsize;

  /* The page directory root (pml4) is stored in CR3 register */

  uintptr_t cr3;
};

typedef struct arch_addrenv_s arch_addrenv_t;
#endif /* __ASSEMBLY__ */
#endif /* CONFIG_ARCH_ADDRENV */

/****************************************************************************
 * Public Data
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

#endif /* __ARCH_X86_64_INCLUDE_ARCH_H */
