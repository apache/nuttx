/****************************************************************************
 * arch/risc-v/include/arch.h
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

#ifndef __ARCH_RISCV_INCLUDE_ARCH_H
#define __ARCH_RISCV_INCLUDE_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stddef.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Provide the maximum amount of page table levels per MMU type */

#ifdef CONFIG_ARCH_MMU_TYPE_SV39
#  define ARCH_PGT_MAX_LEVELS (3)
#endif

/* Amount of static page tables allocated for an address environment */

#ifdef CONFIG_ARCH_ADDRENV
#  define ARCH_SPGTS          (ARCH_PGT_MAX_LEVELS - 1)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
#ifndef __ASSEMBLY__

/* A task group must have its L1 table in memory always, and the rest can
 * be dynamically committed to memory (and even swapped).
 *
 * In this implementation every level tables besides the final level N are
 * kept in memory always, while the level N tables are dynamically allocated.
 *
 * The implications ? They depend on the MMU type.
 *
 * For Sv39 this means that:
 * - A task can not have more than 1GB of memory allocated. This should be
 *   plenty enough...
 * - The minimum amount of memory needed for page tables per task is 12K,
 *   which gives access to 2MB of memory. This is plenty for many tasks.
 */

struct group_addrenv_s
{
  /* Pointers to MAX_LEVELS-1 tables here, one of each are allocated for the
   * task when it is created.
   */

  uintptr_t spgtables[ARCH_SPGTS];

  /* For convenience store the text base here */

  uintptr_t textvbase;

  /* For convenience store the data base here */

  uintptr_t datavbase;

  /* For convenience store the heap base and initial size here */

  uintptr_t heapvbase;
  size_t    heapsize;

  /* For convenience store the satp value here */

  uintptr_t satp;
};

typedef struct group_addrenv_s group_addrenv_t;

/* If an address environment needs to be saved, saving the satp register
 * will suffice. The register width is architecture dependent
 */

typedef uintptr_t save_addrenv_t;
#endif /* __ASSEMBLY__ */
#endif /* CONFIG_ARCH_ADDRENV */

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

#endif /* __ARCH_RISCV_INCLUDE_ARCH_H */
