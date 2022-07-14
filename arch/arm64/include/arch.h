/****************************************************************************
 * arch/arm64/include/arch.h
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

#ifndef __ARCH_ARM64_INCLUDE_ARCH_H
#define __ARCH_ARM64_INCLUDE_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <nuttx/pgalloc.h>
#  include <nuttx/addrenv.h>
#endif

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
#if CONFIG_MM_PGSIZE != 4096
#  error Only pages sizes of 4096 are currently supported (CONFIG_ARCH_ADDRENV)
#endif

#endif /* CONFIG_ARCH_ADDRENV */

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
/* The task group resources are retained in a single structure, task_group_s
 * that is defined in the header file nuttx/include/nuttx/sched.h. The type
 * group_addrenv_t must be defined by platform specific logic in
 * nuttx/arch/<architecture>/include/arch.h.
 *
 * These tables would hold the physical address of the level 2 page tables.
 * All would be initially NULL and would not be backed up with physical
 * memory until mappings in the level 2 page table are required.
 */

struct group_addrenv_s
{
  /* Level 1 page table entries for each group section */

  uintptr_t *text[ARCH_TEXT_NSECTS];
  uintptr_t *data[ARCH_DATA_NSECTS];
#ifdef CONFIG_BUILD_KERNEL
  uintptr_t *heap[ARCH_HEAP_NSECTS];
#ifdef CONFIG_MM_SHM
  uintptr_t *shm[ARCH_SHM_NSECTS];
#endif

  /* Initial heap allocation (in bytes).  This exists only provide an
   * indirect path for passing the size of the initial heap to the heap
   * initialization logic.  These operations are separated in time and
   * architecture.  REVISIT:  I would like a better way to do this.
   */

  size_t heapsize;
#endif
};

typedef struct group_addrenv_s group_addrenv_t;

/* This type is used when the OS needs to temporarily instantiate a
 * different address environment.  Used in the implementation of
 *
 *   int up_addrenv_select(group_addrenv_t addrenv, save_addrenv_t *oldenv);
 *   int up_addrenv_restore(save_addrenv_t oldenv);
 *
 * In this case, the saved value in the L1 page table are returned
 */

struct save_addrenv_s
{
  uint32_t text[ARCH_TEXT_NSECTS];
  uint32_t data[ARCH_DATA_NSECTS];
#ifdef CONFIG_BUILD_KERNEL
  uint32_t heap[ARCH_HEAP_NSECTS];
#ifdef CONFIG_MM_SHM
  uint32_t shm[ARCH_SHM_NSECTS];
#endif
#endif
};

typedef struct save_addrenv_s save_addrenv_t;
#endif

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

#endif /* __ARCH_ARM64_INCLUDE_ARCH_H */
