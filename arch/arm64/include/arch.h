/****************************************************************************
 * arch/arm64/include/arch.h
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

#ifndef __ARCH_ARM64_INCLUDE_ARCH_H
#define __ARCH_ARM64_INCLUDE_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stddef.h>
#endif

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
#if CONFIG_MM_PGSIZE != 4096
#  error Only pages sizes of 4096 are currently supported (CONFIG_ARCH_ADDRENV)
#endif

/* All implementations have 4 levels of page tables */

#define ARCH_PGT_MAX_LEVELS (4)
#define ARCH_SPGTS          (ARCH_PGT_MAX_LEVELS - 1)

#endif /* CONFIG_ARCH_ADDRENV */

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
#ifndef __ASSEMBLY__

/* A task group must have its L1 table in memory always, and the rest can
 * be dynamically committed to memory (and even swapped).
 *
 * In this implementation level tables except the final level N are always
 * kept in static memory, while the level N tables are always dynamically
 * allocated. There is one static page per level in `spgtables[]`.
 *
 * For the VMSAv8-64 address translation system this means that:
 * - A task can not have more than 1GB of memory allocated. This should be
 *   plenty enough...
 * - The minimum amount of memory needed for page tables per task is 12K,
 *   which gives access to 2MB of memory. This is plenty for many tasks.
 */

struct arch_addrenv_s
{
  /* Physical addresses of the static page tables (levels N-1) here, these
   * are allocated when a task is created.
   */

  uintptr_t spgtables[ARCH_SPGTS];

  /* The text, data, heap bases and heap size here */

  uintptr_t textvbase;
  uintptr_t datavbase;
  uintptr_t heapvbase;
  size_t    heapsize;

  /* The page directory root (ttbr0) value */

  uintptr_t ttbr0;
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

#endif /* __ARCH_ARM64_INCLUDE_ARCH_H */
