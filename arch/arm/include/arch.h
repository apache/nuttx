/****************************************************************************
 * arch/arm/include/arch.h
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

#ifndef __ARCH_ARM_INCLUDE_ARCH_H
#define __ARCH_ARM_INCLUDE_ARCH_H

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

#define up_getsp()              (uintptr_t)__builtin_frame_address(0)

#ifdef CONFIG_PIC

/* This identifies the register the is used by the processor as the PIC base
 * register.  It is usually r9 or r10
 */

#define PIC_REG         r10
#define PIC_REG_STRING "r10"

/* Macros to get and set the PIC base register.  picbase is assumed to be
 * of type (void*) and that it will fit into a uint32_t.  These must be
 * inline so that they will be compatible with the ABIs rules for
 * preserving the PIC register
 */

#define up_getpicbase(ppicbase) \
do { \
  uint32_t picbase; \
  __asm__ \
  ( \
    "\tmov %0, " PIC_REG_STRING "\n\t" \
    : "=r"(picbase) \
  ); \
  *ppicbase = (void *)picbase; \
} while (0)

#define up_setpicbase(picbase) \
do { \
  uint32_t _picbase = (uint32_t)picbase; \
  __asm__ \
  ( \
    "\tmov " PIC_REG_STRING ", %0\n\t" \
    : : "r"(_picbase) : PIC_REG_STRING \
  ); \
} while (0)

#endif /* CONFIG_PIC */

#ifdef CONFIG_ARCH_ADDRENV
#if CONFIG_MM_PGSIZE != 4096
#  error Only pages sizes of 4096 are currently supported (CONFIG_ARCH_ADDRENV)
#endif

/* Convert 4KiB pages to 1MiB sections */

#  define __PG2SECT_SHIFT     (20 - MM_PGSHIFT)
#  define __PG2SECT_MASK      ((1 << __PG2SECT_SHIFT) - 1)

#  define ARCH_PG2SECT(p)     (((p) + __PG2SECT_MASK) >> __PG2SECT_SHIFT)
#  define ARCH_SECT2PG(s)     ((s) << __PG2SECT_SHIFT)

#  define ARCH_TEXT_NSECTS    ARCH_PG2SECT(CONFIG_ARCH_TEXT_NPAGES)
#  define ARCH_DATA_NSECTS    ARCH_PG2SECT(CONFIG_ARCH_DATA_NPAGES)
#  define ARCH_HEAP_NSECTS    ARCH_PG2SECT(CONFIG_ARCH_HEAP_NPAGES)

#  ifdef CONFIG_ARCH_VMA_MAPPING
#    define ARCH_SHM_NSECTS   ARCH_PG2SECT(ARCH_SHM_MAXPAGES)
#  endif

#  ifdef CONFIG_ARCH_STACK_DYNAMIC
#    define ARCH_STACK_NSECTS ARCH_PG2SECT(CONFIG_ARCH_STACK_NPAGES)
#  endif
#endif /* CONFIG_ARCH_ADDRENV */

/* Redefine the linker symbols as armlink style */

#ifdef CONFIG_ARM_TOOLCHAIN_ARMCLANG
#  define _sinit   Image$$init_section$$Base
#  define _einit   Image$$init_section$$Limit
#endif

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
#ifdef CONFIG_ARCH_VMA_MAPPING
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
 * In this case, the saved valued in the L1 page table are returned
 */

struct save_addrenv_s
{
  uint32_t text[ARCH_TEXT_NSECTS];
  uint32_t data[ARCH_DATA_NSECTS];
#ifdef CONFIG_BUILD_KERNEL
  uint32_t heap[ARCH_HEAP_NSECTS];
#ifdef CONFIG_ARCH_VMA_MAPPING
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

#endif /* __ARCH_ARM_INCLUDE_ARCH_H */
