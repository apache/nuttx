/****************************************************************************
 * arch/ceva/include/arch.h
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

#ifndef __ARCH_CEVA_INCLUDE_ARCH_H
#define __ARCH_CEVA_INCLUDE_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/chip/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK == 0
/* The interrupt stack is required for CEVA porting */

#  undef CONFIG_ARCH_INTERRUPTSTACK
#  define CONFIG_ARCH_INTERRUPTSTACK CONFIG_IDLETHREAD_STACKSIZE
#endif

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
/* In the kernel build, there are multiple user heaps; one for each task
 * group.  In this build configuration, the user heap structure lies
 * in a reserved region at the beginning of the .bss/.data address
 * space (CONFIG_ARCH_DATA_VBASE).  The size of that region is given by
 * ARCH_DATA_RESERVE_SIZE
 */

#  define UMM_HEAP(i) ((i) ? NULL : &ARCH_DATA_RESERVE->ar_usrheap)

#elif defined(CONFIG_BUILD_PROTECTED) && defined(__KERNEL__)
/* In the protected mode, there are two heaps:  A kernel heap and a single
 * user heap.  Kernel code must obtain the address of the user heap data
 * structure from the userspace interface.
 */

#  define UMM_HEAP(i) ((struct mm_heap_s *const *)USERSPACE->us_heap)[i]

#else
/* Otherwise, the user heap data structures are in common .bss */

#  define UMM_HEAP(i) g_mm_heap[i]
#endif

#ifdef CONFIG_MM_KERNEL_HEAP
/* Kernel has the dedicated heap data structures */

#  define KMM_HEAP(i) g_mm_heap[i]
#else
/* Otherwise, kernel allocate the memory from the user heap data structures */

#  define KMM_HEAP(i) UMM_HEAP(i)
#endif

#ifdef __KERNEL__
#  define MM_HEAP(i) KMM_HEAP(i)
#else
#  define MM_HEAP(i) UMM_HEAP(i)
#endif

#define PM_IDLE_DOMAIN                0

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN struct mm_heap_s *const g_mm_heap[];

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_CEVA_INCLUDE_ARCH_H */
