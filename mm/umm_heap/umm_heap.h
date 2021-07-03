/****************************************************************************
 * mm/umm_heap/umm_heap.h
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

#ifndef __MM_UMM_HEAP_UMM_HEAP_H
#define __MM_UMM_HEAP_UMM_HEAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/addrenv.h>
#include <nuttx/userspace.h>
#include <nuttx/mm/mm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ARCH_ADDRENV) && defined(__KERNEL__)
/* In the kernel build, there are multiple user heaps; one for each task
 * group.  In this build configuration, the user heap structure lies
 * in a reserved region at the beginning of the .bss/.data address
 * space (CONFIG_ARCH_DATA_VBASE).  The size of that region is given by
 * ARCH_DATA_RESERVE_SIZE
 */

#  define USR_HEAP (ARCH_DATA_RESERVE->ar_usrheap)

#elif defined(CONFIG_BUILD_PROTECTED) && defined(__KERNEL__)
/* In the protected mode, there are two heaps:  A kernel heap and a single
 * user heap.  Kernel code must obtain the address of the user heap data
 * structure from the userspace interface.
 */

#  define USR_HEAP (*USERSPACE->us_heap)

#else
/* Otherwise, the user heap data structures are in common .bss */

#  define USR_HEAP g_mmheap
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
void umm_try_initialize(void);
#endif

#endif /* __MM_UMM_HEAP_UMM_HEAP_H */
