/****************************************************************************
 * arch/arm/src/mps/mps_allocateheap.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/userspace.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "mpu.h"
#include "chip.h"
#include "hardware/mps_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Terminology.
 * In the flat build (CONFIG_BUILD_FLAT=y), there is only a
 * single heap access with the standard allocations (malloc/free).  This
 * heap is referred to as the user heap.
 * In the protected build (CONFIG_BUILD_PROTECTED=y) where an MPU is
 * used to protect a region of otherwise flat memory, there will
 * be two allocators:  One that allocates protected (kernel) memory and
 * one that allocates unprotected (user) memory. These are referred to
 * as the kernel and user heaps, respectively.
 *
 * For processor(armv7-a/armv8-a) which has no MPU but does have an MMU.
 * Without an MMU, it cannot support the kernel
 * build (CONFIG_BUILD_KERNEL=y). In that configuration,
 * there would is one kernel heap but multiple user heaps: One per
 * task group. However, in this case, we need only be concerned
 * about initializing the single kernel heap here.
 *
 * Primary RAM:  The Linker script positions the system BLOB's .data and
 * .bss in some RAM.  We refer to that RAM as the primary RAM.  It also
 * holds the IDLE threads stack and any remaining portion of the primary
 * OCRAM is automatically added to the heap.  The linker provided address,
 * ... .sbss, .ebss, .sdat, etc. ...  are expected to lie in the the region
 * defined by the OCRAM configuration settings.
 *
 * Other RAM regions must be selected use configuration options and the
 * start and end of those RAM regions must also be provided in the
 * configuration.  CONFIG_MM_REGIONS must also be set to determined the
 * number of regions to be added to the heap.
 */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* _sbss is the start of the BSS region (see the linker script) _ebss is the
 * end of the BSS regions (see the linker script). The idle task stack starts
 * at the end of BSS and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE
 * thread is the thread that the system boots on and, eventually, becomes the
 * idle, do nothing task that runs only when there is nothing else to run.
 * The heap continues from there until the configured end of memory.
 * g_idle_topstack is the beginning of this heap region (not necessarily
 * aligned).
 */

const uintptr_t g_idle_topstack = (uintptr_t)_ebss +
                                  CONFIG_IDLETHREAD_STACKSIZE;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap/up_allocate_kheap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   - For the normal "flat" build, this function returns the size of the
 *     single heap.
 *   - For the protected build (CONFIG_BUILD_PROTECTED=y) with both kernel-
 *     and user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function
 *     provides the size of the unprotected, user-space heap.
 *   - For the kernel build (CONFIG_BUILD_KERNEL=y), this function provides
 *     the size of the protected, kernel-space heap.
 *
 *   If a protected kernel-space heap is provided, the kernel heap must be
 *   allocated by an analogous up_allocate_kheap(). A custom version of this
 *   file is needed if memory protection of the kernel heap is required.
 *
 *   The following memory map is assumed for the flat build:
 *
 *     .data region.  Size determined at link time.
 *     .bss  region  Size determined at link time.
 *     IDLE thread stack.  Size determined by CONFIG_IDLETHREAD_STACKSIZE.
 *     Heap.  Extends to the end of SRAM.
 *
 *   The following memory map is assumed for the kernel build:
 *
 *     Kernel .data region.  Size determined at link time.
 *     Kernel .bss  region  Size determined at link time.
 *     Kernel IDLE thread stack. (size determined by
 *     CONFIG_IDLETHREAD_STACKSIZE).
 *     Padding for alignment
 *     User .data region.  Size determined at link time.
 *     User .bss region  Size determined at link time.
 *     Kernel heap.  Size determined by CONFIG_MM_KERNEL_HEAPSIZE.
 *     User heap.  Extends to the end of SRAM.
 *
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend +
                     CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = PRIMARY_RAM_END - ubase;
  uint8_t   log2;

  DEBUGASSERT(ubase < PRIMARY_RAM_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the PRIMARY_RAM_END
   * is aligned to the MPU requirement.
   */

  log2  = mpu_log2regionfloor(usize);
  DEBUGASSERT((PRIMARY_RAM_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = PRIMARY_RAM_END - usize;

  /* Return the user-space heap settings */

  *heap_start = (void *)ubase;
  *heap_size  = usize;

  /* Allow user-mode access to the user heap memory */

  mpu_user_intsram(ubase, usize);
#else

  /* Return the heap settings */

  *heap_start = (void *)g_idle_topstack;
  *heap_size  = PRIMARY_RAM_END - g_idle_topstack;
#endif
}

/****************************************************************************
 * Name: up_allocate_kheap
 *
 * Description:
 *   For the kernel build (CONFIG_BUILD_PROTECTED/KERNEL=y) with both kernel-
 *   and user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function allocates
 *   the kernel-space heap.  A custom version of this function is needed if
 *   memory protection of the kernel heap is required.
 *
 ****************************************************************************/

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
void up_allocate_kheap(void **heap_start, size_t *heap_size)
{
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend +
                    CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = PRIMARY_RAM_END - ubase;
  uint8_t   log2;
  DEBUGASSERT(ubase < PRIMARY_RAM_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the CONFIG_RAM_END
   * is aligned to the MPU requirement.
   */

  log2  = mpu_log2regionfloor(usize);
  DEBUGASSERT((PRIMARY_RAM_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = PRIMARY_RAM_END - usize;

  /* Return the kernel heap settings (i.e., the part of the heap region
   * that was not dedicated to the user heap).
   */

  *heap_start = (void *)USERSPACE->us_bssend;
  *heap_size  = ubase - (uintptr_t)USERSPACE->us_bssend;
}
#endif

/****************************************************************************
 * Name: arm_addregion
 *
 * Description:
 *   Memory may be added in non-contiguous chunks.  Additional chunks are
 *   added by calling this function.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void arm_addregion(void)
{
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to region 1 */

  mpu_user_intsram(REGION1_RAM_START, REGION1_RAM_SIZE);
#endif

  /* Add region 1 to the user heap */

  kumm_addregion((void *)REGION1_RAM_START, REGION1_RAM_SIZE);

#if CONFIG_MM_REGIONS > 2

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to region 2 */

  mpu_user_intsram(REGION2_RAM_START, REGION2_RAM_SIZE);
#endif
  /* Add region 2 to the user heap */

  kumm_addregion((void *)REGION2_RAM_START, REGION2_RAM_SIZE);

#endif /* CONFIG_MM_REGIONS > 2 */
}
#endif /* CONFIG_MM_REGIONS > 1 */
