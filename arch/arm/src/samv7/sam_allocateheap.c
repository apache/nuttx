/****************************************************************************
 * arch/arm/src/samv7/sam_allocateheap.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>

#include <arch/samv7/chip.h>
#include <arch/board/board.h>

#include "mpu.h"
#include "arm_internal.h"
#include "sam_mpuinit.h"
#include "sam_periphclks.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* All SAM's have SRAM.  In addition, they may have external SRAM or SDRAM */

#define HAVE_SDRAM_REGION     0 /* Assume no external SDRAM */
#define HAVE_EXTSRAM0_REGION  0 /* Assume no external SRAM at CS0 */
#define HAVE_EXTSRAM1_REGION  0 /* Assume no external SRAM at CS1 */
#define HAVE_EXTSRAM2_REGION  0 /* Assume no external SRAM at CS2 */
#define HAVE_EXTSRAM3_REGION  0 /* Assume no external SRAM at CS3 */

/* Check if external SDRAM is supported and, if so, it is is intended
 * to be used as heap.
 */

#if !defined(CONFIG_SAMV7_SDRAMC) || !defined(CONFIG_SAMV7_SDRAMHEAP)
#  undef  CONFIG_SAMV7_SDRAMSIZE
#  define CONFIG_SAMV7_SDRAMSIZE 0
#endif

/* Check if external SRAM is supported and, if so, is it intended to be used
 * as heap memory?
 */

#if !defined(CONFIG_SAMV7_EXTSRAM0) || !defined(CONFIG_SAMV7_EXTSRAM0HEAP)
#  undef  CONFIG_SAMV7_EXTSRAM0SIZE
#  define CONFIG_SAMV7_EXTSRAM0SIZE 0
#endif

#if !defined(CONFIG_SAMV7_EXTSRAM1) || !defined(CONFIG_SAMV7_EXTSRAM1HEAP)
#  undef  CONFIG_SAMV7_EXTSRAM1SIZE
#  define CONFIG_SAMV7_EXTSRAM1SIZE 0
#endif

#if !defined(CONFIG_SAMV7_EXTSRAM2) || !defined(CONFIG_SAMV7_EXTSRAM2HEAP)
#  undef  CONFIG_SAMV7_EXTSRAM2SIZE
#  define CONFIG_SAMV7_EXTSRAM2SIZE 0
#endif

#if !defined(CONFIG_SAMV7_EXTSRAM3) || !defined(CONFIG_SAMV7_EXTSRAM3HEAP)
#  undef  CONFIG_SAMV7_EXTSRAM3SIZE
#  define CONFIG_SAMV7_EXTSRAM3SIZE 0
#endif

/* Now lets reconcile the number of configured regions with the available
 * memory resource configured for use as a heap region.
 */

#if CONFIG_SAMV7_SDRAMSIZE > 0
#  if CONFIG_MM_REGIONS > 1
#     undef  HAVE_SDRAM_REGION
#     define HAVE_SDRAM_REGION 1
#  else
#    warning "CONFIG_MM_REGIONS < 2: SDRAM not included in HEAP"
#  endif
#endif

#if CONFIG_SAMV7_EXTSRAM0SIZE > 0
#  if CONFIG_MM_REGIONS > (HAVE_SDRAM_REGION + 1)
#     undef  HAVE_EXTSRAM0_REGION
#     define HAVE_EXTSRAM0_REGION 1
#  else
#    warning "CONFIG_MM_REGIONS too small: External SRAM0 not included in HEAP"
#  endif
#endif

#if CONFIG_SAMV7_EXTSRAM1SIZE > 0
#  if CONFIG_MM_REGIONS > (HAVE_SDRAM_REGION + HAVE_EXTSRAM0_REGION + 1)
#     undef  HAVE_EXTSRAM1_REGION
#     define HAVE_EXTSRAM1_REGION 1
#  else
#    warning "CONFIG_MM_REGIONS too small: External SRAM1 not included in HEAP"
#  endif
#endif

#if CONFIG_SAMV7_EXTSRAM2SIZE > 0
#  if CONFIG_MM_REGIONS > (HAVE_SDRAM_REGION + HAVE_EXTSRAM0_REGION + HAVE_EXTSRAM1_REGION + 1)
#     undef  HAVE_EXTSRAM2_REGION
#     define HAVE_EXTSRAM2_REGION 1
#  else
#    warning "CONFIG_MM_REGIONS too small: External SRAM2 not included in HEAP"
#  endif
#endif

#if CONFIG_SAMV7_EXTSRAM3SIZE > 0
#  if CONFIG_MM_REGIONS > (HAVE_SDRAM_REGION + HAVE_EXTSRAM0_REGION + HAVE_EXTSRAM1_REGION + HAVE_EXTSRAM2_REGION + 1)
#     undef  HAVE_EXTSRAM3_REGION
#     define HAVE_EXTSRAM3_REGION 1
#  else
#    warning "CONFIG_MM_REGIONS too small: External SRAM3 not included in HEAP"
#  endif
#endif

/* Check internal SRAM configuration */

#ifdef CONFIG_ARMV7M_DTCM
#  define SRAM_BASE SAM_DTCM_BASE
#else
#  define SRAM_BASE SAM_SRAM_BASE
#endif

#if CONFIG_RAM_END > (SRAM_BASE+SAMV7_SRAM_SIZE)
#  error "CONFIG_RAM_END is beyond the end of SRAM"
#  undef CONFIG_RAM_END
#  define CONFIG_RAM_END (SRAM_BASE+SAMV7_SRAM_SIZE)
#elif CONFIG_RAM_END < (SRAM_BASE+SAMV7_SRAM_SIZE)
#  warning "CONFIG_RAM_END is before end of SRAM... not all of SRAM used"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   For the kernel build (CONFIG_BUILD_PROTECTED=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function provides the
 *   size of the unprotected, user-space heap.
 *
 *   If a protected kernel-space heap is provided, the kernel heap must be
 *   allocated (and protected) by an analogous up_allocate_kheap().
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
 *     Kernel IDLE thread stack.  Size determined by
 *                     CONFIG_IDLETHREAD_STACKSIZE.
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
  size_t    usize = CONFIG_RAM_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)CONFIG_RAM_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the CONFIG_RAM_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(usize);
  DEBUGASSERT((CONFIG_RAM_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = CONFIG_RAM_END - usize;

  /* Return the user-space heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (void *)ubase;
  *heap_size  = usize;

  /* Allow user-mode access to the user heap memory */

  sam_mpu_uheap((uintptr_t)ubase, usize);
#else

  /* Return the heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (void *)g_idle_topstack;
  *heap_size  = CONFIG_RAM_END - g_idle_topstack;
#endif
}

/****************************************************************************
 * Name: up_allocate_kheap
 *
 * Description:
 *   For the kernel build (CONFIG_BUILD_PROTECTED=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function allocates
 *   (and protects) the kernel-space heap.
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
  size_t    usize = CONFIG_RAM_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)CONFIG_RAM_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the CONFIG_RAM_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(usize);
  DEBUGASSERT((CONFIG_RAM_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = CONFIG_RAM_END - usize;

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
#if HAVE_SDRAM_REGION != 0
  /* Allow user access to the heap memory */

  sam_mpu_uheap(SAM_SDRAMCS_BASE, CONFIG_SAMV7_SDRAMSIZE);

  /* Add the region */

  kumm_addregion((void *)SAM_SDRAMCS_BASE, CONFIG_SAMV7_SDRAMSIZE);

#endif /* HAVE_SDRAM_REGION */

#if HAVE_EXTSRAM0_REGION != 0
  /* Allow user access to the heap memory */

  sam_mpu_uheap(SAM_EXTCS0_BASE, CONFIG_SAMV7_EXTSRAM0SIZE);

  /* Add the region */

  kumm_addregion((void *)SAM_EXTCS0_BASE, CONFIG_SAMV7_EXTSRAM0SIZE);

#endif /* HAVE_EXTSRAM0_REGION */

#if HAVE_EXTSRAM1_REGION != 0
  /* Allow user access to the heap memory */

  sam_mpu_uheap(SAM_EXTCS1_BASE, CONFIG_SAMV7_EXTSRAM1SIZE);

  /* Add the region */

  kumm_addregion((void *)SAM_EXTCS1_BASE, CONFIG_SAMV7_EXTSRAM1SIZE);

#endif /* HAVE_EXTSRAM0_REGION */

#if HAVE_EXTSRAM2_REGION != 0
  /* Allow user access to the heap memory */

  sam_mpu_uheap(SAM_EXTCS2_BASE, CONFIG_SAMV7_EXTSRAM2SIZE);

  /* Add the region */

  kumm_addregion((void *)SAM_EXTCS2_BASE, CONFIG_SAMV7_EXTSRAM2SIZE);

#endif /* HAVE_EXTSRAM0_REGION */

#if HAVE_EXTSRAM3_REGION != 0
  /* Allow user access to the heap memory */

  sam_mpu_uheap(SAM_EXTCS3_BASE, CONFIG_SAMV7_EXTSRAM3SIZE);

  /* Add the region */

  kumm_addregion((void *)SAM_EXTCS3_BASE, CONFIG_SAMV7_EXTSRAM3SIZE);

#endif /* HAVE_EXTSRAM0_REGION */
}
#endif /* CONFIG_MM_REGIONS > 1 */
