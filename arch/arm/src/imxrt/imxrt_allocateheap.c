/****************************************************************************
 * arch/arm/src/imxrt/imxrt_allocateheap.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include <arch/imxrt/chip.h>

#include "up_arch.h"
#include "up_internal.h"

#include "chip/imxrt_memorymap.h"
#include "imxrt_mpuinit.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Terminology.  In the flat build (CONFIG_BUILD_FLAT=y), there is only a
 * single heap access with the standard allocations (malloc/free).  This
 * heap is referred to as the user heap.  In the protected build
 * (CONFIG_BUILD_PROTECTED=y) where an MPU is used to protect a region of
 * otherwise flat memory, there will be two allocators:  One that allocates
 * protected (kernel) memory and one that allocates unprotected (user)
 * memory.  These are referred to as the kernel and user heaps,
 * respectively.
 *
 * The ARMv7 has no MPU but does have an MMU.  Without an MMU, it cannot
 * support the kernel build (CONFIG_BUILD_KERNEL=y).  In that configuration,
 * there would is one kernel heap but multiple user heaps:  One per task
 * group.  However, in this case, we need only be concerned about
 * initializing the single kernel heap here.
 *
 * Primary RAM:  The Linker script positions the system BLOB's .data and
 * .bss in some RAM.  We refer to that RAM as the primary RAM.  It also
 * holds the IDLE threads stack and any remaining portion of the primary
 * RAM is automatically added to the heap.  The start and size of the
 * primary RAM are provided by CONFIG_RAM_START and CONFIG_RAM_SIZE.  The
 * linker provided address, ... .sbss, .ebss, .sdat, etc. ...  are expected
 * to lie in the the region defined by those configuration settings.
 *
 * Other RAM regions must be selected use configuration options and the
 * start and end of those RAM regions must also be provided in the
 * configuration.  CONFIG_MM_REGIONS must also be set to determined the
 * number of regions to be added to the heap.
 *
 * REVISIT:  The i.MX RT SEMC will support up to 8 512Mbit memory regions.
 * So it is possible that there could be multiple external SDRAM or SRAM
 * banks.  This logic assumes that there is at most one of each (or at least
 * only one contiguous block of addresses for each).  This would need to
 * be exceed considerably to support multiple SDRAM or SRAM memory regions.
 */

#define IMXRT_DTCM_BASE           0x20000000  /* 512KB DTCM */
                               /* 0x20080000     512KB DTCM Reserved */
                               /* 0x20100000     1MB Reserved */
#define IMXRT_OCRAM_BASE          0x20200000  /* 512KB OCRAM */

/* There there then several memory configurations with a one primary memory
 * region and up to two additional memory regions which may be OCRAM,
 * external SDRAM, or external SRAM.
 */

#undef IMXRT_OCRAM_ASSIGNED
#undef IMXRT_SDRAM_ASSIGNED
#undef IMXRT_SRAM_ASSIGNED

/* REVISIT: Assume that if OCRAM is the primary RAM, then DTCM and ITCM are
 * not being used.
 */

#if defined(CONFIG_IMXRT_OCRAM_PRIMARY)
#  define PRIMARY_RAM_START    IMXRT_OCRAM_BASE         /* CONFIG_RAM_START */
#  define PRIMARY_RAM_SIZE     IMXRT_OCRAM_SIZE         /* CONFIG_RAM_SIZE */
#  define IMXRT_OCRAM_ASSIGNED 1
#elif defined(CONFIG_IMXRT_SDRAM_PRIMARY)
#  define PRIMARY_RAM_START    CONFIG_IMXRT_SDRAM_START /* CONFIG_RAM_START */
#  define PRIMARY_RAM_SIZE     CONFIG_IMXRT_SDRAM_SIZE  /* CONFIG_RAM_SIZE */
#  define IMXRT_SDRAM_ASSIGNED 1
#elif defined(CONFIG_IMXRT_SRAM_PRIMARY)
#  define PRIMARY_RAM_START    CONFIG_IMXRT_SRAM_START /* CONFIG_RAM_START */
#  define PRIMARY_RAM_SIZE     CONFIG_IMXRT_SRAM_SIZE  /* CONFIG_RAM_SIZE */
#  define IMXRT_SRAM_ASSIGNED  1
#else
#  error No primary RAM defined
#endif

#define PRIMARY_RAM_END        (PRIMARY_RAM_START + PRIMARY_RAM_SIZE)

/* REVISIT:  I am not sure how this works.  But I am assuming that if DTCM
 * is enabled, then ITCM is not and we can just use the DTCM base address to
 * access OCRAM.
 */

#ifdef CONFIG_IMXRT_DTCM
#  define IMXRT_OCRAM_START    IMXRT_DTCM_BASE
#else
#  define IMXRT_OCRAM_START    IMXRT_OCRAM_BASE
#endif

#if CONFIG_MM_REGIONS > 1
/* Pick the first region to add to the heap could be any one of OCRAM,
 * SDRAM, or SRAM depending upon which are enabled and which has not
 * already been assigned as the primary RAM.
 */

#if defined(CONFIG_IMXRT_OCRAM_HEAP) && !defined(IMXRT_OCRAM_ASSIGNED)
#  define REGION1_RAM_START    IMXRT_OCRAM_START
#  define REGION1_RAM_SIZE     IMXRT_OCRAM_SIZE
#  define IMXRT_OCRAM_ASSIGNED 1
#elif defined(CONFIG_IMXRT_SDRAM_HEAP) && !defined(IMXRT_SDRAM_ASSIGNED)
#  define REGION1_RAM_START    (CONFIG_IMXRT_SDRAM_START + CONFIG_IMXRT_SDRAM_HEAPOFFSET)
#  define REGION1_RAM_SIZE     (CONFIG_IMXRT_SDRAM_SIZE  - CONFIG_IMXRT_SDRAM_HEAPOFFSET)
#  define IMXRT_SDRAM_ASSIGNED 1
#elif defined(CONFIG_IMXRT_SDRAM_HEAP) && !defined(IMXRT_SDRAM_ASSIGNED)
#  define REGION1_RAM_START    (CONFIG_IMXRT_SRAM_START  + CONFIG_IMXRT_SRAM_HEAPOFFSET)
#  define REGION1_RAM_SIZE     (CONFIG_IMXRT_SRAM_SIZE   - CONFIG_IMXRT_SRAM_HEAPOFFSET)
#  define IMXRT_SDRAM_ASSIGNED 1
#else
#  warning CONFIG_MM_REGIONS > 1 but no available memory region
#endif

#define REGION1_RAM_END        (REGION1_RAM_START + REGION1_RAM_SIZE)
#endif

#if CONFIG_MM_REGIONS > 2
/* Pick the first region to add to the heap could be any one of OCRAM,
 * SDRAM, or SRAM depending upon which are enabled and which has not
 * already been assigned as the primary RAM.
 */

#if defined(CONFIG_IMXRT_OCRAM_HEAP) && !defined(IMXRT_OCRAM_ASSIGNED)
#  define REGION2_RAM_START    IMXRT_OCRAM_START
#  define REGION2_RAM_SIZE     IMXRT_OCRAM_SIZE
#  define IMXRT_OCRAM_ASSIGNED 1
#elif defined(CONFIG_IMXRT_SDRAM_HEAP) && !defined(IMXRT_SDRAM_ASSIGNED)
#  define REGION2_RAM_START    (CONFIG_IMXRT_SDRAM_START + CONFIG_IMXRT_SDRAM_HEAPOFFSET)
#  define REGION2_RAM_SIZE     (CONFIG_IMXRT_SDRAM_SIZE  - CONFIG_IMXRT_SDRAM_HEAPOFFSET)
#  define IMXRT_SDRAM_ASSIGNED 1
#elif defined(CONFIG_IMXRT_SDRAM_HEAP) && !defined(IMXRT_SDRAM_ASSIGNED)
#  define REGION2_RAM_START    (CONFIG_IMXRT_SRAM_START  + CONFIG_IMXRT_SRAM_HEAPOFFSET)
#  define REGION2_RAM_SIZE     (CONFIG_IMXRT_SRAM_SIZE   - CONFIG_IMXRT_SRAM_HEAPOFFSET)
#  define IMXRT_SDRAM_ASSIGNED 1
#else
#  warning CONFIG_MM_REGIONS > 2 but no available memory region
#endif

#define REGION2_RAM_END        (REGION2_RAM_START + REGION2_RAM_SIZE)
#endif

#if CONFIG_MM_REGIONS > 3
#  warning CONFIG_MM_REGIONS > 3 but no available memory region
#endif

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

const uintptr_t g_idle_topstack = (uintptr_t)&_ebss + CONFIG_IDLETHREAD_STACKSIZE;

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
 *     Kernel IDLE thread stack.  Size determined by CONFIG_IDLETHREAD_STACKSIZE.
 *     Padding for alignment
 *     User .data region.  Size determined at link time.
 *     User .bss region  Size determined at link time.
 *     Kernel heap.  Size determined by CONFIG_MM_KERNEL_HEAPSIZE.
 *     User heap.  Extends to the end of SRAM.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
void up_allocate_kheap(FAR void **heap_start, size_t *heap_size)
#else
void up_allocate_heap(FAR void **heap_start, size_t *heap_size)
#endif
{
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend + CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = PRIMARY_RAM_END - ubase;

  DEBUGASSERT(ubase < (uintptr_t)PRIMARY_RAM_END);

  /* Return the user-space heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void *)ubase;
  *heap_size  = usize;
#else

  /* Return the heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void *)g_idle_topstack;
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
void up_allocate_kheap(FAR void **heap_start, size_t *heap_size)
{
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend + CONFIG_MM_KERNEL_HEAPSIZE;
  DEBUGASSERT(ubase < (uintptr_t)PRIMARY_RAM_END);

  /* Return the kernel heap settings (i.e., the part of the heap region
   * that was not dedicated to the user heap).
   */

  *heap_start = (FAR void *)USERSPACE->us_bssend;
  *heap_size  = ubase - (uintptr_t)USERSPACE->us_bssend;
}
#endif

/****************************************************************************
 * Name: up_addregion
 *
 * Description:
 *   Memory may be added in non-contiguous chunks.  Additional chunks are
 *   added by calling this function.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void up_addregion(void)
{
  /* Add region 1 to the user heap */

  kumm_addregion((FAR void *)REGION1_RAM_START, REGION1_RAM_SIZE);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to region 1 */

  imxrt_mpu_uheap((uintptr_t)REGION1_RAM_START, REGION1_RAM_SIZE);
#endif

#if CONFIG_MM_REGIONS > 2
  /* Add region 2 to the user heap */

  kumm_addregion((FAR void *)REGION2_RAM_START, REGION2_RAM_SIZE);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to region 2 */

  imxrt_mpu_uheap((uintptr_t)REGION2_RAM_START, REGION2_RAM_SIZE);
#endif
#endif /* CONFIG_MM_REGIONS > 2 */
}
#endif /* CONFIG_MM_REGIONS > 1 */
