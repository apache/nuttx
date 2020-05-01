/****************************************************************************
 * arch/arm/src/stm32h7/stm32_allocateheap.c
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
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/userspace.h>

#include <arch/stm32h7/chip.h>
#include <arch/board/board.h>

#include "mpu.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include "hardware/stm32_memorymap.h"
#include "stm32_mpuinit.h"
#include "stm32_dtcm.h"
#include "stm32_fmc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* At startup the kernel will invoke up_addregion() so that platform code
 * may register available memories for use as part of system heap.
 * The global configuration option CONFIG_MM_REGIONS defines the maximal
 * number of non-contiguous memory ranges that may be registered with the
 * system heap. You must make sure it is large enough to hold all memory
 * regions you intend to use.
 *
 * The following memory types can be used for heap on STM32H7 platform:
 *
 * - AXI SRAM is a 512kb memory area. This will be automatically registered
 *      with the system heap in up_allocate_heap, all the other memory
 *      regions will be registered in up_addregion().
 *      So, CONFIG_MM_REGIONS must be at least 1 to use AXI SRAM.
 *
 * - Internal SRAM is available in all members of the STM32 family.
 *      This is always registered with system heap.
 *      There are two contiguous regions of internal SRAM:
 *      SRAM1+SRAM2+SRAM3 and SRAM4 at a separate address.
 *      So, add 2 more to CONFIG_MM_REGIONS.
 *
 * - Tightly Coupled Memory (TCM RAM), we can use Data TCM (DTCM) for system
 *      heap. Note that DTCM has a number of limitations, for example DMA
 *      transfers to/from DTCM are limited.
 *      Define CONFIG_STM32H7_DTCMEXCLUDE to exclude the DTCM from heap.
 *      +1 to CONFIG_MM_REGIONS if you want to use DTCM.
 *
 * - External SDRAM can be connected to the FMC peripherial. Initialization
 *      of FMC is done as up_addregion() will invoke stm32_fmc_init().
 *      Please read the comment in stm32_fmc.c how to initialize FMC
 *      correctly.
 *
 *      Then, up to two regions of SDRAM may be registered with the heap:
 *
 *      - BOARD_SDRAM1_SIZE, if defined, declares the size of SDRAM
 *              at address STM32_FMC_BANK5. +1 to CONFIG_MM_REGIONS.
 *      - BOARD_SDRAM2_SIZE, if defined, declares the size of SDRAM
 *              at address STM32_FMC_BANK6. +1 to CONFIG_MM_REGIONS.
 *
 * - Additionaly, you may use the following options to add one more region
 *      of memory to system heap:
 *
 *      - CONFIG_ARCH_HAVE_HEAP2=y
 *      - CONFIG_HEAP2_BASE=base address of memory area.
 *      - CONFIG_HEAP2_SIZE=size of memory area.
 *      - +1 to CONFIG_MM_REGIONS
 */

/* Set the start and end of the SRAMs */

#define SRAM_START STM32_AXISRAM_BASE
#define SRAM_END   (SRAM_START + STM32H7_SRAM_SIZE)

#define SRAM123_START STM32_SRAM123_BASE
#define SRAM123_END   (SRAM123_START + STM32H7_SRAM123_SIZE)

#define SRAM4_START  STM32_SRAM4_BASE
#define SRAM4_END    (SRAM4_START + STM32H7_SRAM4_SIZE)

/* The STM32 H7 has DTCM memory */

#undef HAVE_DTCM
#define HAVE_DTCM 1
#if !defined(DTCM_START) || !defined(DTCM_END)
#  undef HAVE_DTCM
#endif

/* DTCM to be excluded from the main heap. */

#ifdef CONFIG_STM32H7_DTCMEXCLUDE
#  undef HAVE_DTCM
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_heap_color
 *
 * Description:
 *   Set heap memory to a known, non-zero state to checking heap usage.
 *
 ****************************************************************************/

#ifdef CONFIG_HEAP_COLORATION
static inline void up_heap_color(FAR void *start, size_t size)
{
  memset(start, HEAP_COLOR, size);
}
#else
#  define up_heap_color(start,size)
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
 *     Kernel .data region.       Size determined at link time.
 *     Kernel .bss  region        Size determined at link time.
 *     Kernel IDLE thread stack.  Size determined by
 *                                CONFIG_IDLETHREAD_STACKSIZE.
 *     Padding for alignment
 *     User .data region.         Size determined at link time.
 *     User .bss region           Size determined at link time.
 *     Kernel heap.               Size determined by
 *                                CONFIG_MM_KERNEL_HEAPSIZE.
 *     User heap.                 Extends to the end of SRAM.
 *
 ****************************************************************************/

void up_allocate_heap(FAR void **heap_start, size_t *heap_size)
{
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend +
    CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = SRAM123_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)SRAM123_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the SRAM123_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(usize);
  DEBUGASSERT((SRAM123_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = SRAM123_END - usize;

  /* Return the user-space heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void *)ubase;
  *heap_size  = usize;

  /* Colorize the heap for debug */

  up_heap_color((FAR void *)ubase, usize);

  /* Allow user-mode access to the user heap memory */

  stm32_mpu_uheap((uintptr_t)ubase, usize);
#else

  /* Return the heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void *)g_idle_topstack;
  *heap_size  = SRAM_END - g_idle_topstack;

  /* Colorize the heap for debug */

  up_heap_color(*heap_start, *heap_size);
#endif

  /* Display memory ranges to help debugging */

  minfo("%uKb of SRAM at %p\n", *heap_size / 1024, *heap_start);
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
void up_allocate_kheap(FAR void **heap_start, size_t *heap_size)
{
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend +
    CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = SRAM123_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)SRAM123_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the SRAM123_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(usize);
  DEBUGASSERT((SRAM123_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = SRAM123_END - usize;

  /* Return the kernel heap settings (i.e., the part of the heap region
   * that was not dedicated to the user heap).
   */

  *heap_start = (FAR void *)USERSPACE->us_bssend;
  *heap_size  = ubase - (uintptr_t)USERSPACE->us_bssend;
}
#endif

/****************************************************************************
 * Name: addregion
 *
 * Description:
 *   Make a range of memory available for allocation from system heap.
 *   If debug is disabled, compiler should optimize out the "desc" strings.
 *
 ****************************************************************************/

static void addregion (uintptr_t start, uint32_t size, const char *desc)
{
  /* Display memory ranges to help debugging */

  minfo("%uKb of %s at %p\n", size / 1024, desc, (FAR void *)start);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Allow user-mode access to the SRAM123 heap */

  stm32_mpu_uheap(start, size);

#endif

  /* Colorize the heap for debug */

  up_heap_color((FAR void *)start, size);

  /* Add the SRAM123 user heap region. */

  kumm_addregion((FAR void *)start, size);
}

/****************************************************************************
 * Name: up_addregion
 *
 * Description:
 *   Memory may be added in non-contiguous chunks.  Additional chunks are
 *   added by calling this function.
 *
 ****************************************************************************/

void up_addregion(void)
{
  addregion (SRAM123_START, SRAM123_END - SRAM123_START, "SRAM1,2,3");

  unsigned mm_regions = 1;

  if (mm_regions < CONFIG_MM_REGIONS)
    {
      addregion (SRAM4_START, SRAM4_END - SRAM4_START, "SRAM4");
      mm_regions++;
    }

#ifdef HAVE_DTCM
  if (mm_regions < CONFIG_MM_REGIONS)
    {
      addregion (DTCM_START, DTCM_END - DTCM_START, "DTCM");
      mm_regions++;
    }
#endif

#ifdef CONFIG_STM32H7_FMC
  stm32_fmc_init();
#endif

#ifdef BOARD_SDRAM1_SIZE
  if (mm_regions < CONFIG_MM_REGIONS)
    {
      addregion (STM32_FMC_BANK5, BOARD_SDRAM1_SIZE, "SDRAM1");
      mm_regions++;
    }
#endif

#ifdef BOARD_SDRAM2_SIZE
  if (mm_regions < CONFIG_MM_REGIONS)
    {
      addregion (STM32_FMC_BANK6, BOARD_SDRAM2_SIZE, "SDRAM2");
      mm_regions++;
    }
#endif

#ifdef CONFIG_ARCH_HAVE_HEAP2
  if (mm_regions < CONFIG_MM_REGIONS)
    {
      addregion (CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE, "HEAP2");
      mm_regions++;
    }
#endif
}
