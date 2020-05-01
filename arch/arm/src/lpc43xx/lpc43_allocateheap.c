/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_allocateheap.c
 *
 *   Copyright (C) 2012-2013, 2015-2017 Gregory Nutt. All rights reserved.
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

#include <arch/board/board.h>

#include "mpu.h"
#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include "lpc43_mpuinit.h"
#include "lpc43_emacram.h"
#include "lpc43_usbram.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get customizations for each supported chip.
 *
 * SRAM Resources
 * --------------------- -------- ------- ------- ------- ------- -------
 * Local SRAM            LPC4310  LPC4320 LPC4330 LPC4350 LPC4353 LPC4357
 * --------------------- -------- ------- ------- ------- ------- -------
 * BANK 0 (0x1000 0000)     96Kb    96Kb   128Kb   128Kb    32Kb    32Kb
 * BANK 1 (0x1008 0000)     40Kb    40Kb    72Kb    72Kb    40Kb    40Kb
 * --------------------- -------- ------- ------- ------- ------- -------
 * SUBTOTAL                136Kb   136Kb   200Kb   200Kb    72Kb    72Kb
 * --------------------- -------- ------- ------- ------- ------- -------
 * AHB SRAM              LPC4310  LPC4320 LPC4330 LPC4350 LPC4353 LPC4357
 * --------------------- -------- ------- ------- ------- ------- -------
 * BANK 0 (0x2000 0000)     16Kb    48Kb   48Kb    48Kb     48Kb    48Kb
 * BANK 1 (0x2000 8000)             NOTE 1 NOTE 1  NOTE 1  NOTE 1  NOTE 1
 * BANK 2 (0x2000 c000)     16Kb    16Kb   16Kb    16Kb    16Kb    16Kb
 * --------------------- -------- ------- ------- ------- ------- -------
 * SUBTOTAL                 32Kb    64Kb   64Kb    64Kb     64Kb    64Kb
 * --------------------- -------- ------- ------- ------- ------- -------
 * TOTAL                   168Kb   200Kb  264Kb   264Kb    136Kb   136Kb
 * --------------------- -------- ------- ------- ------- ------- -------
 *
 * --------------------- -------- ------- ------- ------- ------- -------
 * FLASH                 LPC4310  LPC4320 LPC4330 LPC4350 LPC4353 LPC4357
 * --------------------- -------- ------- ------- ------- ------- -------
 * BANK A (0x1a00 0000)                                    256Kb   512Kb
 * BANK B (0x1b00 8000)                                    256Kb   512Kb
 * --------------------- -------- ------- ------- ------- ------- -------
 * TOTAL                   None    None    None    None    512Kb  1024Kb
 * --------------------- -------- ------- ------- ------- ------- -------
 *
 * NOTE 1: The 64Kb of AHB of SRAM on the LPC4350/30/20 span all AHB SRAM
 * banks but are treated as two banks of 48 an 16Kb by the NuttX memory
 * manager.  This gives some symmetry to all of the members of the family.
 *
 * ----------------------------------------------------------------------
 * EMC SDRAM
 * ----------------------------------------------------------------------
 * LPC43xx may have dynamic RAM connected on EMC bus. Up to 4 chips can be
 * connected.
 *
 * DYCS0 (0x2800 0000) up to 128MB
 * DYCS1 (0x3000 0000) up to 256MB
 * DYCS2 (0x6000 0000) up to 256MB
 * DYCS3 (0x7000 0000) up to 256MB
 *
 * LPC43xx may have static RAM connected on EMC bus.
 *
 * CS0 (0x1C00 0000) up to 16MB
 * CS1 (0x1D00 0000) up to 16MB
 * CS2 (0x1E00 0000) up to 16MB
 * CS3 (0x1F00 0000) up to 16MB
 *
 */

/* Configuration ************************************************************/

/* Two configurations are supported:
 *
 * Configuration A:
 *   Program memory     = FLASH
 *   Data memory        = Local RAM Bank 0
 *   Additional regions = Local RAM Bank 1 + AHB SRAM (excluding DMA buffers)
 *
 * Configuration B:
 *   Program memory     = Local RAM Bank 0
 *   Data memory        = Local RAM Bank 1
 *   Additional regions = AHB SRAM (excluding DMA buffers)
 *
 * This file supports only memory configuration A.
 *
 * These should be defined in the memory map header file:
 *
 *    LPC43_LOCSRAM_BANK0_BASE   0x10000000
 *    LPC43_LOCSRAM_BANK1_BASE   0x10080000
 *    LPC43_AHBSRAM_BANK0_BASE   0x20000000
 *    LPC43_AHBSRAM_BANK1_BASE   0x20008000
 *    LPC43_AHBSRAM_BANK2_BASE   0x2000c000
 *
 * These should be defined for the specific chip in the chip.h header file.
 * The value will be defined to be zero in size of the bank does not exist.
 * If two banks are contiguous, the combined size will be added to the
 * first bank and the size of the second bank will be defined to be zero.
 *
 *    LPC43_LOCSRAM_BANK0_SIZE
 *    LPC43_LOCSRAM_BANK1_SIZE
 *    LPC43_AHBSRAM_BANK0_SIZE
 *    LPC43_AHBSRAM_BANK1_SIZE
 *    LPC43_AHBSRAM_BANK2_SIZE
 *
 * The config.h file will define only:
 *
 *    CONFIG_RAM_START = The start of the data RAM region which may be
 *      either local SRAM bank 0 (Configuration A) or 1 (Configuration B).
 *    CONFIG_RAM_SIZE  = The size of the data RAM region.
 *    CONFIG_RAM_END   = The sum of the above.
 */

/* External Memory Configuration
 *
 * Dynamic memory configuration
 *   For dynamic memory configuration at least one of LPC43_EXTSDRAMx
 *   should by defined.
 * Also, together with LPC43_EXTSDRAMx should be defined:
 *   LPC43_EXTSDRAMxSIZE = External RAM size in bytes.
 *   LPC43_EXTSDRAMxHEAP = Should this RAM be use as heap space?
 */

/* Check for Configuration A. */

#undef MM_USE_LOCSRAM_BANK0
#undef MM_USE_LOCSRAM_BANK1
#undef MM_USE_AHBSRAM_BANK0
#undef MM_USE_AHBSRAM_BANK1
#undef MM_USE_AHBSRAM_BANK2
#undef MM_USE_EXTSDRAM0
#undef MM_USE_EXTSDRAM1
#undef MM_USE_EXTSDRAM2
#undef MM_USE_EXTSDRAM3
#undef MM_HAVE_REGION

#ifndef CONFIG_LPC43_BOOT_SRAM

/* Configuration A */
/* CONFIG_RAM_START should be set to the base of local SRAM, Bank 0. */

#  if CONFIG_RAM_START != LPC43_LOCSRAM_BANK0_BASE
#    error "CONFIG_RAM_START must be set to the base address of RAM bank 0"
#  endif

/* The configured RAM size should be equal to the size of local SRAM Bank 0. */

#  if CONFIG_RAM_SIZE != LPC43_LOCSRAM_BANK0_SIZE
#    error "CONFIG_RAM_SIZE must be set to size of local SRAM Bank 0"
#  endif

/* Local SRAM Bank 0 will be used as main memory region */

#  define MM_USE_LOCSRAM_BANK0 0

/* Use local SRAM Bank 1 if configured */

#  ifdef CONFIG_LPC43_USE_LOCSRAM_BANK1
#    define MM_USE_LOCSRAM_BANK1 1
#  endif

#else /* CONFIG_LPC43_BOOT_SRAM */

/* Configuration B */
/* CONFIG_RAM_START should be set to the base of local SRAM, Bank 1. */

#  if CONFIG_RAM_START != LPC43_LOCSRAM_BANK1_BASE
#    error "CONFIG_RAM_START must be set to the base address of RAM bank 1"
#  endif

/* The configured RAM size should be equal to the size of local SRAM Bank 1. */

#  if CONFIG_RAM_SIZE != LPC43_LOCSRAM_BANK1_SIZE
#    error "CONFIG_RAM_SIZE must be set to size of local SRAM Bank 1"
#  endif

/* Shouldn't use Local SRAM Bank 0 as system use it for code.
 * Local SRAM Bank1 is used as main memory region.
 */

#  define MM_USE_LOCSRAM_BANK1 0

#endif /* CONFIG_LPC43_BOOT_SRAM */

/* Configure other memory banks */

#ifdef CONFIG_LPC43_AHBSRAM_BANK0
#  define MM_USE_AHBSRAM_BANK0 1
#endif

#ifdef CONFIG_LPC43_AHBSRAM_BANK1
#  define MM_USE_AHBSRAM_BANK1 1
#endif

#define MM_DMAREGION_BASE  LPC43_AHBSRAM_BANK2_BASE
#define MM_DMAREGION_SIZE  LPC43_AHBSRAM_BANK2_SIZE

/* Figure out how much heap we have in the DMA region that is not being
 * used by USB and/or Ethernet (if any).
 */

#warning "Missing Logic"

#ifdef CONFIG_LPC43_AHBSRAM_BANK2
#  define MM_USE_AHBSRAM_BANK2 1
#  define MM_DMAHEAP_BASE MM_DMAREGION_BASE /* For now... use it all */
#  define MM_DMAHEAP_SIZE MM_DMAREGION_SIZE
#endif

/* External RAM configuration */

/* Check if external SDRAM is supported and, if so, it is intended to be used
 * used as heap.
 */

#if !defined(CONFIG_LPC43_EXTSDRAM0) || !defined(CONFIG_LPC43_EXTSDRAM0_HEAP)
#  undef CONFIG_LPC43_EXTSDRAM0_SIZE
#  define CONFIG_LPC43_EXTSDRAM0_SIZE 0
#endif

#if !defined(CONFIG_LPC43_EXTSDRAM1) || !defined(CONFIG_LPC43_EXTSDRAM1_HEAP)
#  undef CONFIG_LPC43_EXTSDRAM1_SIZE
#  define CONFIG_LPC43_EXTSDRAM1_SIZE 0
#endif

#if !defined(CONFIG_LPC43_EXTSDRAM2) || !defined(CONFIG_LPC43_EXTSDRAM2_HEAP)
#  undef CONFIG_LPC43_EXTSDRAM2_SIZE
#  define CONFIG_LPC43_EXTSDRAM2_SIZE 0
#endif

#if !defined(CONFIG_LPC43_EXTSDRAM3) || !defined(CONFIG_LPC43_EXTSDRAM3_HEAP)
#  undef CONFIG_LPC43_EXTSDRAM3_SIZE
#  define CONFIG_LPC43_EXTSDRAM3_SIZE 0
#endif

#if CONFIG_LPC43_EXTSDRAM0_SIZE > 0
#  define MM_USE_EXTSDRAM0 1
#  define MM_EXTSDRAM0_REGION LPC43_DYCS0_BASE
#  define MM_EXTSDRAM0_SIZE   CONFIG_LPC43_EXTSDRAM0_SIZE
#endif /* CONFIG_LPC43_EXTSDRAM0_SIZE */

#if CONFIG_LPC43_EXTSDRAM1_SIZE > 0
#  define MM_USE_EXTSDRAM1 1
#  define MM_EXTSDRAM1_REGION LPC43_DYCS1_BASE
#  define MM_EXTSDRAM1_SIZE   CONFIG_LPC43_EXTSDRAM1_SIZE
#endif /* CONFIG_LPC43_EXTSDRAM1_SIZE */

#if CONFIG_LPC43_EXTSDRAM2_SIZE > 0
#  define MM_USE_EXTSDRAM2 1
#  define MM_EXTSDRAM2_REGION LPC43_DYCS2_BASE
#  define MM_EXTSDRAM2_SIZE   CONFIG_LPC43_EXTSDRAM2_SIZE
#endif /* CONFIG_LPC43_EXTSDRAM1_SIZE */

#if CONFIG_LPC43_EXTSDRAM3_SIZE > 0
#  define HAVE_EXTSDRAM3_REGION 1
#  define MM_EXTSDRAM3_REGION LPC43_DYCS3_BASE
#  define MM_EXTSDRAM3_SIZE   CONFIG_LPC43_EXTSDRAM3_SIZE
#endif /* CONFIG_LPC43_EXTSDRAM3_SIZE */

#if CONFIG_MM_REGIONS > 1 && \
    (defined(MM_USE_LOCSRAM_BANK1) || defined(MM_USE_AHBSRAM_BANK0) || \
     defined(MM_USE_AHBSRAM_BANK1) || defined(MM_USE_AHBSRAM_BANK2) || \
     defined(MM_USE_EXTSDRAM0)     || defined(MM_USE_EXTSDRAM1) || \
     defined(MM_USE_EXTSDRAM2)     || defined(MM_USE_EXTSDRAM3))
#  define MM_HAVE_REGION 1
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* _sbss is the start of the BSS region (see the linker script) _ebss is the
 * end of the BSS region (see the linker script). The idle task stack starts
 * at the end of BSS and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE
 * thread is the thread that the system boots on and, eventually, becomes the
 * idle, do nothing task that runs only when there is nothing else to run.
 * The heap continues from there until the configured end of memory.
 * g_idle_topstack is the beginning of this heap region (not necessarily
 * aligned).
 */

const uint32_t g_idle_topstack = (uint32_t)&_ebss + CONFIG_IDLETHREAD_STACKSIZE;

#ifdef MM_HAVE_REGION
static uint8_t g_mem_region_next = 0;
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
 * Name: mem_addregion
 *
 * Description:
 *   Add one memory region to the user heap
 *
 ****************************************************************************/

#ifdef MM_HAVE_REGION
static void mem_addregion(FAR void *region_start, size_t region_size)
{
  if (g_mem_region_next <= CONFIG_MM_REGIONS)
    {
      kumm_addregion(region_start, region_size);
      g_mem_region_next++;
    }
}
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
 *     Kernel IDLE thread stack.  Size determined by CONFIG_IDLETHREAD_STACKSIZE.
 *     Padding for alignment
 *     User .data region.  Size determined at link time.
 *     User .bss region  Size determined at link time.
 *     Kernel heap.  Size determined by CONFIG_MM_KERNEL_HEAPSIZE.
 *     User heap.  Extends to the end of SRAM.
 *
 ****************************************************************************/

void up_allocate_heap(FAR void **heap_start, size_t *heap_size)
{
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend + CONFIG_MM_KERNEL_HEAPSIZE;
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
  *heap_start = (FAR void *)ubase;
  *heap_size  = usize;

  /* Colorize the heap for debug */

  up_heap_color((FAR void *)ubase, usize);

  /* Allow user-mode access to the user heap memory */

  lpc43_mpu_uheap((uintptr_t)ubase, usize);

#else
  /* Return the heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void *)g_idle_topstack;
  *heap_size  = CONFIG_RAM_END - g_idle_topstack;

  /* Colorize the heap for debug */

  up_heap_color(*heap_start, *heap_size);
#endif
}

/****************************************************************************
 * Name: up_allocate_kheap
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
 ****************************************************************************/

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
void up_allocate_kheap(FAR void **heap_start, size_t *heap_size)
{
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend + CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = CONFIG_RAM_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)SRAM1_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the SRAM1_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(usize);
  DEBUGASSERT((CONFIG_RAM_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = CONFIG_RAM_END - usize;

  /* Return the kernel heap settings (i.e., the part of the heap region
   * that was not dedicated to the user heap).
   */

  *heap_start = (FAR void *)USERSPACE->us_bssend;
  *heap_size  = usize;
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
#ifdef MM_HAVE_REGION
  /* start from second region */

  g_mem_region_next = 2;

#ifdef MM_USE_LOCSRAM_BANK1
  /* Add the SRAM to the user heap */

  mem_addregion((FAR void *)LPC43_LOCSRAM_BANK1_BASE, LPC43_LOCSRAM_BANK1_SIZE);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to the SRAM heap */

  lpc43_mpu_uheap((uintptr_t)LPC43_LOCSRAM_BANK1_BASE, LPC43_LOCSRAM_BANK1_SIZE);
#endif
#endif

#ifdef MM_USE_AHBSRAM_BANK0
  /* Add the SRAM to the user heap */

  mem_addregion((FAR void *)LPC43_AHBSRAM_BANK0_BASE, LPC43_AHBSRAM_BANK0_SIZE);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to the SRAM heap */

  lpc43_mpu_uheap((uintptr_t)LPC43_AHBSRAM_BANK0_BASE, LPC43_AHBSRAM_BANK0_SIZE);
#endif
#endif

#ifdef MM_USE_AHBSRAM_BANK1
  /* Add the SRAM to the user heap */

  mem_addregion((FAR void *)LPC43_AHBSRAM_BANK1_BASE, LPC43_AHBSRAM_BANK1_SIZE);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to the SRAM heap */

  lpc43_mpu_uheap((uintptr_t)LPC43_AHBSRAM_BANK1_BASE, LPC43_AHBSRAM_BANK1_SIZE);
#endif
#endif

#ifdef MM_USE_AHBSRAM_BANK2
  /* Add the SRAM heap to the user heap */

  mem_addregion((FAR void *)MM_DMAREGION_BASE, MM_DMAREGION_SIZE);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to the SRAM heap */

  lpc43_mpu_uheap((uintptr_t)MM_DMAREGION_BASE, MM_DMAREGION_SIZE);
#endif
#endif

#ifdef MM_USE_EXTSDRAM0
  /* Add the SDRAM to the user heap */

  mem_addregion((FAR void *)MM_EXTSDRAM0_REGION, MM_EXTSDRAM0_SIZE);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to the SDRAM heap */

  lpc43_mpu_uheap((uintptr_t)MM_EXTSDRAM0_REGION, MM_EXTSDRAM0_SIZE);
#endif
#endif

#ifdef MM_USE_EXTSDRAM1
  /* Add the SDRAM to the user heap */

  mem_addregion((FAR void *)MM_EXTSDRAM1_REGION, MM_EXTSDRAM1_SIZE);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to the SDRAM heap */

  lpc43_mpu_uheap((uintptr_t)MM_EXTSDRAM1_REGION, MM_EXTSDRAM1_SIZE);
#endif
#endif

#ifdef MM_USE_EXTSDRAM2
  /* Add the SDRAM to the user heap */

  mem_addregion((FAR void *)MM_EXTSDRAM2_REGION, MM_EXTSDRAM2_SIZE);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to the SDRAM heap */

  lpc43_mpu_uheap((uintptr_t)MM_EXTSDRAM2_REGION, MM_EXTSDRAM2_SIZE);
#endif
#endif

#ifdef MM_USE_EXTSDRAM3
  /* Add the SDRAM to the user heap */

  mem_addregion((FAR void *)MM_EXTSDRAM3_REGION, MM_EXTSDRAM3_SIZE);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to the SDRAM heap */

  lpc43_mpu_uheap((uintptr_t)MM_EXTSDRAM3_REGION, MM_EXTSDRAM3_SIZE);
#endif
#endif
#endif /* MM_HAVE_REGION */
}
#endif
