/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_allocateheap.c
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

/* Copyright 2022 NXP */

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

#include <arch/s32k3xx/chip.h>

#include "arm_internal.h"

#include "hardware/s32k3xx_memorymap.h"

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
 * OCRAM is automatically added to the heap.  The linker provided address,
 * ... .sbss, .ebss, .sdat, etc. ...  are expected to lie in the the region
 * defined by the OCRAM configuration settings.
 *
 * Other RAM regions must be selected use configuration options and the
 * start and end of those RAM regions must also be provided in the
 * configuration.  CONFIG_MM_REGIONS must also be set to determined the
 * number of regions to be added to the heap.
 *
 *
 * SOC with 512KiB
 *
 *    DTCM_BASE_ADDR           0x20000000     128K DTCM
 *    ITCM_BASE_ADDR           0x20000000     64K  ITCM
 *    SRAM_BASE_ADDR           0x20400000     512KB OCRAM
 */

/* There there then several memory configurations with a one primary memory
 * region and up to two additional memory regions which may be OCRAM, DTCM
 * external SDRAM, or external SRAM.
 */

#undef S32K3XX_ITCM_ASSIGNED
#undef S32K3XX_DCTM_ASSIGNED
#undef S32K3XX_SRAM_ASSIGNED

#if CONFIG_MM_REGIONS > 1
/* Pick the first region to add to the heap could be any one of OCRAM, DTCM,
 * SDRAM, or SRAM depending upon which are enabled and which has not
 * already been assigned as the primary RAM.
 */

#if defined(CONFIG_S32K3XX_DTCM_HEAP) && !defined(S32K3XX_DCTM_ASSIGNED)
#  define REGION1_RAM_START    DTCM_BASE_ADDR
#  define REGION1_RAM_SIZE     (DTCM_END_ADDR - DTCM_BASE_ADDR)
#  define S32K3XX_DCTM_ASSIGNED 1
#else
#  warning CONFIG_MM_REGIONS > 1 but no available memory region
#endif

#define REGION1_RAM_END        (REGION1_RAM_START + REGION1_RAM_SIZE)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

extern uint8_t SRAM_BASE_ADDR[];
extern uint8_t SRAM_END_ADDR[];
extern uint8_t SRAM_STDBY_BASE_ADDR[];
extern uint8_t SRAM_STDBY_END_ADDR[];
extern uint8_t ITCM_BASE_ADDR[];
extern uint8_t ITCM_END_ADDR[];
extern uint8_t DTCM_BASE_ADDR[];
extern uint8_t DTCM_END_ADDR[];

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

#ifdef CONFIG_BUILD_KERNEL
void up_allocate_kheap(void **heap_start, size_t *heap_size)
#else
void up_allocate_heap(void **heap_start, size_t *heap_size)
#endif
{
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend +
                     CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = (uintptr_t)SRAM_END_ADDR - ubase;

  DEBUGASSERT(ubase < (uintptr_t)SRAM_END_ADDR);

  /* Return the user-space heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (void *)ubase;
  *heap_size  = usize;
#else

  /* Return the heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (void *)g_idle_topstack;
  *heap_size  = SRAM_END_ADDR - (_ebss + CONFIG_IDLETHREAD_STACKSIZE);
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
  DEBUGASSERT(ubase < (uintptr_t)SRAM_END_ADDR);

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
  /* Add region 1 to the user heap */

  kumm_addregion((void *)REGION1_RAM_START, REGION1_RAM_SIZE);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to region 1 */

  s32k3xx_mpu_uheap((uintptr_t)REGION1_RAM_START, REGION1_RAM_SIZE);
#endif

#if CONFIG_MM_REGIONS > 2
  /* Add region 2 to the user heap */

  kumm_addregion((void *)REGION2_RAM_START, REGION2_RAM_SIZE);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to region 2 */

  s32k3xx_mpu_uheap((uintptr_t)REGION2_RAM_START, REGION2_RAM_SIZE);
#endif
#endif /* CONFIG_MM_REGIONS > 2 */
}
#endif /* CONFIG_MM_REGIONS > 1 */
