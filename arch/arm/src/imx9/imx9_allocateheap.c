/****************************************************************************
 * arch/arm/src/imx9/imx9_allocateheap.c
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#include <assert.h>
#include <debug.h>
#include <stdint.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/userspace.h>

#include <arch/imx9/chip.h>

#include "arm_internal.h"
#include "hardware/imx9_memorymap.h"
#include "imx9_mpuinit.h"
#include "mpu.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Primary RAM:  The Linker script positions the system BLOB's .data and
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
 *    IMX9_ITCM_BASE            0x00000000     256KB M7 ITCM
 *    IMX9_DTCM_BASE            0x20000000     256KB M7 DTCM
 *    IMX9_OCRAM_BASE           0x20480000     352KB OCRAM
 */

/* There there then several memory configurations with a one primary memory
 * region and up to two additional memory regions which may be OCRAM, DTCM
 * external DDR.
 */

#undef IMX9_OCRAM_ASSIGNED
#undef IMX9_DCTM_ASSIGNED

#define _IMX9_OCRAM_BASE IMX9_OCRAM_BASE

/* See linker script */

extern const uint32_t _ram_start[];
extern const uint32_t _ram_size[];
extern const uint32_t _ocram_start[];
extern const uint32_t _ocram_size[];

/* by default DTCM size is 256k
 * Can be configured by AON__BLK_CTRL_Secure_AON.M7_CFG  (0x444f0124)
 */

#define PRIMARY_RAM_START (uint32_t) _ram_start
#define PRIMARY_RAM_SIZE  (uint32_t) _ram_size
#define PRIMARY_RAM_END   ((uint32_t)_ram_start + (uint32_t)_ram_size)

#define OCRAM_START (uint32_t) _ocram_start
#define OCRAM_SIZE  (uint32_t) _ocram_size

#if CONFIG_MM_REGIONS > 1
/* Pick the first region to add to the heap could be any one of OCRAM, DTCM,
 * SDRAM, or SRAM depending upon which are enabled and which has not
 * already been assigned as the primary RAM.
 */

#if defined(CONFIG_IMX9_OCRAM_HEAP) && !defined(IMX9_OCRAM_ASSIGNED)
#define REGION1_RAM_START   OCRAM_START
#define REGION1_RAM_SIZE    OCRAM_SIZE
#define IMX9_OCRAM_ASSIGNED 1
#else
#warning CONFIG_MM_REGIONS > 1 but no available memory region
#endif

#define REGION1_RAM_END (REGION1_RAM_START + REGION1_RAM_SIZE)
#endif

#if CONFIG_MM_REGIONS > 2
#warning CONFIG_MM_REGIONS > 2 but no available memory region
#endif

#if CONFIG_MM_REGIONS > 3
#warning CONFIG_MM_REGIONS > 3 but no available memory region
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

const uintptr_t g_idle_topstack
    = (uintptr_t)&_ebss + CONFIG_IDLETHREAD_STACKSIZE;

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

  uintptr_t ubase
      = (uintptr_t)USERSPACE->us_bssend + CONFIG_MM_KERNEL_HEAPSIZE;
  size_t usize = PRIMARY_RAM_END - ubase;
  int log2;

  DEBUGASSERT(ubase < (uintptr_t)PRIMARY_RAM_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the PRIMARY_RAM_END
   * is aligned to the MPU requirement.
   */

  log2 = (int)mpu_log2regionfloor(usize);
  DEBUGASSERT((PRIMARY_RAM_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = PRIMARY_RAM_END - usize;

  /* Return the user-space heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (void *)ubase;
  *heap_size  = usize;

  /* Allow user-mode access to the user heap memory */

  imx9_mpu_uheap((uintptr_t)ubase, usize);
#else

  /* Return the heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
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

  uintptr_t ubase
      = (uintptr_t)USERSPACE->us_bssend + CONFIG_MM_KERNEL_HEAPSIZE;
  size_t usize = PRIMARY_RAM_END - ubase;
  int log2;
  DEBUGASSERT(ubase < (uintptr_t)PRIMARY_RAM_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the CONFIG_RAM_END
   * is aligned to the MPU requirement.
   */

  log2 = (int)mpu_log2regionfloor(usize);
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
  /* Add region 1 to the user heap */

  kumm_addregion((void *)REGION1_RAM_START, REGION1_RAM_SIZE);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to region 1 */

  imx9_mpu_uheap((uintptr_t)REGION1_RAM_START, REGION1_RAM_SIZE);
#endif

#if CONFIG_MM_REGIONS > 2
  /* Add region 2 to the user heap */

  kumm_addregion((void *)REGION2_RAM_START, REGION2_RAM_SIZE);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to region 2 */

  imx9_mpu_uheap((uintptr_t)REGION2_RAM_START, REGION2_RAM_SIZE);
#endif
#endif /* CONFIG_MM_REGIONS > 2 */
}
#endif /* CONFIG_MM_REGIONS > 1 */
