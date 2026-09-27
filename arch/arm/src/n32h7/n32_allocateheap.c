/****************************************************************************
 * arch/arm/src/n32h7/n32_allocateheap.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/compiler.h>
#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "mpu.h"
#include "hardware/n32h7_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

extern const uint8_t _axiram_heap_start[];
extern const uint8_t _ahbram_heap_start[];

/* Adjust memory region definitions */
#define N32_DTCM_SIZE       (128U * 1024U)  /* DTCM 128KB */
#define N32_AHBSRAM_SIZE    (352U * 1024U)  /* AHBSRAM 352KB */
#define N32_AXISRAM_SIZE    (128U * 1024U)  /* AXI SRAM 128KB */
#define N32_BSRAM_SIZE      (  4U * 1024U)  /* BSRAM 4KB */

/* Adjust main heap start address to DTCM */
#define MAIN_HEAP_START     DTCM_HEAP_START
#define DTCM_HEAP_START     g_idle_topstack
#define AHB_HEAP_START      (uint32_t)_ahbram_heap_start
#define AXI_HEAP_START      (uint32_t)_axiram_heap_start
#define BSRAM_HEAP_START    N32_BSRAM_BASE

/* Adjust main heap end address to DTCM */
#define MAIN_HEAP_END       DTCM_HEAP_END
#define DTCM_HEAP_END       (N32_DTCM_SIZE + N32_DTCRAM_BASE)
#define AHB_HEAP_END        (N32_AHBSRAM_SIZE + N32_AHBSRAM_BASE)
#define AXI_HEAP_END        (N32_AXISRAM_SIZE + N32_AXISRAM_BASE)
#define BSRAM_HEAP_END      (N32_BSRAM_SIZE + N32_BSRAM_BASE)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_HEAP_COLORATION
static inline void up_heap_color(void *start, size_t size)
{
  memset(start, HEAP_COLOR, size);
}
#else
#  define up_heap_color(start,size)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Adjust protected mode heap to SRAM1 */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend
    + CONFIG_MM_KERNEL_HEAPSIZE;
  size_t usize = MAIN_HEAP_START + N32_SRAM1_SIZE - ubase;
  int log2;

  DEBUGASSERT(ubase < (DTCM_HEAP_START + N32_DTCM_SIZE));

  /* MPU alignment processing */

  log2 = (int)mpu_log2regionfloor(usize);
  DEBUGASSERT(((DTCM_HEAP_START + N32_DTCM_SIZE) & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = (DTCM_HEAP_START + N32_DTCM_SIZE) - usize;

  *heap_start = (void *)ubase;
  *heap_size = usize;

  /* Initialize kernel memory manager */

  kmm_initialize(*heap_start, *heap_size);

  /* Debug message output */

  minfo("Protected mode heap: %uKB at %p\n", *heap_size / 1024, *heap_start);
#else

  /* Adjust main heap to DTCM */

  *heap_start = (void *)MAIN_HEAP_START;
  *heap_size = MAIN_HEAP_END - MAIN_HEAP_START;

  board_autoled_on(LED_HEAPALLOCATE);
  kmm_initialize(*heap_start, *heap_size);
  minfo("Main heap: %uKB DTCM at %p\n", *heap_size / 1024, *heap_start);
#endif

  /* Debug heap color */

  up_heap_color(*heap_start, *heap_size);
}

#if CONFIG_MM_REGIONS > 1
void arm_addregion(void)
{
  unsigned mm_regions = 1;

  /* Add AXI SRAM region */

  if (mm_regions < CONFIG_MM_REGIONS)
    {
      kumm_addregion((void *)AXI_HEAP_START, AXI_HEAP_END - AXI_HEAP_START);
      minfo("Added AXI SRAM: %luKB at %p\n"
        , (AXI_HEAP_END - AXI_HEAP_START) / 1024U
        , (uint32_t *)AXI_HEAP_START);
      mm_regions++;
    }

  /* Add AHBSRAM region */

  if (mm_regions < CONFIG_MM_REGIONS)
    {
      kumm_addregion((void *)AHB_HEAP_START, AHB_HEAP_END - AHB_HEAP_START);
      minfo("Added AHBSRAM: %luKB at %p\n"
        , (AHB_HEAP_END - AHB_HEAP_START) / 1024U
        , (uint32_t *)AHB_HEAP_START);
      mm_regions++;
    }

  /* Add backup SRAM region */

#if defined(N32_BSRAM_BASE) && !defined(CONFIG_N32H7_BSRAMEXCLUDE)
  if (mm_regions < CONFIG_MM_REGIONS)
    {
      kumm_addregion((void *)BSRAM_HEAP_START
        , BSRAM_HEAP_END - BSRAM_HEAP_START);
      minfo("Added BSRAM: %luKB at %p\n"
        , (BSRAM_HEAP_END - BSRAM_HEAP_START) / 1024U
        , (uint32_t *)BSRAM_HEAP_START);
      mm_regions++;
    }
#endif

  /* Add external memory support region */
#if defined(CONFIG_N32H7_FMC)
  /* Add FMC initialization code here */
#endif
}
#endif
