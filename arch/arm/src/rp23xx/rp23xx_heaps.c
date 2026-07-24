/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_heaps.c
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

#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/mm.h>

#include "arm_internal.h"
#include "rp23xx_psram.h"

#if defined(CONFIG_RP23XX_PSRAM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The PSRAM base address is fixed; the size is whatever rp23xx_psramconfig()
 * detected at boot (0 if no PSRAM is fitted).  psramconfig() runs from
 * rp23xx_boardinitialize(), before any of the heap hooks below, so the
 * detected size is always available here.
 */

#define PSRAM_START ((void *)RP23XX_PSRAM_BASE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#if defined(CONFIG_RP23XX_PSRAM_HEAP_SEPARATE)
static struct mm_heap_s *g_psramheap;
#endif

#if defined(CONFIG_RP23XX_PSRAM_HEAP_SINGLE)

#if defined(CONFIG_MM_KERNEL_HEAP)
#error cannot use CONFIG_MM_KERNEL_HEAP with single heap
#endif

#if CONFIG_MM_REGIONS > 1
void arm_addregion(void)
{
  size_t psram_size = rp23xx_psram_size();

  /* Add the detected PSRAM region to the main heap */

  if (psram_size > 0)
    {
      kumm_addregion(PSRAM_START, psram_size);
    }
}
#endif

#elif defined (CONFIG_RP23XX_PSRAM_HEAP_USER)

#if !defined(CONFIG_MM_KERNEL_HEAP)
#error MM_KERNEL_HEAP is required for separate kernel heap
#endif

/* Use the internal SRAM as the kernel heap */

void up_allocate_kheap(void **heap_start, size_t *heap_size)
{
  *heap_start = (void *)g_idle_topstack;

#ifdef CONFIG_ARCH_PGPOOL_PBASE
  *heap_size  = CONFIG_ARCH_PGPOOL_PBASE - g_idle_topstack;
#else
  *heap_size  = CONFIG_RAM_END - g_idle_topstack;
#endif
}

/* Use the external PSRAM as the default user heap */

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  *heap_start = PSRAM_START;
  *heap_size  = rp23xx_psram_size();
}

#elif defined (CONFIG_RP23XX_PSRAM_HEAP_SEPARATE)

#if !defined(CONFIG_ARCH_HAVE_EXTRA_HEAPS)
#error ARCH_HAVE_EXTRA_HEAPS is required for multiple heaps
#endif

void up_extraheaps_init(void)
{
  size_t psram_size = rp23xx_psram_size();

  if (psram_size > 0)
    {
      g_psramheap = mm_initialize("psram", PSRAM_START, psram_size);
    }
}
#endif

#endif
