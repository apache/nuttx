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

#ifdef CONFIG_BUILD_PROTECTED
#  include <nuttx/userspace.h>
#endif

#include <string.h>

#include "arm_internal.h"
#include "rp23xx_psram.h"
#include "rp23xx_heaps.h"

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
struct mm_heap_s *g_psramheap;
#endif

/* up_initialize() calls arm_addregion() whenever CONFIG_MM_REGIONS > 1.
 * Only the single heap has a region to add: the PSRAM.
 */

#if CONFIG_MM_REGIONS > 1
void arm_addregion(void)
{
#ifdef CONFIG_RP23XX_PSRAM_HEAP_SINGLE
  size_t psram_size = rp23xx_psram_size();

  /* Add the detected PSRAM region to the main heap */

  if (psram_size > 0)
    {
      kumm_addregion(PSRAM_START, psram_size);
    }
#endif
}
#endif

#if defined(CONFIG_RP23XX_PSRAM_HEAP_SINGLE)

#if defined(CONFIG_MM_KERNEL_HEAP)
#error cannot use CONFIG_MM_KERNEL_HEAP with single heap
#endif

#elif defined (CONFIG_RP23XX_PSRAM_HEAP_USER)

#if !defined(CONFIG_MM_KERNEL_HEAP)
#error MM_KERNEL_HEAP is required for separate kernel heap
#endif

/* Use the internal SRAM as the kernel heap */

void up_allocate_kheap(void **heap_start, size_t *heap_size)
{
#ifdef CONFIG_BUILD_PROTECTED
  uintptr_t base = (uintptr_t) USERSPACE->us_bssend;
#else
  uintptr_t base = g_idle_topstack;
#endif

#ifdef CONFIG_ARCH_PGPOOL_PBASE
  size_t end = CONFIG_ARCH_PGPOOL_PBASE;
#else
  size_t end = CONFIG_RAM_END;
#endif

  *heap_start = (void *)base;
  if (rp23xx_psram_size() > 0)
    {
      DEBUGASSERT(base <= end);
      *heap_size = end - base;
    }
  else
    {
      *heap_size = CONFIG_MM_KERNEL_HEAPSIZE;
    }
}

/* Use the external PSRAM as the default user heap */

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  size_t psram_size = rp23xx_psram_size();

  if (psram_size > 0)
    {
      *heap_start = PSRAM_START;
      *heap_size  = psram_size;
    }
  else
    {
#ifdef CONFIG_BUILD_PROTECTED
      uintptr_t base = (uintptr_t)USERSPACE->us_bssend +
                       CONFIG_MM_KERNEL_HEAPSIZE;
#else
      uintptr_t base = g_idle_topstack + CONFIG_MM_KERNEL_HEAPSIZE;
#endif
#ifdef CONFIG_ARCH_PGPOOL_PBASE
      size_t end = CONFIG_ARCH_PGPOOL_PBASE;
#else
      size_t end = CONFIG_RAM_END;
#endif
      *heap_start = (void *)base;
      DEBUGASSERT(base <= end);
      *heap_size  = end - base;
    }
}

#elif defined (CONFIG_RP23XX_PSRAM_HEAP_SEPARATE)

#if !defined(CONFIG_ARCH_HAVE_EXTRA_HEAPS)
#error ARCH_HAVE_EXTRA_HEAPS is required for multiple heaps
#endif

void up_extraheaps_init(void)
{
  struct mm_heap_config_s config;
  size_t psram_size = rp23xx_psram_size();

  if (psram_size > 0)
    {
      /* Keep the heap and its lock in the SRAM: in the PSRAM an atomic
       * operation does not exclude the other core.
       */

      memset(&config, 0, sizeof(config));
      config.allocheap = true;
      config.name  = "psram";
      config.start = PSRAM_START;
      config.size  = psram_size;
      g_psramheap  = mm_initialize_heap(&config);
    }
}
#endif

#endif
