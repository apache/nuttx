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

#if defined(CONFIG_RP23XX_PSRAM)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static void * const psram_start = (void *)0x11000000ul;
static const size_t psram_size = 8 * 1024 * 1024;

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
  /* Add the PSRAM region to main heap */

  kumm_addregion(psram_start, psram_size);
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
  *heap_start = psram_start;
  *heap_size = psram_size;
}

#elif defined (CONFIG_RP23XX_PSRAM_HEAP_SEPARATE)

#if !defined(CONFIG_ARCH_HAVE_EXTRA_HEAPS)
#error ARCH_HAVE_EXTRA_HEAPS is required for multiple heaps
#endif

void up_extraheaps_init(void)
{
    g_psramheap = mm_initialize("psram", psram_start, psram_size);
}
#endif

#endif
