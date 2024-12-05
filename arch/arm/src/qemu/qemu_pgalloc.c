/****************************************************************************
 * arch/arm/src/qemu/qemu_pgalloc.c
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

#include <assert.h>
#include <debug.h>

#ifdef CONFIG_MM_PGALLOC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Currently, page cache memory must be allocated in DRAM.  There are other
 * possibilities, but the logic in this file will have to extended in order
 * handle any other possibility.
 */

#ifndef CONFIG_ARCH_PGPOOL_MAPPING
#  error CONFIG_ARCH_PGPOOL_MAPPING must be selected
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_pgheap
 *
 * Description:
 *   If there is a page allocator in the configuration, then this function
 *   must be provided by the platform-specific code.  The OS initialization
 *   logic will call this function early in the initialization sequence to
 *   get the page heap information needed to configure the page allocator.
 *
 * Input parameters:
 *   heap_start - A double pointer to the start address of the pgheap
 *   heap_size - A pointer to the size of the pgheap
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_allocate_pgheap(void **heap_start, size_t *heap_size)
{
  DEBUGASSERT(heap_start && heap_size);

  *heap_start = (void *)CONFIG_ARCH_PGPOOL_PBASE;
  *heap_size  = (size_t)CONFIG_ARCH_PGPOOL_SIZE;
}

#endif /* CONFIG_MM_PGALLOC */
