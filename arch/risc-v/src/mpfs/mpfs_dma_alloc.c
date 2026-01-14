/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_dma_alloc.c
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
#include <syslog.h>
#include <stdint.h>
#include <errno.h>
#include <nuttx/mm/gran.h>

#if defined(CONFIG_FAT_DMAMEMORY) && defined(CONFIG_GRAN)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_DMA_ALLOC_POOL_SIZE (8*512)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static GRAN_HANDLE dma_allocator;

/* The DMA heap size constrains the total number of things that can be
 * ready to do DMA at a time.
 *
 * For example, FAT DMA depends on one sector-sized buffer per filesystem
 * plus one sector-sized buffer per file.
 *
 * We use a fundamental alignment / granule size of 64B; it fulfills the
 * requirement for any DMA engine.
 */

static uint8_t g_dma_heap[MPFS_DMA_ALLOC_POOL_SIZE] aligned_data(64);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_dma_alloc
 *
 * Description:
 *   All boards may optionally provide this API to instantiate a pool of
 *   memory for uses with FAST FS DMA operations.
 *
 ****************************************************************************/

int mpfs_dma_alloc_init(void)
{
  /* Allocate 128B granules with 64B alignment */

  dma_allocator = gran_initialize(g_dma_heap, sizeof(g_dma_heap), 7, 6);

  if (dma_allocator == NULL)
    {
      return -ENOMEM;
    }

  return OK;
}

void *fat_dma_alloc(size_t size)
{
  return gran_alloc(dma_allocator, size);
}

void fat_dma_free(void *memory, size_t size)
{
  gran_free(dma_allocator, memory, size);
}

#endif /* CONFIG_FAT_DMAMEMORY && CONFIG_GRAN */
