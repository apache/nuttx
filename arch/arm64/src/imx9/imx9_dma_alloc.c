/****************************************************************************
 * arch/arm64/src/imx9/imx9_dma_alloc.c
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

#include <chip.h>

#if defined(CONFIG_IMX9_DMA_ALLOC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA buffers must be aligned with the D-Cache line boundaries to facilitate
 * cache operations on the DMA buffers when the D-Cache is enabled.
 */

#define DMA_ALIGN       ARMV8A_DCACHE_LINESIZE
#define DMA_ALIGN_MASK  (DMA_ALIGN - 1)
#define DMA_ALIGN_UP(n) (((n) + DMA_ALIGN_MASK) & ~DMA_ALIGN_MASK)

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

static uint8_t g_dma_heap[CONFIG_IMX9_DMA_ALLOC_POOL_SIZE]
aligned_data(DMA_ALIGN) locate_data(CONFIG_IMX9_DMA_ALLOC_SECT);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_dma_alloc_init
 *
 * Description:
 *   Initialize the DMA memory allocator.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int imx9_dma_alloc_init(void)
{
  /* Allocate 64B granules with 64B alignment */

  dma_allocator = gran_initialize(g_dma_heap, sizeof(g_dma_heap), 6, 6);

  if (dma_allocator == NULL)
    {
      return -ENOMEM;
    }

  return OK;
}

/****************************************************************************
 * Name: imx9_dma_alloc
 *
 * Description:
 *   Allocate a contiguous block of physical memory for DMA.
 *
 * Input Parameters:
 *   size - Size of the requested block in bytes.
 *
 * Returned Value:
 *   Physical address of the first page on success; NULL on failure.
 *
 ****************************************************************************/

void *imx9_dma_alloc(size_t size)
{
  return gran_alloc(dma_allocator, size);
}

/****************************************************************************
 * Name: imx9_dma_free
 *
 * Description:
 *   Free a previously allocated DMA memory block.
 *
 * Input Parameters:
 *   memory - Physical address of the first page of DMA memory.
 *   size   - Size of the allocated block in bytes.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void imx9_dma_free(void *memory, size_t size)
{
  gran_free(dma_allocator, memory, size);
}

#ifdef CONFIG_FAT_DMAMEMORY
void *fat_dma_alloc(size_t size)
{
  return imx9_dma_alloc(size);
}

void fat_dma_free(void *memory, size_t size)
{
  imx9_dma_free(memory, size);
}
#endif

#endif /* CONFIG_IMX9_DMA_ALLOC */
