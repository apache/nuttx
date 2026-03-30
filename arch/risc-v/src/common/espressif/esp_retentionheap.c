/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_retentionheap.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ESPRESSIF_RETENTION_HEAP

#include <debug.h>
#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>
#include <nuttx/mm/mm.h>

#include "esp_retentionheap.h"
#include "soc/soc.h"
#include "heap_memory_layout.h"
#include "esp_heap_caps.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APP_USABLE_DRAM_END (SOC_ROM_STACK_START - SOC_ROM_STACK_SIZE)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mm_heap_s *g_retentionheap;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_retentionheap_find_region
 *
 * Description:
 *   Find address of retention heap.
 *
 * Input Parameters:
 *   rstart - Starting address of the region
 *   rend   - Ending address of the region
 *
 * Returned Value:
 *   True if region found; false otherwise
 *
 ****************************************************************************/

bool esp_retentionheap_find_region(uintptr_t *rstart, uintptr_t *rend)
{
  for (size_t i = 0; i < soc_memory_region_count; i++)
    {
      const soc_memory_region_t *reg = &soc_memory_regions[i];
      if ((soc_memory_types[reg->type].caps[0] & MALLOC_CAP_RETENTION) != 0
          && (reg->startup_stack == false))
        {
          *rstart = (uintptr_t)reg->start;
          *rend = *rstart + reg->size;
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: esp_retentionheap_initialize
 *
 * Description:
 *   Initialize the retention heap.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_retentionheap_initialize(void)
{
  size_t sz;
  uintptr_t rstart;
  uintptr_t rend;

  if (!esp_retentionheap_find_region(&rstart, &rend))
    {
      _warn("MALLOC_CAP_RETENTION region did not find\n");
      return;
    }

  if (rend <= rstart)
    {
      return;
    }

  sz = (size_t)(rend - rstart);
  g_retentionheap = mm_initialize("retention", (void *)rstart, sz);
  if (g_retentionheap == NULL)
    {
      _err("retention heap mm_initialize failed size=%zu\n", sz);
    }
}

/****************************************************************************
 * Name: esp_retentionheap_malloc
 *
 * Description:
 *   Allocate memory from the retention heap.
 *
 * Input Parameters:
 *   size - Size to allocate memory in bytes
 *
 * Returned Value:
 *   Valid address reference on success; NULL, otherwise.
 *
 ****************************************************************************/

void *esp_retentionheap_malloc(size_t size)
{
  if (g_retentionheap == NULL)
    {
      return NULL;
    }

  return mm_malloc(g_retentionheap, size);
}

/****************************************************************************
 * Name: esp_retentionheap_calloc
 *
 * Description:
 *   Calculates the size of the allocation and allocate memory from
 *   the retention heap.
 *
 * Input Parameters:
 *   n         - Number of elements to allocate.
 *   elem_size - Size of each element, in bytes.
 *
 * Returned Value:
 *   Valid address reference on success; NULL, otherwise.
 *
 ****************************************************************************/

void *esp_retentionheap_calloc(size_t n, size_t elem_size)
{
  if (g_retentionheap == NULL)
    {
      return NULL;
    }

  return mm_calloc(g_retentionheap, n, elem_size);
}

/****************************************************************************
 * Name: esp_retentionheap_memalign
 *
 * Description:
 *   memalign requests more than enough space from malloc, finds a region
 *   within that chunk that meets the alignment request and then frees any
 *   leading or trailing space.
 *
 *   The alignment argument must be a power of two (not checked). 8-byte
 *   alignment is guaranteed by normal malloc calls.
 *
 * Input Parameters:
 *   alignment - Required byte alignment; must be a power of two and usually
 *               at least sizeof(void *).
 *   size      - Number of bytes to allocate.
 *
 * Returned Value:
 *   Valid address reference on success; NULL, otherwise.
 *
 ****************************************************************************/

void *esp_retentionheap_memalign(size_t alignment, size_t size)
{
  if (g_retentionheap == NULL)
    {
      return NULL;
    }

  return mm_memalign(g_retentionheap, alignment, size);
}

/****************************************************************************
 * Name: esp_retentionheap_realloc
 *
 * Description:
 *   Reallocate memory from the retention heap.
 *
 * Input Parameters:
 *   mem  - The address to reallocate
 *   size - New requested size
 *
 * Returned Value:
 *   Valid address reference on success; NULL, otherwise.
 *
 ****************************************************************************/

void *esp_retentionheap_realloc(void *ptr, size_t size)
{
  if (g_retentionheap == NULL)
    {
      return NULL;
    }

  return mm_realloc(g_retentionheap, ptr, size);
}

/****************************************************************************
 * Name: esp_retentionheap_free
 *
 * Description:
 *   Free memory from the retention heap.
 *
 * Input Parameters:
 *   mem - The address to free
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_retentionheap_free(void *mem)
{
  if (g_retentionheap == NULL || mem == NULL)
    {
      return;
    }

  mm_free(g_retentionheap, mem);
}

/****************************************************************************
 * Name: esp_retentionheap_heapmember
 *
 * Description:
 *   Check if an address lies in the retention heap.
 *
 * Parameters:
 *   mem - The address to check
 *
 * Return Value:
 *   True if the address is in the retention heap; false if not.
 *
 ****************************************************************************/

bool esp_retentionheap_heapmember(void *mem)
{
  if (g_retentionheap == NULL || mem == NULL)
    {
      return false;
    }

  return mm_heapmember(g_retentionheap, mem);
}

/****************************************************************************
 * Name: esp_retentionheap_malloc_size
 *
 * Description:
 *   Get size of the allocated space in retention heap
 *
 * Input Parameters:
 *   mem - The address to check
 *
 * Returned Value:
 *   Size of allocated space related to address.
 *
 ****************************************************************************/

size_t esp_retentionheap_malloc_size(void *mem)
{
  if (g_retentionheap == NULL || mem == NULL)
    {
      return 0;
    }

  return mm_malloc_size(g_retentionheap, mem);
}

#endif /* CONFIG_ESPRESSIF_RETENTION_HEAP */
