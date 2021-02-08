/****************************************************************************
 * arch/xtensa/src/esp32/esp32_imm.c
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

#include <nuttx/arch.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/mm/mm.h>
#include <malloc.h>

#include "xtensa.h"

#ifdef CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Region 1 of the heap is the area from the end of the .data section to the
 * begining of the ROM data.  The start address is defined from the linker
 * script as "_sheap".  Then end is defined here, as follows:
 */

#ifndef HEAP_REGION1_END
#define HEAP_REGION1_END    0x3ffdfff0
#endif

/* If define CONFIG_XTENSA_IMEM_MAXIMIZE_HEAP_REGION, it means
 * using maximum separate heap for internal memory, but part of
 * the available memory is reserved for the Region 1 heap.
 */

#ifdef CONFIG_XTENSA_IMEM_MAXIMIZE_HEAP_REGION
#ifndef HEAP_REGION_OFFSET
#define HEAP_REGION_OFFSET  0x2000
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct mm_heap_s g_iheap;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_imm_initialize
 *
 * Description:
 *   Initialize the internal heap.
 *
 ****************************************************************************/

void xtensa_imm_initialize(void)
{
  void  *start;
  size_t size;

  start = (FAR void *)&_sheap;
#ifdef CONFIG_XTENSA_IMEM_MAXIMIZE_HEAP_REGION
  size_t offset = HEAP_REGION_OFFSET;
  size = (size_t)(HEAP_REGION1_END - (uintptr_t)start - offset);
#else

  /* If the following DEBUGASSERT fails,
   * probably you have too large CONFIG_XTENSA_IMEM_REGION_SIZE.
   */

  size = CONFIG_XTENSA_IMEM_REGION_SIZE;
  DEBUGASSERT(HEAP_REGION1_END >  ((uintptr_t)start + size));
#endif

  mm_initialize(&g_iheap, start, size);

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
  static struct procfs_meminfo_entry_s g_imm_procfs;

  g_imm_procfs.name = "esp32-imem";
  g_imm_procfs.mallinfo = (void *)mm_mallinfo;
  g_imm_procfs.user_data = &g_iheap;
  procfs_register_meminfo(&g_imm_procfs);
#endif
}

/****************************************************************************
 * Name: xtensa_imm_malloc
 *
 * Description:
 *   Allocate memory from the internal heap.
 *
 ****************************************************************************/

void *xtensa_imm_malloc(size_t size)
{
  return mm_malloc(&g_iheap, size);
}

/****************************************************************************
 * Name: xtensa_imm_calloc
 *
 * Description:
 *   Calculates the size of the allocation and
 *   allocate memory the internal heap.
 *
 ****************************************************************************/

void *xtensa_imm_calloc(size_t n, size_t elem_size)
{
  return mm_calloc(&g_iheap, n, elem_size);
}

/****************************************************************************
 * Name: xtensa_imm_realloc
 *
 * Description:
 *   Reallocate memory from the internal heap.
 *
 ****************************************************************************/

void *xtensa_imm_realloc(void *ptr, size_t size)
{
  return mm_realloc(&g_iheap, ptr, size);
}

/****************************************************************************
 * Name: xtensa_imm_zalloc
 *
 * Description:
 *   Allocate and zero memory from the internal heap.
 *
 ****************************************************************************/

void *xtensa_imm_zalloc(size_t size)
{
  return mm_zalloc(&g_iheap, size);
}

/****************************************************************************
 * Name: xtensa_imm_free
 *
 * Description:
 *   Free memory from the internal heap.
 *
 ****************************************************************************/

void xtensa_imm_free(FAR void *mem)
{
  mm_free(&g_iheap, mem);
}

/****************************************************************************
 * Name: xtensa_imm_memalign
 *
 * Description:
 *   memalign requests more than enough space from malloc, finds a region
 *   within that chunk that meets the alignment request and then frees any
 *   leading or trailing space.
 *
 *   The alignment argument must be a power of two (not checked).  8-byte
 *   alignment is guaranteed by normal malloc calls.
 *
 ****************************************************************************/

void *xtensa_imm_memalign(size_t alignment, size_t size)
{
  return mm_memalign(&g_iheap, alignment, size);
}

/****************************************************************************
 * Name: xtensa_imm_heapmember
 *
 * Description:
 *   Check if an address lies in the internal heap.
 *
 * Parameters:
 *   mem - The address to check
 *
 * Return Value:
 *   true if the address is a member of the internal heap.  false if not
 *
 ****************************************************************************/

bool xtensa_imm_heapmember(FAR void *mem)
{
  return mm_heapmember(&g_iheap, mem);
}

/****************************************************************************
 * Name: xtensa_imm_mallinfo
 *
 * Description:
 *   mallinfo returns a copy of updated current heap information for the
 *   user heap.
 *
 ****************************************************************************/

int xtensa_imm_mallinfo(FAR struct mallinfo *info)
{
  return mm_mallinfo(&g_iheap, info);
}

#endif /* CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP */
