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
#include <nuttx/mm/mm.h>
#include <malloc.h>

#include "xtensa.h"

#if CONFIG_XTENSA_USE_SEPARATE_IMEM

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
  size = CONFIG_XTENSA_IMEM_REGION_SIZE;
  mm_initialize(&g_iheap, start, size);
}

/****************************************************************************
 * Name: xtensa_imm_malloc
 *
 * Description:
 *   Allocate memory from the internal heap.
 *
 ****************************************************************************/

FAR void *xtensa_imm_malloc(size_t size)
{
  return mm_malloc(&g_iheap, size);
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

FAR void *xtensa_imm_memalign(size_t alignment, size_t size)
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

#endif /* CONFIG_XTENSA_USE_SEPARATE_IMEM */
