/****************************************************************************
 * mm/mm_heap/mm_initialize.c
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

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/mm/mm.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_addregion
 *
 * Description:
 *   This function adds a region of contiguous memory to the selected heap.
 *
 * Input Parameters:
 *   heap      - The selected heap
 *   heapstart - Start of the heap region
 *   heapsize  - Size of the heap region
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void mm_addregion(FAR struct mm_heap_s *heap, FAR void *heapstart,
                  size_t heapsize)
{
  FAR struct mm_freenode_s *node;
  uintptr_t heapbase;
  uintptr_t heapend;
#if CONFIG_MM_REGIONS > 1
  int IDX = heap->mm_nregions;

  /* Writing past CONFIG_MM_REGIONS would have catastrophic consequences */

  DEBUGASSERT(IDX < CONFIG_MM_REGIONS);
  if (IDX >= CONFIG_MM_REGIONS)
    {
      return;
    }

#else
# define IDX 0
#endif

#if defined(CONFIG_MM_SMALL) && !defined(CONFIG_SMALL_MEMORY)
  /* If the MCU handles wide addresses but the memory manager is configured
   * for a small heap, then verify that the caller is  not doing something
   * crazy.
   */

  DEBUGASSERT(heapsize <= MMSIZE_MAX + 1);
#endif

  mm_takesemaphore(heap);

  /* Adjust the provide heap start and size so that they are both aligned
   * with the MM_MIN_CHUNK size.
   */

  heapbase = MM_ALIGN_UP((uintptr_t)heapstart);
  heapend  = MM_ALIGN_DOWN((uintptr_t)heapstart + (uintptr_t)heapsize);
  heapsize = heapend - heapbase;

  minfo("Region %d: base=%p size=%u\n", IDX + 1, heapstart, heapsize);

  /* Add the size of this region to the total size of the heap */

  heap->mm_heapsize += heapsize;

  /* Create two "allocated" guard nodes at the beginning and end of
   * the heap.  These only serve to keep us from allocating outside
   * of the heap.
   *
   * And create one free node between the guard nodes that contains
   * all available memory.
   */

  heap->mm_heapstart[IDX]            = (FAR struct mm_allocnode_s *)heapbase;
  heap->mm_heapstart[IDX]->size      = SIZEOF_MM_ALLOCNODE;
  heap->mm_heapstart[IDX]->preceding = MM_ALLOC_BIT;

  node                               = (FAR struct mm_freenode_s *)
                                       (heapbase + SIZEOF_MM_ALLOCNODE);
  node->size                         = heapsize - 2*SIZEOF_MM_ALLOCNODE;
  node->preceding                    = SIZEOF_MM_ALLOCNODE;

  heap->mm_heapend[IDX]              = (FAR struct mm_allocnode_s *)
                                       (heapend - SIZEOF_MM_ALLOCNODE);
  heap->mm_heapend[IDX]->size        = SIZEOF_MM_ALLOCNODE;
  heap->mm_heapend[IDX]->preceding   = node->size | MM_ALLOC_BIT;

#undef IDX

#if CONFIG_MM_REGIONS > 1
  heap->mm_nregions++;
#endif

  /* Add the single, large free node to the nodelist */

  mm_addfreechunk(heap, node);

  mm_givesemaphore(heap);
}

/****************************************************************************
 * Name: mm_initialize
 *
 * Description:
 *   Initialize the selected heap data structures, providing the initial
 *   heap region.
 *
 * Input Parameters:
 *   heap      - The selected heap
 *   heapstart - Start of the initial heap region
 *   heapsize  - Size of the initial heap region
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void mm_initialize(FAR struct mm_heap_s *heap, FAR void *heapstart,
                   size_t heapsize)
{
  int i;

  minfo("Heap: start=%p size=%u\n", heapstart, heapsize);

  /* The following two lines have cause problems for some older ZiLog
   * compilers in the past (but not the more recent).  Life is easier if we
   * just the suppress them altogther for those tools.
   */

#ifndef __ZILOG__
  CHECK_ALLOCNODE_SIZE;
  CHECK_FREENODE_SIZE;
#endif
  DEBUGASSERT(MM_MIN_CHUNK >= SIZEOF_MM_FREENODE);
  DEBUGASSERT(MM_MIN_CHUNK >= SIZEOF_MM_ALLOCNODE);

  /* Set up global variables */

  heap->mm_heapsize = 0;

#if CONFIG_MM_REGIONS > 1
  heap->mm_nregions = 0;
#endif

  /* Initialize mm_delaylist */

  heap->mm_delaylist = NULL;

  /* Initialize the node array */

  memset(heap->mm_nodelist, 0, sizeof(struct mm_freenode_s) * MM_NNODES);
  for (i = 1; i < MM_NNODES; i++)
    {
      heap->mm_nodelist[i - 1].flink = &heap->mm_nodelist[i];
      heap->mm_nodelist[i].blink     = &heap->mm_nodelist[i - 1];
    }

  /* Initialize the malloc semaphore to one (to support one-at-
   * a-time access to private data sets).
   */

  mm_seminitialize(heap);

  /* Add the initial region of memory to the heap */

  mm_addregion(heap, heapstart, heapsize);
}
