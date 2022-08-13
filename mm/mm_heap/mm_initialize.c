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

#include "mm_heap/mm.h"
#include "kasan/kasan.h"

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
  int IDX;

  IDX = heap->mm_nregions;

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

  /* Register to KASan for access check */

  kasan_register(heapstart, &heapsize);

  DEBUGVERIFY(mm_takesemaphore(heap));

  /* Adjust the provided heap start and size.
   *
   * Note: (uintptr_t)node + SIZEOF_MM_ALLOCNODE is what's actually
   * returned to the malloc user, which should have natural alignment.
   * (that is, in this implementation, MM_MIN_CHUNK-alignment.)
   */

  heapbase = MM_ALIGN_UP((uintptr_t)heapstart + 2 * SIZEOF_MM_ALLOCNODE) -
             2 * SIZEOF_MM_ALLOCNODE;
  heapend  = MM_ALIGN_DOWN((uintptr_t)heapstart + (uintptr_t)heapsize);
  heapsize = heapend - heapbase;

#if defined(CONFIG_FS_PROCFS) && \
    !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO) && \
    (defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__))
  minfo("[%s] Region %d: base=%p size=%zu\n",
        heap->mm_procfs.name, IDX + 1, heapstart, heapsize);
#else
  minfo("Region %d: base=%p size=%zu\n", IDX + 1, heapstart, heapsize);
#endif

  /* Add the size of this region to the total size of the heap */

  heap->mm_heapsize += heapsize;

  /* Create two "allocated" guard nodes at the beginning and end of
   * the heap.  These only serve to keep us from allocating outside
   * of the heap.
   *
   * And create one free node between the guard nodes that contains
   * all available memory.
   */

  heap->mm_heapstart[IDX]            = (FAR struct mm_allocnode_s *)
                                       heapbase;
  MM_ADD_BACKTRACE(heap, heap->mm_heapstart[IDX]);
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
  MM_ADD_BACKTRACE(heap, heap->mm_heapend[IDX]);

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
 *   name      - The heap procfs name
 *   heap      - The selected heap
 *   heapstart - Start of the initial heap region
 *   heapsize  - Size of the initial heap region
 *
 * Returned Value:
 *   Return the address of a new heap instance.
 *
 * Assumptions:
 *
 ****************************************************************************/

FAR struct mm_heap_s *mm_initialize(FAR const char *name,
                                    FAR void *heapstart, size_t heapsize)
{
  FAR struct mm_heap_s *heap;
  uintptr_t             heap_adj;
  int                   i;

  minfo("Heap: name=%s, start=%p size=%zu\n", name, heapstart, heapsize);

  /* First ensure the memory to be used is aligned */

  heap_adj  = MM_ALIGN_UP((uintptr_t)heapstart);
  heapsize -= heap_adj - (uintptr_t)heapstart;

  /* Reserve a block space for mm_heap_s context */

  DEBUGASSERT(heapsize > sizeof(struct mm_heap_s));
  heap = (FAR struct mm_heap_s *)heap_adj;
  heapsize -= sizeof(struct mm_heap_s);
  heapstart = (FAR char *)heap_adj + sizeof(struct mm_heap_s);

  DEBUGASSERT(MM_MIN_CHUNK >= SIZEOF_MM_FREENODE);
  DEBUGASSERT(MM_MIN_CHUNK >= SIZEOF_MM_ALLOCNODE);

  /* Set up global variables */

  memset(heap, 0, sizeof(struct mm_heap_s));

  /* Initialize the node array */

  for (i = 1; i < MM_NNODES; i++)
    {
      heap->mm_nodelist[i - 1].flink = &heap->mm_nodelist[i];
      heap->mm_nodelist[i].blink     = &heap->mm_nodelist[i - 1];
    }

  /* Initialize the malloc semaphore to one (to support one-at-
   * a-time access to private data sets).
   */

  mm_seminitialize(heap);

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
#  if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  heap->mm_procfs.name = name;
  heap->mm_procfs.heap = heap;
#    ifdef CONFIG_MM_BACKTRACE_DEFAULT
  heap->mm_procfs.backtrace = true;
#    endif
#  endif
#endif

  /* Add the initial region of memory to the heap */

  mm_addregion(heap, heapstart, heapsize);

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
#  if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  procfs_register_meminfo(&heap->mm_procfs);
#  endif
#endif

  return heap;
}

/****************************************************************************
 * Name: mm_uninitialize
 *
 * Description:
 *   Uninitialize the selected heap data structures.
 *
 * Input Parameters:
 *   heap - The heap to uninitialize
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mm_uninitialize(FAR struct mm_heap_s *heap)
{
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
#  if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  procfs_unregister_meminfo(&heap->mm_procfs);
#  endif
#endif
  mm_semuninitialize(heap);
}
