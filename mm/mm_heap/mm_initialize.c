/****************************************************************************
 * mm/mm_heap/mm_initialize.c
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

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/sched_note.h>
#include <nuttx/mm/mm.h>
#include <nuttx/mm/kasan.h>

#include "mm_heap/mm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD > 0
#  define MEMPOOL_NPOOLS (CONFIG_MM_HEAP_MEMPOOL_THRESHOLD / MM_MIN_CHUNK)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_MM_HEAP_MEMPOOL) && CONFIG_MM_BACKTRACE >= 0

/****************************************************************************
 * Name: mempool_memalign
 *
 * Description:
 *   This function call mm_memalign and set mm_backtrace pid to free pid
 *   avoid repeated calculation.
 ****************************************************************************/

static FAR void *mempool_memalign(FAR void *arg, size_t alignment,
                                  size_t size)
{
  FAR struct mm_allocnode_s *node;
  FAR void *ret;

  ret = mm_memalign(arg, alignment, size);
  if (ret)
    {
      node = (FAR struct mm_allocnode_s *)
      ((uintptr_t)ret - MM_SIZEOF_ALLOCNODE);
      node->pid = PID_MM_MEMPOOL;
    }

  return ret;
}
#else
#  define mempool_memalign mm_memalign
#endif

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
  int idx;

  DEBUGVERIFY(mm_lock(heap));
  idx = heap->mm_nregions;

  /* Writing past CONFIG_MM_REGIONS would have catastrophic consequences */

  DEBUGASSERT(idx < CONFIG_MM_REGIONS);
  if (idx >= CONFIG_MM_REGIONS)
    {
      mm_unlock(heap);
      return;
    }

#else
#  define idx 0
  DEBUGVERIFY(mm_lock(heap));
#endif

#if defined(CONFIG_MM_SMALL) && !defined(CONFIG_SMALL_MEMORY)
  /* If the MCU handles wide addresses but the memory manager is configured
   * for a small heap, then verify that the caller is  not doing something
   * crazy.
   */

  DEBUGASSERT(heapsize <= MMSIZE_MAX + 1);
#endif

#ifdef CONFIG_MM_FILL_ALLOCATIONS
  /* Use the fill value to mark uninitialized user memory */

  memset(heapstart, MM_INIT_MAGIC, heapsize);
#endif

  /* Adjust the provided heap start and size.
   *
   * Note: (uintptr_t)node + MM_SIZEOF_ALLOCNODE is what's actually
   * returned to the malloc user, which should have natural alignment.
   * (that is, in this implementation, MM_MIN_CHUNK-alignment.)
   */

  heapbase = MM_ALIGN_UP((uintptr_t)heapstart + 2 * MM_SIZEOF_ALLOCNODE) -
             2 * MM_SIZEOF_ALLOCNODE;
  heapsize = heapsize - (heapbase - (uintptr_t)heapstart);

  /* Register KASan for access rights check. We need to register after
   * address alignment.
   */

  kasan_register((void *)heapbase, &heapsize);

  heapend  = MM_ALIGN_DOWN((uintptr_t)heapbase + (uintptr_t)heapsize);
  heapsize = heapend - heapbase;

#if defined(CONFIG_FS_PROCFS) && \
    !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO) && \
    (defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__))
  minfo("[%s] Region %d: base=%p size=%zu\n",
        heap->mm_procfs.name, idx + 1, heapstart, heapsize);
#else
  minfo("Region %d: base=%p size=%zu\n", idx + 1, heapstart, heapsize);
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

  heap->mm_heapstart[idx]          = (FAR struct mm_allocnode_s *)heapbase;
  MM_ADD_BACKTRACE(heap, heap->mm_heapstart[idx]);
  heap->mm_heapstart[idx]->size    = MM_SIZEOF_ALLOCNODE | MM_ALLOC_BIT;
  node                             = (FAR struct mm_freenode_s *)
                                     (heapbase + MM_SIZEOF_ALLOCNODE);
  DEBUGASSERT((((uintptr_t)node + MM_SIZEOF_ALLOCNODE) % MM_ALIGN) == 0);
  node->size                       = heapsize - 2 * MM_SIZEOF_ALLOCNODE;
  heap->mm_heapend[idx]            = (FAR struct mm_allocnode_s *)
                                     (heapend - MM_SIZEOF_ALLOCNODE);
  heap->mm_heapend[idx]->size      = MM_SIZEOF_ALLOCNODE | MM_ALLOC_BIT |
                                     MM_PREVFREE_BIT;
  heap->mm_heapend[idx]->preceding = node->size;
  MM_ADD_BACKTRACE(heap, heap->mm_heapend[idx]);

#undef idx

#if CONFIG_MM_REGIONS > 1
  heap->mm_nregions++;
#endif

  /* Add the single, large free node to the nodelist */

  mm_addfreechunk(heap, node);
  heap->mm_curused += 2 * MM_SIZEOF_ALLOCNODE;
  sched_note_heap(NOTE_HEAP_ADD, heap, heapstart, heapsize,
                  heap->mm_curused);
  mm_unlock(heap);
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

  DEBUGASSERT(MM_MIN_CHUNK >= MM_SIZEOF_ALLOCNODE);

  /* Set up global variables */

  memset(heap, 0, sizeof(struct mm_heap_s));

  /* Initialize the node array */

  for (i = 1; i < MM_NNODES; i++)
    {
      heap->mm_nodelist[i - 1].flink = &heap->mm_nodelist[i];
      heap->mm_nodelist[i].blink     = &heap->mm_nodelist[i - 1];
    }

  /* Initialize the malloc mutex to one (to support one-at-
   * a-time access to private data sets).
   */

  nxmutex_init(&heap->mm_lock);

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

  heap->mm_curused = sizeof(struct mm_heap_s);
  mm_addregion(heap, heapstart, heapsize);

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
#  if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  procfs_register_meminfo(&heap->mm_procfs);
#  endif
#endif

  return heap;
}

#ifdef CONFIG_MM_HEAP_MEMPOOL
FAR struct mm_heap_s *
mm_initialize_pool(FAR const char *name,
                   FAR void *heap_start, size_t heap_size,
                   FAR const struct mempool_init_s *init)
{
  FAR struct mm_heap_s *heap;

#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD > 0
  size_t poolsize[MEMPOOL_NPOOLS];
  struct mempool_init_s def;

  if (init == NULL)
    {
      /* Initialize the multiple mempool default parameter */

      int i;

      for (i = 0; i < MEMPOOL_NPOOLS; i++)
        {
#  if CONFIG_MM_MIN_BLKSIZE != 0
          poolsize[i] = (i + 1) * CONFIG_MM_MIN_BLKSIZE;
#  else
          poolsize[i] = (i + 1) * MM_MIN_CHUNK;
#  endif
        }

      def.poolsize        = poolsize;
      def.npools          = MEMPOOL_NPOOLS;
      def.threshold       = CONFIG_MM_HEAP_MEMPOOL_THRESHOLD;
      def.chunksize       = CONFIG_MM_HEAP_MEMPOOL_CHUNK_SIZE;
      def.expandsize      = CONFIG_MM_HEAP_MEMPOOL_EXPAND_SIZE;
      def.dict_expendsize = CONFIG_MM_HEAP_MEMPOOL_DICTIONARY_EXPAND_SIZE;

      init = &def;
    }
#endif

  heap = mm_initialize(name, heap_start, heap_size);

  /* Initialize the multiple mempool in heap */

  if (init != NULL && init->poolsize != NULL && init->npools != 0)
    {
      heap->mm_threshold = init->threshold;
      heap->mm_mpool     = mempool_multiple_init(name, init->poolsize,
                               init->npools,
                               (mempool_multiple_alloc_t)mempool_memalign,
                               (mempool_multiple_alloc_size_t)mm_malloc_size,
                               (mempool_multiple_free_t)mm_free, heap,
                               init->chunksize, init->expandsize,
                               init->dict_expendsize);
    }

  return heap;
}
#endif

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
  int i;

#ifdef CONFIG_MM_HEAP_MEMPOOL
  mempool_multiple_deinit(heap->mm_mpool);
#endif

  for (i = 0; i < CONFIG_MM_REGIONS; i++)
    {
      kasan_unregister(heap->mm_heapstart[i]);
      sched_note_heap(NOTE_HEAP_REMOVE, heap, heap->mm_heapstart[i],
                      (uintptr_t)heap->mm_heapend[i] -
                      (uintptr_t)heap->mm_heapstart[i], heap->mm_curused);
    }

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
#  if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  procfs_unregister_meminfo(&heap->mm_procfs);
#  endif
#endif
  nxmutex_destroy(&heap->mm_lock);
}
