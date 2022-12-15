/****************************************************************************
 * mm/mm_heap/mm_free.c
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

#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>

#include "mm_heap/mm.h"
#include "kasan/kasan.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void add_delaylist(FAR struct mm_heap_s *heap, FAR void *mem)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  FAR struct mm_delaynode_s *tmp = mem;
  irqstate_t flags;

  /* Delay the deallocation until a more appropriate time. */

  flags = enter_critical_section();

  tmp->flink = heap->mm_delaylist[up_cpu_index()];
  heap->mm_delaylist[up_cpu_index()] = tmp;

  leave_critical_section(flags);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_free
 *
 * Description:
 *   Returns a chunk of memory to the list of free nodes,  merging with
 *   adjacent free chunks if possible.
 *
 ****************************************************************************/

void mm_free(FAR struct mm_heap_s *heap, FAR void *mem)
{
  FAR struct mm_freenode_s *node;
  FAR struct mm_freenode_s *prev;
  FAR struct mm_freenode_s *next;
  size_t nodesize;
  size_t prevsize;

  minfo("Freeing %p\n", mem);

  /* Protect against attempts to free a NULL reference */

  if (!mem)
    {
      return;
    }

#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
  if (mempool_multiple_free(heap->mm_mpool, mem) >= 0)
    {
      return;
    }
#endif

  if (mm_lock(heap) < 0)
    {
      /* Meet -ESRCH return, which means we are in situations
       * during context switching(See mm_lock() & gettid()).
       * Then add to the delay list.
       */

      add_delaylist(heap, mem);
      return;
    }

  kasan_poison(mem, mm_malloc_size(heap, mem));

  DEBUGASSERT(mm_heapmember(heap, mem));

  /* Map the memory chunk into a free node */

  node = (FAR struct mm_freenode_s *)((FAR char *)mem - SIZEOF_MM_ALLOCNODE);
  nodesize = SIZEOF_MM_NODE(node);

  /* Sanity check against double-frees */

  DEBUGASSERT(node->size & MM_ALLOC_BIT);

  node->size &= ~MM_ALLOC_BIT;

  /* Check if the following node is free and, if so, merge it */

  next = (FAR struct mm_freenode_s *)((FAR char *)node + nodesize);
  DEBUGASSERT(next->preceding == nodesize);
  if ((next->size & MM_ALLOC_BIT) == 0)
    {
      FAR struct mm_allocnode_s *andbeyond;
      size_t nextsize = SIZEOF_MM_NODE(next);

      /* Get the node following the next node (which will
       * become the new next node). We know that we can never
       * index past the tail chunk because it is always allocated.
       */

      andbeyond = (FAR struct mm_allocnode_s *)((FAR char *)next + nextsize);

      /* Remove the next node.  There must be a predecessor,
       * but there may not be a successor node.
       */

      DEBUGASSERT(next->blink);
      next->blink->flink = next->flink;
      if (next->flink)
        {
          next->flink->blink = next->blink;
        }

      /* Then merge the two chunks */

      nodesize            += nextsize;
      node->size           = nodesize | (node->size & MM_MASK_BIT);
      andbeyond->preceding = nodesize;
      next                 = (FAR struct mm_freenode_s *)andbeyond;
    }

  /* Check if the preceding node is also free and, if so, merge
   * it with this node
   */

  prev = (FAR struct mm_freenode_s *)((FAR char *)node - node->preceding);
  prevsize = SIZEOF_MM_NODE(prev);
  DEBUGASSERT(node->preceding == prevsize);
  if ((prev->size & MM_ALLOC_BIT) == 0)
    {
      /* Remove the node.  There must be a predecessor, but there may
       * not be a successor node.
       */

      DEBUGASSERT(prev->blink);
      prev->blink->flink = prev->flink;
      if (prev->flink)
        {
          prev->flink->blink = prev->blink;
        }

      /* Then merge the two chunks */

      prevsize       += nodesize;
      prev->size      = prevsize | (prev->size & MM_MASK_BIT);
      next->preceding = prevsize;
      node            = prev;
    }

  /* Add the merged node to the nodelist */

  mm_addfreechunk(heap, node);
  mm_unlock(heap);
}
