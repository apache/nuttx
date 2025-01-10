/****************************************************************************
 * mm/mm_heap/mm_free.c
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

#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/mm/mm.h>
#include <nuttx/mm/kasan.h>
#include <nuttx/sched_note.h>

#include "mm_heap/mm.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void add_delaylist(FAR struct mm_heap_s *heap, FAR void *mem)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  FAR struct mm_delaynode_s *tmp = mem;
  irqstate_t flags;

  /* Delay the deallocation until a more appropriate time. */

  flags = mm_lock_irq(heap);

#  ifdef CONFIG_DEBUG_ASSERTIONS
  FAR struct mm_freenode_s *node;

  node = (FAR struct mm_freenode_s *)((FAR char *)mem - MM_SIZEOF_ALLOCNODE);
  DEBUGASSERT(MM_NODE_IS_ALLOC(node));
#  endif

  tmp->flink = heap->mm_delaylist[this_cpu()];
  heap->mm_delaylist[this_cpu()] = tmp;

#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  heap->mm_delaycount[this_cpu()]++;
#endif

  mm_unlock_irq(heap, flags);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_delayfree
 *
 * Description:
 *   Delay free memory if `delay` is true, otherwise free it immediately.
 *
 ****************************************************************************/

void mm_delayfree(FAR struct mm_heap_s *heap, FAR void *mem, bool delay)
{
  FAR struct mm_freenode_s *node;
  FAR struct mm_freenode_s *prev;
  FAR struct mm_freenode_s *next;
  size_t nodesize;
  size_t prevsize;

  if (mm_lock(heap) < 0)
    {
      /* Meet -ESRCH return, which means we are in situations
       * during context switching(See mm_lock() & gettid()).
       * Then add to the delay list.
       */

      add_delaylist(heap, mem);
      return;
    }

  nodesize = mm_malloc_size(heap, mem);
#ifdef CONFIG_MM_FILL_ALLOCATIONS
#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  /* If delay free is enabled, a memory node will be freed twice.
   * The first time is to add the node to the delay list, and the second
   * time is to actually free the node. Therefore, we only colorize the
   * memory node the first time, when `delay` is set to true.
   */

  if (delay)
#endif
    {
      memset(mem, MM_FREE_MAGIC, nodesize);
    }
#endif

  kasan_poison(mem, nodesize);

  if (delay)
    {
      mm_unlock(heap);
      add_delaylist(heap, mem);
      return;
    }

  /* Map the memory chunk into a free node */

  node = (FAR struct mm_freenode_s *)
         ((FAR char *)kasan_reset_tag(mem) - MM_SIZEOF_ALLOCNODE);
  nodesize = MM_SIZEOF_NODE(node);

  /* Sanity check against double-frees */

  DEBUGASSERT(MM_NODE_IS_ALLOC(node));

  node->size &= ~MM_ALLOC_BIT;

  /* Update heap statistics */

  heap->mm_curused -= nodesize;
  sched_note_heap(NOTE_HEAP_FREE, heap, mem, nodesize, heap->mm_curused);

  /* Check if the following node is free and, if so, merge it */

  next = (FAR struct mm_freenode_s *)((FAR char *)node + nodesize);
  DEBUGASSERT(MM_PREVNODE_IS_ALLOC(next));
  if (MM_NODE_IS_FREE(next))
    {
      FAR struct mm_allocnode_s *andbeyond;
      size_t nextsize = MM_SIZEOF_NODE(next);

      /* Get the node following the next node (which will
       * become the new next node). We know that we can never
       * index past the tail chunk because it is always allocated.
       */

      andbeyond = (FAR struct mm_allocnode_s *)((FAR char *)next + nextsize);
      DEBUGASSERT(MM_PREVNODE_IS_FREE(andbeyond) &&
                  andbeyond->preceding == nextsize);

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
  else
    {
      next->size     |= MM_PREVFREE_BIT;
      next->preceding = nodesize;
    }

  /* Check if the preceding node is also free and, if so, merge
   * it with this node
   */

  if (MM_PREVNODE_IS_FREE(node))
    {
      prev = (FAR struct mm_freenode_s *)
        ((FAR char *)node - node->preceding);
      prevsize = MM_SIZEOF_NODE(prev);
      DEBUGASSERT(MM_NODE_IS_FREE(prev) && node->preceding == prevsize);

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
  minfo("Freeing %p\n", mem);

  /* Protect against attempts to free a NULL reference */

  if (mem == NULL)
    {
      return;
    }

  DEBUGASSERT(mm_heapmember(heap, mem));

#ifdef CONFIG_MM_HEAP_MEMPOOL
  if (heap->mm_mpool)
    {
      if (mempool_multiple_free(heap->mm_mpool, mem) >= 0)
        {
          return;
        }
    }
#endif

  mm_delayfree(heap, mem, CONFIG_MM_FREE_DELAYCOUNT_MAX > 0);
}
