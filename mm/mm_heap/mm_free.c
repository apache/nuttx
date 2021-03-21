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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
static void mm_add_delaylist(FAR struct mm_heap_s *heap, FAR void *mem)
{
  FAR struct mm_heap_impl_s *heap_impl;
  FAR struct mm_delaynode_s *tmp = mem;
  irqstate_t flags;

  DEBUGASSERT(MM_IS_VALID(heap));
  heap_impl = heap->mm_impl;

  /* Delay the deallocation until a more appropriate time. */

  flags = enter_critical_section();

  tmp->flink = heap_impl->mm_delaylist;
  heap_impl->mm_delaylist = tmp;

  leave_critical_section(flags);
}
#endif

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
  int ret;

  UNUSED(ret);
  minfo("Freeing %p\n", mem);

  /* Protect against attempts to free a NULL reference */

  if (!mem)
    {
      return;
    }

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  /* Check current environment */

  if (up_interrupt_context())
    {
      /* We are in ISR, add to mm_delaylist */

      mm_add_delaylist(heap, mem);
      return;
    }
  else if ((ret = mm_trysemaphore(heap)) == 0)
    {
      /* Got the sem, do free immediately */
    }
  else if (ret == -ESRCH || sched_idletask())
    {
      /* We are in IDLE task & can't get sem, or meet -ESRCH return,
       * which means we are in situations during context switching(See
       * mm_trysemaphore() & getpid()). Then add to mm_delaylist.
       */

      mm_add_delaylist(heap, mem);
      return;
    }
  else
#endif
    {
      /* We need to hold the MM semaphore while we muck with the
       * nodelist.
       */

      mm_takesemaphore(heap);
    }

  DEBUGASSERT(mm_heapmember(heap, mem));

  /* Map the memory chunk into a free node */

  node = (FAR struct mm_freenode_s *)((FAR char *)mem - SIZEOF_MM_ALLOCNODE);

  /* Sanity check against double-frees */

  DEBUGASSERT(node->preceding & MM_ALLOC_BIT);

  node->preceding &= ~MM_ALLOC_BIT;

  /* Check if the following node is free and, if so, merge it */

  next = (FAR struct mm_freenode_s *)((FAR char *)node + node->size);
  DEBUGASSERT((next->preceding & ~MM_ALLOC_BIT) == node->size);
  if ((next->preceding & MM_ALLOC_BIT) == 0)
    {
      FAR struct mm_allocnode_s *andbeyond;

      /* Get the node following the next node (which will
       * become the new next node). We know that we can never
       * index past the tail chunk because it is always allocated.
       */

      andbeyond = (FAR struct mm_allocnode_s *)
                    ((FAR char *)next + next->size);

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

      node->size          += next->size;
      andbeyond->preceding =  node->size |
                              (andbeyond->preceding & MM_ALLOC_BIT);
      next                 = (FAR struct mm_freenode_s *)andbeyond;
    }

  /* Check if the preceding node is also free and, if so, merge
   * it with this node
   */

  prev = (FAR struct mm_freenode_s *)((FAR char *)node - node->preceding);
  DEBUGASSERT((node->preceding & ~MM_ALLOC_BIT) == prev->size);
  if ((prev->preceding & MM_ALLOC_BIT) == 0)
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

      prev->size     += node->size;
      next->preceding = prev->size | (next->preceding & MM_ALLOC_BIT);
      node            = prev;
    }

  /* Add the merged node to the nodelist */

  mm_addfreechunk(heap, node);
  mm_givesemaphore(heap);
}
