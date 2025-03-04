/****************************************************************************
 * mm/mm_heap/mm_realloc.c
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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/param.h>
#include <assert.h>

#include <nuttx/mm/mm.h>
#include <nuttx/mm/kasan.h>
#include <nuttx/sched_note.h>

#include "mm_heap/mm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_realloc
 *
 * Description:
 *   If the reallocation is for less space, then:
 *
 *     (1) the current allocation is reduced in size
 *     (2) the remainder at the end of the allocation is returned to the
 *         free list.
 *
 *  If the request is for more space and the current allocation can be
 *  extended, it will be extended by:
 *
 *     (1) Taking the additional space from the following free chunk, or
 *     (2) Taking the additional space from the preceding free chunk.
 *     (3) Or both
 *
 *  If the request is for more space but the current chunk cannot be
 *  extended, then malloc a new buffer, copy the data into the new buffer,
 *  and free the old buffer.
 *
 ****************************************************************************/

FAR void *mm_realloc(FAR struct mm_heap_s *heap, FAR void *oldmem,
                     size_t size)
{
  FAR struct mm_allocnode_s *oldnode;
  FAR struct mm_freenode_s  *prev = NULL;
  FAR struct mm_freenode_s  *next;
  size_t newsize;
  size_t oldsize;
  size_t prevsize = 0;
  size_t nextsize = 0;
  FAR void *newmem;

  /* If oldmem is NULL, then realloc is equivalent to malloc */

  if (oldmem == NULL)
    {
      return mm_malloc(heap, size);
    }

  DEBUGASSERT(mm_heapmember(heap, oldmem));

#ifdef CONFIG_MM_HEAP_MEMPOOL
  if (heap->mm_mpool)
    {
      newmem = mempool_multiple_realloc(heap->mm_mpool, oldmem, size);
      if (newmem != NULL)
        {
          return newmem;
        }
      else if (size <= heap->mm_threshold ||
               mempool_multiple_alloc_size(heap->mm_mpool, oldmem) >= 0)
        {
          newmem = mm_malloc(heap, size);
          if (newmem != NULL)
            {
              memcpy(newmem, oldmem,
                     MIN(size, mm_malloc_size(heap, oldmem)));
              mm_free(heap, oldmem);
            }

          return newmem;
        }
    }
#endif

  /* Adjust the size to account for (1) the size of the allocated node and
   * (2) to make sure that it is aligned with MM_ALIGN and its size is at
   * least MM_MIN_CHUNK.
   */

  if (size < MM_MIN_CHUNK - MM_ALLOCNODE_OVERHEAD)
    {
      size = MM_MIN_CHUNK - MM_ALLOCNODE_OVERHEAD;
    }

  newsize = MM_ALIGN_UP(size + MM_ALLOCNODE_OVERHEAD);
  if (newsize < size)
    {
      /* There must have been an integer overflow */

      DEBUGPANIC();
      return NULL;
    }

  /* Map the memory chunk into an allocated node structure */

  oldnode = (FAR struct mm_allocnode_s *)
    ((FAR char *)kasan_reset_tag(oldmem) - MM_SIZEOF_ALLOCNODE);

  /* We need to hold the MM mutex while we muck with the nodelist. */

  DEBUGVERIFY(mm_lock(heap));
  DEBUGASSERT(MM_NODE_IS_ALLOC(oldnode));

  /* Check if this is a request to reduce the size of the allocation. */

  oldsize = MM_SIZEOF_NODE(oldnode);
  if (newsize <= oldsize)
    {
      /* Handle the special case where we are not going to change the size
       * of the allocation.
       */

      if (newsize < oldsize)
        {
          heap->mm_curused += newsize - oldsize;
          mm_shrinkchunk(heap, oldnode, newsize);
          kasan_poison((FAR char *)oldnode + MM_SIZEOF_NODE(oldnode) +
                       sizeof(mmsize_t), oldsize - MM_SIZEOF_NODE(oldnode));
        }

      /* Then return the original address */

      mm_unlock(heap);
      MM_ADD_BACKTRACE(heap, oldnode);

      return oldmem;
    }

  /* This is a request to increase the size of the allocation,  Get the
   * available sizes before and after the oldnode so that we can make the
   * best decision
   */

  next = (FAR struct mm_freenode_s *)((FAR char *)oldnode + oldsize);
  if (MM_NODE_IS_FREE(next))
    {
      DEBUGASSERT(MM_PREVNODE_IS_ALLOC(next));
      nextsize = MM_SIZEOF_NODE(next);
    }

  if (MM_PREVNODE_IS_FREE(oldnode))
    {
      prev = (FAR struct mm_freenode_s *)
        ((FAR char *)oldnode - oldnode->preceding);
      DEBUGASSERT(MM_NODE_IS_FREE(prev));
      prevsize = MM_SIZEOF_NODE(prev);
    }

  /* Now, check if we can extend the current allocation or not */

  if (nextsize + prevsize + oldsize >= newsize)
    {
      size_t needed = newsize - oldsize;
      size_t nodesize = oldsize;
      size_t takeprev;
      size_t takenext;

      /* Check if we can extend into the previous chunk and if the
       * previous chunk is smaller than the next chunk.
       */

      if (nextsize > prevsize)
        {
          /* Can we get everything we need from the previous chunk? */

          if (needed > prevsize)
            {
              /* No, take the whole previous chunk and get the
               * rest that we need from the next chunk.
               */

              takeprev = prevsize;
              takenext = needed - prevsize;
            }
          else
            {
              /* Yes, take what we need from the previous chunk */

              takeprev = needed;
              takenext = 0;
            }
        }

      /* Check if we can extend into the next chunk and if we still need
       * more memory.
       */

      else
        {
          /* Can we get everything we need from the next chunk? */

          if (needed > nextsize)
            {
              /* No, take the whole next chunk and get the rest that we
               * need from the previous chunk.
               */

              takeprev = needed - nextsize;
              takenext = nextsize;
            }
          else
            {
              /* Yes, take what we need from the previous chunk */

              takeprev = 0;
              takenext = needed;
            }
        }

      /* Extend into the previous free chunk */

      newmem = oldmem;
      if (takeprev)
        {
          FAR struct mm_allocnode_s *newnode;

          /* Remove the previous node.  There must be a predecessor, but
           * there may not be a successor node.
           */

          DEBUGASSERT(prev && prev->blink);
          prev->blink->flink = prev->flink;
          if (prev->flink)
            {
              prev->flink->blink = prev->blink;
            }

          /* Make sure the new previous node has enough space */

          if (prevsize < takeprev + MM_MIN_CHUNK)
            {
              heap->mm_curused += prevsize - takeprev;
              takeprev          = prevsize;
            }

          /* Extend the node into the previous free chunk */

          newnode = (FAR struct mm_allocnode_s *)
            ((FAR char *)oldnode - takeprev);

          /* Did we consume the entire preceding chunk? */

          if (takeprev < prevsize)
            {
              /* No.. just take what we need from the previous chunk and put
               * it back into the free list
               */

              prevsize          -= takeprev;
              prev->size         = prevsize | (prev->size & MM_MASK_BIT);
              nodesize          += takeprev;
              newnode->size      = nodesize | MM_ALLOC_BIT | MM_PREVFREE_BIT;
              newnode->preceding = prevsize;

              /* Return the previous free node to the nodelist
               * (with the new size)
               */

              mm_addfreechunk(heap, prev);
            }
          else
            {
              /* Yes.. update its size (newnode->preceding is already set) */

              nodesize     += prevsize;
              newnode->size = nodesize | MM_ALLOC_BIT |
                              (newnode->size & MM_MASK_BIT);
            }

          newmem = (FAR void *)((FAR char *)newnode + MM_SIZEOF_ALLOCNODE);

          /* Now we want to return newnode */

          oldnode = newnode;
        }

      /* Extend into the next free chunk */

      if (takenext)
        {
          FAR struct mm_freenode_s *newnode;
          FAR struct mm_allocnode_s *andbeyond;

          /* Get the chunk following the next node (which could be the tail
           * chunk)
           */

          andbeyond = (FAR struct mm_allocnode_s *)
                      ((FAR char *)next + nextsize);

          /* Remove the next node.  There must be a predecessor, but there
           * may not be a successor node.
           */

          DEBUGASSERT(next->blink);
          next->blink->flink = next->flink;
          if (next->flink)
            {
              next->flink->blink = next->blink;
            }

          /* Make sure the new next node has enough space */

          if (nextsize < takenext + MM_MIN_CHUNK)
            {
              heap->mm_curused += nextsize - takenext;
              takenext          = nextsize;
            }

          /* Extend the node into the next chunk */

          nodesize += takenext;
          oldnode->size = nodesize | (oldnode->size & MM_MASK_BIT);

          /* Did we consume the entire preceding chunk? */

          if (takenext < nextsize)
            {
              /* No, take what we need from the next chunk and return it to
               * the free nodelist.
               */

              newnode              = (FAR struct mm_freenode_s *)
                                     ((FAR char *)oldnode + nodesize);
              newnode->size        = nextsize - takenext;
              andbeyond->preceding = newnode->size;

              /* Add the new free node to the nodelist (with the new size) */

              mm_addfreechunk(heap, newnode);
            }
          else
            {
              /* Yes, just update some pointers. */

              andbeyond->size &= ~MM_PREVFREE_BIT;
            }
        }

      /* Update heap statistics */

      heap->mm_curused += newsize - oldsize;
      if (heap->mm_curused > heap->mm_maxused)
        {
          heap->mm_maxused = heap->mm_curused;
        }

      sched_note_heap(NOTE_HEAP_FREE, heap, oldmem, oldsize,
                      heap->mm_curused - newsize);
      sched_note_heap(NOTE_HEAP_ALLOC, heap, newmem, newsize,
                      heap->mm_curused);
      mm_unlock(heap);
      MM_ADD_BACKTRACE(heap, (FAR char *)newmem - MM_SIZEOF_ALLOCNODE);

      newmem = kasan_unpoison(newmem, MM_SIZEOF_NODE(oldnode) -
                              MM_ALLOCNODE_OVERHEAD);
      if (kasan_reset_tag(newmem) != kasan_reset_tag(oldmem))
        {
          /* Now we have to move the user contents 'down' in memory.  memcpy
           * should be safe for this.
           */

          memcpy(newmem, oldmem, oldsize - MM_ALLOCNODE_OVERHEAD);
        }

      return newmem;
    }

  /* The current chunk cannot be extended.
   * Just allocate a new chunk and copy
   */

  else
    {
      /* Allocate a new block.  On failure, realloc must return NULL but
       * leave the original memory in place.
       */

      mm_unlock(heap);
      newmem = mm_malloc(heap, size);
      if (newmem)
        {
          memcpy(newmem, oldmem, oldsize - MM_ALLOCNODE_OVERHEAD);
          mm_free(heap, oldmem);
        }

      return newmem;
    }
}
