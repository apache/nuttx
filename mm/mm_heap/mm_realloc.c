/****************************************************************************
 * mm/mm_heap/mm_realloc.c
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
#include <assert.h>

#include <nuttx/mm/mm.h>

#include "mm_heap/mm.h"
#include "kasan/kasan.h"

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
  FAR struct mm_freenode_s  *prev;
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

  /* If size is zero, then realloc is equivalent to free */

  if (size < 1)
    {
      mm_free(heap, oldmem);
      return NULL;
    }

  /* Adjust the size to account for (1) the size of the allocated node and
   * (2) to make sure that it is an even multiple of our granule size.
   */

  newsize = MM_ALIGN_UP(size + SIZEOF_MM_ALLOCNODE);
  if (newsize < size)
    {
      /* There must have been an integer overflow */

      DEBUGPANIC();
      return NULL;
    }

  /* Map the memory chunk into an allocated node structure */

  oldnode = (FAR struct mm_allocnode_s *)
    ((FAR char *)oldmem - SIZEOF_MM_ALLOCNODE);

  /* We need to hold the MM semaphore while we muck with the nodelist. */

  DEBUGVERIFY(mm_takesemaphore(heap));
  DEBUGASSERT(oldnode->preceding & MM_ALLOC_BIT);
  DEBUGASSERT(mm_heapmember(heap, oldmem));

  /* Check if this is a request to reduce the size of the allocation. */

  oldsize = oldnode->size;
  if (newsize <= oldsize)
    {
      /* Handle the special case where we are not going to change the size
       * of the allocation.
       */

      if (newsize < oldsize)
        {
          mm_shrinkchunk(heap, oldnode, newsize);
          kasan_poison((FAR char *)oldnode + oldnode->size,
                       oldsize - oldnode->size);
        }

      MM_ADD_BACKTRACE(heap, oldnode);

      /* Then return the original address */

      mm_givesemaphore(heap);
      return oldmem;
    }

  /* This is a request to increase the size of the allocation,  Get the
   * available sizes before and after the oldnode so that we can make the
   * best decision
   */

  next = (FAR struct mm_freenode_s *)
    ((FAR char *)oldnode + oldnode->size);
  if ((next->preceding & MM_ALLOC_BIT) == 0)
    {
      nextsize = next->size;
    }

  prev = (FAR struct mm_freenode_s *)
    ((FAR char *)oldnode - (oldnode->preceding & ~MM_ALLOC_BIT));
  if ((prev->preceding & MM_ALLOC_BIT) == 0)
    {
      prevsize = prev->size;
    }

  /* Now, check if we can extend the current allocation or not */

  if (nextsize + prevsize + oldsize >= newsize)
    {
      size_t needed = newsize - oldsize;
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

          DEBUGASSERT(prev->blink);
          prev->blink->flink = prev->flink;
          if (prev->flink)
            {
              prev->flink->blink = prev->blink;
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

              prev->size        -= takeprev;
              DEBUGASSERT(prev->size >= SIZEOF_MM_FREENODE);
              newnode->size      = oldsize + takeprev;
              newnode->preceding = prev->size | MM_ALLOC_BIT;
              next->preceding    = newnode->size |
                                   (next->preceding & MM_ALLOC_BIT);

              /* Return the previous free node to the nodelist
               * (with the new size)
               */

              mm_addfreechunk(heap, prev);
            }
          else
            {
              /* Yes.. update its size (newnode->preceding is already set) */

              newnode->size      += oldsize;
              newnode->preceding |= MM_ALLOC_BIT;
              next->preceding     = newnode->size |
                                    (next->preceding & MM_ALLOC_BIT);
            }

          newmem = (FAR void *)((FAR char *)newnode + SIZEOF_MM_ALLOCNODE);

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

          /* Extend the node into the next chunk */

          oldnode->size += takenext;

          /* Did we consume the entire preceding chunk? */

          if (takenext < nextsize)
            {
              /* No, take what we need from the next chunk and return it to
               * the free nodelist.
               */

              newnode              = (FAR struct mm_freenode_s *)
                                     ((FAR char *)oldnode + oldnode->size);
              newnode->size        = nextsize - takenext;
              DEBUGASSERT(newnode->size >= SIZEOF_MM_FREENODE);
              newnode->preceding   = oldnode->size;
              andbeyond->preceding = newnode->size |
                                     (andbeyond->preceding & MM_ALLOC_BIT);

              /* Add the new free node to the nodelist (with the new size) */

              mm_addfreechunk(heap, newnode);
            }
          else
            {
              /* Yes, just update some pointers. */

              andbeyond->preceding = oldnode->size |
                                     (andbeyond->preceding & MM_ALLOC_BIT);
            }
        }

      MM_ADD_BACKTRACE(heap, (FAR char *)newmem - SIZEOF_MM_ALLOCNODE);

      mm_givesemaphore(heap);

      kasan_unpoison(newmem, mm_malloc_size(newmem));
      if (newmem != oldmem)
        {
          /* Now we have to move the user contents 'down' in memory.  memcpy
           * should be safe for this.
           */

          memcpy(newmem, oldmem, oldsize - SIZEOF_MM_ALLOCNODE);
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

      mm_givesemaphore(heap);
      newmem = mm_malloc(heap, size);
      if (newmem)
        {
          memcpy(newmem, oldmem, oldsize - SIZEOF_MM_ALLOCNODE);
          mm_free(heap, oldmem);
        }

      return newmem;
    }
}
