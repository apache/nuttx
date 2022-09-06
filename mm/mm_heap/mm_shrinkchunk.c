/****************************************************************************
 * mm/mm_heap/mm_shrinkchunk.c
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

#include <nuttx/mm/mm.h>

#include "mm_heap/mm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_shrinkchunk
 *
 * Description:
 *   Reduce the size of the chunk specified by the node structure to the
 *   specified size.  this internal logic is used both from memalign to
 *   dispose of any trailing memory in the aligned allocation and also by
 *   realloc when there is a request to reduce the size of an allocation.
 *
 *   NOTES:
 *     (1) size is the whole chunk size (payload and header)
 *     (2) the caller must hold the MM mutex.
 *
 ****************************************************************************/

void  mm_shrinkchunk(FAR struct mm_heap_s *heap,
                     FAR struct mm_allocnode_s *node, size_t size)
{
  FAR struct mm_freenode_s *next;

  DEBUGASSERT((size & MM_GRAN_MASK) == 0);

  /* Get a reference to the next node */

  next = (FAR struct mm_freenode_s *)((FAR char *)node + node->size);

  /* Check if it is free */

  if ((next->preceding & MM_ALLOC_BIT) == 0)
    {
      FAR struct mm_allocnode_s *andbeyond;
      FAR struct mm_freenode_s *newnode;

      /* Get the chunk next the next node (which could be the tail chunk) */

      andbeyond = (FAR struct mm_allocnode_s *)
                  ((FAR char *)next + next->size);

      /* Remove the next node.  There must be a predecessor, but there may
       * not be a successor node.
       */

      DEBUGASSERT(next->blink);
      next->blink->flink = next->flink;
      if (next->flink)
        {
          next->flink->blink = next->blink;
        }

      /* Create a new chunk that will hold both the next chunk and the
       * tailing memory from the aligned chunk.
       */

      newnode = (FAR struct mm_freenode_s *)((FAR char *)node + size);

      /* Set up the size of the new node */

      newnode->size        = next->size + node->size - size;
      newnode->preceding   = size;
      node->size           = size;
      andbeyond->preceding = newnode->size |
                             (andbeyond->preceding & MM_ALLOC_BIT);

      /* Add the new node to the freenodelist */

      mm_addfreechunk(heap, newnode);
    }

  /* The next chunk is allocated.  Try to free the end portion at the end
   * chunk to be shrunk.
   */

  else if (node->size >= size + SIZEOF_MM_FREENODE)
    {
      FAR struct mm_freenode_s *newnode;

      /* Create a new chunk that will hold both the next chunk and the
       * tailing memory from the aligned chunk.
       */

      newnode = (FAR struct mm_freenode_s *)((FAR char *)node + size);

      /* Set up the size of the new node */

      newnode->size        = node->size - size;
      newnode->preceding   = size;
      node->size           = size;
      next->preceding      = newnode->size | MM_ALLOC_BIT;

      /* Add the new node to the freenodelist */

      mm_addfreechunk(heap, newnode);
    }
}
