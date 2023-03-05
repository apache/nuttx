/****************************************************************************
 * mm/mm_heap/mm_addfreechunk.c
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
 * Name: mm_addfreechunk
 *
 * Description:
 *   Add a free chunk to the nodes list.  It is assumed that the caller holds
 *   the mm mutex.
 *
 ****************************************************************************/

void mm_addfreechunk(FAR struct mm_heap_s *heap,
                     FAR struct mm_freenode_s *node)
{
  FAR struct mm_freenode_s *next;
  FAR struct mm_freenode_s *prev;
  size_t nodesize = SIZEOF_MM_NODE(node);
  int ndx;

  DEBUGASSERT(nodesize >= MM_MIN_CHUNK);
  DEBUGASSERT((node->size & MM_ALLOC_BIT) == 0);

  /* Convert the size to a nodelist index */

  ndx = mm_size2ndx(nodesize);

  /* Now put the new node into the next */

  for (prev = &heap->mm_nodelist[ndx],
       next = heap->mm_nodelist[ndx].flink;
       next && next->size && SIZEOF_MM_NODE(next) < nodesize;
       prev = next, next = next->flink);

  /* Does it go in mid next or at the end? */

  prev->flink = node;
  node->blink = prev;
  node->flink = next;

  if (next)
    {
      /* The new node goes between prev and next */

      next->blink = node;
    }
}
