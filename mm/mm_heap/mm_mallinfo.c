/****************************************************************************
 * mm/mm_heap/mm_mallinfo.c
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

#include <malloc.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/mm/mm.h>

#include "mm_heap/mm.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void mallinfo_handler(FAR struct mm_allocnode_s *node, FAR void *arg)
{
  FAR struct mallinfo *info = arg;

  minfo("node=%p size=%u preceding=%u (%c)\n",
        node, (unsigned int)node->size,
        (unsigned int)(node->preceding & ~MM_ALLOC_BIT),
        (node->preceding & MM_ALLOC_BIT) ? 'A' : 'F');

  /* Check if the node corresponds to an allocated memory chunk */

  if ((node->preceding & MM_ALLOC_BIT) != 0)
    {
      DEBUGASSERT(node->size >= SIZEOF_MM_ALLOCNODE);
      info->aordblks++;
      info->uordblks += node->size;
    }
  else
    {
      FAR struct mm_freenode_s *fnode = (FAR void *)node;

      DEBUGASSERT(node->size >= SIZEOF_MM_FREENODE);
      DEBUGASSERT(fnode->blink->flink == fnode);
      DEBUGASSERT(fnode->blink->size <= fnode->size);
      DEBUGASSERT(fnode->flink == NULL ||
                  fnode->flink->blink == fnode);
      DEBUGASSERT(fnode->flink == NULL ||
                  fnode->flink->size == 0 ||
                  fnode->flink->size >= fnode->size);

      info->ordblks++;
      info->fordblks += node->size;
      if (node->size > info->mxordblk)
        {
          info->mxordblk = node->size;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_mallinfo
 *
 * Description:
 *   mallinfo returns a copy of updated current heap information.
 *
 ****************************************************************************/

int mm_mallinfo(FAR struct mm_heap_s *heap, FAR struct mallinfo *info)
{
#if CONFIG_MM_REGIONS > 1
  int region = heap->mm_nregions;
#else
# define region 1
#endif

  DEBUGASSERT(info);

  memset(info, 0, sizeof(*info));
  mm_foreach(heap, mallinfo_handler, info);

  info->arena = heap->mm_heapsize;
  info->uordblks += region * SIZEOF_MM_ALLOCNODE; /* account for the tail node */

  DEBUGASSERT(info->uordblks + info->fordblks == heap->mm_heapsize);

  return OK;
}
