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
  FAR struct mm_allocnode_s *node;
  FAR struct mm_allocnode_s *prev;
  size_t mxordblk = 0;
  int    ordblks  = 0;  /* Number of non-inuse chunks */
  int    aordblks = 0;  /* Number of inuse chunks */
  size_t uordblks = 0;  /* Total allocated space */
  size_t fordblks = 0;  /* Total non-inuse space */
#if CONFIG_MM_REGIONS > 1
  int region;
#else
# define region 0
#endif

  DEBUGASSERT(info);

  /* Visit each region */

#if CONFIG_MM_REGIONS > 1
  for (region = 0; region < heap->mm_nregions; region++)
#endif
    {
      prev = NULL;

      /* Visit each node in the region
       * Retake the semaphore for each region to reduce latencies
       */

      DEBUGVERIFY(mm_takesemaphore(heap));

      for (node = heap->mm_heapstart[region];
           node < heap->mm_heapend[region];
           node = (FAR struct mm_allocnode_s *)
                  ((FAR char *)node + node->size))
        {
          minfo("region=%d node=%p size=%u preceding=%u (%c)\n",
                region, node, (unsigned int)node->size,
                (unsigned int)(node->preceding & ~MM_ALLOC_BIT),
                (node->preceding & MM_ALLOC_BIT) ? 'A' : 'F');

          /* Check if the node corresponds to an allocated memory chunk */

          if ((node->preceding & MM_ALLOC_BIT) != 0)
            {
              DEBUGASSERT(node->size >= SIZEOF_MM_ALLOCNODE);
              aordblks++;
              uordblks += node->size;
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
              ordblks++;
              fordblks += node->size;
              if (node->size > mxordblk)
                {
                  mxordblk = node->size;
                }
            }

          DEBUGASSERT(prev == NULL ||
                      prev->size == (node->preceding & ~MM_ALLOC_BIT));
          prev = node;
        }

      minfo("region=%d node=%p heapend=%p\n",
            region, node, heap->mm_heapend[region]);
      DEBUGASSERT(node == heap->mm_heapend[region]);

      mm_givesemaphore(heap);

      uordblks += SIZEOF_MM_ALLOCNODE; /* account for the tail node */
    }
#undef region

  DEBUGASSERT(uordblks + fordblks == heap->mm_heapsize);

  info->arena    = heap->mm_heapsize;
  info->ordblks  = ordblks;
  info->aordblks = aordblks;
  info->mxordblk = mxordblk;
  info->uordblks = uordblks;
  info->fordblks = fordblks;
  return OK;
}
