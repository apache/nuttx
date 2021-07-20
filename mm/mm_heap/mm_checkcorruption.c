/****************************************************************************
 * mm/mm_heap/mm_checkcorruption.c
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
#include <sched.h>

#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>
#include <nuttx/irq.h>

#include "mm_heap/mm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_checkcorruption
 *
 * Description:
 *   mm_checkcorruption is used to check whether memory heap is normal.
 *
 ****************************************************************************/

void mm_checkcorruption(FAR struct mm_heap_s *heap)
{
  FAR struct mm_allocnode_s *node;
  FAR struct mm_allocnode_s *prev;

#if CONFIG_MM_REGIONS > 1
  int region;
#else
# define region 0
#endif

  /* Visit each region */

#if CONFIG_MM_REGIONS > 1
  for (region = 0; region < heap->mm_nregions; region++)
#endif
    {
      prev = NULL;

      /* Visit each node in the region
       * Retake the semaphore for each region to reduce latencies
       */

      if (mm_takesemaphore(heap) == false)
        {
          return;
        }

      for (node = heap->mm_heapstart[region];
           node < heap->mm_heapend[region];
           node = (FAR struct mm_allocnode_s *)
                  ((FAR char *)node + node->size))
        {
          if ((node->preceding & MM_ALLOC_BIT) != 0)
            {
              assert(node->size >= SIZEOF_MM_ALLOCNODE);
            }
          else
            {
              FAR struct mm_freenode_s *fnode = (FAR void *)node;

              assert(node->size >= SIZEOF_MM_FREENODE);
              assert(fnode->blink->flink == fnode);
              assert(fnode->blink->size <= fnode->size);
              assert(fnode->flink == NULL ||
                     fnode->flink->blink == fnode);
              assert(fnode->flink == NULL ||
                     fnode->flink->size == 0 ||
                     fnode->flink->size >= fnode->size);
            }

          assert(prev == NULL ||
                 prev->size == (node->preceding & ~MM_ALLOC_BIT));
          prev = node;
        }

      assert(node == heap->mm_heapend[region]);

      mm_givesemaphore(heap);
    }
}
