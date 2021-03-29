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
  FAR struct mm_heap_impl_s *heap_impl;
  FAR struct mm_allocnode_s *node;
  FAR struct mm_allocnode_s *prev;

  DEBUGASSERT(MM_IS_VALID(heap));
  heap_impl = heap->mm_impl;

#if CONFIG_MM_REGIONS > 1
  int region;
#else
# define region 0
#endif

  /* Visit each region */

#if CONFIG_MM_REGIONS > 1
  for (region = 0; region < heap_impl->mm_nregions; region++)
#endif
    {
      irqstate_t flags = 0;

      prev = NULL;

      /* Visit each node in the region
       * Retake the semaphore for each region to reduce latencies
       */

      if (up_interrupt_context() || sched_idletask())
        {
          if (heap_impl->mm_counts_held)
            {
#if CONFIG_MM_REGIONS > 1
              continue;
#else
              return;
#endif
            }

          flags = enter_critical_section();
        }
      else
        {
          mm_takesemaphore(heap);
        }

      for (node = heap_impl->mm_heapstart[region];
           node < heap_impl->mm_heapend[region];
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

      assert(node == heap_impl->mm_heapend[region]);

      if (up_interrupt_context() || sched_idletask())
        {
          leave_critical_section(flags);
        }
      else
        {
          mm_givesemaphore(heap);
        }
    }
}
