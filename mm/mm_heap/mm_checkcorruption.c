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
 * Private Functions
 ****************************************************************************/

static void checkcorruption_handler(FAR struct mm_allocnode_s *node,
                                    FAR void *arg)
{
  size_t nodesize = SIZEOF_MM_NODE(node);

  if ((node->size & MM_ALLOC_BIT) != 0)
    {
      assert(nodesize >= SIZEOF_MM_ALLOCNODE);
    }
  else
    {
      FAR struct mm_freenode_s *fnode = (FAR void *)node;

      assert(nodesize >= SIZEOF_MM_FREENODE);
      assert(fnode->blink->flink == fnode);
      assert(SIZEOF_MM_NODE(fnode->blink) <= nodesize);
      assert(fnode->flink == NULL ||
             fnode->flink->blink == fnode);
      assert(fnode->flink == NULL ||
             SIZEOF_MM_NODE(fnode->flink) == 0 ||
             SIZEOF_MM_NODE(fnode->flink) >= nodesize);
    }
}

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
  mm_foreach(heap, checkcorruption_handler, NULL);
}
