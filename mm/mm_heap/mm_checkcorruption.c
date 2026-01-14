/****************************************************************************
 * mm/mm_heap/mm_checkcorruption.c
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
  size_t nodesize = MM_SIZEOF_NODE(node);

  if (MM_NODE_IS_ALLOC(node))
    {
      ASSERT(nodesize >= MM_SIZEOF_ALLOCNODE);
    }
  else
    {
      FAR struct mm_freenode_s *fnode = (FAR void *)node;

      ASSERT(nodesize >= MM_MIN_CHUNK);
      ASSERT(fnode->blink->flink == fnode);
      ASSERT(MM_SIZEOF_NODE(fnode->blink) <= nodesize);
      ASSERT(fnode->flink == NULL ||
             fnode->flink->blink == fnode);
      ASSERT(fnode->flink == NULL ||
             MM_SIZEOF_NODE(fnode->flink) == 0 ||
             MM_SIZEOF_NODE(fnode->flink) >= nodesize);
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
