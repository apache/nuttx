/****************************************************************************
 * mm/mm_heap/mm_mallinfo.c
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
#include <debug.h>

#include <nuttx/mm/mm.h>

#include "mm_heap/mm.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mm_mallinfo_handler_s
{
  FAR const struct malltask *task;
  FAR struct mallinfo_task *info;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void mallinfo_handler(FAR struct mm_allocnode_s *node, FAR void *arg)
{
  FAR struct mallinfo *info = arg;
  size_t nodesize = MM_SIZEOF_NODE(node);

  minfo("node=%p size=%zu preceding=%u (%c)\n",
        node, nodesize, (unsigned int)node->preceding,
        MM_NODE_IS_ALLOC(node) ? 'A' : 'F');

  /* Check if the node corresponds to an allocated memory chunk */

  if (MM_NODE_IS_ALLOC(node))
    {
      DEBUGASSERT(nodesize >= MM_SIZEOF_ALLOCNODE);
      info->aordblks++;
      info->uordblks += nodesize;
    }
  else
    {
      FAR struct mm_freenode_s *fnode = (FAR void *)node;

      DEBUGASSERT(nodesize >= MM_MIN_CHUNK);
      DEBUGASSERT(fnode->blink->flink == fnode);
      DEBUGASSERT(MM_SIZEOF_NODE(fnode->blink) <= nodesize);
      DEBUGASSERT(fnode->flink == NULL ||
                  fnode->flink->blink == fnode);
      DEBUGASSERT(fnode->flink == NULL ||
                  MM_SIZEOF_NODE(fnode->flink) == 0 ||
                  MM_SIZEOF_NODE(fnode->flink) >= nodesize);

      info->ordblks++;
      info->fordblks += nodesize;
      if (node->size > (size_t)info->mxordblk)
        {
          info->mxordblk = nodesize;
        }
    }
}

static void mallinfo_task_handler(FAR struct mm_allocnode_s *node,
                                  FAR void *arg)
{
  FAR struct mm_mallinfo_handler_s *handler = arg;
  FAR const struct malltask *task = handler->task;
  FAR struct mallinfo_task *info = handler->info;
  size_t nodesize = MM_SIZEOF_NODE(node);

  /* Check if the node corresponds to an allocated memory chunk */

  if (MM_NODE_IS_ALLOC(node))
    {
      DEBUGASSERT(nodesize >= MM_SIZEOF_ALLOCNODE);
      if ((MM_DUMP_ASSIGN(task, node) || MM_DUMP_ALLOC(task, node) ||
           MM_DUMP_LEAK(task, node)) && MM_DUMP_SEQNO(task, node))
        {
          info->aordblks++;
          info->uordblks += nodesize;
        }
    }
  else if (task->pid == PID_MM_FREE)
    {
      info->aordblks++;
      info->uordblks += nodesize;
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

struct mallinfo mm_mallinfo(FAR struct mm_heap_s *heap)
{
  struct mallinfo info;
#ifdef CONFIG_MM_HEAP_MEMPOOL
  struct mallinfo poolinfo;
#endif

  memset(&info, 0, sizeof(info));
  mm_foreach(heap, mallinfo_handler, &info);
  info.arena = heap->mm_heapsize;
  info.arena += sizeof(struct mm_heap_s);
  info.uordblks += sizeof(struct mm_heap_s);
  info.usmblks = heap->mm_maxused + sizeof(struct mm_heap_s);

#ifdef CONFIG_MM_HEAP_MEMPOOL
  poolinfo = mempool_multiple_mallinfo(heap->mm_mpool);

  info.uordblks -= poolinfo.fordblks;
  info.fordblks += poolinfo.fordblks;
#endif

  DEBUGASSERT(info.uordblks + info.fordblks == info.arena);

  return info;
}

/****************************************************************************
 * Name: mm_mallinfo_task
 *
 * Description:
 *   mallinfo returns a copy of updated current heap information for task
 *   with pid.
 *
 ****************************************************************************/

struct mallinfo_task mm_mallinfo_task(FAR struct mm_heap_s *heap,
                                      FAR const struct malltask *task)
{
  struct mm_mallinfo_handler_s handle;
  struct mallinfo_task info =
    {
      0, 0
    };

#ifdef CONFIG_MM_HEAP_MEMPOOL
  info = mempool_multiple_info_task(heap->mm_mpool, task);
#endif

  handle.task = task;
  handle.info = &info;
  mm_foreach(heap, mallinfo_task_handler, &handle);

  return info;
}

/****************************************************************************
 * Name: mm_heapfree
 *
 * Description:
 *   Return the total free size (in bytes) in the heap
 *
 ****************************************************************************/

size_t mm_heapfree(FAR struct mm_heap_s *heap)
{
  return heap->mm_heapsize - heap->mm_curused;
}

/****************************************************************************
 * Name: mm_heapfree_largest
 *
 * Description:
 *   Return the largest chunk of contiguous memory in the heap
 *
 ****************************************************************************/

size_t mm_heapfree_largest(FAR struct mm_heap_s *heap)
{
  FAR struct mm_freenode_s *node;
  for (node = heap->mm_nodelist[MM_NNODES - 1].blink; node;
       node = node->blink)
    {
      size_t nodesize = MM_SIZEOF_NODE(node);
      if (nodesize != 0)
        {
          return nodesize;
        }
    }

  return 0;
}
