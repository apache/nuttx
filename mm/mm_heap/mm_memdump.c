/****************************************************************************
 * mm/mm_heap/mm_memdump.c
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

#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/mm/mm.h>

#include "mm_heap/mm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if UINTPTR_MAX <= UINT32_MAX
#  define MM_PTR_FMT_WIDTH 11
#elif UINTPTR_MAX <= UINT64_MAX
#  define MM_PTR_FMT_WIDTH 19
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void memdump_handler(FAR struct mm_allocnode_s *node, FAR void *arg)
{
  FAR const struct mm_memdump_s *dump = arg;
  size_t nodesize = MM_SIZEOF_NODE(node);

  if (MM_NODE_IS_ALLOC(node))
    {
      DEBUGASSERT(nodesize >= MM_SIZEOF_ALLOCNODE);
#if CONFIG_MM_BACKTRACE < 0
      if (dump->pid == PID_MM_ALLOC)
        {
          syslog(LOG_INFO, "%12zu%*p\n",
                 nodesize, MM_PTR_FMT_WIDTH,
                 ((FAR char *)node + MM_SIZEOF_ALLOCNODE));
        }
#elif CONFIG_MM_BACKTRACE == 0
      if ((MM_DUMP_ASSIGN(dump->pid, node->pid) ||
           MM_DUMP_ALLOC(dump->pid, node->pid) ||
           MM_DUMP_LEAK(dump->pid, node->pid)) &&
          node->seqno >= dump->seqmin && node->seqno <= dump->seqmax)
        {
          syslog(LOG_INFO, "%6d%12zu%12lu%*p\n",
                 node->pid, nodesize, node->seqno,
                 MM_PTR_FMT_WIDTH,
                 ((FAR char *)node + MM_SIZEOF_ALLOCNODE));
        }
#else
      if ((MM_DUMP_ASSIGN(dump->pid, node->pid) ||
           MM_DUMP_ALLOC(dump->pid, node->pid) ||
           MM_DUMP_LEAK(dump->pid, node->pid)) &&
          node->seqno >= dump->seqmin && node->seqno <= dump->seqmax)
        {
          char buf[CONFIG_MM_BACKTRACE * MM_PTR_FMT_WIDTH + 1] = "";

          FAR const char *format = " %0*p";
          int i;

          for (i = 0; i < CONFIG_MM_BACKTRACE && node->backtrace[i]; i++)
            {
              snprintf(buf + i * MM_PTR_FMT_WIDTH,
                       sizeof(buf) - i * MM_PTR_FMT_WIDTH,
                       format, MM_PTR_FMT_WIDTH - 1, node->backtrace[i]);
            }

          syslog(LOG_INFO, "%6d%12zu%12lu%*p%s\n",
                 node->pid, nodesize, node->seqno,
                 MM_PTR_FMT_WIDTH,
                 ((FAR char *)node + MM_SIZEOF_ALLOCNODE), buf);
        }
#endif
    }
  else if (dump->pid == PID_MM_FREE)
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

      syslog(LOG_INFO, "%12zu%*p\n",
             nodesize, MM_PTR_FMT_WIDTH,
             ((FAR char *)node + MM_SIZEOF_ALLOCNODE));
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_memdump
 *
 * Description:
 *   mm_memdump returns a memory info about specified pid of task/thread.
 *   if pid equals -1, this function will dump all allocated node and output
 *   backtrace for every allocated node for this heap, if pid equals -2, this
 *   function will dump all free node for this heap, and if pid is greater
 *   than or equal to 0, will dump pid allocated node and output backtrace.
 ****************************************************************************/

void mm_memdump(FAR struct mm_heap_s *heap,
                FAR const struct mm_memdump_s *dump)
{
  struct mallinfo_task info;

  if (dump->pid >= PID_MM_ALLOC)
    {
      syslog(LOG_INFO, "Dump all used memory node info:\n");
#if CONFIG_MM_BACKTRACE < 0
      syslog(LOG_INFO, "%12s%*s\n", "Size", MM_PTR_FMT_WIDTH, "Address");
#else
      syslog(LOG_INFO, "%6s%12s%12s%*s %s\n", "PID", "Size", "Sequence",
                        MM_PTR_FMT_WIDTH, "Address", "Backtrace");
#endif
    }
  else
    {
      syslog(LOG_INFO, "Dump all free memory node info:\n");
      syslog(LOG_INFO, "%12s%*s\n", "Size", MM_PTR_FMT_WIDTH, "Address");
    }

#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
  mempool_multiple_memdump(heap->mm_mpool, dump);
#endif
  mm_foreach(heap, memdump_handler, (FAR void *)dump);

  info = mm_mallinfo_task(heap, dump);

  syslog(LOG_INFO, "%12s%12s\n", "Total Blks", "Total Size");
  syslog(LOG_INFO, "%12d%12d\n", info.aordblks, info.uordblks);
}
