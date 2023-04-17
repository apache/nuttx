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
#include <malloc.h>
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
  pid_t pid = *(FAR pid_t *)arg;
  size_t nodesize = SIZEOF_MM_NODE(node);

  if ((node->size & MM_ALLOC_BIT) != 0)
    {
      DEBUGASSERT(nodesize >= SIZEOF_MM_ALLOCNODE);
#if CONFIG_MM_BACKTRACE < 0
      if (pid == MM_BACKTRACE_ALLOC_PID)
#else
      if (pid == MM_BACKTRACE_ALLOC_PID || node->pid == pid)
#endif
        {
#if CONFIG_MM_BACKTRACE < 0
          syslog(LOG_INFO, "%12zu%*p\n",
                 nodesize, MM_PTR_FMT_WIDTH,
                 ((FAR char *)node + SIZEOF_MM_ALLOCNODE));
#else
#  if CONFIG_MM_BACKTRACE > 0
          int i;
          FAR const char *format = " %0*p";
#  endif
          char buf[CONFIG_MM_BACKTRACE * MM_PTR_FMT_WIDTH + 1];

          buf[0] = '\0';
#  if CONFIG_MM_BACKTRACE > 0
          for (i = 0; i < CONFIG_MM_BACKTRACE && node->backtrace[i]; i++)
            {
              sprintf(buf + i * MM_PTR_FMT_WIDTH, format,
                      MM_PTR_FMT_WIDTH - 1, node->backtrace[i]);
            }
#  endif

          syslog(LOG_INFO, "%6d%12zu%*p%s\n",
                 (int)node->pid, nodesize, MM_PTR_FMT_WIDTH,
                 ((FAR char *)node + SIZEOF_MM_ALLOCNODE), buf);
#endif
        }
    }
  else
    {
      FAR struct mm_freenode_s *fnode = (FAR void *)node;

      DEBUGASSERT(nodesize >= MM_MIN_CHUNK);
      DEBUGASSERT(fnode->blink->flink == fnode);
      DEBUGASSERT(SIZEOF_MM_NODE(fnode->blink) <= nodesize);
      DEBUGASSERT(fnode->flink == NULL ||
                  fnode->flink->blink == fnode);
      DEBUGASSERT(fnode->flink == NULL ||
                  SIZEOF_MM_NODE(fnode->flink) == 0 ||
                  SIZEOF_MM_NODE(fnode->flink) >= nodesize);

      if (pid <= MM_BACKTRACE_FREE_PID)
        {
          syslog(LOG_INFO, "%12zu%*p\n",
                 nodesize, MM_PTR_FMT_WIDTH,
                 ((FAR char *)node + SIZEOF_MM_ALLOCNODE));
        }
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

void mm_memdump(FAR struct mm_heap_s *heap, pid_t pid)
{
  struct mallinfo_task info;

  if (pid >= -1)
    {
      syslog(LOG_INFO, "Dump all used memory node info:\n");
#if CONFIG_MM_BACKTRACE < 0
      syslog(LOG_INFO, "%12s%*s\n", "Size", MM_PTR_FMT_WIDTH, "Address");
#else
      syslog(LOG_INFO, "%6s%12s%*s %s\n", "PID", "Size", MM_PTR_FMT_WIDTH,
             "Address", "Backtrace");
#endif
    }
  else
    {
      syslog(LOG_INFO, "Dump all free memory node info:\n");
      syslog(LOG_INFO, "%12s%*s\n", "Size", MM_PTR_FMT_WIDTH, "Address");
    }

#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
  mempool_multiple_memdump(heap->mm_mpool, pid);
#endif
  mm_foreach(heap, memdump_handler, &pid);

  info.pid = pid;
  mm_mallinfo_task(heap, &info);
  syslog(LOG_INFO, "%12s%12s\n", "Total Blks", "Total Size");
  syslog(LOG_INFO, "%12d%12d\n", info.aordblks, info.uordblks);
}
