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

struct memdump_info_s
{
  pid_t pid;
  int   blks;
  int   size;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void memdump_handler(FAR struct mm_allocnode_s *node, FAR void *arg)
{
  FAR struct memdump_info_s *info = arg;

  if ((node->preceding & MM_ALLOC_BIT) != 0)
    {
      DEBUGASSERT(node->size >= SIZEOF_MM_ALLOCNODE);
      if (info->pid == -1)
        {
          info->blks++;
          info->size += node->size;
          syslog(LOG_INFO, "%12p%12" PRIu32 "\n",
                 ((FAR char *)node + SIZEOF_MM_ALLOCNODE),
                 node->size);
        }
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

      if (info->pid == -2)
        {
          info->blks++;
          info->size += node->size;
          syslog(LOG_INFO, "%12p%12" PRIu32 "\n",
                 ((FAR char *)node + SIZEOF_MM_ALLOCNODE),
                 node->size);
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
 *
 ****************************************************************************/

void mm_memdump(FAR struct mm_heap_s *heap, pid_t pid)
{
  struct memdump_info_s info;

  if (pid == -1)
    {
      syslog(LOG_INFO, "Dump all used memory node info:\n");
    }
  else if (pid == -2)
    {
      syslog(LOG_INFO, "Dump all free memory node info:\n");
    }

  syslog(LOG_INFO, "%12s%12s\n", "Address", "Size");

  info.blks = 0;
  info.size = 0;
  info.pid  = pid;
  mm_foreach(heap, memdump_handler, &info);

  syslog(LOG_INFO, "%12s%12s\n", "Total Blks", "Total Size");
  syslog(LOG_INFO, "%12d%12d\n", info.blks, info.size);
}
