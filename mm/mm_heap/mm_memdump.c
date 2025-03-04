/****************************************************************************
 * mm/mm_heap/mm_memdump.c
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

#include <stdio.h>
#include <assert.h>
#include <debug.h>
#include <execinfo.h>

#include <nuttx/mm/mm.h>

#include "mm_heap/mm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mm_memdump_priv_s
{
  FAR const struct mm_memdump_s *dump;
  struct mallinfo_task info;
#if CONFIG_MM_HEAP_BIGGEST_COUNT > 0
  FAR struct mm_allocnode_s *node[CONFIG_MM_HEAP_BIGGEST_COUNT];
  size_t filled;
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void memdump_allocnode(FAR struct mm_allocnode_s *node)
{
  size_t nodesize = MM_SIZEOF_NODE(node);
  size_t overhead = MM_SIZEOF_ALLOCNODE;
#if CONFIG_MM_BACKTRACE < 0
  syslog(LOG_INFO, "%12zu%9zu%*p\n",
         nodesize, overhead, BACKTRACE_PTR_FMT_WIDTH,
         (FAR const char *)node + MM_SIZEOF_ALLOCNODE);
#elif CONFIG_MM_BACKTRACE == 0
  syslog(LOG_INFO, "%6d%12zu%9zu%12lu%*p\n",
         node->pid, nodesize, overhead, node->seqno,
         BACKTRACE_PTR_FMT_WIDTH,
         (FAR const char *)node + MM_SIZEOF_ALLOCNODE);
#else
  char buf[BACKTRACE_BUFFER_SIZE(CONFIG_MM_BACKTRACE)];

  backtrace_format(buf, sizeof(buf), node->backtrace,
                   CONFIG_MM_BACKTRACE);

  syslog(LOG_INFO, "%6d%12zu%9zu%12lu%*p %s\n",
         node->pid, nodesize, overhead, node->seqno,
         BACKTRACE_PTR_FMT_WIDTH,
         (FAR const char *)node + MM_SIZEOF_ALLOCNODE, buf);
#endif
}

#if CONFIG_MM_HEAP_BIGGEST_COUNT > 0
static int memdump_record_comare(FAR const void *a, FAR const void *b)
{
  FAR struct mm_allocnode_s *node_a = *(FAR struct mm_allocnode_s **)a;
  FAR struct mm_allocnode_s *node_b = *(FAR struct mm_allocnode_s **)b;
  size_t size_a = MM_SIZEOF_NODE(node_a);
  size_t size_b = MM_SIZEOF_NODE(node_b);
  return size_a > size_b ? 1 : -1;
}

static void memdump_record_biggest(FAR struct mm_memdump_priv_s *priv,
                                   FAR struct mm_allocnode_s *node)
{
  if (priv->filled < CONFIG_MM_HEAP_BIGGEST_COUNT)
    {
      priv->node[priv->filled] = node;
      priv->filled++;
    }
  else
    {
      if (MM_SIZEOF_NODE(node) <= MM_SIZEOF_NODE(priv->node[0]))
        {
          return;
        }

      priv->node[0] = node;
    }

  if (priv->filled > 1)
    {
      qsort(priv->node, priv->filled, sizeof(struct mm_allocnode_s *),
            memdump_record_comare);
    }
}

static void memdump_dump_biggestnodes(FAR struct mm_memdump_priv_s *priv)
{
  size_t i;
  for (i = 0; i < priv->filled; i++)
    {
      priv->info.uordblks += MM_SIZEOF_NODE(priv->node[i]);
      memdump_allocnode(priv->node[i]);
    }

  priv->info.aordblks = priv->filled;
}

#endif

#ifdef CONFIG_MM_HEAP_MEMPOOL
static inline_function
void memdump_info_pool(FAR struct mm_memdump_priv_s *priv,
                       FAR struct mm_heap_s *heap)
{
  priv->info = mempool_multiple_info_task(heap->mm_mpool, priv->dump);
}

static inline_function
void memdump_dump_pool(FAR struct mm_memdump_priv_s *priv,
                       FAR struct mm_heap_s *heap)
{
  if (priv->info.aordblks > 0)
    {
      mempool_multiple_memdump(heap->mm_mpool, priv->dump);
    }
}
#else
#  define memdump_info_pool(priv,heap)
#  define memdump_dump_pool(priv,heap)
#endif

static void memdump_handler(FAR struct mm_allocnode_s *node, FAR void *arg)
{
  FAR struct mm_memdump_priv_s *priv = arg;
  FAR const struct mm_memdump_s *dump = priv->dump;
  size_t nodesize = MM_SIZEOF_NODE(node);

  if (MM_NODE_IS_ALLOC(node))
    {
      DEBUGASSERT(nodesize >= MM_SIZEOF_ALLOCNODE);
      if ((MM_DUMP_ASSIGN(dump, node) || MM_DUMP_ALLOC(dump, node) ||
           MM_DUMP_LEAK(dump, node)) && MM_DUMP_SEQNO(dump, node))
        {
          priv->info.aordblks++;
          priv->info.uordblks += nodesize;
          memdump_allocnode(node);
        }
      else if(dump->pid == PID_MM_ORPHAN && MM_DUMP_SEQNO(dump, node))
        {
          FAR struct mm_allocnode_s *next = (FAR struct mm_allocnode_s *)
                                            ((FAR char *)node + nodesize);
          if (MM_PREVNODE_IS_FREE(node) || MM_NODE_IS_FREE(next))
            {
              priv->info.aordblks++;
              priv->info.uordblks += nodesize;
              memdump_allocnode(node);
            }
        }
#if CONFIG_MM_HEAP_BIGGEST_COUNT > 0
      else if (dump->pid == PID_MM_BIGGEST && MM_DUMP_SEQNO(dump, node))
        {
          memdump_record_biggest(priv, node);
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

      priv->info.aordblks++;
      priv->info.uordblks += nodesize;
      syslog(LOG_INFO, "%12zu%9zu%*p\n",
             nodesize, MM_ALLOCNODE_OVERHEAD, BACKTRACE_PTR_FMT_WIDTH,
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
  struct mm_memdump_priv_s priv;
  pid_t pid = dump->pid;

  memset(&priv, 0, sizeof(struct mm_memdump_priv_s));
  priv.dump = dump;

  if (pid == PID_MM_MEMPOOL)
    {
      syslog(LOG_INFO, "Memdump mempool\n");
    }
  else if (pid == PID_MM_LEAK)
    {
      syslog(LOG_INFO, "Memdump leak\n");
      memdump_info_pool(&priv, heap);
    }
  else if (pid == PID_MM_ALLOC || pid >= 0)
    {
      FAR struct tcb_s *tcb = NULL;
      FAR const char   *name;

      if (pid == PID_MM_ALLOC)
        {
          name = "ALL";
        }
      else
        {
          name = "Unkown";
          tcb  = nxsched_get_tcb(pid);
        }

      if (tcb == NULL)
        {
          syslog(LOG_INFO, "Memdump task %s\n", name);
        }
      else
        {
          name = get_task_name(tcb);
          syslog(LOG_INFO, "Memdump task stack_alloc_ptr: %p,"
                           " adj_stack_size: %zu, name: %s\n",
                           tcb->stack_alloc_ptr, tcb->adj_stack_size, name);
        }

      memdump_info_pool(&priv, heap);
    }
  else if (pid == PID_MM_FREE)
    {
      syslog(LOG_INFO, "Dump all free memory node info\n");
      memdump_info_pool(&priv, heap);
    }
#if CONFIG_MM_HEAP_BIGGEST_COUNT > 0
  else if (pid == PID_MM_BIGGEST)
    {
      syslog(LOG_INFO, "Memdump biggest allocated top %d\n",
                       CONFIG_MM_HEAP_BIGGEST_COUNT);
    }
#endif
  else if (pid == PID_MM_ORPHAN)
    {
      syslog(LOG_INFO, "Dump allocated orphan nodes\n");
    }

#if CONFIG_MM_BACKTRACE < 0
  syslog(LOG_INFO, "%12s%9s%*s\n", "Size", "Overhead",
                   BACKTRACE_PTR_FMT_WIDTH,
                   "Address");
#else
  syslog(LOG_INFO, "%6s%12s%9s%12s%*s %s\n", "PID", "Size", "Overhead",
                   "Sequence", BACKTRACE_PTR_FMT_WIDTH,
                   "Address", "Backtrace");
#endif

  memdump_dump_pool(&priv, heap);

  mm_foreach(heap, memdump_handler, &priv);

#if CONFIG_MM_HEAP_BIGGEST_COUNT > 0
  if (pid == PID_MM_BIGGEST)
    {
      memdump_dump_biggestnodes(&priv);
    }
#endif

  syslog(LOG_INFO, "%12s%12s\n", "Total Blks", "Total Size");
  syslog(LOG_INFO, "%12d%12d\n", priv.info.aordblks, priv.info.uordblks);
}
