/****************************************************************************
 * mm/mm_heap/mm_malloc.c
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
#include <string.h>
#include <malloc.h>

#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>
#include <nuttx/sched.h>

#include "mm_heap/mm.h"
#include "kasan/kasan.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void free_delaylist(FAR struct mm_heap_s *heap)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  FAR struct mm_delaynode_s *tmp;
  irqstate_t flags;

  /* Move the delay list to local */

  flags = enter_critical_section();

  tmp = heap->mm_delaylist[up_cpu_index()];
  heap->mm_delaylist[up_cpu_index()] = NULL;

  leave_critical_section(flags);

  /* Test if the delayed is empty */

  while (tmp)
    {
      FAR void *address;

      /* Get the first delayed deallocation */

      address = tmp;
      tmp = tmp->flink;

      /* The address should always be non-NULL since that was checked in the
       * 'while' condition above.
       */

      mm_free(heap, address);
    }
#endif
}

#if CONFIG_MM_BACKTRACE >= 0
void mm_dump_handler(FAR struct tcb_s *tcb, FAR void *arg)
{
  struct mallinfo_task info;
  struct mm_memdump_s dump;

  dump.pid = tcb->pid;
  dump.seqmin = 0;
  dump.seqmax = ULONG_MAX;
  info = mm_mallinfo_task(arg, &dump);
  mwarn("pid:%5d, used:%10d, nused:%10d\n",
        tcb->pid, info.uordblks, info.aordblks);
}
#endif

#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
void mm_mempool_dump_handle(FAR struct mempool_s *pool, FAR void *arg)
{
  struct mempoolinfo_s info;

  mempool_info(pool, &info);
  mwarn("%9lu%11lu%9lu%9lu%9lu%9lu%9zu\n",
        info.sizeblks, info.arena, info.aordblks,
        info.ordblks, info.iordblks,
        info.nwaiter, pool->nexpend);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_malloc
 *
 * Description:
 *  Find the smallest chunk that satisfies the request. Take the memory from
 *  that chunk, save the remaining, smaller chunk (if any).
 *
 *  8-byte alignment of the allocated data is assured.
 *
 ****************************************************************************/

FAR void *mm_malloc(FAR struct mm_heap_s *heap, size_t size)
{
  FAR struct mm_freenode_s *node;
  size_t alignsize;
  size_t nodesize;
  FAR void *ret = NULL;
  int ndx;

  /* Free the delay list first */

  free_delaylist(heap);

#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
  ret = mempool_multiple_alloc(heap->mm_mpool, size);
  if (ret != NULL)
    {
      return ret;
    }
#endif

  /* Adjust the size to account for (1) the size of the allocated node and
   * (2) to make sure that it is aligned with MM_ALIGN and its size is at
   * least MM_MIN_CHUNK.
   */

  if (size < MM_MIN_CHUNK - OVERHEAD_MM_ALLOCNODE)
    {
      size = MM_MIN_CHUNK - OVERHEAD_MM_ALLOCNODE;
    }

  alignsize = MM_ALIGN_UP(size + OVERHEAD_MM_ALLOCNODE);
  if (alignsize < size)
    {
      /* There must have been an integer overflow */

      return NULL;
    }

  DEBUGASSERT(alignsize >= MM_ALIGN);

  /* We need to hold the MM mutex while we muck with the nodelist. */

  DEBUGVERIFY(mm_lock(heap));

  /* Convert the request size into a nodelist index */

  ndx = mm_size2ndx(alignsize);

  /* Search for a large enough chunk in the list of nodes. This list is
   * ordered by size, but will have occasional zero sized nodes as we visit
   * other mm_nodelist[] entries.
   */

  for (node = heap->mm_nodelist[ndx].flink; node; node = node->flink)
    {
      DEBUGASSERT(node->blink->flink == node);
      nodesize = SIZEOF_MM_NODE(node);
      if (nodesize >= alignsize)
        {
          break;
        }
    }

  /* If we found a node with non-zero size, then this is one to use. Since
   * the list is ordered, we know that it must be the best fitting chunk
   * available.
   */

  if (node)
    {
      FAR struct mm_freenode_s *remainder;
      FAR struct mm_freenode_s *next;
      size_t remaining;

      /* Remove the node.  There must be a predecessor, but there may not be
       * a successor node.
       */

      DEBUGASSERT(node->blink);
      node->blink->flink = node->flink;
      if (node->flink)
        {
          node->flink->blink = node->blink;
        }

      /* Get a pointer to the next node in physical memory */

      next = (FAR struct mm_freenode_s *)(((FAR char *)node) + nodesize);
      DEBUGASSERT((next->size & MM_ALLOC_BIT) != 0 &&
                  (next->size & MM_PREVFREE_BIT) != 0 &&
                  next->preceding == nodesize);

      /* Check if we have to split the free node into one of the allocated
       * size and another smaller freenode.  In some cases, the remaining
       * bytes can be smaller (they may be SIZEOF_MM_ALLOCNODE).  In that
       * case, we will just carry the few wasted bytes at the end of the
       * allocation.
       */

      remaining = nodesize - alignsize;
      if (remaining >= MM_MIN_CHUNK)
        {
          /* Create the remainder node */

          remainder = (FAR struct mm_freenode_s *)
            (((FAR char *)node) + alignsize);

          remainder->size = remaining;

          /* Adjust the size of the node under consideration */

          node->size = alignsize | (node->size & MM_MASK_BIT);

          /* Adjust the 'preceding' size of the (old) next node. */

          next->preceding = remaining;

          /* Add the remainder back into the nodelist */

          mm_addfreechunk(heap, remainder);
        }
      else
        {
          /* Previous physical memory node is alloced, so clear the previous
           * free bit in next->size.
           */

          next->size &= ~MM_PREVFREE_BIT;
        }

      /* Handle the case of an exact size match */

      node->size |= MM_ALLOC_BIT;
      ret = (FAR void *)((FAR char *)node + SIZEOF_MM_ALLOCNODE);
    }

  DEBUGASSERT(ret == NULL || mm_heapmember(heap, ret));
  mm_unlock(heap);

  if (ret)
    {
      MM_ADD_BACKTRACE(heap, node);
      kasan_unpoison(ret, mm_malloc_size(heap, ret));
#ifdef CONFIG_MM_FILL_ALLOCATIONS
      memset(ret, 0xaa, alignsize - OVERHEAD_MM_ALLOCNODE);
#endif
#ifdef CONFIG_DEBUG_MM
      minfo("Allocated %p, size %zu\n", ret, alignsize);
#endif
    }
#ifdef CONFIG_DEBUG_MM
  else
    {
#ifdef CONFIG_MM_DUMP_ON_FAILURE
      struct mallinfo minfo;
#endif

      mwarn("WARNING: Allocation failed, size %zu\n", alignsize);
#ifdef CONFIG_MM_DUMP_ON_FAILURE
      mm_mallinfo(heap, &minfo);
      mwarn("Total:%d, used:%d, free:%d, largest:%d, nused:%d, nfree:%d\n",
            minfo.arena, minfo.uordblks, minfo.fordblks,
            minfo.mxordblk, minfo.aordblks, minfo.ordblks);
#  if CONFIG_MM_BACKTRACE >= 0
      nxsched_foreach(mm_dump_handler, heap);
#  endif
#  if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
      mwarn("%11s%9s%9s%9s%9s%9s%9s\n", "bsize", "total", "nused",
            "nfree", "nifree", "nwaiter", "nexpend");
      mempool_multiple_foreach(heap->mm_mpool,
                               mm_mempool_dump_handle, NULL);
#  endif

#endif
#ifdef CONFIG_MM_PANIC_ON_FAILURE
      PANIC();
#endif
    }
#endif

  DEBUGASSERT(ret == NULL || ((uintptr_t)ret) % MM_ALIGN == 0);
  return ret;
}
