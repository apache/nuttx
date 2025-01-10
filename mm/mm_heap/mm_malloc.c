/****************************************************************************
 * mm/mm_heap/mm_malloc.c
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
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>
#include <nuttx/mm/kasan.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>

#include "mm_heap/mm.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: free_delaylist
 *
 * Description:
 *  Free the memory in delay list either added because of mm_lock failed or
 *  added because of CONFIG_MM_FREE_DELAYCOUNT_MAX.
 *  Set force to true to free all the memory in delay list immediately, set
 *  to false will only free delaylist when time is up if
 *  CONFIG_MM_FREE_DELAYCOUNT_MAX is enabled.
 *
 *  Return true if there is memory freed.
 *
 ****************************************************************************/

static bool free_delaylist(FAR struct mm_heap_s *heap, bool force)
{
  bool ret = false;
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  FAR struct mm_delaynode_s *tmp;
  irqstate_t flags;

  /* Move the delay list to local */

  flags = mm_lock_irq(heap);

  tmp = heap->mm_delaylist[this_cpu()];

#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  if (tmp == NULL ||
      (!force &&
        heap->mm_delaycount[this_cpu()] < CONFIG_MM_FREE_DELAYCOUNT_MAX))
    {
      mm_unlock_irq(heap, flags);
      return false;
    }

  heap->mm_delaycount[this_cpu()] = 0;
#endif

  heap->mm_delaylist[this_cpu()] = NULL;

  mm_unlock_irq(heap, flags);

  /* Test if the delayed is empty */

  ret = tmp != NULL;

  while (tmp)
    {
      FAR void *address;

      /* Get the first delayed deallocation */

      address = tmp;
      tmp = tmp->flink;

      /* The address should always be non-NULL since that was checked in the
       * 'while' condition above.
       */

      mm_delayfree(heap, address, false);
    }

#endif
  return ret;
}

#if CONFIG_MM_BACKTRACE >= 0
void mm_dump_handler(FAR struct tcb_s *tcb, FAR void *arg)
{
  struct mallinfo_task info;
  struct malltask task;

  task.pid = tcb ? tcb->pid : PID_MM_LEAK;
  task.seqmin = 0;
  task.seqmax = ULONG_MAX;
  info = mm_mallinfo_task(arg, &task);
  mwarn("pid:%5d, used:%10d, nused:%10d\n",
        task.pid, info.uordblks, info.aordblks);
}
#endif

#ifdef CONFIG_MM_HEAP_MEMPOOL
void mm_mempool_dump_handle(FAR struct mempool_s *pool, FAR void *arg)
{
  struct mempoolinfo_s info;

  mempool_info(pool, &info);
  mwarn("%9lu%11lu%9lu%9lu%9lu%9lu\n",
        info.sizeblks, info.arena, info.aordblks,
        info.ordblks, info.iordblks, info.nwaiter);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_free_delaylist
 *
 * Description:
 *   force freeing the delaylist of this heap.
 *
 ****************************************************************************/

void mm_free_delaylist(FAR struct mm_heap_s *heap)
{
  if (heap)
    {
       free_delaylist(heap, true);
    }
}

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

  free_delaylist(heap, false);

#ifdef CONFIG_MM_HEAP_MEMPOOL
  if (heap->mm_mpool)
    {
      ret = mempool_multiple_alloc(heap->mm_mpool, size);
      if (ret != NULL)
        {
          return ret;
        }
    }
#endif

  /* Adjust the size to account for (1) the size of the allocated node and
   * (2) to make sure that it is aligned with MM_ALIGN and its size is at
   * least MM_MIN_CHUNK.
   */

  if (size < MM_MIN_CHUNK - MM_ALLOCNODE_OVERHEAD)
    {
      size = MM_MIN_CHUNK - MM_ALLOCNODE_OVERHEAD;
    }

  alignsize = MM_ALIGN_UP(size + MM_ALLOCNODE_OVERHEAD);
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
      nodesize = MM_SIZEOF_NODE(node);
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

      /* Node next must be alloced, otherwise it should be merged.
       * Its prenode(the founded node) must be free and preceding should
       * match with nodesize.
       */

      DEBUGASSERT(MM_NODE_IS_ALLOC(next) && MM_PREVNODE_IS_FREE(next) &&
                  next->preceding == nodesize);

      /* Check if we have to split the free node into one of the allocated
       * size and another smaller freenode.  In some cases, the remaining
       * bytes can be smaller (they may be MM_SIZEOF_ALLOCNODE).  In that
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

      /* Update heap statistics */

      nodesize = MM_SIZEOF_NODE(node);
      heap->mm_curused += nodesize;
      if (heap->mm_curused > heap->mm_maxused)
        {
          heap->mm_maxused = heap->mm_curused;
        }

      /* Handle the case of an exact size match */

      node->size |= MM_ALLOC_BIT;
      ret = (FAR void *)((FAR char *)node + MM_SIZEOF_ALLOCNODE);
    }

  DEBUGASSERT(ret == NULL || mm_heapmember(heap, ret));

  if (ret)
    {
      sched_note_heap(NOTE_HEAP_ALLOC, heap, ret, nodesize,
                      heap->mm_curused);
    }

  mm_unlock(heap);

  if (ret)
    {
      MM_ADD_BACKTRACE(heap, node);
      ret = kasan_unpoison(ret, nodesize - MM_ALLOCNODE_OVERHEAD);
#ifdef CONFIG_MM_FILL_ALLOCATIONS
      memset(ret, MM_ALLOC_MAGIC, alignsize - MM_ALLOCNODE_OVERHEAD);
#endif
#ifdef CONFIG_DEBUG_MM
      minfo("Allocated %p, size %zu\n", ret, alignsize);
#endif
    }

#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  /* Try again after free delay list */

  else if (free_delaylist(heap, true))
    {
      return mm_malloc(heap, size);
    }
#endif

#ifdef CONFIG_DEBUG_MM
  else if (MM_INTERNAL_HEAP(heap))
    {
#ifdef CONFIG_MM_DUMP_ON_FAILURE
      struct mallinfo minfo;
#  ifdef CONFIG_MM_DUMP_DETAILS_ON_FAILURE
      struct mm_memdump_s dump =
      {
#if CONFIG_MM_BACKTRACE >= 0
        PID_MM_ALLOC, 0, ULONG_MAX
#else
        PID_MM_ALLOC
#endif
      };
#  endif
#endif

      mwarn("WARNING: Allocation failed, size %zu\n", alignsize);
#ifdef CONFIG_MM_DUMP_ON_FAILURE
      minfo = mm_mallinfo(heap);
      mwarn("Total:%d, used:%d, free:%d, largest:%d, nused:%d, nfree:%d\n",
            minfo.arena, minfo.uordblks, minfo.fordblks,
            minfo.mxordblk, minfo.aordblks, minfo.ordblks);
#  if CONFIG_MM_BACKTRACE >= 0
      nxsched_foreach(mm_dump_handler, heap);
      mm_dump_handler(NULL, heap);
#  endif
#  ifdef CONFIG_MM_HEAP_MEMPOOL
      mwarn("%11s%9s%9s%9s%9s%9s\n",
            "bsize", "total", "nused",
            "nfree", "nifree", "nwaiter");
      mempool_multiple_foreach(heap->mm_mpool,
                               mm_mempool_dump_handle, NULL);
#  endif
#  ifdef CONFIG_MM_DUMP_DETAILS_ON_FAILURE
      mm_memdump(heap, &dump);
      mwarn("Dump leak memory(thread exit, but memory not free):\n");
      dump.pid = PID_MM_LEAK;
      mm_memdump(heap, &dump);
#    ifdef CONFIG_MM_HEAP_MEMPOOL
      mwarn("Dump block used by mempool expand/trunk:\n");
      dump.pid = PID_MM_MEMPOOL;
      mm_memdump(heap, &dump);
#    endif
#    if CONFIG_MM_BACKTRACE >= 0
      mwarn("Dump allocated orphan nodes. (neighbor of free nodes):\n");
      dump.pid = PID_MM_ORPHAN;
      mm_memdump(heap, &dump);
#    endif
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
