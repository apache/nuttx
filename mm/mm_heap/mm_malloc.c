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
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef NULL
#  define NULL ((void *)0)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void mm_free_delaylist(FAR struct mm_heap_s *heap)
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

  info.pid = tcb->pid;
  mm_mallinfo_task(arg, &info);
  mwarn("pid:%5d, used:%10d, nused:%10d\n",
        tcb->pid, info.uordblks, info.aordblks);
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
  FAR void *ret = NULL;
  int ndx;

  /* Free the delay list first */

  mm_free_delaylist(heap);

  /* Ignore zero-length allocations */

  if (size < 1)
    {
      return NULL;
    }

  /* Adjust the size to account for (1) the size of the allocated node and
   * (2) to make sure that it is an even multiple of our granule size.
   */

  alignsize = MM_ALIGN_UP(size + SIZEOF_MM_ALLOCNODE);
  if (alignsize < size)
    {
      /* There must have been an integer overflow */

      return NULL;
    }

  DEBUGASSERT(alignsize >= MM_MIN_CHUNK);
  DEBUGASSERT(alignsize >= SIZEOF_MM_FREENODE);

  /* We need to hold the MM semaphore while we muck with the nodelist. */

  DEBUGVERIFY(mm_takesemaphore(heap));

  /* Get the location in the node list to start the search. Special case
   * really big allocations
   */

  if (alignsize >= MM_MAX_CHUNK)
    {
      ndx = MM_NNODES - 1;
    }
  else
    {
      /* Convert the request size into a nodelist index */

      ndx = mm_size2ndx(alignsize);
    }

  /* Search for a large enough chunk in the list of nodes. This list is
   * ordered by size, but will have occasional zero sized nodes as we visit
   * other mm_nodelist[] entries.
   */

  for (node = heap->mm_nodelist[ndx].flink;
       node && node->size < alignsize;
       node = node->flink)
    {
      DEBUGASSERT(node->blink->flink == node);
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

      /* Check if we have to split the free node into one of the allocated
       * size and another smaller freenode.  In some cases, the remaining
       * bytes can be smaller (they may be SIZEOF_MM_ALLOCNODE).  In that
       * case, we will just carry the few wasted bytes at the end of the
       * allocation.
       */

      remaining = node->size - alignsize;
      if (remaining >= SIZEOF_MM_FREENODE)
        {
          /* Get a pointer to the next node in physical memory */

          next = (FAR struct mm_freenode_s *)
                 (((FAR char *)node) + node->size);

          /* Create the remainder node */

          remainder = (FAR struct mm_freenode_s *)
            (((FAR char *)node) + alignsize);

          remainder->size      = remaining;
          remainder->preceding = alignsize;

          /* Adjust the size of the node under consideration */

          node->size = alignsize;

          /* Adjust the 'preceding' size of the (old) next node, preserving
           * the allocated flag.
           */

          next->preceding = remaining | (next->preceding & MM_ALLOC_BIT);

          /* Add the remainder back into the nodelist */

          mm_addfreechunk(heap, remainder);
        }

      /* Handle the case of an exact size match */

      node->preceding |= MM_ALLOC_BIT;
      MM_ADD_BACKTRACE(heap, node);
      ret = (FAR void *)((FAR char *)node + SIZEOF_MM_ALLOCNODE);
    }

  DEBUGASSERT(ret == NULL || mm_heapmember(heap, ret));
  mm_givesemaphore(heap);

  if (ret)
    {
      kasan_unpoison(ret, mm_malloc_size(ret));
#ifdef CONFIG_MM_FILL_ALLOCATIONS
      memset(ret, 0xaa, alignsize - SIZEOF_MM_ALLOCNODE);
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
#endif
#ifdef CONFIG_MM_PANIC_ON_FAILURE
      PANIC();
#endif
    }
#endif

  return ret;
}
