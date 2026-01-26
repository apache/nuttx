/****************************************************************************
 * arch/sim/src/sim/sim_ummheap.c
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
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/atomic.h>
#include <nuttx/mm/mm.h>
#include <nuttx/sched_note.h>

#include "sim_internal.h"

#ifdef CONFIG_MM_UMM_CUSTOMIZE_MANAGER

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This describes one heap (possibly with multiple regions) */

struct mm_delaynode_s
{
  struct mm_delaynode_s *flink;
};

struct mm_heap_s
{
  struct mm_delaynode_s *delaylist[CONFIG_SMP_NCPUS];

#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  size_t delaycount[CONFIG_SMP_NCPUS];
#endif

  atomic_t aordblks;
  atomic_t uordblks;
  atomic_t usmblks;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void delay_free(struct mm_heap_s *heap, void *mem, bool delay);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mm_heap_s g_heap;

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct mm_heap_s *g_mmheap = &g_heap;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void add_delaylist(struct mm_heap_s *heap, void *mem)
{
  struct mm_delaynode_s *tmp = mem;
  irqstate_t flags;

  /* Delay the deallocation until a more appropriate time. */

  flags = up_irq_save();

  tmp->flink = heap->delaylist[this_cpu()];
  heap->delaylist[this_cpu()] = tmp;

#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  heap->delaycount[this_cpu()]++;
#endif

  up_irq_restore(flags);
}

static bool free_delaylist(struct mm_heap_s *heap, bool force)
{
  bool ret = false;
  struct mm_delaynode_s *tmp;
  irqstate_t flags;

  /* Move the delay list to local */

  flags = up_irq_save();

  tmp = heap->delaylist[this_cpu()];

#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  if (tmp == NULL ||
      (!force &&
        heap->delaycount[this_cpu()] < CONFIG_MM_FREE_DELAYCOUNT_MAX))
    {
      up_irq_restore(flags);
      return false;
    }

  heap->delaycount[this_cpu()] = 0;
#endif
  heap->delaylist[this_cpu()] = NULL;

  up_irq_restore(flags);

  /* Test if the delayed is empty */

  ret = (tmp != NULL);

  while (tmp)
    {
      void *address;

      /* Get the first delayed deallocation */

      address = tmp;
      tmp = tmp->flink;

      /* The address should always be non-NULL since that was checked in the
       * 'while' condition above.
       */

      delay_free(heap, address, false);
    }

  return ret;
}

/****************************************************************************
 * Name: delay_free
 *
 * Description:
 *   Delay free memory if `delay` is true, otherwise free it immediately.
 *
 ****************************************************************************/

static void delay_free(struct mm_heap_s *heap, void *mem, bool delay)
{
  /* Check current environment */

  if (up_interrupt_context())
    {
      /* We are in ISR, add to the delay list */

      add_delaylist(heap, mem);
    }
  else if (nxsched_gettid() < 0 || delay)
    {
      /* nxsched_gettid() return -ESRCH, means we are in situations
       * during context switching(See nxsched_gettid's comment).
       * Then add to the delay list.
       */

      add_delaylist(heap, mem);
    }
  else
    {
      int size = host_mallocsize(mem);
      atomic_fetch_sub(&heap->aordblks, 1);
      atomic_fetch_sub(&heap->uordblks, size);
      sched_note_heap(NOTE_HEAP_FREE, heap, mem, size, 0);
      host_free(mem);
    }
}

static void *reallocate(void *oldmem, size_t size)
{
  struct mm_heap_s *heap = g_mmheap;
  void *mem;
  int uordblks;
  int usmblks;
  int newsize;
  int oldsize;

  free_delaylist(heap, false);

  if (size == 0)
    {
      size = 1;
    }

  oldsize = host_mallocsize(oldmem);
  atomic_fetch_sub(&heap->uordblks, oldsize);
  mem = host_realloc(oldmem, size);

  atomic_fetch_add(&heap->aordblks, oldmem == NULL && mem != NULL);
  newsize = host_mallocsize(mem ? mem : oldmem);
  atomic_fetch_add(&heap->uordblks, newsize);
  usmblks = atomic_read(&heap->usmblks);
  if (mem != NULL)
    {
      if (oldmem != NULL)
        {
          sched_note_heap(NOTE_HEAP_FREE, heap, oldmem, oldsize, 0);
        }

      sched_note_heap(NOTE_HEAP_ALLOC, heap, mem, newsize, 0);
    }

  do
    {
      uordblks = atomic_read(&heap->uordblks);
      if (uordblks <= usmblks)
        {
          break;
        }
    }
  while (atomic_try_cmpxchg(&heap->usmblks, &usmblks, uordblks));

#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  if (mem == NULL && free_delaylist(heap, true))
    {
      return reallocate(oldmem, size);
    }
#endif

  return mem;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: umm_addregion
 *
 * Description:
 *   This function adds a region of contiguous memory to the selected heap.
 *
 * Input Parameters:
 *   heapstart - Start of the heap region
 *   heapsize  - Size of the heap region
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void umm_addregion(void *heapstart, size_t heapsize)
{
}

/****************************************************************************
 * Name: malloc
 *
 * Description:
 *  Find the smallest chunk that satisfies the request. Take the memory from
 *  that chunk, save the remaining, smaller chunk (if any).
 *
 *  8-byte alignment of the allocated data is assured.
 *
 ****************************************************************************/

void *malloc(size_t size)
{
  return reallocate(NULL, size);
}

/****************************************************************************
 * Name: free
 *
 * Description:
 *   Returns a chunk of memory to the list of free nodes,  merging with
 *   adjacent free chunks if possible.
 *
 ****************************************************************************/

void free(void *mem)
{
  /* Protect against attempts to free a NULL reference */

  if (mem == NULL)
    {
      return;
    }

  delay_free(g_mmheap, mem, CONFIG_MM_FREE_DELAYCOUNT_MAX > 0);
}

/****************************************************************************
 * Name: realloc
 *
 * Description:
 *   If the reallocation is for less space, then:
 *
 *     (1) the current allocation is reduced in size
 *     (2) the remainder at the end of the allocation is returned to the
 *         free list.
 *
 *  If the request is for more space and the current allocation can be
 *  extended, it will be extended by:
 *
 *     (1) Taking the additional space from the following free chunk, or
 *     (2) Taking the additional space from the preceding free chunk.
 *     (3) Or both
 *
 *  If the request is for more space but the current chunk cannot be
 *  extended, then malloc a new buffer, copy the data into the new buffer,
 *  and free the old buffer.
 *
 ****************************************************************************/

void *realloc(void *oldmem, size_t size)
{
  return reallocate(oldmem, size);
}

/****************************************************************************
 * Name: calloc
 *
 * Descriptor:
 *   calloc() calculates the size of the allocation and calls zalloc()
 *
 ****************************************************************************/

void *calloc(size_t n, size_t elem_size)
{
  size_t size = n * elem_size;

  if (size < elem_size)
    {
      return NULL;
    }

  return zalloc(size);
}

/****************************************************************************
 * Name: zalloc
 *
 * Description:
 *   zalloc calls malloc, then zeroes out the allocated chunk.
 *
 ****************************************************************************/

void *zalloc(size_t size)
{
  void *ptr;

  ptr = malloc(size);
  if (ptr != NULL)
    {
      memset(ptr, 0, size);
    }

  return ptr;
}

/****************************************************************************
 * Name: memalign
 *
 * Description:
 *   memalign requests more than enough space from malloc, finds a region
 *   within that chunk that meets the alignment request and then frees any
 *   leading or trailing space.
 *
 *   The alignment argument must be a power of two (not checked).  8-byte
 *   alignment is guaranteed by normal malloc calls.
 *
 ****************************************************************************/

void *memalign(size_t alignment, size_t size)
{
  struct mm_heap_s *heap = g_mmheap;
  void *mem;
  int uordblks;
  int usmblks;

  free_delaylist(heap, false);
  mem = host_memalign(alignment, size);

  if (mem == NULL)
    {
      return NULL;
    }

  size = host_mallocsize(mem);
  sched_note_heap(NOTE_HEAP_ALLOC, heap, mem, size, 0);
  atomic_fetch_add(&heap->aordblks, 1);
  atomic_fetch_add(&heap->uordblks, size);
  usmblks = atomic_read(&heap->usmblks);

  do
    {
      uordblks = atomic_read(&heap->uordblks);
      if (uordblks <= usmblks)
        {
          break;
        }
    }
  while (atomic_try_cmpxchg(&heap->usmblks, &usmblks, uordblks));

#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  if (mem == NULL && free_delaylist(heap, true))
    {
      return memalign(alignment, size);
    }
#endif

  return mem;
}

/****************************************************************************
 * Name: umm_heapmember
 *
 * Description:
 *   Check if an address lies in the heap.
 *
 * Parameters:
 *   heap - The heap to check
 *   mem  - The address to check
 *
 * Return Value:
 *   true if the address is a member of the heap.  false if not
 *   not.  If the address is not a member of the heap, then it
 *   must be a member of the user-space heap (unchecked)
 *
 ****************************************************************************/

bool umm_heapmember(void *mem)
{
  return true;
}

/****************************************************************************
 * Name: umm_brkaddr
 *
 * Description:
 *   Return the break address of a heap region.  Zero is returned if the
 *   memory region is not initialized.
 *
 ****************************************************************************/

void *umm_brkaddr(int region)
{
  return NULL;
}

/****************************************************************************
 * Name: umm_extend
 *
 * Description:
 *   Extend a heap region by add a block of (virtually) contiguous memory
 *   to the end of the heap.
 *
 ****************************************************************************/

void umm_extend(void *mem, size_t size, int region)
{
}

/****************************************************************************
 * Name: mallinfo
 *
 * Description:
 *   mallinfo returns a copy of updated current heap information.
 *
 ****************************************************************************/

struct mallinfo mallinfo(void)
{
  struct mm_heap_s *heap = g_mmheap;
  struct mallinfo info;

  memset(&info, 0, sizeof(struct mallinfo));
  info.aordblks = atomic_read(&heap->aordblks);
  info.uordblks = atomic_read(&heap->uordblks);
  info.usmblks  = atomic_read(&heap->usmblks);
  info.arena    = SIM_HEAP_SIZE;
  info.fordblks = SIM_HEAP_SIZE - info.uordblks;
  info.mxordblk = info.fordblks;
  return info;
}

/****************************************************************************
 * Name: mallinfo_task
 *
 * Description:
 *   mallinfo_task returns a copy of updated current task's heap information.
 *
 ****************************************************************************/

struct mallinfo_task mallinfo_task(const struct malltask *task)
{
  struct mallinfo_task info =
    {
      0, 0
    };

  return info;
}

/****************************************************************************
 * Name: umm_memdump
 *
 * Description:
 *   umm_memdump returns a memory info about specified pid of task/thread.
 *
 ****************************************************************************/

void umm_memdump(const struct mm_memdump_s *dump)
{
}

#ifdef CONFIG_DEBUG_MM

/****************************************************************************
 * Name:umm_checkcorruption
 *
 * Description:
 *   umm_checkcorruption is used to check whether memory heap is normal.
 *
 ****************************************************************************/

void umm_checkcorruption(void)
{
}

#endif /* CONFIG_DEBUG_MM */

/****************************************************************************
 * Name: malloc_size
 ****************************************************************************/

size_t malloc_size(void *mem)
{
  return host_mallocsize(mem);
}

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   If a protected kernel-space heap is provided, the kernel heap must be
 *   allocated (and protected) by an analogous up_allocate_kheap().
 *
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  *heap_start = NULL;
  *heap_size  = 0;
}

/****************************************************************************
 * Name: umm_initialize
 *
 * Description:
 *   Initialize the selected heap data structures, providing the initial
 *   heap region.
 *
 ****************************************************************************/

void umm_initialize(void *heap_start, size_t heap_size)
{
  sched_note_heap(NOTE_HEAP_ADD, g_mmheap, heap_start, heap_size, 0);
  UNUSED(heap_start);
  UNUSED(heap_size);
}

#else /* CONFIG_MM_UMM_CUSTOMIZE_MANAGER */

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  *heap_start = host_allocheap(SIM_HEAP_SIZE, false);
  *heap_size  = SIM_HEAP_SIZE;
}

#endif /* CONFIG_MM_UMM_CUSTOMIZE_MANAGER */
