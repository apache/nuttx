/****************************************************************************
 * arch/sim/src/sim/up_heap.c
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

#include <string.h>
#include <malloc.h>
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>

#include "up_internal.h"

#ifdef CONFIG_MM_CUSTOMIZE_MANAGER

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This describes one heap (possibly with multiple regions) */

struct mm_delaynode_s
{
  FAR struct mm_delaynode_s *flink;
};

struct mm_heap_impl_s
{
  struct mm_delaynode_s *mm_delaylist;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
static void mm_add_delaylist(FAR struct mm_heap_s *heap, FAR void *mem)
{
  FAR struct mm_delaynode_s *tmp = mem;
  irqstate_t flags;

  /* Delay the deallocation until a more appropriate time. */

  flags = enter_critical_section();

  tmp->flink = heap->mm_impl->mm_delaylist;
  heap->mm_impl->mm_delaylist = tmp;

  leave_critical_section(flags);
}

#endif

static void mm_free_delaylist(FAR struct mm_heap_s *heap)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  FAR struct mm_delaynode_s *tmp;
  irqstate_t flags;

  /* Move the delay list to local */

  flags = enter_critical_section();

  tmp = heap->mm_impl->mm_delaylist;
  heap->mm_impl->mm_delaylist = NULL;

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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_initialize
 *
 * Description:
 *   Initialize the selected heap data structures, providing the initial
 *   heap region.
 *
 * Input Parameters:
 *   heap      - The selected heap
 *   heapstart - Start of the initial heap region
 *   heapsize  - Size of the initial heap region
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void mm_initialize(FAR struct mm_heap_s *heap, FAR void *heap_start,
                   size_t heap_size)
{
  FAR struct mm_heap_impl_s *impl;
  impl = host_malloc(sizeof(struct mm_heap_impl_s));
  impl->mm_delaylist = NULL;
  heap->mm_impl = impl;
}

/****************************************************************************
 * Name: mm_addregion
 *
 * Description:
 *   This function adds a region of contiguous memory to the selected heap.
 *
 * Input Parameters:
 *   heap      - The selected heap
 *   heapstart - Start of the heap region
 *   heapsize  - Size of the heap region
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void mm_addregion(FAR struct mm_heap_s *heap, FAR void *heapstart,
                  size_t heapsize)
{
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
  /* Firstly, free mm_delaylist */

  mm_free_delaylist(heap);
  return host_malloc(size);
}

/****************************************************************************
 * Name: mm_free
 *
 * Description:
 *   Returns a chunk of memory to the list of free nodes,  merging with
 *   adjacent free chunks if possible.
 *
 ****************************************************************************/

FAR void mm_free(FAR struct mm_heap_s *heap, FAR void *mem)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  int ret = (int)getpid();

  /* Check current environment */

  if (up_interrupt_context())
    {
      /* We are in ISR, add to mm_delaylist */

      mm_add_delaylist(heap, mem);
    }
  else if (ret == -ESRCH || sched_idletask())
    {
      /* We are in IDLE task & can't get sem, or meet -ESRCH return,
       * which means we are in situations during context switching(See
       * mm_trysemaphore() & getpid()). Then add to mm_delaylist.
       */

      mm_add_delaylist(heap, mem);
    }
  else
#endif
    {
      host_free(mem);
    }

  return;
}

/****************************************************************************
 * Name: mm_realloc
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

FAR void *mm_realloc(FAR struct mm_heap_s *heap, FAR void *oldmem,
                    size_t size)
{
  mm_free_delaylist(heap);
  return host_realloc(oldmem, size);
}

/****************************************************************************
 * Name: mm_calloc
 *
 * Descriptor:
 *   mm_calloc() calculates the size of the allocation and calls mm_zalloc()
 *
 ****************************************************************************/

FAR void *mm_calloc(FAR struct mm_heap_s *heap, size_t n, size_t elem_size)
{
  mm_free_delaylist(heap);
  return host_calloc(n, elem_size);
}

/****************************************************************************
 * Name: mm_zalloc
 *
 * Description:
 *   mm_zalloc calls mm_malloc, then zeroes out the allocated chunk.
 *
 ****************************************************************************/

FAR void *mm_zalloc(FAR struct mm_heap_s *heap, size_t size)
{
  FAR void *ptr;

  ptr = mm_malloc(heap, size);
  if (ptr != NULL)
    {
      memset(ptr, 0, size);
    }

  return ptr;
}

/****************************************************************************
 * Name: mm_memalign
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

FAR void *mm_memalign(FAR struct mm_heap_s *heap, size_t alignment,
                      size_t size)
{
  mm_free_delaylist(heap);
  return host_memalign(alignment, size);
}

/****************************************************************************
 * Name: mm_heapmember
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

bool mm_heapmember(FAR struct mm_heap_s *heap, FAR void *mem)
{
  return true;
}

/****************************************************************************
 * Name: mm_brkaddr
 *
 * Description:
 *   Return the break address of a heap region.  Zero is returned if the
 *   memory region is not initialized.
 *
 ****************************************************************************/

FAR void *mm_brkaddr(FAR struct mm_heap_s *heap, int region)
{
  return NULL;
}

/****************************************************************************
 * Name: mm_sbrk
 *
 * Description:
 *    The sbrk() function is used to change the amount of space allocated
 *    for the calling process. The change is made by resetting the process's
 *    break value and allocating the appropriate amount of space.  The amount
 *    of allocated space increases as the break value increases.
 *
 *    The sbrk() function adds 'incr' bytes to the break value and changes
 *    the allocated space accordingly. If incr is negative, the amount of
 *    allocated space is decreased by incr bytes. The current value of the
 *    program break is returned by sbrk(0).
 *
 * Input Parameters:
 *    heap - A reference to the data structure that defines this heap.
 *    incr - Specifies the number of bytes to add or to remove from the
 *      space allocated for the process.
 *    maxbreak - The maximum permissible break address.
 *
 * Returned Value:
 *    Upon successful completion, sbrk() returns the prior break value.
 *    Otherwise, it returns (void *)-1 and sets errno to indicate the
 *    error:
 *
 *      ENOMEM - The requested change would allocate more space than
 *        allowed under system limits.
 *      EAGAIN - The total amount of system memory available for allocation
 *        to this process is temporarily insufficient. This may occur even
 *        though the space requested was less than the maximum data segment
 *        size.
 *
 ****************************************************************************/

FAR void *mm_sbrk(FAR struct mm_heap_s *heap, intptr_t incr,
                  uintptr_t maxbreak)
{
  return NULL;
}

/****************************************************************************
 * Name: mm_extend
 *
 * Description:
 *   Extend a heap region by add a block of (virtually) contiguous memory
 *   to the end of the heap.
 *
 ****************************************************************************/

void mm_extend(FAR struct mm_heap_s *heap, FAR void *mem, size_t size,
               int region)
{
}

/****************************************************************************
 * Name: mm_mallinfo
 *
 * Description:
 *   mallinfo returns a copy of updated current heap information.
 *
 ****************************************************************************/

int mm_mallinfo(FAR struct mm_heap_s *heap, FAR struct mallinfo *info)
{
  memset(info, 0, sizeof(struct mallinfo));
  return 0;
}

#ifdef CONFIG_DEBUG_MM

/****************************************************************************
 * Name: mm_checkcorruption
 *
 * Description:
 *   mm_checkcorruption is used to check whether memory heap is normal.
 *
 ****************************************************************************/

void mm_checkcorruption(FAR struct mm_heap_s *heap)
{
}

#endif /* CONFIG_DEBUG_MM */

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

#else /* CONFIG_MM_CUSTOMIZE_MANAGER */

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  /* Note: Some subsystems like modlib and binfmt need to allocate
   * executable memory.
   */

  /* We make the entire heap executable here to keep
   * the sim simpler. If it turns out to be a problem, the
   * ARCH_HAVE_MODULE_TEXT mechanism can be an alternative.
   */

  uint8_t *sim_heap = host_alloc_heap(SIM_HEAP_SIZE);

  *heap_start = sim_heap;
  *heap_size  = SIM_HEAP_SIZE;
}

#endif /* CONFIG_MM_CUSTOMIZE_MANAGER */
