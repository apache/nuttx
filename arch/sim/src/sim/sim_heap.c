/****************************************************************************
 * arch/sim/src/sim/sim_heap.c
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
#include <string.h>
#include <malloc.h>
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/mm/mm.h>

#include "sim_internal.h"

#ifdef CONFIG_MM_CUSTOMIZE_MANAGER

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
  struct mm_delaynode_s *mm_delaylist[CONFIG_SMP_NCPUS];

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
  struct procfs_meminfo_entry_s mm_procfs;
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void mm_add_delaylist(struct mm_heap_s *heap, void *mem)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  struct mm_delaynode_s *tmp = mem;
  irqstate_t flags;

  /* Delay the deallocation until a more appropriate time. */

  flags = enter_critical_section();

  tmp->flink = heap->mm_delaylist[up_cpu_index()];
  heap->mm_delaylist[up_cpu_index()] = tmp;

  leave_critical_section(flags);
#endif
}

static void mm_free_delaylist(struct mm_heap_s *heap)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  struct mm_delaynode_s *tmp;
  irqstate_t flags;

  /* Move the delay list to local */

  flags = enter_critical_section();

  tmp = heap->mm_delaylist[up_cpu_index()];
  heap->mm_delaylist[up_cpu_index()] = NULL;

  leave_critical_section(flags);

  /* Test if the delayed is empty */

  while (tmp)
    {
      void *address;

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

struct mm_heap_s *mm_initialize(const char *name,
                                    void *heap_start, size_t heap_size)
{
  struct mm_heap_s *heap;

  heap = host_memalign(sizeof(void *), sizeof(*heap));
  DEBUGASSERT(heap);

  memset(heap, 0, sizeof(struct mm_heap_s));

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
  heap->mm_procfs.name = name;
  heap->mm_procfs.heap = heap;
  procfs_register_meminfo(&heap->mm_procfs);
#endif

  return heap;
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

void mm_addregion(struct mm_heap_s *heap, void *heapstart,
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

void *mm_malloc(struct mm_heap_s *heap, size_t size)
{
  return mm_realloc(heap, NULL, size);
}

/****************************************************************************
 * Name: mm_free
 *
 * Description:
 *   Returns a chunk of memory to the list of free nodes,  merging with
 *   adjacent free chunks if possible.
 *
 ****************************************************************************/

void mm_free(struct mm_heap_s *heap, void *mem)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  /* Check current environment */

  if (up_interrupt_context())
    {
      /* We are in ISR, add to the delay list */

      mm_add_delaylist(heap, mem);
    }
  else
#endif

  if (nxsched_gettid() < 0)
    {
      /* nxsched_gettid() return -ESRCH, means we are in situations
       * during context switching(See nxsched_gettid's comment).
       * Then add to the delay list.
       */

      mm_add_delaylist(heap, mem);
    }
  else
    {
      host_free(mem);
    }
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

void *mm_realloc(struct mm_heap_s *heap, void *oldmem,
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

void *mm_calloc(struct mm_heap_s *heap, size_t n, size_t elem_size)
{
  size_t size = n * elem_size;

  if (size < elem_size)
    {
      return NULL;
    }

  return mm_zalloc(heap, size);
}

/****************************************************************************
 * Name: mm_zalloc
 *
 * Description:
 *   mm_zalloc calls mm_malloc, then zeroes out the allocated chunk.
 *
 ****************************************************************************/

void *mm_zalloc(struct mm_heap_s *heap, size_t size)
{
  void *ptr;

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

void *mm_memalign(struct mm_heap_s *heap, size_t alignment,
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

bool mm_heapmember(struct mm_heap_s *heap, void *mem)
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

void *mm_brkaddr(struct mm_heap_s *heap, int region)
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

void mm_extend(struct mm_heap_s *heap, void *mem, size_t size,
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

int mm_mallinfo(struct mm_heap_s *heap, struct mallinfo *info)
{
  memset(info, 0, sizeof(struct mallinfo));
  host_mallinfo(&info->aordblks, &info->uordblks);
  return 0;
}

/****************************************************************************
 * Name: mm_mallinfo_task
 *
 * Description:
 *   mallinfo_task returns a copy of updated current task's heap information.
 *
 ****************************************************************************/

int mm_mallinfo_task(struct mm_heap_s *heap, struct mallinfo_task *info)
{
  info->aordblks = 0;
  info->uordblks = 0;
  return 0;
}

/****************************************************************************
 * Name: mm_memdump
 *
 * Description:
 *   mm_memdump returns a memory info about specified pid of task/thread.
 *
 ****************************************************************************/

void mm_memdump(struct mm_heap_s *heap, pid_t pid)
{
}

#ifdef CONFIG_DEBUG_MM

/****************************************************************************
 * Name: mm_checkcorruption
 *
 * Description:
 *   mm_checkcorruption is used to check whether memory heap is normal.
 *
 ****************************************************************************/

void mm_checkcorruption(struct mm_heap_s *heap)
{
}

#endif /* CONFIG_DEBUG_MM */

/****************************************************************************
 * Name: malloc_size
 ****************************************************************************/

size_t mm_malloc_size(struct mm_heap_s *heap, void *mem)
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

#else /* CONFIG_MM_CUSTOMIZE_MANAGER */

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  /* Note: Some subsystems like modlib and binfmt need to allocate
   * executable memory.
   */

  /* We make the entire heap executable here to keep
   * the sim simpler. If it turns out to be a problem, the
   * ARCH_HAVE_TEXT_HEAP mechanism can be an alternative.
   */

  uint8_t *sim_heap = host_allocheap(SIM_HEAP_SIZE);

  *heap_start = sim_heap;
  *heap_size  = SIM_HEAP_SIZE;
}

#endif /* CONFIG_MM_CUSTOMIZE_MANAGER */
