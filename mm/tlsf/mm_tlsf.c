/****************************************************************************
 * mm/tlsf/mm_tlsf.c
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

#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <malloc.h>
#include <sched.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/semaphore.h>
#include <nuttx/mm/mm.h>

#include "tlsf/tlsf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is a special value that indicates that there is no holder of the
 * semaphore.  The valid range of PIDs is 0-32767 and any value outside of
 * that range could be used (except -ESRCH which is a special return value
 * from getpid())
 */

#define NO_HOLDER ((pid_t)-1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mm_delaynode_s
{
  FAR struct mm_delaynode_s *flink;
};

struct mm_heap_s
{
  /* Mutually exclusive access to this data set is enforced with
   * the following un-named semaphore.
   */

  sem_t mm_semaphore;
  pid_t mm_holder;
  int mm_counts_held;

  /* This is the size of the heap provided to mm */

  size_t mm_heapsize;

  /* This is the first and last of the heap */

  FAR void *mm_heapstart[CONFIG_MM_REGIONS];
  FAR void *mm_heapend[CONFIG_MM_REGIONS];

#if CONFIG_MM_REGIONS > 1
  int mm_nregions;
#endif

  tlsf_t mm_tlsf; /* The tlfs context */

  /* Free delay list, for some situation can't do free immdiately */

#ifdef CONFIG_SMP
  struct mm_delaynode_s *mm_delaylist[CONFIG_SMP_NCPUS];
#else
  struct mm_delaynode_s *mm_delaylist[1];
#endif

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
  struct procfs_meminfo_entry_s mm_procfs;
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_add_delaylist
 ****************************************************************************/

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
static void mm_add_delaylist(FAR struct mm_heap_s *heap, FAR void *mem)
{
  FAR struct mm_delaynode_s *tmp = mem;
  irqstate_t flags;

  /* Delay the deallocation until a more appropriate time. */

  flags = enter_critical_section();

  tmp->flink = heap->mm_delaylist[up_cpu_index()];
  heap->mm_delaylist[up_cpu_index()] = tmp;

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: mm_free_delaylist
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

/****************************************************************************
 * Name: mm_mallinfo_walker
 ****************************************************************************/

static void mm_mallinfo_walker(FAR void *ptr, size_t size, int used,
                               FAR void *user)
{
  FAR struct mallinfo *info = user;

  if (!used)
    {
      info->ordblks++;
      info->fordblks += size;
      if (size > info->mxordblk)
        {
          info->mxordblk = size;
        }
    }
}

/****************************************************************************
 * Name: mm_seminitialize
 *
 * Description:
 *   Initialize the MM mutex
 *
 ****************************************************************************/

static void mm_seminitialize(FAR struct mm_heap_s *heap)
{
  /* Initialize the MM semaphore to one (to support one-at-a-time access to
   * private data sets).
   */

  _SEM_INIT(&heap->mm_semaphore, 0, 1);

  heap->mm_holder      = NO_HOLDER;
  heap->mm_counts_held = 0;
}

/****************************************************************************
 * Name: mm_trysemaphore
 *
 * Description:
 *   Try to take the MM mutex.  This is called only from the OS in certain
 *   conditions when it is necessary to have exclusive access to the memory
 *   manager but it is impossible to wait on a semaphore (e.g., the idle
 *   process when it performs its background memory cleanup).
 *
 ****************************************************************************/

static int mm_trysemaphore(FAR struct mm_heap_s *heap)
{
  pid_t my_pid = getpid();
  int ret;

  /* getpid() returns the task ID of the task at the head of the ready-to-
   * run task list.  mm_trysemaphore() may be called during context
   * switches.  There are certain situations during context switching when
   * the OS data structures are in flux and where the current task (i.e.,
   * the task at the head of the ready-to-run task list) is not actually
   * running. Granting the semaphore access in that case is known to result
   * in heap corruption as in the following failure scenario.
   *
   * ----------------------------    -------------------------------
   * TASK A                          TASK B
   * ----------------------------    -------------------------------
   *                                 Begins memory allocation.
   *                                 - Holder is set to TASK B
   *                             <---- Task B preempted, Task A runs
   * Task A exits
   * - Current task set to Task B
   * Free tcb and stack memory
   * - Since holder is Task B,
   *   memory manager is re-
   *   entered, and
   * - Heap is corrupted.
   * ----------------------------    -------------------------------
   *
   * This is handled by getpid():  If the case where Task B is not actually
   * running, then getpid() will return the special value -ESRCH.  That will
   * avoid taking the fatal 'if' logic and will fall through to use the
   * 'else', albeit with a nonsensical PID value.
   */

  if (my_pid < 0)
    {
      ret = my_pid;
      goto errout;
    }

  /* Does the current task already hold the semaphore?  Is the current
   * task actually running?
   */

  if (heap->mm_holder == my_pid)
    {
      /* Yes, just increment the number of references held by the current
       * task.
       */

      heap->mm_counts_held++;
      ret = OK;
    }
  else
    {
      /* Try to take the semaphore */

      ret = _SEM_TRYWAIT(&heap->mm_semaphore);
      if (ret < 0)
        {
          ret = _SEM_ERRVAL(ret);
          goto errout;
        }

      /* We have it.  Claim the heap for the current task and return */

      heap->mm_holder      = my_pid;
      heap->mm_counts_held = 1;
      ret = OK;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: mm_takesemaphore
 *
 * Description:
 *   Take the MM mutex.  This is the normal action before all memory
 *   management actions.
 *
 ****************************************************************************/

static void mm_takesemaphore(FAR struct mm_heap__s *heap)
{
  pid_t my_pid = getpid();

  /* Does the current task already hold the semaphore? */

  if (heap->mm_holder == my_pid)
    {
      /* Yes, just increment the number of references held by the current
       * task.
       */

      heap->mm_counts_held++;
    }
  else
    {
      int ret;

      /* Take the semaphore (perhaps waiting) */

      do
        {
          ret = _SEM_WAIT(&heap->mm_semaphore);

          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          if (ret < 0)
            {
              ret = _SEM_ERRVAL(ret);
              DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
            }
        }
      while (ret == -EINTR);

      /* We have it (or some awful, unexpected error occurred).  Claim
       * the semaphore for the current task and return.
       */

      heap->mm_holder      = my_pid;
      heap->mm_counts_held = 1;
    }
}

/****************************************************************************
 * Name: mm_givesemaphore
 *
 * Description:
 *   Release the MM mutex when it is not longer needed.
 *
 ****************************************************************************/

static void mm_givesemaphore(FAR struct mm_heap_s *heap)
{
  /* The current task should be holding at least one reference to the
   * semaphore.
   */

  DEBUGASSERT(heap->mm_holder == getpid());

  /* Does the current task hold multiple references to the semaphore */

  if (heap->mm_counts_held > 1)
    {
      /* Yes, just release one count and return */

      heap->mm_counts_held--;
    }
  else
    {
      /* Nope, this is the last reference held by the current task. */

      heap->mm_holder      = NO_HOLDER;
      heap->mm_counts_held = 0;
      DEBUGVERIFY(_SEM_POST(&heap->mm_semaphore));
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
#if CONFIG_MM_REGIONS > 1
  int idx;

  idx = heap->mm_nregions;

  /* Writing past CONFIG_MM_REGIONS would have catastrophic consequences */

  DEBUGASSERT(idx < CONFIG_MM_REGIONS);
  if (idx >= CONFIG_MM_REGIONS)
    {
      return;
    }

#else
# define idx 0
#endif

  mm_takesemaphore(heap);

  minfo("Region %d: base=%p size=%zu\n", idx + 1, heapstart, heapsize);

  /* Add the size of this region to the total size of the heap */

  heap->mm_heapsize += heapsize;

  /* Save the start and end of the heap */

  heap->mm_heapstart[idx] = heapstart;
  heap->mm_heapend[idx]   = heapstart + heapsize;

#undef idx

#if CONFIG_MM_REGIONS > 1
  heap->mm_nregions++;
#endif

  /* Add memory to the tlsf pool */

  tlsf_add_pool(heap->mm_tlsf, heapstart, heapsize);
  mm_givesemaphore(heap);
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
#if CONFIG_MM_REGIONS > 1
  DEBUGASSERT(region >= 0 && region < heap->mm_nregions);
#else
  DEBUGASSERT(region == 0);
#endif

  return heap->mm_heapend[region];
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
  FAR void *ret = NULL;

  /* Verify input parameters */

  if (n > 0 && elem_size > 0)
    {
      /* Assure that the following multiplication cannot overflow the size_t
       * type, i.e., that:  SIZE_MAX >= n * elem_size
       *
       * Refer to SEI CERT C Coding Standard.
       */

      if (n <= (SIZE_MAX / elem_size))
        {
          ret = mm_zalloc(heap, n * elem_size);
        }
    }

  return ret;
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
#if CONFIG_MM_REGIONS > 1
  int region;
#else
# define region 0
#endif

  /* Visit each region */

#if CONFIG_MM_REGIONS > 1
  for (region = 0; region < heap->mm_nregions; region++)
#endif
    {
      /* Retake the semaphore for each region to reduce latencies */

      if (up_interrupt_context())
        {
          return;
        }
      else if (sched_idletask())
        {
          if (mm_trysemaphore(heap))
            {
              return;
            }
        }
      else
        {
          mm_takesemaphore(heap);
        }

      /* Check tlsf control block in the first pass */

      if (region == 0)
        {
          tlsf_check(heap->mm_tlsf);
        }

      /* Check tlsf pool in each iteration temporarily */

      tlsf_check_pool(heap->mm_heapstart[region]);

      /* Release the semaphore */

      mm_givesemaphore(heap);
    }
#undef region
}
#endif

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
  size_t oldsize;

  /* Make sure that we were passed valid parameters */

#if CONFIG_MM_REGIONS > 1
  DEBUGASSERT(region >= 0 && region < heap->mm_nregions);
#else
  DEBUGASSERT(region == 0);
#endif
  DEBUGASSERT(mem == heap->mm_heapend[region]);

  /* Take the memory manager semaphore */

  mm_takesemaphore(heap);

  /* Extend the tlsf pool */

  oldsize = heap->mm_heapend[region] - heap->mm_heapstart[region];
  tlsf_extend_pool(heap->mm_tlsf, heap->mm_heapstart[region], oldsize, size);

  /* Save the new size */

  heap->mm_heapsize += size;
  heap->mm_heapend[region] += size;

  mm_givesemaphore(heap);
}

/****************************************************************************
 * Name: mm_free
 *
 * Description:
 *   Returns a chunk of memory to the list of free nodes,  merging with
 *   adjacent free chunks if possible.
 *
 ****************************************************************************/

void mm_free(FAR struct mm_heap_s *heap, FAR void *mem)
{
  int ret;

  UNUSED(ret);
  minfo("Freeing %p\n", mem);

  /* Protect against attempts to free a NULL reference */

  if (!mem)
    {
      return;
    }

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  /* Check current environment */

  if (up_interrupt_context())
    {
      /* We are in ISR, add to mm_delaylist */

      mm_add_delaylist(heap, mem);
      return;
    }
  else if ((ret = mm_trysemaphore(heap)) == 0)
    {
      /* Got the sem, do free immediately */
    }
  else if (ret == -ESRCH || sched_idletask())
    {
      /* We are in IDLE task & can't get sem, or meet -ESRCH return,
       * which means we are in situations during context switching(See
       * mm_trysemaphore() & getpid()). Then add to mm_delaylist.
       */

      mm_add_delaylist(heap, mem);
      return;
    }
  else
#endif
    {
      /* We need to hold the MM semaphore while we muck with the
       * nodelist.
       */

      mm_takesemaphore(heap);
    }

  /* Return to the tlsf pool */

  tlsf_free(heap->mm_tlsf, mem);
  mm_givesemaphore(heap);
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
#if CONFIG_MM_REGIONS > 1
  int i;

  /* A valid address from the heap for this region would have to lie
   * between the region's two guard nodes.
   */

  for (i = 0; i < heap->mm_nregions; i++)
    {
      if (mem >= heap->mm_heapstart[i] &&
          mem < heap->mm_heapend[i])
        {
          return true;
        }
    }

  /* The address does not like any any region assigned to the heap */

  return false;

#else
  /* A valid address from the heap would have to lie between the
   * two guard nodes.
   */

  if (mem >= heap->mm_heapstart[0] &&
      mem < heap->mm_heapend[0])
    {
      return true;
    }

  /* Otherwise, the address does not lie in the heap */

  return false;

#endif
}

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

FAR struct mm_heap_s *mm_initialize(FAR const char *name,
                                    FAR void *heapstart, size_t heapsize)
{
  FAR struct mm_heap_s *heap;

  minfo("Heap: name=%s start=%p size=%zu\n", name, heapstart, heapsize);

  /* Reserve a block space for mm_heap_s context */

  DEBUGASSERT(heapsize > sizeof(struct mm_heap_s));
  heap = (FAR struct mm_heap_s *)heapstart;
  memset(heap, 0, sizeof(struct mm_heap_s));
  heapstart += sizeof(struct mm_heap_s);
  heapsize -= sizeof(struct mm_heap_s);

  /* Allocate and create TLSF context */

  DEBUGASSERT(heapsize > tlsf_size());
  heap->mm_tlsf = tlsf_create(heapstart);
  heapstart += tlsf_size();
  heapsize -= tlsf_size();

  /* Initialize the malloc semaphore to one (to support one-at-
   * a-time access to private data sets).
   */

  mm_seminitialize(heap);

  /* Add the initial region of memory to the heap */

  mm_addregion(heap, heapstart, heapsize);

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  heap->mm_procfs.name = name;
  heap->mm_procfs.mallinfo = (FAR void *)mm_mallinfo;
  heap->mm_procfs.user_data = heap;
  procfs_register_meminfo(&heap->mm_procfs);
#endif
#endif

  return heap;
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
#if CONFIG_MM_REGIONS > 1
  int region;
#else
# define region 0
#endif

  DEBUGASSERT(info);

  memset(info, 0, sizeof(struct mallinfo));

  /* Visit each region */

#if CONFIG_MM_REGIONS > 1
  for (region = 0; region < heap->mm_nregions; region++)
#endif
    {
      /* Retake the semaphore for each region to reduce latencies */

      mm_takesemaphore(heap);
      tlsf_walk_pool(heap->mm_heapstart[region],
                     mm_mallinfo_walker, info);
      mm_givesemaphore(heap);
    }
#undef region

  info->arena    = heap->mm_heapsize;
  info->uordblks = info->arena - info->fordblks;

  return OK;
}

/****************************************************************************
 * Name: mm_malloc_size
 ****************************************************************************/

size_t mm_malloc_size(FAR void *mem)
{
  return tlsf_block_size(mem);
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
  FAR void *ret;

  /* Firstly, free mm_delaylist */

  mm_free_delaylist(heap);

  /* Allocate from the tlsf pool */

  mm_takesemaphore(heap);
  ret = tlsf_malloc(heap->mm_tlsf, size);
  mm_givesemaphore(heap);

  return ret;
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
  FAR void *ret;

  /* Firstly, free mm_delaylist */

  mm_free_delaylist(heap);

  /* Allocate from the tlsf pool */

  mm_takesemaphore(heap);
  ret = tlsf_memalign(heap->mm_tlsf, alignment, size);
  mm_givesemaphore(heap);

  return ret;
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
  FAR void *newmem;

  /* Firstly, free mm_delaylist */

  mm_free_delaylist(heap);

  /* Allocate from the tlsf pool */

  mm_takesemaphore(heap);
  newmem = tlsf_realloc(heap->mm_tlsf, oldmem, size);
  mm_givesemaphore(heap);

  return newmem;
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
  FAR void *alloc = mm_malloc(heap, size);

  if (alloc)
    {
       memset(alloc, 0, size);
    }

  return alloc;
}
