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
#include <nuttx/semaphore.h>
#include <nuttx/mm/mm.h>
#include <nuttx/pgalloc.h>
#include <nuttx/irq.h>

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

struct mm_heap_impl_s
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

  FAR struct mm_delaynode_s *mm_delaylist;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_seminitialize
 *
 * Description:
 *   Initialize the MM mutex
 *
 ****************************************************************************/

static void mm_seminitialize(FAR struct mm_heap_impl_s *impl)
{
  /* Initialize the MM semaphore to one (to support one-at-a-time access to
   * private data sets).
   */

  _SEM_INIT(&impl->mm_semaphore, 0, 1);

  impl->mm_holder      = NO_HOLDER;
  impl->mm_counts_held = 0;
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

static int mm_trysemaphore(FAR struct mm_heap_impl_s *impl)
{
#ifdef CONFIG_SMP
  irqstate_t flags = enter_critical_section();
#endif
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

  if (impl->mm_holder == my_pid)
    {
      /* Yes, just increment the number of references held by the current
       * task.
       */

      impl->mm_counts_held++;
      ret = OK;
    }
  else
    {
      /* Try to take the semaphore */

      ret = _SEM_TRYWAIT(&impl->mm_semaphore);
      if (ret < 0)
        {
          ret = _SEM_ERRVAL(ret);
          goto errout;
        }

      /* We have it.  Claim the heap for the current task and return */

      impl->mm_holder      = my_pid;
      impl->mm_counts_held = 1;
      ret = OK;
    }

errout:
#ifdef CONFIG_SMP
  leave_critical_section(flags);
#endif
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

static void mm_takesemaphore(FAR struct mm_heap_impl_s *impl)
{
#ifdef CONFIG_SMP
  irqstate_t flags = enter_critical_section();
#endif
  pid_t my_pid = getpid();

  /* Does the current task already hold the semaphore? */

  if (impl->mm_holder == my_pid)
    {
      /* Yes, just increment the number of references held by the current
       * task.
       */

      impl->mm_counts_held++;
    }
  else
    {
      int ret;

      /* Take the semaphore (perhaps waiting) */

      do
        {
          ret = _SEM_WAIT(&impl->mm_semaphore);

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

      impl->mm_holder      = my_pid;
      impl->mm_counts_held = 1;
    }

#ifdef CONFIG_SMP
  leave_critical_section(flags);
#endif
}

/****************************************************************************
 * Name: mm_givesemaphore
 *
 * Description:
 *   Release the MM mutex when it is not longer needed.
 *
 ****************************************************************************/

static void mm_givesemaphore(FAR struct mm_heap_impl_s *impl)
{
#ifdef CONFIG_SMP
  irqstate_t flags = enter_critical_section();
#endif

  /* The current task should be holding at least one reference to the
   * semaphore.
   */

  DEBUGASSERT(impl->mm_holder == getpid());

  /* Does the current task hold multiple references to the semaphore */

  if (impl->mm_counts_held > 1)
    {
      /* Yes, just release one count and return */

      impl->mm_counts_held--;
    }
  else
    {
      /* Nope, this is the last reference held by the current task. */

      impl->mm_holder      = NO_HOLDER;
      impl->mm_counts_held = 0;
      DEBUGVERIFY(_SEM_POST(&impl->mm_semaphore));
    }

#ifdef CONFIG_SMP
  leave_critical_section(flags);
#endif
}

/****************************************************************************
 * Name: mm_add_delaylist
 ****************************************************************************/

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
static void mm_add_delaylist(FAR struct mm_heap_s *heap, FAR void *mem)
{
  FAR struct mm_heap_impl_s *impl;
  FAR struct mm_delaynode_s *tmp = mem;
  irqstate_t flags;

  DEBUGASSERT(MM_IS_VALID(heap));
  impl = heap->mm_impl;

  /* Delay the deallocation until a more appropriate time. */

  flags = enter_critical_section();

  tmp->flink = impl->mm_delaylist;
  impl->mm_delaylist = tmp;

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: mm_free_delaylist
 ****************************************************************************/

static void mm_free_delaylist(FAR struct mm_heap_s *heap)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  FAR struct mm_heap_impl_s *impl;
  FAR struct mm_delaynode_s *tmp;
  irqstate_t flags;

  DEBUGASSERT(MM_IS_VALID(heap));
  impl = heap->mm_impl;

  /* Move the delay list to local */

  flags = enter_critical_section();

  tmp = impl->mm_delaylist;
  impl->mm_delaylist = NULL;

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
  FAR struct mm_heap_impl_s *impl;
#if CONFIG_MM_REGIONS > 1
  int idx;

  DEBUGASSERT(MM_IS_VALID(heap));
  impl = heap->mm_impl;
  idx = impl->mm_nregions;

  /* Writing past CONFIG_MM_REGIONS would have catastrophic consequences */

  DEBUGASSERT(idx < CONFIG_MM_REGIONS);
  if (idx >= CONFIG_MM_REGIONS)
    {
      return;
    }

#else
# define idx 0

  DEBUGASSERT(MM_IS_VALID(heap));
  impl = heap->mm_impl;
#endif

  mm_takesemaphore(impl);

  minfo("Region %d: base=%p size=%zu\n", idx + 1, heapstart, heapsize);

  /* Add the size of this region to the total size of the heap */

  impl->mm_heapsize += heapsize;

  /* Save the start and end of the heap */

  impl->mm_heapstart[idx] = heapstart;
  impl->mm_heapend[idx]   = heapstart + heapsize;

#undef idx

#if CONFIG_MM_REGIONS > 1
  impl->mm_nregions++;
#endif

  /* Add memory to the tlsf pool */

  tlsf_add_pool(impl->mm_tlsf, heapstart, heapsize);
  mm_givesemaphore(impl);
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
  FAR struct mm_heap_impl_s *impl;

  DEBUGASSERT(MM_IS_VALID(heap));
  impl = heap->mm_impl;

#if CONFIG_MM_REGIONS > 1
  DEBUGASSERT(region >= 0 && region < impl->mm_nregions);
#else
  DEBUGASSERT(region == 0);
#endif

  return impl->mm_heapend[region];
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
  FAR struct mm_heap_impl_s *impl;

  DEBUGASSERT(MM_IS_VALID(heap));
  impl = heap->mm_impl;

#if CONFIG_MM_REGIONS > 1
  int region;
#else
# define region 0
#endif

  /* Visit each region */

#if CONFIG_MM_REGIONS > 1
  for (region = 0; region < impl->mm_nregions; region++)
#endif
    {
      irqstate_t flags = 0;

      /* Retake the semaphore for each region to reduce latencies */

      if (up_interrupt_context() || sched_idletask())
        {
          if (impl->mm_counts_held)
            {
#if CONFIG_MM_REGIONS > 1
              continue;
#else
              return;
#endif
            }

          flags = enter_critical_section();
        }
      else
        {
          mm_takesemaphore(heap);
        }

      /* Check tlsf control block in the first pass */

      if (region == 0)
        {
          tlsf_check(impl->mm_tlsf);
        }

      /* Check tlsf pool in each iteration temporarily */

      tlsf_check_pool(impl->mm_heapstart[region]);

      /* Release the semaphore */

      if (up_interrupt_context() || sched_idletask())
        {
          leave_critical_section(flags);
        }
      else
        {
          mm_givesemaphore(heap);
        }
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
  FAR struct mm_heap_impl_s *impl;
  size_t oldsize;

  DEBUGASSERT(MM_IS_VALID(heap));
  impl = heap->mm_impl;

  /* Make sure that we were passed valid parameters */

#if CONFIG_MM_REGIONS > 1
  DEBUGASSERT(region >= 0 && region < impl->mm_nregions);
#else
  DEBUGASSERT(region == 0);
#endif
  DEBUGASSERT(mem == impl->mm_heapend[region]);

  /* Take the memory manager semaphore */

  mm_takesemaphore(impl);

  /* Extend the tlsf pool */

  oldsize = impl->mm_heapend[region] - impl->mm_heapstart[region];
  tlsf_extend_pool(impl->mm_tlsf, impl->mm_heapstart[region], oldsize, size);

  /* Save the new size */

  impl->mm_heapsize += size;
  impl->mm_heapend[region] += size;

  mm_givesemaphore(impl);
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
  FAR struct mm_heap_impl_s *impl;
  int ret;

  UNUSED(ret);
  minfo("Freeing %p\n", mem);

  /* Protect against attempts to free a NULL reference */

  if (!mem)
    {
      return;
    }

  DEBUGASSERT(MM_IS_VALID(heap));
  impl = heap->mm_impl;

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  /* Check current environment */

  if (up_interrupt_context())
    {
      /* We are in ISR, add to mm_delaylist */

      mm_add_delaylist(heap, mem);
      return;
    }
  else if ((ret = mm_trysemaphore(impl)) == 0)
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

      mm_takesemaphore(impl);
    }

  /* Return to the tlsf pool */

  tlsf_free(impl->mm_tlsf, mem);
  mm_givesemaphore(impl);
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
  FAR struct mm_heap_impl_s *impl;

  DEBUGASSERT(MM_IS_VALID(heap));
  impl = heap->mm_impl;

#if CONFIG_MM_REGIONS > 1
  int i;

  /* A valid address from the heap for this region would have to lie
   * between the region's two guard nodes.
   */

  for (i = 0; i < impl->mm_nregions; i++)
    {
      if (mem >= impl->mm_heapstart[i] &&
          mem < impl->mm_heapend[i])
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

  if (mem >= impl->mm_heapstart[0] &&
      mem < impl->mm_heapend[0])
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

void mm_initialize(FAR struct mm_heap_s *heap, FAR void *heapstart,
                   size_t heapsize)
{
  FAR struct mm_heap_impl_s *impl;

  minfo("Heap: start=%p size=%zu\n", heapstart, heapsize);

  /* Reserve a block space for mm_heap_impl_s context */

  DEBUGASSERT(heapsize > sizeof(struct mm_heap_impl_s));
  heap->mm_impl = (FAR struct mm_heap_impl_s *)heapstart;
  heapstart += sizeof(struct mm_heap_impl_s);
  heapsize -= sizeof(struct mm_heap_impl_s);

  /* Zero implmeentation context */

  impl = heap->mm_impl;
  memset(impl, 0, sizeof(struct mm_heap_impl_s));

  /* Allocate and create TLSF context */

  DEBUGASSERT(heapsize > tlsf_size());
  impl->mm_tlsf = tlsf_create(heapstart);
  heapstart += tlsf_size();
  heapsize -= tlsf_size();

  /* Initialize the malloc semaphore to one (to support one-at-
   * a-time access to private data sets).
   */

  mm_seminitialize(impl);

  /* Add the initial region of memory to the heap */

  mm_addregion(heap, heapstart, heapsize);
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
  FAR struct mm_heap_impl_s *impl;
#if CONFIG_MM_REGIONS > 1
  int region;
#else
# define region 0
#endif

  DEBUGASSERT(info);
  DEBUGASSERT(MM_IS_VALID(heap));
  impl = heap->mm_impl;

  memset(info, 0, sizeof(struct mallinfo));

  /* Visit each region */

#if CONFIG_MM_REGIONS > 1
  for (region = 0; region < impl->mm_nregions; region++)
#endif
    {
      /* Retake the semaphore for each region to reduce latencies */

      mm_takesemaphore(impl);
      tlsf_walk_pool(impl->mm_heapstart[region],
                     mm_mallinfo_walker, info);
      mm_givesemaphore(impl);
    }
#undef region

  info->arena    = impl->mm_heapsize;
  info->uordblks = info->arena - info->fordblks;

  return OK;
}

/****************************************************************************
 * Name: malloc_usable_size
 ****************************************************************************/

size_t malloc_usable_size(FAR void *mem)
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
  FAR struct mm_heap_impl_s *impl;
  FAR void *ret;

  DEBUGASSERT(MM_IS_VALID(heap));
  impl = heap->mm_impl;

  /* Firstly, free mm_delaylist */

  mm_free_delaylist(heap);

  /* Allocate from the tlsf pool */

  mm_takesemaphore(impl);
  ret = tlsf_malloc(impl->mm_tlsf, size);
  mm_givesemaphore(impl);

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
  FAR struct mm_heap_impl_s *impl;
  FAR void *ret;

  DEBUGASSERT(MM_IS_VALID(heap));
  impl = heap->mm_impl;

  /* Firstly, free mm_delaylist */

  mm_free_delaylist(heap);

  /* Allocate from the tlsf pool */

  mm_takesemaphore(impl);
  ret = tlsf_memalign(impl->mm_tlsf, alignment, size);
  mm_givesemaphore(impl);

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
  FAR struct mm_heap_impl_s *impl;
  FAR void *newmem;

  DEBUGASSERT(MM_IS_VALID(heap));
  impl = heap->mm_impl;

  /* Firstly, free mm_delaylist */

  mm_free_delaylist(heap);

  /* Allocate from the tlsf pool */

  mm_takesemaphore(impl);
  newmem = tlsf_realloc(impl->mm_tlsf, oldmem, size);
  mm_givesemaphore(impl);

  return newmem;
}

#ifdef CONFIG_BUILD_KERNEL
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
  uintptr_t brkaddr;
  uintptr_t allocbase;
  unsigned int pgincr;
  size_t bytesize;
  int errcode;

  DEBUGASSERT(incr >= 0);
  if (incr < 0)
    {
      errcode = ENOSYS;
      goto errout;
    }

  /* Get the current break address (NOTE: assumes region 0).  If
   * the memory manager is uninitialized, mm_brkaddr() will return
   * zero.
   */

  brkaddr = (uintptr_t)mm_brkaddr(heap, 0);
  if (incr > 0)
    {
      /* Convert the increment to multiples of the page size */

      pgincr = MM_NPAGES(incr);

      /* Check if this increment would exceed the maximum break value */

      if ((brkaddr > 0) && ((maxbreak - brkaddr) < (pgincr << MM_PGSHIFT)))
        {
          errcode = ENOMEM;
          goto errout;
        }

      /* Allocate the requested number of pages and map them to the
       * break address.  If we provide a zero brkaddr to pgalloc(),  it
       * will create the first block in the correct virtual address
       * space and return the start address of that block.
       */

      allocbase = pgalloc(brkaddr, pgincr);
      if (allocbase == 0)
        {
          errcode = EAGAIN;
          goto errout;
        }

      /* Has the been been initialized?  brkaddr will be zero if the
       * memory manager has not yet been initialized.
       */

      bytesize = pgincr << MM_PGSHIFT;
      if (brkaddr == 0)
        {
          /* No... then initialize it now */

          mm_initialize(heap, (FAR void *)allocbase, bytesize);
        }
      else
        {
          /* Extend the heap (region 0) */

          mm_extend(heap, (FAR void *)allocbase, bytesize, 0);
        }
    }

  return (FAR void *)brkaddr;

errout:
  set_errno(errcode);
  return (FAR void *)-1;
}
#endif

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
