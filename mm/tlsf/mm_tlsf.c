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
#include <execinfo.h>
#include <malloc.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/mutex.h>
#include <nuttx/mm/mm.h>
#include <nuttx/sched.h>
#include <nuttx/mm/mempool.h>

#include "tlsf/tlsf.h"
#include "kasan/kasan.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if UINTPTR_MAX <= UINT32_MAX
#  define MM_PTR_FMT_WIDTH 11
#elif UINTPTR_MAX <= UINT64_MAX
#  define MM_PTR_FMT_WIDTH 19
#endif

#define MIN(x, y) ((x) < (y) ? (x) : (y))

#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
#  define MEMPOOL_NPOOLS (CONFIG_MM_HEAP_MEMPOOL_THRESHOLD / tlsf_align_size())
#endif

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
   * the following un-named mutex.
   */

  mutex_t mm_lock;

  /* This is the size of the heap provided to mm */

  size_t mm_heapsize;

  /* This is the first and last of the heap */

  FAR void *mm_heapstart[CONFIG_MM_REGIONS];
  FAR void *mm_heapend[CONFIG_MM_REGIONS];

#if CONFIG_MM_REGIONS > 1
  int mm_nregions;
#endif

  tlsf_t mm_tlsf; /* The tlfs context */

  /* The is a multiple mempool of the heap */

#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
  FAR struct mempool_multiple_s *mm_mpool;
#endif

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

struct memdump_info_s
{
  pid_t pid;
  int   blks;
  int   size;
};

#if CONFIG_MM_BACKTRACE >= 0
struct memdump_backtrace_s
{
  pid_t pid;                                /* The pid for caller */
#if CONFIG_MM_BACKTRACE > 0
  FAR void *backtrace[CONFIG_MM_BACKTRACE]; /* The backtrace buffer for caller */
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if CONFIG_MM_BACKTRACE >= 0

/****************************************************************************
 * Name: memdump_backtrace
 ****************************************************************************/

static void memdump_backtrace(FAR struct mm_heap_s *heap,
                              FAR struct memdump_backtrace_s *dump)
{
#  if CONFIG_MM_BACKTRACE > 0
  FAR struct tcb_s *tcb;
#  endif

  dump->pid = getpid();
#  if CONFIG_MM_BACKTRACE > 0
  tcb = nxsched_get_tcb(dump->pid);
  if (heap->mm_procfs.backtrace ||
      (tcb && tcb->flags & TCB_FLAG_HEAPDUMP))
    {
      int ret = backtrace(dump->backtrace, CONFIG_MM_BACKTRACE);
      if (ret < CONFIG_MM_BACKTRACE)
        {
          dump->backtrace[ret] = NULL;
        }
    }
#  endif
}
#endif

/****************************************************************************
 * Name: add_delaylist
 ****************************************************************************/

static void add_delaylist(FAR struct mm_heap_s *heap, FAR void *mem)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  FAR struct mm_delaynode_s *tmp = mem;
  irqstate_t flags;

  /* Delay the deallocation until a more appropriate time. */

  flags = enter_critical_section();

  tmp->flink = heap->mm_delaylist[up_cpu_index()];
  heap->mm_delaylist[up_cpu_index()] = tmp;

  leave_critical_section(flags);
#endif
}

/****************************************************************************
 * Name: free_delaylist
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

/****************************************************************************
 * Name: mallinfo_handler
 ****************************************************************************/

static void mallinfo_handler(FAR void *ptr, size_t size, int used,
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
  else
    {
      info->aordblks++;
    }
}

/****************************************************************************
 * Name: mallinfo_task_handler
 ****************************************************************************/

static void mallinfo_task_handler(FAR void *ptr, size_t size, int used,
                                  FAR void *user)
{
#if CONFIG_MM_BACKTRACE >= 0
  FAR struct memdump_backtrace_s *dump;
#endif
  FAR struct mallinfo_task *info = user;

#if CONFIG_MM_BACKTRACE >= 0
  size -= sizeof(struct memdump_backtrace_s);
  dump = ptr + size;

  if (used)
    {
#if CONFIG_MM_BACKTRACE < 0
      if (info->pid = -1)
#else
      if (info->pid == -1 || info->pid == dump->pid)
#endif
        {
          info->aordblks++;
          info->uordblks += size;
        }
    }
  else if (info->pid == -2)
    {
      info->aordblks++;
      info->uordblks += size;
    }
#endif
}

/****************************************************************************
 * Name: mm_lock
 *
 * Description:
 *   Take the MM mutex. This may be called from the OS in certain conditions
 *   when it is impossible to wait on a mutex:
 *     1.The idle process performs the memory corruption check.
 *     2.The task/thread free the memory in the exiting process.
 *
 * Input Parameters:
 *   heap  - heap instance want to take mutex
 *
 * Returned Value:
 *   0 if the lock can be taken, otherwise negative errno.
 *
 ****************************************************************************/

static int mm_lock(FAR struct mm_heap_s *heap)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  /* Check current environment */

  if (up_interrupt_context())
    {
#if !defined(CONFIG_SMP)
      /* Check the mutex value, if held by someone, then return false.
       * Or, touch the heap internal data directly.
       */

      return nxmutex_is_locked(&heap->mm_lock) ? -EAGAIN : 0;
#else
      /* Can't take mutex in SMP interrupt handler */

      return -EAGAIN;
#endif
    }
  else
#endif

  /* gettid() returns the task ID of the task at the head of the ready-to-
   * run task list.  mm_lock() may be called during context
   * switches.  There are certain situations during context switching when
   * the OS data structures are in flux and then can't be freed immediately
   * (e.g. the running thread stack).
   *
   * This is handled by gettid() to return the special value -ESRCH to
   * indicate this special situation.
   */

  if (gettid() < 0)
    {
      return -ESRCH;
    }
  else
    {
      return nxmutex_lock(&heap->mm_lock);
    }
}

/****************************************************************************
 * Name: mm_unlock
 *
 * Description:
 *   Release the MM mutex when it is not longer needed.
 *
 ****************************************************************************/

static void mm_unlock(FAR struct mm_heap_s *heap)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  if (up_interrupt_context())
    {
      return;
    }
#endif

  DEBUGVERIFY(nxmutex_unlock(&heap->mm_lock));
}

/****************************************************************************
 * Name: memdump_handler
 ****************************************************************************/

static void memdump_handler(FAR void *ptr, size_t size, int used,
                            FAR void *user)
{
  pid_t pid = *(FAR pid_t *)user;
#if CONFIG_MM_BACKTRACE >= 0
  FAR struct memdump_backtrace_s *dump;

  size -= sizeof(struct memdump_backtrace_s);
  dump = ptr + size;
#endif

  if (used)
    {
#if CONFIG_MM_BACKTRACE < 0
      if (pid == -1)
#else
      if (pid == -1 || dump->pid == pid)
#endif
        {
#if CONFIG_MM_BACKTRACE < 0
          syslog(LOG_INFO, "%12zu%*p\n", size, MM_PTR_FMT_WIDTH, ptr);
#else
#  if CONFIG_MM_BACKTRACE > 0
          int i;
          FAR const char *format = " %0*p";
#  endif
          char buf[CONFIG_MM_BACKTRACE * MM_PTR_FMT_WIDTH + 1];

          buf[0] = '\0';
#  if CONFIG_MM_BACKTRACE > 0
          for (i = 0; i < CONFIG_MM_BACKTRACE && dump->backtrace[i]; i++)
            {
              sprintf(buf + i * MM_PTR_FMT_WIDTH, format,
                      MM_PTR_FMT_WIDTH - 1, dump->backtrace[i]);
            }
#  endif

         syslog(LOG_INFO, "%6d%12zu%*p%s\n",
                (int)dump->pid, size, MM_PTR_FMT_WIDTH,
                ptr, buf);
#endif
        }
    }
  else if (pid <= -2)
    {
      syslog(LOG_INFO, "%12zu%*p\n", size, MM_PTR_FMT_WIDTH, ptr);
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

  /* Register to KASan for access check */

  kasan_register(heapstart, &heapsize);

  DEBUGVERIFY(mm_lock(heap));

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
  mm_unlock(heap);
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
      /* Retake the mutex for each region to reduce latencies */

      if (mm_lock(heap) < 0)
        {
          return;
        }

      /* Check tlsf control block in the first pass */

      if (region == 0)
        {
          tlsf_check(heap->mm_tlsf);
        }

      /* Check tlsf pool in each iteration temporarily */

      tlsf_check_pool(heap->mm_heapstart[region]);

      /* Release the mutex */

      mm_unlock(heap);
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

  /* Take the memory manager mutex */

  DEBUGVERIFY(mm_lock(heap));

  /* Extend the tlsf pool */

  oldsize = heap->mm_heapend[region] - heap->mm_heapstart[region];
  tlsf_extend_pool(heap->mm_tlsf, heap->mm_heapstart[region], oldsize, size);

  /* Save the new size */

  heap->mm_heapsize += size;
  heap->mm_heapend[region] += size;

  mm_unlock(heap);
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

#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
  if (mempool_multiple_free(heap->mm_mpool, mem) >= 0)
    {
      return;
    }
#endif

  if (mm_lock(heap) == 0)
    {
      kasan_poison(mem, mm_malloc_size(heap, mem));

      /* Pass, return to the tlsf pool */

      tlsf_free(heap->mm_tlsf, mem);
      mm_unlock(heap);
    }
  else
    {
      /* Add to the delay list(see the comment in mm_lock) */

      add_delaylist(heap, mem);
    }
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
#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
  size_t poolsize[MEMPOOL_NPOOLS];
  int i;
#endif

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

  /* Initialize the malloc mutex (to support one-at-
   * a-time access to private data sets).
   */

  nxmutex_init(&heap->mm_lock);

  /* Add the initial region of memory to the heap */

  mm_addregion(heap, heapstart, heapsize);

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  heap->mm_procfs.name = name;
  heap->mm_procfs.heap = heap;
#  ifdef CONFIG_MM_BACKTRACE_DEFAULT
  heap->mm_procfs.backtrace = true;
#  endif
  procfs_register_meminfo(&heap->mm_procfs);
#endif
#endif

#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
  for (i = 0; i < MEMPOOL_NPOOLS; i++)
    {
      poolsize[i] = (i + 1) * tlsf_align_size();
    }

  heap->mm_mpool = mempool_multiple_init(name, poolsize, MEMPOOL_NPOOLS,
                                    (mempool_multiple_alloc_t)mm_memalign,
                                    (mempool_multiple_free_t)mm_free, heap,
                                    CONFIG_MM_HEAP_MEMPOOL_EXPAND);
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
      /* Retake the mutex for each region to reduce latencies */

      DEBUGVERIFY(mm_lock(heap));
      tlsf_walk_pool(heap->mm_heapstart[region],
                     mallinfo_handler, info);
      mm_unlock(heap);
    }
#undef region

  info->arena    = heap->mm_heapsize;
  info->uordblks = info->arena - info->fordblks;

  return OK;
}

int mm_mallinfo_task(FAR struct mm_heap_s *heap,
                     FAR struct mallinfo_task *info)
{
#if CONFIG_MM_REGIONS > 1
  int region;
#else
#define region 0
#endif

  DEBUGASSERT(info);
  info->uordblks = 0;
  info->aordblks = 0;

#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
  mempool_multiple_info_task(heap->mm_mpool, info);
#endif

#if CONFIG_MM_REGIONS > 1
  for (region = 0; region < heap->mm_nregions; region++)
#endif
    {
      /* Retake the mutex for each region to reduce latencies */

      DEBUGVERIFY(mm_lock(heap));
      tlsf_walk_pool(heap->mm_heapstart[region],
                     mallinfo_task_handler, info);
      mm_unlock(heap);
    }
#undef region

  return OK;
}

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

void mm_memdump(FAR struct mm_heap_s *heap, pid_t pid)
{
#if CONFIG_MM_REGIONS > 1
  int region;
#else
# define region 0
#endif
  struct memdump_info_s info;

  if (pid >= -1)
    {
      syslog(LOG_INFO, "Dump all used memory node info:\n");
#if CONFIG_MM_BACKTRACE < 0
      syslog(LOG_INFO, "%12s%*s\n", "Size", MM_PTR_FMT_WIDTH, "Address");
#else
      syslog(LOG_INFO, "%6s%12s%*s %s\n", "PID", "Size", MM_PTR_FMT_WIDTH,
            "Address", "Backtrace");
#endif
    }
  else
    {
      syslog(LOG_INFO, "Dump all free memory node info:\n");
      syslog(LOG_INFO, "%12s%*s\n", "Size", MM_PTR_FMT_WIDTH, "Address");
    }

#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
  mempool_multiple_memdump(heap->mm_mpool, pid);
#endif

#if CONFIG_MM_REGIONS > 1
  for (region = 0; region < heap->mm_nregions; region++)
#endif
    {
      DEBUGVERIFY(mm_lock(heap));
      tlsf_walk_pool(heap->mm_heapstart[region],
                     memdump_handler, &pid);
      mm_unlock(heap);
    }
#undef region

  info.pid = pid;
  mm_mallinfo_task(heap, &info);
  syslog(LOG_INFO, "%12s%12s\n", "Total Blks", "Total Size");
  syslog(LOG_INFO, "%12d%12d\n", info.blks, info.size);
}

/****************************************************************************
 * Name: mm_malloc_size
 ****************************************************************************/

size_t mm_malloc_size(FAR struct mm_heap_s *heap, FAR void *mem)
{
#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
  ssize_t size = mempool_multiple_alloc_size(heap->mm_mpool, mem);
  if (size >= 0)
    {
      return size;
    }
#endif

#if CONFIG_MM_BACKTRACE >= 0
  return tlsf_block_size(mem) - sizeof(struct memdump_backtrace_s);
#else
  return tlsf_block_size(mem);
#endif
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

#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
  ret = mempool_multiple_alloc(heap->mm_mpool, size);
  if (ret != NULL)
    {
      return ret;
    }
#endif

  /* Free the delay list first */

  free_delaylist(heap);

  /* Allocate from the tlsf pool */

  DEBUGVERIFY(mm_lock(heap));
#if CONFIG_MM_BACKTRACE >= 0
  ret = tlsf_malloc(heap->mm_tlsf, size +
                    sizeof(struct memdump_backtrace_s));
#else
  ret = tlsf_malloc(heap->mm_tlsf, size);
#endif

  mm_unlock(heap);

  if (ret)
    {
#if CONFIG_MM_BACKTRACE >= 0
      FAR struct memdump_backtrace_s *dump = ret + mm_malloc_size(heap, ret);

      memdump_backtrace(heap, dump);
#endif
      kasan_unpoison(ret, mm_malloc_size(heap, ret));
    }

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

#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
  ret = mempool_multiple_memalign(heap->mm_mpool, alignment, size);
  if (ret != NULL)
    {
      return ret;
    }
#endif

  /* Free the delay list first */

  free_delaylist(heap);

  /* Allocate from the tlsf pool */

  DEBUGVERIFY(mm_lock(heap));
#if CONFIG_MM_BACKTRACE >= 0
  ret = tlsf_memalign(heap->mm_tlsf, alignment, size +
                      sizeof(struct memdump_backtrace_s));
#else
  ret = tlsf_memalign(heap->mm_tlsf, alignment, size);
#endif
  mm_unlock(heap);

  if (ret)
    {
#if CONFIG_MM_BACKTRACE >= 0
      FAR struct memdump_backtrace_s *dump = ret + mm_malloc_size(heap, ret);

      memdump_backtrace(heap, dump);
#endif
      kasan_unpoison(ret, mm_malloc_size(heap, ret));
    }

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

  if (oldmem == NULL)
    {
      return mm_malloc(heap, size);
    }

  if (size == 0)
    {
      mm_free(heap, oldmem);
      return NULL;
    }

#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
  newmem = mempool_multiple_realloc(heap->mm_mpool, oldmem, size);
  if (newmem != NULL)
    {
      return newmem;
    }
  else if (size <= CONFIG_MM_HEAP_MEMPOOL_THRESHOLD ||
           mempool_multiple_alloc_size(heap->mm_mpool, oldmem) >= 0)
    {
      newmem = mm_malloc(heap, size);
      if (newmem != 0)
        {
          memcpy(newmem, oldmem, size);
          mm_free(heap, oldmem);
          return newmem;
        }
    }
#endif

#ifdef CONFIG_MM_KASAN

  newmem = mm_malloc(heap, size);
  if (newmem)
    {
      if (size > mm_malloc_size(heap, oldmem))
        {
          size = mm_malloc_size(heap, oldmem);
        }

      memcpy(newmem, oldmem, size);
      mm_free(heap, oldmem);
    }
#else
  /* Free the delay list first */

  free_delaylist(heap);

  /* Allocate from the tlsf pool */

  DEBUGVERIFY(mm_lock(heap));
#if CONFIG_MM_BACKTRACE >= 0
  newmem = tlsf_realloc(heap->mm_tlsf, oldmem, size +
                        sizeof(struct memdump_backtrace_s));
#else
  newmem = tlsf_realloc(heap->mm_tlsf, oldmem, size);
#endif
  mm_unlock(heap);

#if CONFIG_MM_BACKTRACE >= 0
  if (newmem)
    {
      FAR struct memdump_backtrace_s *dump = newmem +
                                             mm_malloc_size(heap, newmem);

      memdump_backtrace(heap, dump);
    }
#endif

#endif

  return newmem;
}

/****************************************************************************
 * Name: mm_uninitialize
 *
 * Description:
 *   Uninitialize the selected heap data structures.
 *
 * Input Parameters:
 *   heap - The heap to uninitialize
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mm_uninitialize(FAR struct mm_heap_s *heap)
{
#if CONFIG_MM_HEAP_MEMPOOL_THRESHOLD != 0
  mempool_multiple_deinit(heap->mm_mpool);
#endif

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
#  if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  procfs_unregister_meminfo(&heap->mm_procfs);
#  endif
#endif
  nxmutex_destroy(&heap->mm_lock);
  tlsf_destroy(&heap->mm_tlsf);
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
