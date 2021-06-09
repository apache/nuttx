/****************************************************************************
 * mm/dmalloc/mm_dmalloc.c
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

#define DMALLOC_DISABLE

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>
#include <malloc.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/mm/mm.h>
#include <nuttx/pgalloc.h>
#include <nuttx/semaphore.h>

#include "error_val.h"
#include "dmalloc.h"
#include "dmalloc_loc.h"
#include "return.h"

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

  FAR sem_t *mm_semaphore;

  /* This is the first and last of the heap */

  FAR void *mm_heapstart[CONFIG_MM_REGIONS];
  size_t mm_heapsize[CONFIG_MM_REGIONS];

  size_t mm_heapnext;

#if CONFIG_MM_REGIONS > 1
  int mm_region;
  int mm_nregions;
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

/****************************************************************************
 * Public Data
 ****************************************************************************/

FAR struct mm_heap_s *g_mmheap;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: umm_add_delaylist
 ****************************************************************************/

static void umm_add_delaylist(FAR void *mem)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  FAR struct mm_heap_s *heap = g_mmheap;
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
 * Name: umm_free_delaylist
 ****************************************************************************/

static void umm_free_delaylist(void)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  FAR struct mm_heap_s *heap = g_mmheap;
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

      free(address);
    }
#endif
}

/****************************************************************************
 * Name: umm_mallinfo
 ****************************************************************************/

static void umm_mallinfo(FAR void *data, FAR struct mallinfo *info)
{
  *info = mallinfo();
}

/****************************************************************************
 * Name: umm_try_initialize
 *
 * Description:
 *   Allocate and initialize the user heap if not yet.
 *
 * Input Parameters:
 * None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
static void umm_try_initialize(void)
{
  uintptr_t allocbase;

  /* Return if the user heap is already initialized. */

  if (g_mmheap != NULL)
    {
      return;
    }

  /* Allocate one page. If we provide a zero brkaddr to pgalloc(),
   * it will create the first block in the correct virtual address
   * space and return the start address of that block.
   */

  allocbase = pgalloc(0, 1);
  DEBUGASSERT(allocbase != 0);

  /* Let umm_initialize do the real work. */

  umm_initialize((FAR void *)allocbase, CONFIG_MM_PGSIZE);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: umm_initsemaphore
 *
 * Description:
 *   Initialize the MM mutex
 *
 ****************************************************************************/

void umm_initsemaphore(FAR sem_t *sem, int val)
{
  FAR struct mm_heap_s *heap = g_mmheap;

  /* Initialize the MM semaphore to one (to support one-at-a-time access to
   * private data sets).
   */

  _SEM_INIT(sem, 0, val);
  heap->mm_semaphore = sem;
}

/****************************************************************************
 * Name: umm_takesemaphore
 *
 * Description:
 *   Take the MM mutex. This may be called from the OS in certain conditions
 *   when it is impossible to wait on a semaphore:
 *     1.The idle process performs the memory corruption check.
 *     2.The task/thread free the memory in the exiting process.
 *
 * Input Parameters:
 *   sem  - semaphore to be taken
 *
 * Returned Value:
 *   true if the semaphore can be taken, otherwise false.
 *
 ****************************************************************************/

bool umm_takesemaphore(FAR sem_t *sem)
{
  if (!sem)
    {
      return true;
    }

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  /* Check current environment */

  if (up_interrupt_context())
    {
      /* Can't take semaphore in the interrupt handler */

      return false;
    }
  else
#endif

  /* getpid() returns the task ID of the task at the head of the ready-to-
   * run task list.  umm_takesemaphore() may be called during context
   * switches.  There are certain situations during context switching when
   * the OS data structures are in flux and then can't be freed immediately
   * (e.g. the running thread stack).
   *
   * This is handled by getpid() to return the special value -ESRCH to
   * indicate this special situation.
   */

  if (getpid() < 0)
    {
      return false;
    }
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  else if (sched_idletask())
    {
      /* Try to take the semaphore */

      return _SEM_TRYWAIT(sem) >= 0;
    }
#endif
  else
    {
      int ret;

      /* Take the semaphore (perhaps waiting) */

      do
        {
          ret = _SEM_WAIT(sem);

          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          if (ret < 0)
            {
              ret = _SEM_ERRVAL(ret);
              DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
            }
        }
      while (ret < 0);

      return true;
    }
}

/****************************************************************************
 * Name: umm_givesemaphore
 *
 * Description:
 *   Release the MM mutex when it is not longer needed.
 *
 ****************************************************************************/

void umm_givesemaphore(FAR sem_t *sem)
{
  if (sem)
    {
      DEBUGVERIFY(_SEM_POST(sem));
    }
}

/****************************************************************************
 * Name: umm_addregion
 *
 * Description:
 *   This function adds a region of contiguous memory to the user heap. This
 *   function is exported from the user-space blob so that the kernel
 *   can initialize the user-mode allocator.
 *
 * Input Parameters:
 *   heapstart - Address of the beginning of the memory region
 *   heapsize  - The size (in bytes) of the memory region.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void umm_addregion(FAR void *heapstart, size_t heapsize)
{
  FAR struct mm_heap_s *heap = g_mmheap;
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

  DEBUGVERIFY(umm_takesemaphore(heap->mm_semaphore));

  minfo("Region %d: base=%p size=%zu\n", idx + 1, heapstart, heapsize);

  /* Save the start and end of the heap */

  heap->mm_heapstart[idx] = heapstart;
  heap->mm_heapsize[idx]  = heapsize;

#undef idx

#if CONFIG_MM_REGIONS > 1
  heap->mm_nregions++;
#endif

  umm_givesemaphore(heap->mm_semaphore);
}

/****************************************************************************
 * Name: umm_brkaddr
 *
 * Description:
 *   Return the break address of a region in the user heap.  Zero is returned
 *   if the memory region is not initialized.
 *
 ****************************************************************************/

FAR void *umm_brkaddr(int region)
{
  FAR struct mm_heap_s *heap = g_mmheap;

#if CONFIG_MM_REGIONS > 1
  DEBUGASSERT(region >= 0 && region < heap->mm_nregions);
#else
  DEBUGASSERT(region == 0);
#endif

  return heap->mm_heapstart[region] + heap->mm_heapsize[region];
}

/****************************************************************************
 * Name: calloc
 *
 * Description:
 *   calloc() calculates the size of the allocation and calls dmalloc_malloc()
 *
 ****************************************************************************/

FAR void *calloc(size_t n, size_t elem_size)
{
  FAR const char *file;

  n *= elem_size;
  if (n < elem_size)
    {
      return NULL;
    }

  /* Free the dealy list first */

  umm_free_delaylist();

  /* Allocate from the dmalloc */

  GET_RET_ADDR(file);
  return dmalloc_malloc(file, DMALLOC_DEFAULT_LINE,
                        n, DMALLOC_FUNC_CALLOC,
                        0 /* no alignment */,
                        0 /* no xalloc messages */);
}

#ifdef CONFIG_DEBUG_MM
/****************************************************************************
 * Name: umm_checkcorruption
 *
 * Description:
 *   umm_checkcorruption is used to check whether memory heap is normal.
 *
 ****************************************************************************/

void umm_checkcorruption(void)
{
  dmalloc_verify(NULL);
}
#endif

/****************************************************************************
 * Name: umm_extend
 *
 * Description:
 *   Extend a region in the user heap by add a block of (virtually)
 *   contiguous memory to the end of the heap.
 *
 ****************************************************************************/

void umm_extend(FAR void *mem, size_t size, int region)
{
  FAR struct mm_heap_s *heap = g_mmheap;

  /* Make sure that we were passed valid parameters */

#if CONFIG_MM_REGIONS > 1
  DEBUGASSERT(region >= 0 && region < heap->mm_nregions);
#else
  DEBUGASSERT(region == 0);
#endif
  DEBUGASSERT(mem == heap->mm_heapstart[region] + heap->mm_heapsize[region]);

  /* Take the memory manager semaphore */

  DEBUGVERIFY(umm_takesemaphore(heap->mm_semaphore));

  /* Save the new size */

  heap->mm_heapsize[region] += size;

  umm_givesemaphore(heap->mm_semaphore);
}

/****************************************************************************
 * Name: free
 *
 * Description:
 *   Returns a chunk of user memory to the list of free nodes,  merging with
 *   adjacent free chunks if possible.
 *
 ****************************************************************************/

void free(FAR void *mem)
{
  FAR const char *file;
  int ret;

  GET_RET_ADDR(file);
  ret = dmalloc_free(file, DMALLOC_DEFAULT_LINE,
                     mem, DMALLOC_FUNC_FREE);
  if (ret == FREE_ERROR && dmalloc_errno == DMALLOC_ERROR_LOCK_FAIL)
    {
      umm_add_delaylist(mem);
    }
}

/****************************************************************************
 * Name: umm_heapmember
 *
 * Description:
 *   Check if an address lies in the user heap.
 *
 * Parameters:
 *   mem - The address to check
 *
 * Return Value:
 *   true if the address is a member of the user heap.  false if not
 *   not.  If the address is not a member of the user heap, then it
 *   must be a member of the user-space heap (unchecked)
 *
 ****************************************************************************/

bool umm_heapmember(FAR void *mem)
{
  FAR struct mm_heap_s *heap = g_mmheap;
#if CONFIG_MM_REGIONS > 1
  int i;

  /* A valid address from the heap for this region would have to lie
   * between the region's two guard nodes.
   */

  for (i = 0; i < heap->mm_nregions; i++)
    {
      if (mem >= heap->mm_heapstart[i] &&
          mem < heap->mm_heapstart[i] + heap->mm_heapsize[i])
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
      mem < heap->mm_heapstart[0] + heap->mm_heapsize[0])
    {
      return true;
    }

  /* Otherwise, the address does not lie in the heap */

  return false;

#endif
}

/****************************************************************************
 * Name: umm_initialize
 *
 * Description:
 *   Initialize the selected heap data structures, providing the initial
 *   heap region. This function will initialize the user heap.
 *
 *   CONFIG_BUILD_FLAT:
 *     There is only kernel mode "blob" containing both kernel and
 *     application code.  There is only one heap that is used by both the
 *     kernel and application logic.
 *
 *     In this configuration, this function is called early in nx_start()
 *     to initialize the common heap.
 *
 *   CONFIG_BUILD_PROTECTED
 *     In this configuration, there are two "blobs", one containing
 *     protected kernel logic and one containing unprotected application
 *     logic.  Depending upon the setting of CONFIG_MM_KERNEL_HEAP there
 *     may be only a single shared heap, much as with CONFIG_BUILD_FLAT.
 *     Or there may be separate protected/kernel and unprotected/user
 *     heaps.
 *
 *     In either case, this function is still called early in nx_start()
 *     to initialize the user heap.
 *
 *   CONFIG_BUILD_KERNEL
 *     In this configuration there are multiple user heaps, one for each
 *     user process.  Furthermore, each heap is initially empty; memory
 *     is added to each heap dynamically via sbrk().  The heap data
 *     structure was set to zero when the address environment was created.
 *     Otherwise, the heap is uninitialized.
 *
 *     This function is not called at all.  Rather, this function is called
 *     when each user process is created before the first allocation is made.
 *
 * Input Parameters:
 *   heapstart - Address of the beginning of the (initial) memory region
 *   heapsize  - The size (in bytes) if the (initial) memory region.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void umm_initialize(FAR void *heapstart, size_t heapsize)
{
  FAR struct mm_heap_s *heap;

  minfo("Heap: start=%p size=%zu\n", heapstart, heapsize);

  /* Reserve a block space for mm_heap_s context */

  DEBUGASSERT(heapsize > sizeof(struct mm_heap_s));
  heap = (FAR struct mm_heap_s *)heapstart;
  memset(heap, 0, sizeof(struct mm_heap_s));
  heapstart += sizeof(struct mm_heap_s);
  heapsize -= sizeof(struct mm_heap_s);
  g_mmheap = heap;

  /* Add the initial region of memory to the heap */

  umm_addregion(heapstart, heapsize);

  /* Initialize the default options */

  dmalloc_debug_setup(CONFIG_MM_DMALLOC_OPTIONS);

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  heap->mm_procfs.name = "Umem";
  heap->mm_procfs.mallinfo = umm_mallinfo;
  heap->mm_procfs.user_data = heap;
  procfs_register_meminfo(&heap->mm_procfs);
#endif
#endif
}

/****************************************************************************
 * Name: mallinfo
 *
 * Description:
 *   mallinfo returns a copy of updated current heap information for the
 *   user heap.
 *
 ****************************************************************************/

struct mallinfo mallinfo(void)
{
  unsigned long total_space;
  unsigned long user_space;
  unsigned long current_allocated;
  unsigned long current_pnt;
  struct mallinfo info;

  dmalloc_get_stats(NULL, NULL, &total_space,
                    &user_space, &current_allocated, &current_pnt,
                    NULL, NULL, NULL);

  info.arena    = total_space;
  info.ordblks  = 0;
  info.aordblks = current_pnt;
  info.mxordblk = 0;
  info.uordblks = current_allocated;
  info.fordblks = user_space - current_allocated;

  return info;
}

/****************************************************************************
 * Name: malloc_size
 ****************************************************************************/

size_t malloc_size(FAR void *mem)
{
  DMALLOC_SIZE total_size;

  return dmalloc_examine(mem, NULL, &total_size, NULL,
                         NULL, NULL, NULL, NULL, NULL,
                         NULL) == DMALLOC_NOERROR ? total_size : 0;
}

/****************************************************************************
 * Name: malloc
 *
 * Description:
 *   Allocate memory from the user heap.
 *
 * Input Parameters:
 *   size - Size (in bytes) of the memory region to be allocated.
 *
 * Returned Value:
 *   The address of the allocated memory (NULL on failure to allocate)
 *
 ****************************************************************************/

FAR void *malloc(size_t size)
{
  FAR const char *file;

  /* Free the dealy list first */

  umm_free_delaylist();

  /* Allocate from the dmalloc */

  GET_RET_ADDR(file);
  return dmalloc_malloc(file, DMALLOC_DEFAULT_LINE,
                        size, DMALLOC_FUNC_MALLOC,
                        0 /* no alignment */,
                        0 /* no xalloc messages */);
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

FAR void *memalign(size_t alignment, size_t size)
{
  FAR const char *file;

  /* Free the dealy list first */

  umm_free_delaylist();

  /* Allocate from the dmalloc */

  GET_RET_ADDR(file);
  return dmalloc_malloc(file, DMALLOC_DEFAULT_LINE,
                        size, DMALLOC_FUNC_MEMALIGN,
                        alignment,
                        0 /* no xalloc messages */);
}

/****************************************************************************
 * Name: realloc
 *
 * Description:
 *   Re-allocate memory in the user heap.
 *
 * Input Parameters:
 *   oldmem  - The old memory allocated
 *   newsize - Size (in bytes) of the new memory region to be re-allocated.
 *
 * Returned Value:
 *   The address of the re-allocated memory (NULL on failure to re-allocate)
 *
 ****************************************************************************/

FAR void *realloc(FAR void *oldmem, size_t newsize)
{
  FAR const char *file;

  /* Free the dealy list first */

  umm_free_delaylist();

  /* Allocate from the dmalloc */

  GET_RET_ADDR(file);
  return dmalloc_realloc(file, DMALLOC_DEFAULT_LINE,
                         oldmem, newsize,
                         DMALLOC_FUNC_REALLOC,
                         0 /* no xalloc messages */);
}

/****************************************************************************
 * Name: sbrk
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
 *    incr - Specifies the number of bytes to add or to remove from the
 *      space allocated for the process.
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

FAR void *sbrk(intptr_t incr)
{
  int errcode;

  DEBUGASSERT(incr >= 0);
  if (incr < 0)
    {
      errcode = ENOSYS;
      goto errout;
    }

#ifdef CONFIG_BUILD_KERNEL
  uintptr_t brkaddr;
  uintptr_t allocbase;
  unsigned int pgincr;
  size_t bytesize;

  /* Initialize the user heap if not yet */

  umm_try_initialize();

  /* Get the current break address (NOTE: assumes region 0). */

  brkaddr = (uintptr_t)umm_brkaddr(0);
  if (incr > 0)
    {
      /* Convert the increment to multiples of the page size */

      pgincr = MM_NPAGES(incr);

      /* Allocate the requested number of pages and map them to the
       * break address.
       */

      allocbase = pgalloc(brkaddr, pgincr);
      if (allocbase == 0)
        {
          errcode = EAGAIN;
          goto errout;
        }

      /* Extend the heap (region 0) */

      bytesize = pgincr << MM_PGSHIFT;
      umm_extend((FAR void *)allocbase, bytesize, 0);
    }

  return (FAR void *)brkaddr;
#else
  FAR struct mm_heap_s *heap = g_mmheap;
  FAR void *brkaddr = NULL;

# if CONFIG_MM_REGIONS > 1
  for (; heap->mm_region < heap->mm_nregions; heap->mm_region++)
    {
      if (heap->mm_heapsize[heap->mm_region] - heap->mm_heapnext >= incr)
        {
          brkaddr = heap->mm_heapstart[heap->mm_region];
          break;
        }

      heap->mm_heapnext = 0;
    }
# else
  if (heap->mm_heapsize[0] - heap->mm_heapnext >= incr)
    {
      brkaddr = heap->mm_heapstart[0];
    }
#endif

  if (brkaddr == NULL)
    {
      errcode = EAGAIN;
      goto errout;
    }

  brkaddr += heap->mm_heapnext;
  heap->mm_heapnext += incr;
  return brkaddr;
#endif

errout:
  set_errno(errcode);
  return (FAR void *)-1;
}

/****************************************************************************
 * Name: zalloc
 *
 * Description:
 *   Allocate and zero memory from the user heap.
 *
 * Input Parameters:
 *   size - Size (in bytes) of the memory region to be allocated.
 *
 * Returned Value:
 *   The address of the allocated memory (NULL on failure to allocate)
 *
 ****************************************************************************/

FAR void *zalloc(size_t size)
{
  FAR const char *file;

  /* Free the dealy list first */

  umm_free_delaylist();

  /* Allocate from the dmalloc */

  GET_RET_ADDR(file);
  return dmalloc_malloc(file, DMALLOC_DEFAULT_LINE,
                        size, DMALLOC_FUNC_CALLOC,
                        0 /* no alignment */,
                        0 /* no xalloc messages */);
}

/****************************************************************************
 * Name: umm_fwrite/umm_write
 *
 * Description:
 *   Redirect dmalloc output to syslog
 ****************************************************************************/

ssize_t umm_fwrite(FAR const void *ptr, size_t size, size_t items,
                   FAR FILE *stream)
{
  if (stream == stdout)
    {
      syslog(LOG_NOTICE, "%.*s", size * items, (FAR const char *)ptr);
      return size * items;
    }
  else if (stream == stderr)
    {
      syslog(LOG_ERR, "%.*s", size * items, (FAR const char *)ptr);
      return size * items;
    }
  else
    {
      return fwrite(ptr, size, items, stream);
    }
}

ssize_t umm_write(int fd, FAR const void *buf, size_t nbytes)
{
  if (fd == STDOUT_FILENO)
    {
      syslog(LOG_NOTICE, "%.*s", nbytes, (FAR const char *)buf);
      return nbytes;
    }
  else if (fd == STDERR_FILENO)
    {
      syslog(LOG_ERR, "%.*s", nbytes, (FAR const char *)buf);
      return nbytes;
    }
  else
    {
      return write(fd, buf, nbytes);
    }
}
