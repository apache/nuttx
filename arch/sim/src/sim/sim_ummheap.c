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
#include <execinfo.h>
#include <string.h>
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/atomic.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/list.h>
#include <nuttx/mm/mm.h>
#include <nuttx/nuttx.h>
#include <nuttx/sched_note.h>
#include <nuttx/spinlock.h>

#include "sim_internal.h"

#ifdef CONFIG_MM_UMM_CUSTOMIZE_MANAGER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_MM_BACKTRACE == 0
#  define MM_ADD_BACKTRACE(heap, ptr) \
     do { \
         FAR struct mm_allocnode_s *tmp = (FAR struct mm_allocnode_s *)(ptr); \
         tmp->pid = _SCHED_GETTID(); \
         MM_INCSEQNO(tmp); \
     } while (0)

#elif CONFIG_MM_BACKTRACE > 0
#  define MM_ADD_BACKTRACE(heap, ptr) \
     do { \
         FAR struct mm_allocnode_s *tmp = (FAR struct mm_allocnode_s *)(ptr); \
         FAR struct tcb_s *tcb; \
         pid_t tid = _SCHED_GETTID(); \
         tmp->pid = tid; \
         tcb = nxsched_get_tcb(tid); \
         if ((heap)->procfs.backtrace || (tcb && tcb->flags & TCB_FLAG_HEAP_DUMP)) { \
             int n = sched_backtrace(tid, tmp->backtrace, CONFIG_MM_BACKTRACE, \
                                     CONFIG_MM_BACKTRACE_SKIP); \
             if (n < CONFIG_MM_BACKTRACE) { \
                 tmp->backtrace[n] = NULL; \
             } \
         } else { \
             tmp->backtrace[0] = NULL; \
         } \
         if (tcb) nxsched_put_tcb(tcb); \
         MM_INCSEQNO(tmp); \
     } while (0)

#else
#  define MM_ADD_BACKTRACE(heap, ptr)
#endif

#define MM_ALLOCNODE_SIZE    sizeof(struct mm_allocnode_s)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This describes an allocated chunk */

struct mm_allocnode_s
{
  struct list_node node;            /* Supports a doubly linked list */
  size_t size;                      /* Size of this chunk */
  size_t padding;                   /* Padding size for alignment */
  void *allocmem;                   /* Start address of this chunk */
#ifdef CONFIG_MM_RECORD_PID
  pid_t pid;                        /* The pid for caller */
#endif
#ifdef CONFIG_MM_RECORD_SEQNO
  unsigned long seqno;              /* The sequence of memory malloc */
#endif
#ifdef CONFIG_MM_RECORD_STACK
  void *stack;                      /* The backtrace buffer for caller */
#endif
};

struct mm_delaynode_s
{
  struct mm_delaynode_s *flink;
};

struct mm_heap_s
{
  spinlock_t lock;

  struct mm_delaynode_s *delaylist[CONFIG_SMP_NCPUS];
  struct list_node alloclist;

#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  size_t delaycount[CONFIG_SMP_NCPUS];
#endif

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
  struct procfs_meminfo_entry_s procfs;
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

#ifdef CONFIG_MM_RECORD_SEQNO
unsigned long g_mm_seqno;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void update_stats(struct mm_heap_s *heap, void *mem, size_t size,
                         bool alloc)
{
  struct mm_allocnode_s *node;
  irqstate_t flags;
  size_t uordblks;
  size_t usmblks;

  node = (struct mm_allocnode_s *)((uintptr_t)mem - MM_ALLOCNODE_SIZE);
  flags = spin_lock_irqsave(&heap->lock);

  if (alloc)
    {
      list_add_tail(&heap->alloclist, &node->node);
      spin_unlock_irqrestore(&heap->lock, flags);

      atomic_fetch_add(&heap->aordblks, 1);
      atomic_fetch_add(&heap->uordblks, size);
      usmblks = atomic_read(&heap->usmblks);
      do
        {
          uordblks = atomic_read(&heap->uordblks);
          if (uordblks <= usmblks) break;
        }
      while (!atomic_try_cmpxchg(&heap->usmblks, &usmblks, uordblks));
    }
  else
    {
      list_delete(&node->node);
      spin_unlock_irqrestore(&heap->lock, flags);

      atomic_fetch_sub(&heap->aordblks, 1);
      atomic_fetch_sub(&heap->uordblks, size);
    }
}

static void *init_allocnode(struct mm_heap_s *heap, void *alloc_mem,
                            size_t aligned_size, size_t padding_size)
{
  uintptr_t node_addr;
  struct mm_allocnode_s *node;
  void *mem;

  node_addr = (uintptr_t)alloc_mem + padding_size;
  node = (struct mm_allocnode_s *)node_addr;
  mem = (void *)(node_addr + MM_ALLOCNODE_SIZE);

  node->size = aligned_size;
  node->padding = padding_size;
  node->allocmem = alloc_mem;

  MM_ADD_BACKTRACE(heap, node);
  update_stats(heap, mem, aligned_size, true);

  return mem;
}

static void add_delaylist(struct mm_heap_s *heap, void *mem)
{
  struct mm_delaynode_s *tmp = mem;
  irqstate_t flags;

  /* Delay the deallocation until a more appropriate time. */

  flags = spin_lock_irqsave(&heap->lock);

  tmp->flink = heap->delaylist[this_cpu()];
  heap->delaylist[this_cpu()] = tmp;

#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  heap->delaycount[this_cpu()]++;
#endif

  spin_unlock_irqrestore(&heap->lock, flags);
}

static bool free_delaylist(struct mm_heap_s *heap, bool force)
{
  bool ret = false;
  struct mm_delaynode_s *tmp;
  irqstate_t flags;

  /* Move the delay list to local */

  flags = spin_lock_irqsave(&heap->lock);

  tmp = heap->delaylist[this_cpu()];

#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  if (tmp == NULL ||
      (!force &&
        heap->delaycount[this_cpu()] < CONFIG_MM_FREE_DELAYCOUNT_MAX))
    {
      spin_unlock_irqrestore(&heap->lock, flags);
      return false;
    }

  heap->delaycount[this_cpu()] = 0;
#endif
  heap->delaylist[this_cpu()] = NULL;

  spin_unlock_irqrestore(&heap->lock, flags);

  /* Test if the delayed is empty */

  ret = !!tmp;

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
  struct mm_allocnode_s *node;

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
      node = (struct mm_allocnode_s *)((uintptr_t)mem - MM_ALLOCNODE_SIZE);
      update_stats(heap, mem, node->size, false);
      sched_note_heap(NOTE_HEAP_FREE, heap, mem, node->size, 0);
      host_free(node->allocmem);
    }
}

static void *reallocate(void *oldmem, size_t alignment, size_t size)
{
  size_t aligned_size;
  size_t padding_size;
  void *new_alloc_addr;
  struct mm_allocnode_s *old_node;
  size_t old_size;
  void *old_alloc_addr;
  void *mem = oldmem;

  free_delaylist(g_mmheap, false);

  /* NEW ALLOCATION (MALLOC/MEMALIGN) */

  if (oldmem == NULL)
    {
      size = size ? size : 1;
      if (alignment > MM_ALIGN)  /* CUSTOM ALIGNMENT HANDLING (MEMALIGN) */
        {
          if ((alignment & (alignment - 1)) != 0 || alignment > SIZE_MAX / 2)
            {
              return NULL;
            }

          padding_size = (MM_ALLOCNODE_SIZE < alignment)
                       ? (alignment - MM_ALLOCNODE_SIZE)
                       : (-MM_ALLOCNODE_SIZE) & (alignment - 1);

          if (padding_size > SIZE_MAX - MM_ALLOCNODE_SIZE - size)
            {
              return NULL;
            }

          aligned_size = padding_size + MM_ALLOCNODE_SIZE + size;
          new_alloc_addr = host_memalign(alignment, aligned_size);
        }
      else  /* DEFAULT ALIGNMENT HANDLING (MALLOC) */
        {
          padding_size = ALIGN_UP_MASK(MM_ALLOCNODE_SIZE, MM_ALIGN - 1)
                         - MM_ALLOCNODE_SIZE;
          aligned_size = padding_size + MM_ALLOCNODE_SIZE + size;
          new_alloc_addr = host_realloc(NULL, aligned_size);
        }

      if (new_alloc_addr == NULL)
        {
          return NULL;
        }

      mem = init_allocnode(g_mmheap, new_alloc_addr, aligned_size,
                           padding_size);
      sched_note_heap(NOTE_HEAP_ALLOC, g_mmheap, mem, aligned_size, 0);
      return mem;
    }

  /* DEALLOCATION (FREE) */

  if (size == 0)
    {
      free(oldmem);
      return NULL;
    }

  /* REALLOCATION (REALLOC) */

  old_node = (struct mm_allocnode_s *)
             ((uintptr_t)oldmem - MM_ALLOCNODE_SIZE);
  old_alloc_addr = old_node->allocmem;
  old_size = old_node->size;

  update_stats(g_mmheap, oldmem, old_size, false);

  padding_size = ALIGN_UP_MASK(MM_ALLOCNODE_SIZE, MM_ALIGN - 1)
                 - MM_ALLOCNODE_SIZE;
  aligned_size = padding_size + MM_ALLOCNODE_SIZE + size;

  new_alloc_addr = host_realloc(old_alloc_addr, aligned_size);
  if (new_alloc_addr == NULL)
    {
      update_stats(g_mmheap, oldmem, old_size, true);
      return NULL;
    }

  if (new_alloc_addr == old_alloc_addr)
    {
      old_node->size = aligned_size;
      old_node->padding = padding_size;
      MM_ADD_BACKTRACE(g_mmheap, old_node);
      update_stats(g_mmheap, oldmem, aligned_size, true);
      sched_note_heap(NOTE_HEAP_ALLOC, g_mmheap, mem, aligned_size, 0);
      return oldmem;
    }

  sched_note_heap(NOTE_HEAP_FREE, g_mmheap, oldmem, old_size, 0);
  mem = init_allocnode(g_mmheap, new_alloc_addr, aligned_size, padding_size);
  sched_note_heap(NOTE_HEAP_ALLOC, g_mmheap, mem, aligned_size, 0);
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
  return reallocate(NULL, 1, size);
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
  return reallocate(oldmem, 1, size);
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
  return reallocate(NULL, alignment, size);
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
 * Name: mallinfo_callback
 *
 * Description:
 *   mallinfo callback function of the procfs filesystem.
 *
 ****************************************************************************/

struct mallinfo mallinfo_callback(struct mm_heap_s *heap)
{
  struct mallinfo info;
  memset(&info, 0, sizeof(struct mallinfo));

  info.aordblks = atomic_read(&heap->aordblks);
  info.ordblks  = 1;
  info.uordblks = atomic_read(&heap->uordblks);
  info.usmblks  = atomic_read(&heap->usmblks);
  info.arena    = SIM_HEAP_SIZE;
  info.fordblks = SIM_HEAP_SIZE - info.uordblks;
  info.mxordblk = info.fordblks;

  return info;
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
  return mallinfo_callback(g_mmheap);
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
 * Name: memdump_callback
 *
 * Description:
 *   memdump callback function of the procfs filesystem.
 *
 ****************************************************************************/

void memdump_callback(struct mm_heap_s *heap,
                      const struct mm_memdump_s *dump)
{
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
  memdump_callback(g_mmheap, dump);
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
  struct mm_allocnode_s *node;

  if (mem == NULL)
    {
      return 0;
    }

  node = (struct mm_allocnode_s *)((uintptr_t)mem - MM_ALLOCNODE_SIZE);
  return node->size - MM_ALLOCNODE_SIZE - node->padding;
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

  list_initialize(&g_mmheap->alloclist);
  spin_lock_init(&g_mmheap->lock);

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
  g_mmheap->procfs.name = "Umem";
  g_mmheap->procfs.heap = g_mmheap;
  g_mmheap->procfs.mallinfo = mallinfo_callback;
  g_mmheap->procfs.memdump = memdump_callback;
  procfs_register_meminfo(&g_mmheap->procfs);
#endif
}

#else /* CONFIG_MM_UMM_CUSTOMIZE_MANAGER */

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  *heap_start = host_allocheap(SIM_HEAP_SIZE, false);
  *heap_size  = SIM_HEAP_SIZE;
}

#endif /* CONFIG_MM_UMM_CUSTOMIZE_MANAGER */
