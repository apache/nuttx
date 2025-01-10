/****************************************************************************
 * mm/mm_heap/mm.h
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

#ifndef __MM_MM_HEAP_MM_H
#define __MM_MM_HEAP_MM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/mutex.h>
#include <nuttx/sched.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/lib/math32.h>
#include <nuttx/mm/mempool.h>
#include <nuttx/mm/mm.h>

#include <assert.h>
#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Chunk Header Definitions *************************************************/

/* These definitions define the characteristics of the allocator:
 *
 * MM_MIN_SHIFT is used to define MM_MIN_CHUNK.
 * MM_MIN_CHUNK - is the smallest physical chunk that can be allocated.
 *   It must be at least as large as sizeof(struct mm_freenode_s). Larger
 *   values may improve performance slightly, but will waste memory due to
 *   quantization losses.
 *
 * MM_MAX_SHIFT is used to define MM_MAX_CHUNK
 * MM_MAX_CHUNK is the largest, contiguous chunk of memory that can be
 *   allocated.  It can range from 16-bytes to 4Gb.  Larger values of
 *   MM_MAX_SHIFT can cause larger data structure sizes and, perhaps,
 *   minor performance losses.
 */

#define MM_MIN_SHIFT      LOG2_CEIL(sizeof(struct mm_freenode_s))
#if defined(CONFIG_MM_SMALL) && UINTPTR_MAX <= UINT32_MAX
#  define MM_MAX_SHIFT    (15)  /* 32 Kb */
#else
#  define MM_MAX_SHIFT    (22)  /*  4 Mb */
#endif

#if CONFIG_MM_BACKTRACE == 0
#  define MM_ADD_BACKTRACE(heap, ptr) \
     do \
       { \
         FAR struct mm_allocnode_s *tmp = (FAR struct mm_allocnode_s *)(ptr); \
         tmp->pid = _SCHED_GETTID(); \
         tmp->seqno = g_mm_seqno++; \
       } \
     while (0)
#elif CONFIG_MM_BACKTRACE > 0
#  define MM_ADD_BACKTRACE(heap, ptr) \
     do \
       { \
         FAR struct mm_allocnode_s *tmp = (FAR struct mm_allocnode_s *)(ptr); \
         FAR struct tcb_s *tcb; \
         tmp->pid = _SCHED_GETTID(); \
         tcb = nxsched_get_tcb(tmp->pid); \
         if ((heap)->mm_procfs.backtrace || (tcb && tcb->flags & TCB_FLAG_HEAP_DUMP)) \
           { \
             int n = sched_backtrace(tmp->pid, tmp->backtrace, CONFIG_MM_BACKTRACE, \
                                     CONFIG_MM_BACKTRACE_SKIP); \
             if (n < CONFIG_MM_BACKTRACE) \
               { \
                 tmp->backtrace[n] = NULL; \
               } \
           } \
         else \
           { \
             tmp->backtrace[0] = NULL; \
           } \
         tmp->seqno = g_mm_seqno++; \
       } \
     while (0)
#else
#  define MM_ADD_BACKTRACE(heap, ptr)
#endif

/* All other definitions derive from these two */

#define MM_MIN_CHUNK     (1 << MM_MIN_SHIFT)
#define MM_MAX_CHUNK     (1 << MM_MAX_SHIFT)
#define MM_NNODES        (MM_MAX_SHIFT - MM_MIN_SHIFT + 1)

#define MM_GRAN_MASK     (MM_ALIGN - 1)
#define MM_ALIGN_UP(a)   (((a) + MM_GRAN_MASK) & ~MM_GRAN_MASK)
#define MM_ALIGN_DOWN(a) ((a) & ~MM_GRAN_MASK)

/* Due to alignment, the lowest two bits of valid chunk size are always
 * zero, thus the two bits are reused to depict allocation status: bit
 * 0 depicts the allocation state of current chunk, and bit 1 depicts that
 * of the physically preceding chunk.
 */

#define MM_ALLOC_BIT     0x1
#define MM_PREVFREE_BIT  0x2
#define MM_MASK_BIT      (MM_ALLOC_BIT | MM_PREVFREE_BIT)
#ifdef CONFIG_MM_SMALL
#  define MMSIZE_MAX     UINT16_MAX
#else
#  define MMSIZE_MAX     UINT32_MAX
#endif

/* What is the size of the allocnode? */

#define MM_SIZEOF_ALLOCNODE sizeof(struct mm_allocnode_s)

/* What is the overhead of the allocnode
 * Remove the space of preceding field since it locates at the end of the
 * previous freenode
 */

#define MM_ALLOCNODE_OVERHEAD (MM_SIZEOF_ALLOCNODE - sizeof(mmsize_t))

/* Get the node size */

#define MM_SIZEOF_NODE(node) ((node)->size & (~MM_MASK_BIT))

/* Check if node/prenode is free */

#define MM_NODE_IS_ALLOC(node) (((node)->size & MM_ALLOC_BIT) != 0)
#define MM_NODE_IS_FREE(node) (((node)->size & MM_ALLOC_BIT) == 0)

#define MM_PREVNODE_IS_ALLOC(node) (((node)->size & MM_PREVFREE_BIT) == 0)
#define MM_PREVNODE_IS_FREE(node) (((node)->size & MM_PREVFREE_BIT) != 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Determines the size of the chunk size/offset type */

#ifdef CONFIG_MM_SMALL
typedef uint16_t mmsize_t;
#else
typedef size_t mmsize_t;
#endif

/* This describes an allocated chunk */

struct mm_allocnode_s
{
  mmsize_t preceding;                       /* Physical preceding chunk size */
  mmsize_t size;                            /* Size of this chunk */
#if CONFIG_MM_BACKTRACE >= 0
  pid_t pid;                                /* The pid for caller */
  unsigned long seqno;                      /* The sequence of memory malloc */
#  if CONFIG_MM_BACKTRACE > 0
  FAR void *backtrace[CONFIG_MM_BACKTRACE]; /* The backtrace buffer for caller */
#  endif
#endif
};

/* This describes a free chunk */

struct mm_freenode_s
{
  mmsize_t preceding;                       /* Physical preceding chunk size */
  mmsize_t size;                            /* Size of this chunk */
#if CONFIG_MM_BACKTRACE >= 0
  pid_t pid;                                /* The pid for caller */
  unsigned long seqno;                      /* The sequence of memory malloc */
#  if CONFIG_MM_BACKTRACE > 0
  FAR void *backtrace[CONFIG_MM_BACKTRACE]; /* The backtrace buffer for caller */
#  endif
#endif
  FAR struct mm_freenode_s *flink;          /* Supports a doubly linked list */
  FAR struct mm_freenode_s *blink;
};

static_assert(MM_SIZEOF_ALLOCNODE <= MM_MIN_CHUNK,
              "Error size for struct mm_allocnode_s\n");

static_assert(MM_ALIGN >= sizeof(uintptr_t) &&
              (MM_ALIGN & MM_GRAN_MASK) == 0,
              "Error memory alignment\n");

struct mm_delaynode_s
{
  FAR struct mm_delaynode_s *flink;
};

/* This describes one heap (possibly with multiple regions) */

struct mm_heap_s
{
  /* Mutex for controling access to this heap */

  mutex_t mm_lock;

  /* This is the size of the heap provided to mm */

  size_t mm_heapsize;

  /* This is the heap maximum used memory size */

  size_t mm_maxused;

  /* This is the current used size of the heap */

  size_t mm_curused;

  /* The first and last allocated nodes of each region */

  FAR struct mm_allocnode_s *mm_heapstart[CONFIG_MM_REGIONS];
  FAR struct mm_allocnode_s *mm_heapend[CONFIG_MM_REGIONS];

#if CONFIG_MM_REGIONS > 1
  int mm_nregions;
#endif

  /* All free nodes are maintained in a doubly linked list.  This
   * array provides some hooks into the list at various points to
   * speed up searching of free nodes.
   */

  struct mm_freenode_s mm_nodelist[MM_NNODES];

  /* Free delay list, as sometimes we can't do free immdiately. */

  FAR struct mm_delaynode_s *mm_delaylist[CONFIG_SMP_NCPUS];

#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  size_t mm_delaycount[CONFIG_SMP_NCPUS];
#endif

  /* The is a multiple mempool of the heap */

#ifdef CONFIG_MM_HEAP_MEMPOOL
  size_t                         mm_threshold;
  FAR struct mempool_multiple_s *mm_mpool;
#endif

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
  struct procfs_meminfo_entry_s mm_procfs;
#endif
};

/* This describes the callback for mm_foreach */

typedef CODE void (*mm_node_handler_t)(FAR struct mm_allocnode_s *node,
                                       FAR void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Functions contained in mm_lock.c *****************************************/

int mm_lock(FAR struct mm_heap_s *heap);
void mm_unlock(FAR struct mm_heap_s *heap);
irqstate_t mm_lock_irq(FAR struct mm_heap_s *heap);
void mm_unlock_irq(FAR struct mm_heap_s *heap, irqstate_t state);

/* Functions contained in mm_shrinkchunk.c **********************************/

void mm_shrinkchunk(FAR struct mm_heap_s *heap,
                    FAR struct mm_allocnode_s *node, size_t size);

/* Functions contained in mm_foreach.c **************************************/

void mm_foreach(FAR struct mm_heap_s *heap, mm_node_handler_t handler,
                FAR void *arg);

/* Functions contained in mm_free.c *****************************************/

void mm_delayfree(FAR struct mm_heap_s *heap, FAR void *mem, bool delay);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline_function int mm_size2ndx(size_t size)
{
  DEBUGASSERT(size >= MM_MIN_CHUNK);
  if (size >= MM_MAX_CHUNK)
    {
      return MM_NNODES - 1;
    }

  size >>= MM_MIN_SHIFT;
  return flsl(size) - 1;
}

static inline_function void mm_addfreechunk(FAR struct mm_heap_s *heap,
                                            FAR struct mm_freenode_s *node)
{
  FAR struct mm_freenode_s *next;
  FAR struct mm_freenode_s *prev;
  size_t nodesize = MM_SIZEOF_NODE(node);
  int ndx;

  DEBUGASSERT(nodesize >= MM_MIN_CHUNK);
  DEBUGASSERT(MM_NODE_IS_FREE(node));

  /* Convert the size to a nodelist index */

  ndx = mm_size2ndx(nodesize);

  /* Now put the new node into the next */

  for (prev = &heap->mm_nodelist[ndx],
       next = heap->mm_nodelist[ndx].flink;
       next && next->size && MM_SIZEOF_NODE(next) < nodesize;
       prev = next, next = next->flink);

  /* Does it go in mid next or at the end? */

  prev->flink = node;
  node->blink = prev;
  node->flink = next;

  if (next)
    {
      /* The new node goes between prev and next */

      next->blink = node;
    }
}

#endif /* __MM_MM_HEAP_MM_H */
