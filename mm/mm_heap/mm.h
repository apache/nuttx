/****************************************************************************
 * mm/mm_heap/mm.h
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

#include <assert.h>
#include <execinfo.h>
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
         tmp->pid = gettid(); \
       } \
     while (0)
#elif CONFIG_MM_BACKTRACE > 0
#  define MM_ADD_BACKTRACE(heap, ptr) \
     do \
       { \
         FAR struct mm_allocnode_s *tmp = (FAR struct mm_allocnode_s *)(ptr); \
         kasan_unpoison(tmp, SIZEOF_MM_ALLOCNODE); \
         FAR struct tcb_s *tcb; \
         tmp->pid = gettid(); \
         tcb = nxsched_get_tcb(tmp->pid); \
         if ((heap)->mm_procfs.backtrace || (tcb && tcb->flags & TCB_FLAG_HEAP_DUMP)) \
           { \
             memset(tmp->backtrace, 0, sizeof(tmp->backtrace)); \
             backtrace(tmp->backtrace, CONFIG_MM_BACKTRACE); \
           } \
         else \
           { \
             tmp->backtrace[0] = 0; \
           } \
         kasan_poison(tmp, SIZEOF_MM_ALLOCNODE); \
       } \
     while (0)
#else
#  define MM_ADD_BACKTRACE(heap, ptr)
#endif

/* All other definitions derive from these two */

#define MM_MIN_CHUNK     (1 << MM_MIN_SHIFT)
#define MM_MAX_CHUNK     (1 << MM_MAX_SHIFT)
#define MM_NNODES        (MM_MAX_SHIFT - MM_MIN_SHIFT + 1)

#define MM_GRAN_MASK     (MM_MIN_CHUNK - 1)
#define MM_ALIGN_UP(a)   (((a) + MM_GRAN_MASK) & ~MM_GRAN_MASK)
#define MM_ALIGN_DOWN(a) ((a) & ~MM_GRAN_MASK)

/* An allocated chunk is distinguished from a free chunk by bit 0
 * of the 'preceding' chunk size.  If set, then this is an allocated chunk.
 */

#define MM_ALLOC_BIT     0x1
#define MM_MASK_BIT      MM_ALLOC_BIT
#ifdef CONFIG_MM_SMALL
# define MMSIZE_MAX      UINT16_MAX
#else
# define MMSIZE_MAX      UINT32_MAX
#endif

/* What is the size of the allocnode? */

#define SIZEOF_MM_ALLOCNODE sizeof(struct mm_allocnode_s)

/* What is the size of the freenode? */

#define SIZEOF_MM_FREENODE sizeof(struct mm_freenode_s)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Determines the size of the chunk size/offset type */

#ifdef CONFIG_MM_SMALL
typedef uint16_t mmsize_t;
#else
typedef uint32_t mmsize_t;
#endif

/* This describes an allocated chunk.  An allocated chunk is
 * distinguished from a free chunk by bit 15/31 of the 'preceding' chunk
 * size.  If set, then this is an allocated chunk.
 */

struct mm_allocnode_s
{
#if CONFIG_MM_BACKTRACE >= 0
  pid_t pid;                                /* The pid for caller */
#  if CONFIG_MM_BACKTRACE > 0
  FAR void *backtrace[CONFIG_MM_BACKTRACE]; /* The backtrace buffer for caller */
#  endif
#endif
  mmsize_t size;                            /* Size of this chunk */
  mmsize_t preceding;                       /* Size of the preceding chunk */
};

/* This describes a free chunk */

struct mm_freenode_s
{
#if CONFIG_MM_BACKTRACE >= 0
  pid_t pid;                                /* The pid for caller */
#  if CONFIG_MM_BACKTRACE > 0
  FAR void *backtrace[CONFIG_MM_BACKTRACE]; /* The backtrace buffer for caller */
#  endif
#endif
  mmsize_t size;                            /* Size of this chunk */
  mmsize_t preceding;                       /* Size of the preceding chunk */
  FAR struct mm_freenode_s *flink;          /* Supports a doubly linked list */
  FAR struct mm_freenode_s *blink;
};

static_assert(SIZEOF_MM_ALLOCNODE <= MM_MIN_CHUNK,
              "Error size for struct mm_allocnode_s\n");

static_assert(SIZEOF_MM_FREENODE <= MM_MIN_CHUNK,
              "Error size for struct mm_freenode_s\n");

struct mm_delaynode_s
{
  FAR struct mm_delaynode_s *flink;
};

/* This describes one heap (possibly with multiple regions) */

struct mm_heap_s
{
  /* Mutually exclusive access to this data set is enforced with
   * the following un-named mutex.
   */

  mutex_t mm_lock;

  /* This is the size of the heap provided to mm */

  size_t mm_heapsize;

  /* This is the first and last nodes of the heap */

  FAR struct mm_allocnode_s *mm_heapstart[CONFIG_MM_REGIONS];
  FAR struct mm_allocnode_s *mm_heapend[CONFIG_MM_REGIONS];

#if CONFIG_MM_REGIONS > 1
  int mm_nregions;
#endif

  /* All free nodes are maintained in a doubly linked list.  This
   * array provides some hooks into the list at various points to
   * speed searches for free nodes.
   */

  struct mm_freenode_s mm_nodelist[MM_NNODES];

  /* Free delay list, for some situations where we can't do free
   * immdiately.
   */

  FAR struct mm_delaynode_s *mm_delaylist[CONFIG_SMP_NCPUS];

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

/* Functions contained in mm_shrinkchunk.c **********************************/

void mm_shrinkchunk(FAR struct mm_heap_s *heap,
                    FAR struct mm_allocnode_s *node, size_t size);

/* Functions contained in mm_addfreechunk.c *********************************/

void mm_addfreechunk(FAR struct mm_heap_s *heap,
                     FAR struct mm_freenode_s *node);

/* Functions contained in mm_size2ndx.c *************************************/

int mm_size2ndx(size_t size);

/* Functions contained in mm_foreach.c **************************************/

void mm_foreach(FAR struct mm_heap_s *heap, mm_node_handler_t handler,
                FAR void *arg);

#endif /* __MM_MM_HEAP_MM_H */
