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

#include <nuttx/fs/procfs.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>

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

#if defined(CONFIG_MM_SMALL) && UINTPTR_MAX <= UINT32_MAX
/* Two byte offsets; Pointers may be 2 or 4 bytes;
 * sizeof(struct mm_freenode_s) is 8 or 12 bytes.
 * REVISIT: We could do better on machines with 16-bit addressing.
 */

#  define MM_MIN_SHIFT   ( 4)  /* 16 bytes */
#  define MM_MAX_SHIFT   (15)  /* 32 Kb */

#elif defined(CONFIG_HAVE_LONG_LONG)
/* Four byte offsets; Pointers may be 4 or 8 bytes
 * sizeof(struct mm_freenode_s) is 16 or 24 bytes.
 */

#  if UINTPTR_MAX <= UINT32_MAX
#    define MM_MIN_SHIFT ( 4)  /* 16 bytes */
#  elif UINTPTR_MAX <= UINT64_MAX
#    define MM_MIN_SHIFT ( 5)  /* 32 bytes */
#  endif
#  define MM_MAX_SHIFT   (22)  /*  4 Mb */

#else
/* Four byte offsets; Pointers must be 4 bytes.
 * sizeof(struct mm_freenode_s) is 16 bytes.
 */

#  define MM_MIN_SHIFT   ( 4)  /* 16 bytes */
#  define MM_MAX_SHIFT   (22)  /*  4 Mb */
#endif

/* All other definitions derive from these two */

#define MM_MIN_CHUNK     (1 << MM_MIN_SHIFT)
#define MM_MAX_CHUNK     (1 << MM_MAX_SHIFT)
#define MM_NNODES        (MM_MAX_SHIFT - MM_MIN_SHIFT + 1)

#define MM_GRAN_MASK     (MM_MIN_CHUNK - 1)
#define MM_ALIGN_UP(a)   (((a) + MM_GRAN_MASK) & ~MM_GRAN_MASK)
#define MM_ALIGN_DOWN(a) ((a) & ~MM_GRAN_MASK)

/* An allocated chunk is distinguished from a free chunk by bit 31 (or 15)
 * of the 'preceding' chunk size.  If set, then this is an allocated chunk.
 */

#ifdef CONFIG_MM_SMALL
# define MM_ALLOC_BIT    0x8000
#else
# define MM_ALLOC_BIT    0x80000000
#endif
#define MM_IS_ALLOCATED(n) \
  ((int)((FAR struct mm_allocnode_s *)(n)->preceding) < 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Determines the size of the chunk size/offset type */

#ifdef CONFIG_MM_SMALL
typedef uint16_t mmsize_t;
#  define MMSIZE_MAX UINT16_MAX
#else
typedef uint32_t mmsize_t;
#  define MMSIZE_MAX UINT32_MAX
#endif

/* This describes an allocated chunk.  An allocated chunk is
 * distinguished from a free chunk by bit 15/31 of the 'preceding' chunk
 * size.  If set, then this is an allocated chunk.
 */

struct mm_allocnode_s
{
  mmsize_t size;           /* Size of this chunk */
  mmsize_t preceding;      /* Size of the preceding chunk */
};

/* What is the size of the allocnode? */

#ifdef CONFIG_MM_SMALL
# define SIZEOF_MM_ALLOCNODE   (4)
#else
# define SIZEOF_MM_ALLOCNODE   (8)
#endif

#define CHECK_ALLOCNODE_SIZE \
  DEBUGASSERT(sizeof(struct mm_allocnode_s) == SIZEOF_MM_ALLOCNODE)

/* This describes a free chunk */

struct mm_freenode_s
{
  mmsize_t size;                   /* Size of this chunk */
  mmsize_t preceding;              /* Size of the preceding chunk */
  FAR struct mm_freenode_s *flink; /* Supports a doubly linked list */
  FAR struct mm_freenode_s *blink;
};

struct mm_delaynode_s
{
  FAR struct mm_delaynode_s *flink;
};

/* What is the size of the freenode? */

#define MM_PTR_SIZE sizeof(FAR struct mm_freenode_s *)
#define SIZEOF_MM_FREENODE (SIZEOF_MM_ALLOCNODE + 2*MM_PTR_SIZE)

#define CHECK_FREENODE_SIZE \
  DEBUGASSERT(sizeof(struct mm_freenode_s) == SIZEOF_MM_FREENODE)

/* This describes one heap (possibly with multiple regions) */

struct mm_heap_s
{
  /* Mutually exclusive access to this data set is enforced with
   * the following un-named semaphore.
   */

  sem_t mm_semaphore;

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

#ifdef CONFIG_SMP
  FAR struct mm_delaynode_s *mm_delaylist[CONFIG_SMP_NCPUS];
#else
  FAR struct mm_delaynode_s *mm_delaylist[1];
#endif

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
  struct procfs_meminfo_entry_s mm_procfs;
#endif
};

/* Functions contained in mm_sem.c ******************************************/

void mm_seminitialize(FAR struct mm_heap_s *heap);
bool mm_takesemaphore(FAR struct mm_heap_s *heap);
void mm_givesemaphore(FAR struct mm_heap_s *heap);

/* Functions contained in mm_shrinkchunk.c **********************************/

void mm_shrinkchunk(FAR struct mm_heap_s *heap,
                    FAR struct mm_allocnode_s *node, size_t size);

/* Functions contained in mm_addfreechunk.c *********************************/

void mm_addfreechunk(FAR struct mm_heap_s *heap,
                     FAR struct mm_freenode_s *node);

/* Functions contained in mm_size2ndx.c.c ***********************************/

int mm_size2ndx(size_t size);

#endif /* __MM_MM_HEAP_MM_H */
