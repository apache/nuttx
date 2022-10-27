/****************************************************************************
 * include/nuttx/mm/mempool.h
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

#ifndef __INCLUDE_NUTTX_MM_MEMPOOL_H
#define __INCLUDE_NUTTX_MM_MEMPOOL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

#include <nuttx/list.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/spinlock.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mempool_s;
typedef CODE void *(*mempool_alloc_t)(FAR struct mempool_s *pool,
                                      size_t size);
typedef CODE void (*mempool_free_t)(FAR struct mempool_s *pool,
                                    FAR void *addr);

#ifndef CONFIG_FS_PROCFS_EXCLUDE_MEMPOOL
struct mempool_procfs_entry_s
{
  FAR const char *name;
  FAR struct mempool_procfs_entry_s *next;
};
#endif

/* This structure describes memory buffer pool */

struct mempool_s
{
  size_t     blocksize;     /* The size for every block in mempool */
  size_t     initialsize;   /* The initialize size in normal mempool */
  size_t     interruptsize; /* The initialize size in interrupt mempool */
  size_t     expandsize;    /* The size of expand block every time for mempool */
  bool       wait;          /* The flag of need to wait when mempool is empty */
  mempool_alloc_t alloc;    /* The alloc function for mempool */
  mempool_free_t  free;     /* The free function for mempool */

  /* Private data for memory pool */

  struct list_node list;    /* The free block list in normal mempool */
  struct list_node ilist;   /* The free block list in interrupt mempool */
  struct list_node elist;   /* The expand block list for normal mempool */
  size_t           nused;   /* The number of used block in mempool */
  spinlock_t       lock;    /* The protect lock to mempool */
  sem_t            waitsem; /* The semaphore of waiter get free block */
#ifndef CONFIG_FS_PROCFS_EXCLUDE_MEMPOOL
  struct mempool_procfs_entry_s procfs; /* The entry of procfs */
#endif
};

struct mempool_multiple_s
{
  FAR struct mempool_s *pools;  /* The memory pool array */
  size_t                npools; /* The number of memory pool array elements */

  /* Private data for multiple memory pool */

  /* This delta describes the relationship between the block size of each
   * mempool in multiple mempool by user initialized. It is automatically
   * detected by the mempool_multiple_init function. If the delta is not
   * equal to 0, the block size of the pool in the multiple mempool is an
   * arithmetic progressions, otherwise it is an increasing progressions.
   */

  size_t                delta;
};

struct mempoolinfo_s
{
  unsigned long arena;    /* This is the total size of mempool */
  unsigned long ordblks;  /* This is the number of free blocks for normal mempool */
  unsigned long iordblks; /* This is the number of free blocks for interrupt mempool */
  unsigned long aordblks; /* This is the number of used blocks */
  unsigned long sizeblks; /* This is the size of a mempool blocks */
  unsigned long nwaiter;  /* This is the number of waiter for mempool */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mempool_init
 *
 * Description:
 *   Initialize a memory pool.
 *   The user needs to specify the initialization information of mempool
 *   including blocksize, initialsize, expandsize, interruptsize.
 *
 * Input Parameters:
 *   pool - Address of the memory pool to be used.
 *   name - The name of memory pool.
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int mempool_init(FAR struct mempool_s *pool, FAR const char *name);

/****************************************************************************
 * Name: mempool_alloc
 *
 * Description:
 *   Allocate an block from a specific memory pool.
 *
 *   If there isn't enough memory blocks, This function will expand memory
 *   pool if expandsize isn't zero.
 *
 * Input Parameters:
 *   pool - Address of the memory pool to be used.
 *
 * Returned Value:
 *   The pointer to the allocated block on success; NULL on any failure.
 *
 ****************************************************************************/

FAR void *mempool_alloc(FAR struct mempool_s *pool);

/****************************************************************************
 * Name: mempool_free
 *
 * Description:
 *   Release an memory block to the pool.
 *
 * Input Parameters:
 *   pool - Address of the memory pool to be used.
 *   blk  - The pointer of memory block.
 ****************************************************************************/

void mempool_free(FAR struct mempool_s *pool, FAR void *blk);

/****************************************************************************
 * Name: mempool_info
 *
 * Description:
 *   mempool_info returns a copy of updated current mempool information.
 *
 * Input Parameters:
 *   pool    - Address of the memory pool to be used.
 *   info    - The pointer of mempoolinfo.
 *
 * Returned Value:
 *   OK on success; A negated errno value on any failure.
 ****************************************************************************/

int mempool_info(FAR struct mempool_s *pool, struct mempoolinfo_s *info);

/****************************************************************************
 * Name: mempool_deinit
 *
 * Description:
 *   Deallocate a memory pool.
 *
 * Input Parameters:
 *   pool    - Address of the memory pool to be used.
 ****************************************************************************/

int mempool_deinit(FAR struct mempool_s *pool);

/****************************************************************************
 * Name: mempool_procfs_register
 *
 * Description:
 *   Add a new mempool entry to the procfs file system.
 *
 * Input Parameters:
 *   entry - Describes the entry to be registered.
 *   name  - The name of mempool.
 *
 ****************************************************************************/

#ifndef CONFIG_FS_PROCFS_EXCLUDE_MEMPOOL
void mempool_procfs_register(FAR struct mempool_procfs_entry_s *entry,
                             FAR const char *name);
#endif

/****************************************************************************
 * Name: mempool_procfs_unregister
 *
 * Description:
 *   Remove a mempool entry from the procfs file system.
 *
 * Input Parameters:
 *   entry - Describes the entry to be unregistered.
 *
 ****************************************************************************/

#ifndef CONFIG_FS_PROCFS_EXCLUDE_MEMPOOL
void mempool_procfs_unregister(FAR struct mempool_procfs_entry_s *entry);
#endif

/****************************************************************************
 * Name: mempool_multiple_init
 *
 * Description:
 *   Initialize multiple memory pool, each element represents a memory pool.
 *   The user needs to specify the initialization information of each mempool
 *   in the array, including blocksize, initialsize, expandsize,
 *   interruptsize, wait. These mempool will be initialized by mempool_init.
 *   The name of all mempool are "name".
 *
 *   This function will initialize the member delta by detecting the
 *   relationship between the each block size of mempool in multiple mempool.
 *
 * Input Parameters:
 *   name  - The name of memory pool.
 *   mpool - The handle of the multiple memory pool to be used.
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int mempool_multiple_init(FAR struct mempool_multiple_s *mpool,
                          FAR const char *name);

/****************************************************************************
 * Name: mempool_multiple_alloc
 *
 * Description:
 *   Allocate an block from specific multiple memory pool.
 *   If the mempool of the corresponding size doesn't have free block,
 *   it will continue to alloc memory for a larger memory pool until last
 *   mempool in multiple mempools.
 *
 * Input Parameters:
 *   mpool - The handle of multiple memory pool to be used.
 *   size  - The size of alloc blk.
 *
 * Returned Value:
 *   The pointer to the allocated block on success; NULL on any failure.
 *
 ****************************************************************************/

FAR void *mempool_multiple_alloc(FAR struct mempool_multiple_s *mpool,
                                 size_t size);

/****************************************************************************
 * Name: mempool_multiple_realloc
 *
 * Description:
 *   Change the size of the block memory pointed to by oldblk to size bytes.
 *
 * Input Parameters:
 *   mpool  - The handle of multiple memory pool to be used.
 *   oldblk - The pointer to change the size of the block memory.
 *   size   - The size of alloc blk.
 *
 * Returned Value:
 *   The pointer to the allocated block on success; NULL on any failure.
 *
 ****************************************************************************/

FAR void *mempool_multiple_realloc(FAR struct mempool_multiple_s *mpool,
                                   FAR void *oldblk, size_t size);

/****************************************************************************
 * Name: mempool_multiple_free
 *
 * Description:
 *   Release an memory block to the multiple mempry pool. The blk must have
 *   been returned by a previous call to mempool_multiple_alloc.
 *
 * Input Parameters:
 *   mpool - The handle of multiple memory pool to be used.
 *   blk  - The pointer of memory block.
 ****************************************************************************/

void mempool_multiple_free(FAR struct mempool_multiple_s *mpool,
                           FAR void *blk);

/****************************************************************************
 * Name: mempool_multiple_alloc_size
 *
 * Description:
 *   Get size of memory block from multiple memory.
 *
 * Input Parameters:
 *   blk  - The pointer of memory block.
 *
 * Returned Value:
 *   The size of memory block.
 *
 ****************************************************************************/

size_t mempool_multiple_alloc_size(FAR void *blk);

/****************************************************************************
 * Name: mempool_multiple_fixed_alloc
 *
 * Description:
 *   Allocate an block from specific multiple memory pool.
 *   If the mempool of the corresponding size doesn't have free block,
 *   then wait until free happened or return NULL.
 *
 * Input Parameters:
 *   mpool - The handle of multiple memory pool to be used.
 *   size  - The size of alloc blk.
 *
 * Returned Value:
 *   The pointer to the allocated block on success; NULL on any failure.
 *
 ****************************************************************************/

FAR void *mempool_multiple_fixed_alloc(FAR struct mempool_multiple_s *mpool,
                                       size_t size);

/****************************************************************************
 * Name: mempool_multiple_fixed_realloc
 *
 * Description:
 *   Change the size of the block memory pointed to by oldblk to size bytes.
 *
 * Input Parameters:
 *   mpool   - The handle of multiple memory pool to be used.
 *   oldblk  - The pointer to change the size of the block memory.
 *   oldsize - The size of block memory to oldblk.
 *   size    - The size of alloc blk.
 *
 * Returned Value:
 *   The pointer to the allocated block on success; NULL on any failure.
 *
 ****************************************************************************/

FAR void *
mempool_multiple_fixed_realloc(FAR struct mempool_multiple_s *mpool,
                               FAR void *oldblk, size_t oldsize,
                               size_t size);

/****************************************************************************
 * Name: mempool_multiple_fixed_free
 *
 * Description:
 *   Release an memory block to the multiple mempry pool. The blk must have
 *   been returned by a previous call to mempool_multiple_fixed_alloc.
 *
 * Input Parameters:
 *   mpool - The handle of multiple memory pool to be used.
 *   blk   - The pointer of memory block.
 *   size  - The size of alloc blk.
 ****************************************************************************/

void mempool_multiple_fixed_free(FAR struct mempool_multiple_s *mpool,
                                 FAR void *blk, size_t size);

/****************************************************************************
 * Name: mempool_multiple_deinit
 *
 * Description:
 *   Deallocate multiple memory pool.
 *
 * Input Parameters:
 *   mpool - The handle of multiple memory pool to be used.
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int mempool_multiple_deinit(FAR struct mempool_multiple_s *mpool);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif
