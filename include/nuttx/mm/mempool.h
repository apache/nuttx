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
#include <nuttx/queue.h>
#include <nuttx/mm/mm.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/spinlock.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_MM_BACKTRACE >= 0
#  define MEMPOOL_REALBLOCKSIZE(pool) (ALIGN_UP((pool)->blocksize + \
                                       sizeof(struct mempool_backtrace_s), \
                                       (pool)->blockalign))
#else
#  define MEMPOOL_REALBLOCKSIZE(pool) ((pool)->blocksize)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mempool_s;
typedef CODE FAR void *(*mempool_alloc_t)(FAR struct mempool_s *pool,
                                          size_t size);
typedef CODE void (*mempool_free_t)(FAR struct mempool_s *pool,
                                    FAR void *addr);

typedef CODE FAR void *(*mempool_multiple_alloc_t)(FAR void *arg,
                                                   size_t alignment,
                                                   size_t size);
typedef CODE void (*mempool_multiple_free_t)(FAR void *arg, FAR void *addr);
typedef CODE size_t (*mempool_multiple_alloc_size_t)(FAR void *arg,
                                                     FAR void *addr);

typedef CODE void (mempool_multiple_foreach_t)(FAR struct mempool_s *pool,
                                               FAR void *arg);

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMPOOL)
struct mempool_procfs_entry_s
{
  FAR const char *name;
  FAR struct mempool_procfs_entry_s *next;
#if CONFIG_MM_BACKTRACE >= 0

  /* This is dynamic control flag whether to turn on backtrace in the heap,
   * you can set it by /proc/mempool.
   */

  bool backtrace;
#endif
};
#endif

/* This structure describes memory buffer pool */

struct mempool_s
{
  size_t     blocksize;     /* The size for every block in mempool */
#if CONFIG_MM_BACKTRACE >= 0
  size_t     blockalign;    /* The alignment of the blocksize */
#endif
  size_t     initialsize;   /* The initialize size in normal mempool */
  size_t     interruptsize; /* The initialize size in interrupt mempool */
  size_t     expandsize;    /* The size of expand block every time for mempool */
  bool       wait;          /* The flag of need to wait when mempool is empty */
  FAR void  *priv;          /* This pointer is used to store the user's private data */
  mempool_alloc_t alloc;    /* The alloc function for mempool */
  mempool_free_t  free;     /* The free function for mempool */

  /* Private data for memory pool */

  FAR char  *ibase;   /* The inerrupt mempool base pointer */
  sq_queue_t queue;   /* The free block queue in normal mempool */
  sq_queue_t iqueue;  /* The free block queue in interrupt mempool */
  sq_queue_t equeue;  /* The expand block queue for normal mempool */
#if CONFIG_MM_BACKTRACE >= 0
  struct list_node alist;     /* The used block list in mempool */
#else
  size_t     nalloc;    /* The number of used block in mempool */
#endif
  spinlock_t lock;      /* The protect lock to mempool */
  sem_t      waitsem;   /* The semaphore of waiter get free block */
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMPOOL)
  struct mempool_procfs_entry_s procfs; /* The entry of procfs */
#endif
};

#if CONFIG_MM_BACKTRACE >= 0
struct mempool_backtrace_s
{
  struct list_node node;
  pid_t pid;
  unsigned long seqno; /* The sequence of memory malloc */
#  if CONFIG_MM_BACKTRACE > 0
  FAR void *backtrace[CONFIG_MM_BACKTRACE];
#  endif
};
#endif

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

int mempool_info(FAR struct mempool_s *pool, FAR struct mempoolinfo_s *info);

/****************************************************************************
 * Name: mempool_memdump
 *
 * Description:
 *   mempool_memdump returns a memory info about specified pid of
 *   task/thread. if pid equals -1, this function will dump all allocated
 *   node and output backtrace for every allocated node for this mempool,
 *   if pid equals -2, this function will dump all free node for this
 *   mempool, and if pid is greater than or equal to 0, will dump pid
 *   allocated node and output backtrace.
 *
 * Input Parameters:
 *   pool    - Address of the memory pool to be used.
 *   dump    - The information of what need dump.
 *
 * Returned Value:
 *   OK on success; A negated errno value on any failure.
 ****************************************************************************/

void mempool_memdump(FAR struct mempool_s *pool,
                     FAR const struct mm_memdump_s *dump);

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
 * Name: mempool_info_task
 *
 * Description:
 *   Get memory pool's memory used info.
 *
 * Input Parameters:
 *   pool    - Address of the memory pool to be used.
 *   task    - The information of what need retrieve.
 *
 * Returned Value:
 *   Statistics of memory information based on dump.
 ****************************************************************************/

struct mallinfo_task
mempool_info_task(FAR struct mempool_s *pool,
                  FAR const struct malltask *task);

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

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMPOOL)
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

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMPOOL)
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
 *   name            - The name of memory pool.
 *   poolsize        - The block size array for pools in multiples pool.
 *   npools          - How many pools in multiples pool.
 *   alloc           - The alloc memory function for multiples pool.
 *   alloc_size      - Get the address size of the alloc function.
 *   free            - The free memory function for multiples pool.
 *   arg             - The alloc & free memory fuctions used arg.
 *   chunksize       - The multiples pool chunk size.
 *   expandsize      - The expend mempry for all pools in multiples pool.
 *   dict_expendsize - The expend size for multiple dictnoary.
 * Returned Value:
 *   Return an initialized multiple pool pointer on success,
 *   otherwise NULL is returned.
 *
 ****************************************************************************/

struct mempool_multiple_s;

FAR struct mempool_multiple_s *
mempool_multiple_init(FAR const char *name,
                      FAR size_t *poolsize, size_t npools,
                      mempool_multiple_alloc_t alloc,
                      mempool_multiple_alloc_size_t alloc_size,
                      mempool_multiple_free_t free, FAR void *arg,
                      size_t chunksize, size_t expandsize,
                      size_t dict_expendsize);

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
 *
 * Returned Value:
 *   Zero on success; Negative number mean the block doesn't come from pool.
 *
 ****************************************************************************/

int mempool_multiple_free(FAR struct mempool_multiple_s *mpool,
                          FAR void *blk);

/****************************************************************************
 * Name: mempool_multiple_alloc_size
 *
 * Description:
 *   Get size of memory block from multiple memory.
 *
 * Input Parameters:
 *   mpool - The handle of multiple memory pool to be used.
 *   blk  - The pointer of memory block.
 *
 * Returned Value:
 *   The size of memory block on success. Negative number mean the block
 *   doesn't come from pool.
 *
 ****************************************************************************/

ssize_t mempool_multiple_alloc_size(FAR struct mempool_multiple_s *mpool,
                                    FAR void *blk);

/****************************************************************************
 * Name: mempool_multiple_memalign
 *
 * Description:
 *   This function requests more than enough space from malloc, finds a
 *   region within that chunk that meets the alignment request.
 *
 *   The alignment argument must be a power of two.
 *
 *   The memalign is special to multiple mempool because multiple mempool
 *   doesn't support split and shrink chunk operate. So When you alloc a
 *   memory block and find an aligned address in this block, you need to
 *   occupy 8 bytes before the address to save the address of the padding
 *   size and pool to ensure correct use in realloc and free operations.
 *   So we will use bit1 in the previous address of the address to represent
 *   that it is applied by memalign.
 *
 * Input Parameters:
 *   mpool     - The handle of multiple memory pool to be used.
 *   alignment - The alignment request of memory block.
 *   size      - The size of alloc blk.
 *
 * Returned Value:
 *   The size of memory block.
 *
 ****************************************************************************/

FAR void *mempool_multiple_memalign(FAR struct mempool_multiple_s *mpool,
                                    size_t alignment, size_t size);

/****************************************************************************
 * Name: mempool_multiple_memdump
 *
 * Description:
 *   mempool_multiple_memdump returns a memory info about specified pid of
 *   task/thread. if pid equals -1, this function will dump all allocated
 *   node and output backtrace for every allocated node for this multiple
 *   mempool, if pid equals -2, this function will dump all free node for
 *   this multiple mempool, and if pid is greater than or equal to 0, will
 *   dump pid allocated node and output backtrace.
 *
 * Input Parameters:
 *   mpool - The handle of multiple memory pool to be used.
 *   dump  - The information of what need dump.
 *
 ****************************************************************************/

void mempool_multiple_memdump(FAR struct mempool_multiple_s *mpool,
                              FAR const struct mm_memdump_s *dump);

/****************************************************************************
 * Name: mempool_multiple_deinit
 *
 * Description:
 *   Deallocate multiple memory pool.
 *
 * Input Parameters:
 *   mpool - The handle of multiple memory pool to be used.
 *
 ****************************************************************************/

void mempool_multiple_deinit(FAR struct mempool_multiple_s *mpool);

/****************************************************************************
 * Name: mempool_multiple_foreach
 * Description:
 *   Traverse mempool under multiple pool to execute handle.
 ****************************************************************************/

void mempool_multiple_foreach(FAR struct mempool_multiple_s *mpool,
                              mempool_multiple_foreach_t handle,
                              FAR void *arg);

/****************************************************************************
 * Name: mempool_multiple_mallinfo
 * Description:
 *   mallinfo returns a copy of updated current multiples pool information.
 ****************************************************************************/

struct mallinfo
mempool_multiple_mallinfo(FAR struct mempool_multiple_s *mpool);

/****************************************************************************
 * Name: mempool_multiple_info_task
 * Description:
 *   Get multiple memory pool's memory used info.
 *
 * Input Parameters:
 *   mpool - The handle of multiple memory pool to be used.
 *   task  - The information of what need retrieve.
 *
 * Returned Value:
 *    Statistics of memory information based on dump.
 ****************************************************************************/

struct mallinfo_task
mempool_multiple_info_task(FAR struct mempool_multiple_s *mpool,
                           FAR const struct malltask *task);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif
