/****************************************************************************
 * mm/mempool/mempool_multiple.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/mm/mempool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SIZEOF_HEAD sizeof(FAR struct mempool_s *)
#define MAX(a, b)   ((a) > (b) ? (a) : (b))
#define MIN(a, b)   ((a) < (b) ? (a) : (b))
#define ALIGN_BIT   (1 << 1)

struct mempool_multiple_s
{
  FAR struct mempool_s *   pools;      /* The memory pool array */
  size_t                   npools;     /* The number of memory pool array elements */

  FAR void                *arg;     /* This pointer is used to store the user's
                                     * private data
                                     */
  mempool_multiple_alloc_t alloc;   /* The alloc function for mempool */
  mempool_multiple_free_t  free;    /* The free function for mempool */

  /* This delta describes the relationship between the block size of each
   * mempool in multiple mempool by user initialized. It is automatically
   * detected by the mempool_multiple_init function. If the delta is not
   * equal to 0, the block size of the pool in the multiple mempool is an
   * arithmetic progressions, otherwise it is an increasing progressions.
   */

  size_t                   delta;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline struct mempool_s *
mempool_multiple_find(FAR struct mempool_multiple_s *mpool, size_t size)
{
  size_t right;
  size_t left = 0;
  size_t mid;

  if (mpool == NULL)
    {
      return NULL;
    }

  right = mpool->npools;
  if (mpool->delta != 0)
    {
      left = mpool->pools[0].blocksize;
      mid = (size - left + mpool->delta - 1) / mpool->delta;
      return mid < right ? &mpool->pools[mid] : NULL;
    }

  while (left < right)
    {
      mid = (left + right) >> 1;
      if (mpool->pools[mid].blocksize > size)
        {
          right = mid;
        }
      else
        {
          left = mid + 1;
        }
    }

  if (left == mpool->npools)
    {
      return NULL;
    }

  return &mpool->pools[left];
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
 *   name        - The name of memory pool.
 *   poolsize    - The block size array for pools in multiples pool.
 *   npools      - How many pools in multiples pool.
 *   alloc       - The alloc memory function for multiples pool.
 *   free        - The free memory function for multiples pool.
 *   arg         - The alloc & free memory fuctions used arg.
 *   expandsize  - The expend mempry for all pools in multiples pool.
 *
 * Returned Value:
 *   Return an initialized multiple pool pointer on success,
 *   otherwise NULL is returned.
 *
 ****************************************************************************/

FAR struct mempool_multiple_s *
mempool_multiple_init(FAR const char *name,
                      FAR size_t *poolsize, size_t npools,
                      mempool_multiple_alloc_t alloc,
                      mempool_multiple_free_t free,
                      FAR void *arg, size_t expandsize)
{
  FAR struct mempool_multiple_s *mpool;
  FAR struct mempool_s *pools;
  int ret;
  int i;

  mpool = alloc(arg, sizeof(uintptr_t), sizeof(struct mempool_multiple_s));
  if (mpool == NULL)
    {
      return NULL;
    }

  pools = alloc(arg, sizeof(uintptr_t),
                npools * sizeof(FAR struct mempool_s));
  if (pools == NULL)
    {
      goto err_with_mpool;
    }

  mpool->pools = pools;
  mpool->npools = npools;
  mpool->alloc = alloc;
  mpool->free = free;
  mpool->arg = arg;

  for (i = 0; i < npools; i++)
    {
      pools[i].blocksize = poolsize[i];
      pools[i].expandsize = expandsize;
      pools[i].initialsize = 0;
      pools[i].interruptsize = 0;
      ret = mempool_init(pools + i, name);
      if (ret < 0)
        {
          while (--i >= 0)
            {
              mempool_deinit(pools + i);
            }

          goto err_with_pools;
        }

      if (i + 1 != npools)
        {
          size_t delta = pools[i + 1].blocksize - pools[i].blocksize;

          if (i == 0)
            {
              mpool->delta = delta;
            }
          else if (delta != mpool->delta)
            {
              mpool->delta = 0;
            }
        }
    }

  return mpool;

err_with_pools:
  free(arg, pools);
err_with_mpool:
  free(arg, mpool);
  return NULL;
}

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
                                 size_t size)
{
  FAR struct mempool_s *end = mpool->pools + mpool->npools;
  FAR struct mempool_s *pool;

  pool = mempool_multiple_find(mpool, size + SIZEOF_HEAD);
  if (pool == NULL)
    {
      return NULL;
    }

  do
    {
      FAR void *blk = mempool_alloc(pool);
      if (blk != NULL)
        {
          *(FAR struct mempool_s **)blk = pool;
          return (FAR char *)blk + SIZEOF_HEAD;
        }
    }
  while (++pool < end);

  return NULL;
}

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
                                   FAR void *oldblk, size_t size)
{
  FAR void *blk;

  if (size < 1)
    {
      mempool_multiple_free(mpool, oldblk);
      return NULL;
    }

  blk = mempool_multiple_alloc(mpool, size);
  if (blk != NULL && oldblk != NULL)
    {
      FAR struct mempool_s *oldpool;

      oldpool = *(FAR struct mempool_s **)
                ((FAR char *)oldblk - SIZEOF_HEAD);
      if ((uintptr_t)oldpool & ALIGN_BIT)
        {
          oldpool = (FAR struct mempool_s *)
                    ((uintptr_t)oldpool & ~ALIGN_BIT);
          size = MIN(size, oldpool->blocksize -
                     *(FAR size_t *)((FAR char *)oldblk - 2 * SIZEOF_HEAD));
        }
      else
        {
          size = MIN(size, oldpool->blocksize - SIZEOF_HEAD);
        }

      memcpy(blk, oldblk, size);
      mempool_multiple_free(mpool, oldblk);
    }

  return blk;
}

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
                           FAR void *blk)
{
  FAR struct mempool_s *pool;
  FAR char *mem;

  DEBUGASSERT(mpool != NULL && blk != NULL);

  mem = (FAR char *)blk - SIZEOF_HEAD;
  pool = *(FAR struct mempool_s **)mem;
  if ((uintptr_t)pool & ALIGN_BIT)
    {
      pool = (FAR struct mempool_s *)((uintptr_t)pool & ~ALIGN_BIT);
      mem = (FAR char *)blk - *(FAR size_t *)(mem - SIZEOF_HEAD);
    }

  mempool_free(pool, mem);
}

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
 *   The size of memory block.
 *
 ****************************************************************************/

size_t mempool_multiple_alloc_size(FAR struct mempool_multiple_s *mpool,
                                   FAR void *blk)
{
  FAR struct mempool_s *pool;
  FAR char *mem;

  DEBUGASSERT(blk != NULL);

  mem = (FAR char *)blk - SIZEOF_HEAD;
  pool = *(FAR struct mempool_s **)mem;
  if ((uintptr_t)pool & ALIGN_BIT)
    {
      pool = (FAR struct mempool_s *)((uintptr_t)pool & ~ALIGN_BIT);
      return pool->blocksize - *(FAR size_t *)(mem - SIZEOF_HEAD);
    }
  else
    {
      return pool->blocksize - SIZEOF_HEAD;
    }
}

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
                                    size_t alignment, size_t size)
{
  FAR struct mempool_s *end = mpool->pools + mpool->npools;
  FAR struct mempool_s *pool;

  DEBUGASSERT((alignment & (alignment - 1)) == 0);

  pool = mempool_multiple_find(mpool, size + alignment + 2 * SIZEOF_HEAD);
  if (pool == NULL)
    {
      return NULL;
    }

  do
    {
      FAR char *blk = mempool_alloc(pool);
      if (blk != NULL)
        {
          FAR char *mem;

          mem = blk + 2 * SIZEOF_HEAD;
          mem = (FAR char *)(((uintptr_t)mem + alignment - 1) &
                             ~(alignment - 1));
          *(FAR uintptr_t *)(mem - SIZEOF_HEAD) =
                                                 (uintptr_t)pool | ALIGN_BIT;
          *(FAR size_t *)(mem - 2 * SIZEOF_HEAD) = mem - blk;
          return mem;
        }
    }
  while (++pool < end);

  return NULL;
}

/****************************************************************************
 * Name: mempool_multiple_info_task
 ****************************************************************************/

void mempool_multiple_info_task(FAR struct mempool_multiple_s *mpool,
                                FAR struct mempoolinfo_task *info)
{
  size_t i;

  for (i = 0; i < mpool->npools; i++)
    {
      mempool_info_task(mpool->pools + i, info);
    }
}

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
 *   pid   - The pid of task.
 *
 ****************************************************************************/

void mempool_multiple_memdump(FAR struct mempool_multiple_s *mpool,
                              pid_t pid)
{
  size_t i;

  for (i = 0; i < mpool->npools; i++)
    {
      mempool_memdump(mpool->pools + i, pid);
    }
}

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

void mempool_multiple_deinit(FAR struct mempool_multiple_s *mpool)
{
  size_t i;

  DEBUGASSERT(mpool != NULL);

  for (i = 0; i < mpool->npools; i++)
    {
      DEBUGVERIFY(mempool_deinit(mpool->pools + i));
    }

  mpool->free(mpool->arg, mpool->pools);
  mpool->free(mpool->arg, mpool);
}
