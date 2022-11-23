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

#define MIN(a, b)             ((a) < (b) ? (a) : (b))
#undef  ALIGN_UP
#define ALIGN_UP(x, a)        ((((size_t)x) + ((a) - 1)) & (~((a) - 1)))
#undef  ALIGN_DOWN
#define ALIGN_DOWN(x, a)      ((size_t)(x) & (~((a) - 1)))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mpool_dict_s
{
  FAR struct mempool_s *pool; /* Record pool when expanding */
  FAR void             *addr; /* Record expand memary address */
  size_t                size; /* Record expand memary size */
};

struct mempool_multiple_s
{
  FAR struct mempool_s *   pools;       /* The memory pool array */
  size_t                   npools;      /* The number of memory pool array elements */
  size_t                   expandsize;  /* The number not will use it to init erery
                                         * pool expandsize
                                         */
  size_t                   minpoolsize; /* The number is align for each memory pool */
  FAR void                *arg;         /* This pointer is used to store the user's
                                         * private data
                                         */
  mempool_multiple_alloc_t alloc;       /* The alloc function for mempool */
  mempool_multiple_free_t  free;        /* The free function for mempool */

  /* This delta describes the relationship between the block size of each
   * mempool in multiple mempool by user initialized. It is automatically
   * detected by the mempool_multiple_init function. If the delta is not
   * equal to 0, the block size of the pool in the multiple mempool is an
   * arithmetic progressions, otherwise it is an increasing progressions.
   */

  size_t                   delta;

  /* It is used to record the information recorded by the mempool during
   * expansion, and find the mempool by adding an index
   */

  size_t                   dict_alloc;
  size_t                   dict_used;
  FAR struct mpool_dict_s *dict;
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
      if (left >= size)
        {
          return &mpool->pools[0];
        }

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

static FAR void *mempool_multiple_alloc_callback(FAR struct mempool_s *pool,
                                                 size_t size)
{
  FAR struct mempool_multiple_s *mpool = pool->priv;
  FAR void *ret;

  if (mpool->dict_used >= mpool->dict_alloc)
    {
      ret = mpool->alloc(mpool->arg, sizeof(uintptr_t),
                         mpool->dict_alloc *
                         sizeof(struct mpool_dict_s) * 2);
      if (ret == NULL)
        {
          return NULL;
        }

      memcpy(ret, mpool->dict,
             mpool->dict_alloc * sizeof(struct mpool_dict_s));
      mpool->free(mpool->arg, mpool->dict);
      mpool->dict = ret;
      mpool->dict_alloc *= 2;
    }

  ret = mpool->alloc(mpool->arg, mpool->expandsize,
                     mpool->minpoolsize + size);
  if (ret == NULL)
    {
      return NULL;
    }

  mpool->dict[mpool->dict_used].pool = pool;
  mpool->dict[mpool->dict_used].addr = ret;
  mpool->dict[mpool->dict_used].size = mpool->minpoolsize + size;
  *(FAR size_t *)ret = mpool->dict_used++;
  return (FAR char *)ret + mpool->minpoolsize;
}

static void mempool_multiple_free_callback(FAR struct mempool_s *pool,
                                           FAR void *addr)
{
  FAR struct mempool_multiple_s *mpool = pool->priv;

  return mpool->free(mpool->arg, (FAR char *)addr - mpool->minpoolsize);
}

/****************************************************************************
 * Name: mempool_multiple_get_dict
 *
 * Description:
 *   Obtain the dict through mpool and blk
 *
 * Input Parameters:
 *   mpool - The handle of the multiple memory pool to be used.
 *   blk   - The pointer of memory block.
 *
 * Returned Value:
 *   Address of the dict to be used or NULL is not find.
 *
 ****************************************************************************/

static FAR struct mpool_dict_s *
mempool_multiple_get_dict(FAR struct mempool_multiple_s *mpool,
                          FAR void *blk)
{
  FAR void *addr;
  size_t index;

  if (mpool == NULL || blk == NULL)
    {
      return NULL;
    }

  addr = (FAR void *)ALIGN_DOWN(blk, mpool->expandsize);

  index = *(FAR size_t *)addr;
  if (index >= mpool->dict_used)
    {
      return NULL;
    }

  if (mpool->dict[index].addr != addr ||
      blk - addr >= mpool->dict[index].size)
    {
      return NULL;
    }

  return &mpool->dict[index];
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
  size_t maxpoolszie;
  size_t minpoolsize;
  int ret;
  int i;

  if (expandsize & (expandsize - 1))
    {
      return NULL;
    }

  maxpoolszie = poolsize[0];
  minpoolsize = poolsize[0];
  for (i = 0; i < npools; i++)
    {
      if (maxpoolszie < poolsize[i])
        {
          maxpoolszie = poolsize[i];
        }

      if (minpoolsize > poolsize[i])
        {
          minpoolsize = poolsize[i];
        }
    }

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
  mpool->expandsize = expandsize;
  mpool->minpoolsize = minpoolsize;
  mpool->alloc = alloc;
  mpool->free = free;
  mpool->arg = arg;

  for (i = 0; i < npools; i++)
    {
      pools[i].blocksize = poolsize[i];
      pools[i].expandsize = expandsize - mpool->minpoolsize;
      pools[i].initialsize = 0;
      pools[i].interruptsize = 0;
      pools[i].priv = mpool;
      pools[i].alloc = mempool_multiple_alloc_callback;
      pools[i].free = mempool_multiple_free_callback;

      ret = mempool_init(pools + i, name);
      if (ret < 0)
        {
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

  mpool->dict_alloc = maxpoolszie / sizeof(struct mpool_dict_s) + 1;
  mpool->dict_used = 0;
  mpool->dict = alloc(arg, sizeof(uintptr_t),
                      mpool->dict_alloc * sizeof(struct mpool_dict_s));
  if (mpool->dict == NULL)
    {
      goto err_with_pools;
    }

  return mpool;

err_with_pools:
  while (--i >= 0)
    {
      mempool_deinit(pools + i);
    }

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
  FAR struct mempool_s *end;
  FAR struct mempool_s *pool;

  if (size < 1)
    {
      return NULL;
    }

  pool = mempool_multiple_find(mpool, size);
  if (pool == NULL)
    {
      return NULL;
    }

  end = mpool->pools + mpool->npools;
  do
    {
      FAR void *blk = mempool_alloc(pool);

      if (blk)
        {
          return blk;
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
  FAR struct mpool_dict_s *dict;
  FAR void *blk;

  if (oldblk == NULL)
    {
      return mempool_multiple_alloc(mpool, size);
    }

  if (size < 1)
    {
      mempool_multiple_free(mpool, oldblk);
      return NULL;
    }

  dict = mempool_multiple_get_dict(mpool, oldblk);
  if (dict == NULL)
    {
      return NULL;
    }

  blk = mempool_multiple_alloc(mpool, size);
  if (blk != NULL && oldblk != NULL)
    {
      size = MIN(size, dict->pool->blocksize);
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
 *
 * Returned Value:
 *   Zero on success; Negative number on any failure.
 *
 ****************************************************************************/

int mempool_multiple_free(FAR struct mempool_multiple_s *mpool,
                          FAR void *blk)
{
  FAR struct mpool_dict_s *dict;

  dict = mempool_multiple_get_dict(mpool, blk);
  if (dict == NULL)
    {
      return -EINVAL;
    }

  blk = (FAR char *)blk - (((FAR char *)blk -
                           ((FAR char *)dict->addr + mpool->minpoolsize)) %
                           dict->pool->blocksize);
  mempool_free(dict->pool, blk);
  return 0;
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
 *   The size of memory block on success. Negative number on any failure.
 *
 ****************************************************************************/

ssize_t mempool_multiple_alloc_size(FAR struct mempool_multiple_s *mpool,
                                    FAR void *blk)
{
  FAR struct mpool_dict_s *dict;

  DEBUGASSERT(blk != NULL);

  dict = mempool_multiple_get_dict(mpool, blk);
  if (dict == NULL)
    {
      return -EINVAL;
    }

  return dict->pool->blocksize;
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
  FAR struct mempool_s *end;
  FAR struct mempool_s *pool;

  DEBUGASSERT((alignment & (alignment - 1)) == 0);

  pool = mempool_multiple_find(mpool, size + alignment);
  if (pool == NULL)
    {
      return NULL;
    }

  end = mpool->pools + mpool->npools;
  do
    {
      FAR char *blk = mempool_alloc(pool);
      if (blk != NULL)
        {
          return (FAR void *)ALIGN_UP(blk, alignment);
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

  mpool->free(mpool->arg, mpool->dict);
  mpool->free(mpool->arg, mpool->pools);
  mpool->free(mpool->arg, mpool);
}
