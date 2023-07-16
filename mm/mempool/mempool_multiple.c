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

#include <strings.h>
#include <syslog.h>
#include <sys/param.h>

#include <nuttx/mutex.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/mempool.h>

#include <assert.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

struct mpool_chunk_s
{
  sq_entry_t entry;
  FAR void *start;
  FAR void *next;
  FAR void *end;
  size_t used;
};

struct mempool_multiple_s
{
  FAR struct mempool_s         *pools;       /* The memory pool array */
  size_t                        npools;      /* The number of memory pool array elements */
  size_t                        expandsize;  /* The number not will use it to init erery
                                              * pool expandsize
                                              */
  size_t                        minpoolsize; /* The number is align for each memory pool */
  FAR void                     *arg;         /* This pointer is used to store the user's
                                              * private data
                                              */
  mempool_multiple_alloc_t      alloc;       /* The alloc function for mempool */
  mempool_multiple_alloc_size_t alloc_size;  /* Get the address size of the
                                              * alloc function
                                              */
  mempool_multiple_free_t       free;        /* The free function for mempool */
  size_t                        alloced;     /* Total size of alloc */

  /* This delta describes the relationship between the block size of each
   * mempool in multiple mempool by user initialized. It is automatically
   * detected by the mempool_multiple_init function. If the delta is not
   * equal to 0, the block size of the pool in the multiple mempool is an
   * arithmetic progressions, otherwise it is an increasing progressions.
   */

  size_t                        delta;

  /* It is used to record the information recorded by the mempool during
   * expansion, and find the mempool by adding an index
   */

  mutex_t                       lock;
  sq_queue_t                    chunk_queue;
  size_t                        chunk_size;
  size_t                        dict_used;
  size_t                        dict_col_num_log2;
  size_t                        dict_row_num;
  FAR struct mpool_dict_s     **dict;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline FAR struct mempool_s *
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

static FAR void *
mempool_multiple_alloc_chunk(FAR struct mempool_multiple_s *mpool,
                             size_t align, size_t size)
{
  FAR struct mpool_chunk_s *chunk;
  FAR char *tmp;
  FAR void *ret;

  if (mpool->chunk_size < mpool->expandsize)
    {
      ret = mpool->alloc(mpool->arg, align, size);
      if (ret)
        {
          mpool->alloced += mpool->alloc_size(mpool->arg, ret);
        }

      return ret;
    }

  chunk = (FAR struct mpool_chunk_s *)sq_peek(&mpool->chunk_queue);
  if (chunk == NULL)
    {
retry:
      tmp = mpool->alloc(mpool->arg, mpool->expandsize,
                         mpool->chunk_size +
                         sizeof(struct mpool_chunk_s));

      if (tmp == NULL)
        {
          return NULL;
        }

      mpool->alloced += mpool->alloc_size(mpool->arg, tmp);
      chunk = (FAR struct mpool_chunk_s *)(tmp + mpool->chunk_size);
      chunk->end = tmp + mpool->chunk_size;
      chunk->start = tmp;
      chunk->next = tmp;
      chunk->used = 0;
      sq_addfirst(&chunk->entry, &mpool->chunk_queue);
    }

  ret = (FAR void *)ALIGN_UP(chunk->next, align);
  if ((uintptr_t)chunk->end - (uintptr_t)ret < size)
    {
      goto retry;
    }

  chunk->used++;
  chunk->next = (FAR char *)ret + size;
  return ret;
}

static void
mempool_multiple_free_chunk(FAR struct mempool_multiple_s *mpool,
                            FAR void *ptr)
{
  FAR struct mpool_chunk_s *chunk;
  FAR sq_entry_t *entry;

  if (mpool->chunk_size < mpool->expandsize)
    {
      mpool->free(mpool->arg, ptr);
      return;
    }

  nxmutex_lock(&mpool->lock);
  sq_for_every(&mpool->chunk_queue, entry)
    {
      chunk = (FAR struct mpool_chunk_s *)entry;
      if (ptr >= chunk->start && ptr < chunk->next)
        {
          if (--chunk->used == 0)
            {
              sq_rem(&chunk->entry, &mpool->chunk_queue);
              mpool->free(mpool->arg, chunk->start);
            }

          break;
        }
    }

  nxmutex_unlock(&mpool->lock);
}

static FAR void *mempool_multiple_alloc_callback(FAR struct mempool_s *pool,
                                                 size_t size)
{
  FAR struct mempool_multiple_s *mpool = pool->priv;
  FAR void *ret;
  size_t row;
  size_t col;

  nxmutex_lock(&mpool->lock);
  ret = mempool_multiple_alloc_chunk(mpool, mpool->expandsize,
                                     mpool->minpoolsize + size);
  if (ret == NULL)
    {
      nxmutex_unlock(&mpool->lock);
      return NULL;
    }

  row = mpool->dict_used >> mpool->dict_col_num_log2;

  /* There is no new pointer address to store the dictionarys */

  DEBUGASSERT(mpool->dict_row_num > row);

  col = mpool->dict_used - (row << mpool->dict_col_num_log2);

  if (mpool->dict[row] == NULL)
    {
      mpool->dict[row] =
        mempool_multiple_alloc_chunk(mpool, sizeof(uintptr_t),
                                     (1 << mpool->dict_col_num_log2)
                                     * sizeof(struct mpool_dict_s));
    }

  mpool->dict[row][col].pool = pool;
  mpool->dict[row][col].addr = ret;
  mpool->dict[row][col].size = mpool->minpoolsize + size;
  *(FAR size_t *)ret = mpool->dict_used++;
  nxmutex_unlock(&mpool->lock);
  return (FAR char *)ret + mpool->minpoolsize;
}

static void mempool_multiple_free_callback(FAR struct mempool_s *pool,
                                           FAR void *addr)
{
  FAR struct mempool_multiple_s *mpool = pool->priv;

  mempool_multiple_free_chunk(mpool,
                              (FAR char *)addr - mpool->minpoolsize);
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
  size_t row;
  size_t col;

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

  row = index >> mpool->dict_col_num_log2;
  col = index - (row << mpool->dict_col_num_log2);
  if (mpool->dict[row] == NULL ||
      mpool->dict[row][col].addr != addr ||
      (FAR char *)blk - (FAR char *)addr >= mpool->dict[row][col].size)
    {
      return NULL;
    }

  return &mpool->dict[row][col];
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

FAR struct mempool_multiple_s *
mempool_multiple_init(FAR const char *name,
                      FAR size_t *poolsize, size_t npools,
                      mempool_multiple_alloc_t alloc,
                      mempool_multiple_alloc_size_t alloc_size,
                      mempool_multiple_free_t free, FAR void *arg,
                      size_t chunksize, size_t expandsize,
                      size_t dict_expendsize)
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

  mpool->alloc_size = alloc_size;
  mpool->expandsize = expandsize;
  mpool->chunk_size = chunksize;
  mpool->alloc = alloc;
  mpool->free = free;
  mpool->arg = arg;
  mpool->alloced = alloc_size(arg, mpool);
  sq_init(&mpool->chunk_queue);
  pools = mempool_multiple_alloc_chunk(mpool, sizeof(uintptr_t),
                                       npools * sizeof(struct mempool_s));
  if (pools == NULL)
    {
      goto err_with_mpool;
    }

  mpool->pools = pools;
  mpool->npools = npools;
  mpool->minpoolsize = minpoolsize;
  mpool->delta = 0;

  for (i = 0; i < npools; i++)
    {
      pools[i].blocksize = poolsize[i];
      pools[i].expandsize = expandsize - mpool->minpoolsize;
      pools[i].initialsize = 0;
      pools[i].interruptsize = 0;
      pools[i].priv = mpool;
      pools[i].alloc = mempool_multiple_alloc_callback;
      pools[i].free = mempool_multiple_free_callback;
#if CONFIG_MM_BACKTRACE >= 0
      pools[i].blockalign = mpool->minpoolsize;
#endif
      ret = mempool_init(pools + i, name);
      if (ret < 0)
        {
          goto err_with_pools;
        }

      if (i + 1 != npools)
        {
          size_t delta = poolsize[i + 1] - poolsize[i];

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

  mpool->dict_used = 0;
  mpool->dict_col_num_log2 = fls(dict_expendsize /
                                 sizeof(struct mpool_dict_s));

  mpool->dict_row_num = dict_expendsize / sizeof(FAR struct mpool_dict_s *);
  mpool->dict = mempool_multiple_alloc_chunk(
                mpool, sizeof(FAR struct mpool_dict_s *),
                sizeof(FAR struct mpool_dict_s *) * mpool->dict_row_num);
  if (mpool->dict == NULL)
    {
      goto err_with_pools;
    }

  memset(mpool->dict, 0,
         mpool->dict_row_num * sizeof(FAR struct mpool_dict_s *));
  nxmutex_init(&mpool->lock);

  return mpool;

err_with_pools:
  while (--i >= 0)
    {
      mempool_deinit(pools + i);
    }

  mempool_multiple_free_chunk(mpool, pools);
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
                           MEMPOOL_REALBLOCKSIZE(dict->pool));
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
 * Name: mempool_multiple_foreach
 ****************************************************************************/

void mempool_multiple_foreach(FAR struct mempool_multiple_s *mpool,
                              mempool_multiple_foreach_t handle,
                              FAR void *arg)
{
  size_t i;
  for (i = 0; i < mpool->npools; i++)
    {
      handle(mpool->pools + i, arg);
    }
}

/****************************************************************************
 * Name: mempool_multiple_mallinfo
 ****************************************************************************/

struct mallinfo
mempool_multiple_mallinfo(FAR struct mempool_multiple_s *mpool)
{
  struct mallinfo info;
  size_t i;

  memset(&info, 0, sizeof(struct mallinfo));

  nxmutex_lock(&mpool->lock);
  info.arena = mpool->alloced;

  if (mpool->chunk_size >= mpool->expandsize)
    {
      FAR struct mpool_chunk_s *chunk;

      chunk = (FAR struct mpool_chunk_s *)sq_peek(&mpool->chunk_queue);
      info.fordblks += (uintptr_t)chunk->end - (uintptr_t)chunk->next;
    }

  nxmutex_unlock(&mpool->lock);

  for (i = 0; i < mpool->npools; i++)
    {
      struct mempoolinfo_s poolinfo;

      mempool_info(mpool->pools + i, &poolinfo);
      info.fordblks += (poolinfo.ordblks + poolinfo.iordblks)
                       * poolinfo.sizeblks;
      info.ordblks += poolinfo.ordblks + poolinfo.iordblks;
      info.aordblks += poolinfo.aordblks;
      if (info.mxordblk < poolinfo.sizeblks)
        {
          info.mxordblk = poolinfo.sizeblks;
        }
    }

  info.uordblks += mpool->alloced - info.fordblks;
  return info;
}

/****************************************************************************
 * Name: mempool_multiple_info_task
 ****************************************************************************/

struct mallinfo_task
mempool_multiple_info_task(FAR struct mempool_multiple_s *mpool,
                           FAR const struct malltask *task)
{
  int i;
  struct mallinfo_task info;
  struct mallinfo_task ret =
    {
      0, 0
    };

  for (i = 0; i < mpool->npools; i++)
    {
      info = mempool_info_task(mpool->pools + i, task);
      ret.aordblks += info.aordblks;
      ret.uordblks += info.uordblks;
    }

  return ret;
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
 *   dump  - The information of what need dump.
 *
 ****************************************************************************/

void mempool_multiple_memdump(FAR struct mempool_multiple_s *mpool,
                              FAR const struct mm_memdump_s *dump)
{
  size_t i;

  for (i = 0; i < mpool->npools; i++)
    {
      mempool_memdump(mpool->pools + i, dump);
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

  for (i = 0; i < mpool->dict_row_num; i++)
    {
      if (mpool->dict[i] != NULL)
        {
          mempool_multiple_free_chunk(mpool, mpool->dict[i]);
        }
      else
        {
          break;
        }
    }

  mempool_multiple_free_chunk(mpool, mpool->dict);
  mempool_multiple_free_chunk(mpool, mpool->pools);
  nxmutex_destroy(&mpool->lock);
  mpool->free(mpool, mpool);
}
