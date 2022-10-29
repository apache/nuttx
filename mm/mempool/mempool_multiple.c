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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline struct mempool_s *
mempool_multiple_find(FAR struct mempool_multiple_s *mpool, size_t size)
{
  size_t right = mpool->npools;
  size_t left = 0;
  size_t mid;

  while (left < right)
    {
      mid = (left + right) >> 1;
      if (mpool->pools[mid].bsize > size)
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
 *   in the array, including bsize, ninitial, nexpand, ninterrupt, wait.
 *   These mempool will be initialized by mempool_init. The name of all
 *   mempool are "name".
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
                          FAR const char *name)
{
  int i;

  DEBUGASSERT(mpool != NULL && mpool->pools != NULL);

  for (i = 0; i < mpool->npools; i++)
    {
      int ret = mempool_init(mpool->pools + i, name);
      if (ret < 0)
        {
          while (--i >= 0)
            {
              mempool_deinit(mpool->pools + i);
            }

          return ret;
        }
    }

  return 0;
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
      memcpy(blk, oldblk, MIN(oldpool->bsize, size));
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
  FAR void *mem;

  DEBUGASSERT(mpool != NULL && blk != NULL);

  mem = (FAR char *)blk - SIZEOF_HEAD;
  pool = *(FAR struct mempool_s **)mem;
  mempool_free(pool, mem);
}

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

size_t mempool_multiple_alloc_size(FAR void *blk)
{
  FAR struct mempool_s *pool;
  FAR void *mem;

  DEBUGASSERT(blk != NULL);

  mem = (FAR char *)blk - SIZEOF_HEAD;
  pool = *(FAR struct mempool_s **)mem;
  return pool->bsize;
}

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
                                       size_t size)
{
  FAR struct mempool_s *pool;

  pool = mempool_multiple_find(mpool, size);
  if (pool == NULL)
    {
      return NULL;
    }

  return mempool_alloc(pool);
}

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
                               FAR void *oldblk, size_t oldsize, size_t size)
{
  FAR void *blk;

  if (size < 1)
    {
      mempool_multiple_fixed_free(mpool, oldblk, oldsize);
      return NULL;
    }

  blk = mempool_multiple_fixed_alloc(mpool, size);
  if (blk != NULL && oldblk != NULL)
    {
      memcpy(blk, oldblk, MIN(oldsize, size));
      mempool_multiple_fixed_free(mpool, oldblk, oldsize);
    }

  return blk;
}

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
                                 FAR void *blk, size_t size)
{
  FAR struct mempool_s *pool;

  DEBUGASSERT(mpool != NULL && blk != NULL);

  pool = mempool_multiple_find(mpool, size);
  DEBUGASSERT(pool != NULL);
  mempool_free(pool, blk);
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
 * Returned Value:
 *   Zero on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int mempool_multiple_deinit(FAR struct mempool_multiple_s *mpool)
{
  int i;

  DEBUGASSERT(mpool != NULL);

  for (i = 0; i < mpool->npools; i++)
    {
      if (mpool->pools[i].nused != 0)
        {
          return -EBUSY;
        }
    }

  for (i = 0; i < mpool->npools; i++)
    {
      mempool_deinit(mpool->pools + i);
    }

  return 0;
}
