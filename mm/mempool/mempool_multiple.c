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


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline struct mempool_s *
mempool_multiple_find(FAR struct mempool_multiple_s *mpool, size_t size)
{
  FAR struct mempool_s *low = mpool->pools;
  FAR struct mempool_s *mid;
  size_t n = mpool->npools;

  while (1)
    {
      n >>= 1;
      mid = low + n;
      if (size > mid->bsize)
        {
          if (n == 0)
            {
              return NULL;
            }

          low = ++mid;
        }
      else if (n == 0 || size == mid->bsize)
        {
          return mid;
        }
    }
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

  if (mpool == NULL || mpool->pools == NULL)
    {
      return -EINVAL;
    }

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
  FAR struct mempool_s *pool;

  pool = mempool_multiple_find(mpool, size + SIZEOF_HEAD);
  if (pool != NULL)
    {
      do
        {
          FAR void *blk = mempool_alloc(pool);
          if (blk != NULL)
            {
              *(FAR struct mempool_s **)blk = pool;
              return (FAR char *)blk + SIZEOF_HEAD;
            }
        }
      while (++pool< mpool->pools + mpool->npools);
    }

  return NULL;
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

  if (blk != NULL)
    {
      mem = (FAR char *)blk - SIZEOF_HEAD;
      pool = *(FAR struct mempool_s **)mem;

      mempool_free(pool, mem);
    }
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
  if (pool != NULL)
    {
      return mempool_alloc(pool);
    }

  return NULL;
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
  if (blk != NULL)
    {
      FAR struct mempool_s *pool = mempool_multiple_find(mpool, size);
      if (pool != NULL)
        {
          mempool_free(pool, blk);
        }
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
 * Returned Value:
 *   Zero on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int mempool_multiple_deinit(FAR struct mempool_multiple_s *mpool)
{
  int i;

  if (mpool == NULL)
    {
      return -EINVAL;
    }

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
