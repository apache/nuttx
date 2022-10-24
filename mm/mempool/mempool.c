/****************************************************************************
 * mm/mempool/mempool.c
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

#include <stdbool.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mm/mempool.h>

#include "kasan/kasan.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void mempool_add_list(FAR sq_queue_t *list, FAR void *base,
                                    size_t nblks, size_t bsize)
{
  while (nblks-- > 0)
    {
      sq_addfirst(((FAR sq_entry_t *)((FAR char *)base + bsize * nblks)),
                  list);
    }
}

static inline FAR void *mempool_malloc(FAR struct mempool_s *pool,
                                       size_t size)
{
  if (pool->alloc != NULL)
    {
      return pool->alloc(pool, size);
    }
  else
    {
      return kmm_malloc(size);
    }
}

static inline void mempool_mfree(FAR struct mempool_s *pool, FAR void *addr)
{
  if (pool->free != NULL)
    {
      return pool->free(pool, addr);
    }
  else
    {
      return kmm_free(addr);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mempool_init
 *
 * Description:
 *   Initialize a memory pool.
 *   The user needs to specify the initialization information of mempool
 *   including bsize, ninitial, nexpand, ninterrupt.
 *
 * Input Parameters:
 *   pool - Address of the memory pool to be used.
 *   name - The name of memory pool.
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int mempool_init(FAR struct mempool_s *pool, FAR const char *name)
{
  FAR sq_entry_t *base;
  size_t count;

  DEBUGASSERT(pool != NULL && pool->bsize != 0);

  pool->nused = 0;
  sq_init(&pool->list);
  sq_init(&pool->ilist);
  sq_init(&pool->elist);

  count = pool->ninitial + pool->ninterrupt;
  if (count != 0)
    {
      base = mempool_malloc(pool, sizeof(*base) +
                            pool->bsize * count);
      if (base == NULL)
        {
          return -ENOMEM;
        }

      sq_addfirst(base, &pool->elist);
      mempool_add_list(&pool->ilist, base + 1,
                       pool->ninterrupt, pool->bsize);
      mempool_add_list(&pool->list, (FAR char *)(base + 1) +
                       pool->ninterrupt * pool->bsize,
                       pool->ninitial, pool->bsize);
      kasan_poison(base + 1, pool->bsize * count);
    }

  if (pool->wait && pool->nexpand == 0)
    {
      nxsem_init(&pool->waitsem, 0, 0);
    }

#ifndef CONFIG_FS_PROCFS_EXCLUDE_MEMPOOL
  mempool_procfs_register(&pool->procfs, name);
#endif

  return 0;
}

/****************************************************************************
 * Name: mempool_alloc
 *
 * Description:
 *   Allocate an block from a specific memory pool.
 *
 *   If there isn't enough memory blocks, This function will expand memory
 *   pool if nexpand isn't zero.
 *
 * Input Parameters:
 *   pool - Address of the memory pool to be used.
 *
 * Returned Value:
 *   The pointer to the allocated block on success; NULL on any failure.
 *
 ****************************************************************************/

FAR void *mempool_alloc(FAR struct mempool_s *pool)
{
  FAR sq_entry_t *blk;
  irqstate_t flags;

  DEBUGASSERT(pool != NULL);

retry:
  flags = spin_lock_irqsave(&pool->lock);
  blk = sq_remfirst(&pool->list);
  if (blk == NULL)
    {
      if (up_interrupt_context())
        {
          blk = sq_remfirst(&pool->ilist);
          if (blk == NULL)
            {
              goto out_with_lock;
            }
        }
      else
        {
          spin_unlock_irqrestore(&pool->lock, flags);
          if (pool->nexpand != 0)
            {
              blk = mempool_malloc(pool, sizeof(*blk) + pool->bsize *
                                   pool->nexpand);
              if (blk == NULL)
                {
                  return NULL;
                }

              kasan_poison(blk + 1, pool->bsize * pool->nexpand);
              flags = spin_lock_irqsave(&pool->lock);
              sq_addlast(blk, &pool->elist);
              mempool_add_list(&pool->list, blk + 1, pool->nexpand,
                               pool->bsize);
              blk = sq_remfirst(&pool->list);
            }
          else if (!pool->wait ||
                   nxsem_wait_uninterruptible(&pool->waitsem) < 0)
            {
              return NULL;
            }
          else
            {
              goto retry;
            }
        }
    }

  pool->nused++;
  kasan_unpoison(blk, pool->bsize);
out_with_lock:
  spin_unlock_irqrestore(&pool->lock, flags);
  return blk;
}

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

void mempool_free(FAR struct mempool_s *pool, FAR void *blk)
{
  irqstate_t flags;

  DEBUGASSERT(pool != NULL && blk != NULL);

  flags = spin_lock_irqsave(&pool->lock);
  if (pool->ninterrupt)
    {
      FAR char *base;

      base = (FAR char *)(sq_peek(&pool->elist) + 1);
      if ((FAR char *)blk >= base &&
          (FAR char *)blk < base + pool->ninterrupt * pool->bsize)
        {
          sq_addfirst(blk, &pool->ilist);
        }
      else
        {
          sq_addfirst(blk, &pool->list);
        }
    }
  else
    {
      sq_addfirst(blk, &pool->list);
    }

  pool->nused--;
  kasan_poison(blk, pool->bsize);
  spin_unlock_irqrestore(&pool->lock, flags);
  if (pool->wait && pool->nexpand == 0)
    {
      int semcount;

      nxsem_get_value(&pool->waitsem, &semcount);
      if (semcount < 1)
        {
          nxsem_post(&pool->waitsem);
        }
    }
}

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

int mempool_info(FAR struct mempool_s *pool, FAR struct mempoolinfo_s *info)
{
  irqstate_t flags;

  DEBUGASSERT(pool != NULL && info != NULL);

  flags = spin_lock_irqsave(&pool->lock);
  info->ordblks = sq_count(&pool->list);
  info->iordblks = sq_count(&pool->ilist);
  info->aordblks = pool->nused;
  info->arena = (pool->nused + info->ordblks + info->iordblks) * pool->bsize;
  spin_unlock_irqrestore(&pool->lock, flags);
  info->sizeblks = pool->bsize;
  if (pool->wait && pool->nexpand == 0)
    {
      int semcount;

      nxsem_get_value(&pool->waitsem, &semcount);
      info->nwaiter = -semcount;
    }
  else
    {
      info->nwaiter = 0;
    }

  return 0;
}

/****************************************************************************
 * Name: mempool_deinit
 *
 * Description:
 *   Deallocate a memory pool.
 *
 * Input Parameters:
 *   pool    - Address of the memory pool to be used.
 ****************************************************************************/

int mempool_deinit(FAR struct mempool_s *pool)
{
  FAR sq_entry_t *blk;

  DEBUGASSERT(pool != NULL);

  if (pool->nused != 0)
    {
      return -EBUSY;
    }

#ifndef CONFIG_FS_PROCFS_EXCLUDE_MEMPOOL
  mempool_procfs_unregister(&pool->procfs);
#endif

  while ((blk = sq_remfirst(&pool->elist)) != NULL)
    {
      kasan_unpoison(blk, mm_malloc_size(blk));
      mempool_mfree(pool, blk);
    }

  if (pool->wait && pool->nexpand == 0)
    {
      nxsem_destroy(&pool->waitsem);
    }

  return 0;
}
