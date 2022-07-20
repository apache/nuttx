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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mempool_init
 *
 * Description:
 *   Initialize a memory pool.
 *
 * Input Parameters:
 *   pool       - Address of the memory pool to be used.
 *   bsize      - The block size of memory blocks in pool.
 *   ninitial   - The initial count of memory blocks in pool.
 *   nexpand    - The increment count of memory blocks in pool.
 *                If there is not enough memory blocks and it isn't zero,
 *                mempool_alloc will alloc nexpand memory blocks.
 *   ninterrupt - The block count of memory blocks in pool for interrupt
 *                context. These blocks only can use in interrupt context.
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int mempool_init(FAR struct mempool_s *pool, size_t bsize, size_t ninitial,
                 size_t nexpand, size_t ninterrupt)
{
  size_t count = ninitial + ninterrupt;

  if (pool == NULL || bsize == 0)
    {
      return -EINVAL;
    }

  pool->nused = 0;
  pool->bsize = bsize;
  pool->nexpand = nexpand;
  pool->ninterrupt = ninterrupt;
  sq_init(&pool->list);
  sq_init(&pool->ilist);
  sq_init(&pool->elist);
  if (count != 0)
    {
      FAR sq_entry_t *base;

      base = kmm_malloc(sizeof(*base) + bsize * count);
      if (base == NULL)
        {
          return -ENOMEM;
        }

      sq_addfirst(base, &pool->elist);
      mempool_add_list(&pool->ilist, base + 1, ninterrupt, bsize);
      mempool_add_list(&pool->list, (FAR char *)(base + 1) +
                                    ninterrupt * bsize, ninitial, bsize);
    }

  nxsem_init(&pool->wait, 0, 0);
  nxsem_set_protocol(&pool->wait, SEM_PRIO_NONE);

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

  if (pool == NULL)
    {
      return NULL;
    }

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
              blk = kmm_malloc(sizeof(*blk) + pool->bsize * pool->nexpand);
              if (blk == NULL)
                {
                  return NULL;
                }

              flags = spin_lock_irqsave(&pool->lock);
              sq_addlast(blk, &pool->elist);
              mempool_add_list(&pool->list, blk + 1, pool->nexpand,
                               pool->bsize);
              blk = sq_remfirst(&pool->list);
            }
          else if (nxsem_wait_uninterruptible(&pool->wait) < 0)
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
  FAR char *base;

  if (blk == NULL || pool == NULL)
    {
      return;
    }

  flags = spin_lock_irqsave(&pool->lock);
  base = (FAR char *)(sq_peek(&pool->elist) + 1);
  if (pool->ninterrupt && (FAR char *)blk >= base &&
      (FAR char *)blk < base + pool->ninterrupt * pool->bsize)
    {
      sq_addfirst(blk, &pool->ilist);
    }
  else
    {
      sq_addfirst(blk, &pool->list);
    }

  pool->nused--;
  spin_unlock_irqrestore(&pool->lock, flags);
  if (pool->nexpand == 0)
    {
      int semcount;

      nxsem_get_value(&pool->wait, &semcount);
      if (semcount < 1)
        {
          nxsem_post(&pool->wait);
        }
    }
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

  if (pool == NULL)
    {
      return -EINVAL;
    }

  if (pool->nused != 0)
    {
      return -EBUSY;
    }

  while ((blk = sq_remfirst(&pool->elist)) != NULL)
    {
      kmm_free(blk);
    }

  nxsem_destroy(&pool->wait);
  return 0;
}
