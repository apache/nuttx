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

static inline void mempool_add_list(FAR struct list_node *list,
                                    FAR void *base, size_t nblks,
                                    size_t blocksize)
{
  while (nblks--)
    {
      list_add_head(list, ((FAR struct list_node *)((FAR char *)base +
                                                    blocksize * nblks)));
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

int mempool_init(FAR struct mempool_s *pool, FAR const char *name)
{
  FAR struct list_node *base;
  size_t ninterrupt;
  size_t ninitial;
  size_t count;

  DEBUGASSERT(pool != NULL && pool->blocksize != 0);

  pool->nused = 0;
  list_initialize(&pool->list);
  list_initialize(&pool->ilist);
  list_initialize(&pool->elist);

  ninitial = pool->initialsize / pool->blocksize;
  ninterrupt = pool->interruptsize / pool->blocksize;
  count = ninitial + ninterrupt;
  if (count != 0)
    {
      base = mempool_malloc(pool, sizeof(*base) +
                            pool->blocksize * count);
      if (base == NULL)
        {
          return -ENOMEM;
        }

      list_add_head(&pool->elist, base);
      mempool_add_list(&pool->ilist, base + 1,
                       ninterrupt, pool->blocksize);
      mempool_add_list(&pool->list, (FAR char *)(base + 1) +
                       ninterrupt * pool->blocksize,
                       ninitial, pool->blocksize);
      kasan_poison(base + 1, pool->blocksize * count);
    }

  if (pool->wait && pool->expandsize == 0)
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
 *   pool if expandsize isn't zero.
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
  FAR struct list_node *blk;
  irqstate_t flags;

  DEBUGASSERT(pool != NULL);

retry:
  flags = spin_lock_irqsave(&pool->lock);
  blk = list_remove_head(&pool->list);
  if (blk == NULL)
    {
      if (up_interrupt_context())
        {
          blk = list_remove_head(&pool->ilist);
          if (blk == NULL)
            {
              goto out_with_lock;
            }
        }
      else
        {
          spin_unlock_irqrestore(&pool->lock, flags);
          if (pool->expandsize != 0)
            {
              size_t nexpand = pool->expandsize / pool->blocksize;
              blk = mempool_malloc(pool, sizeof(*blk) + pool->blocksize *
                                   nexpand);
              if (blk == NULL)
                {
                  return NULL;
                }

              kasan_poison(blk + 1, pool->blocksize * nexpand);
              flags = spin_lock_irqsave(&pool->lock);
              list_add_head(&pool->elist, blk);
              mempool_add_list(&pool->list, blk + 1, nexpand,
                               pool->blocksize);
              blk = list_remove_head(&pool->list);
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
  kasan_unpoison(blk, pool->blocksize);
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
  if (pool->interruptsize != 0)
    {
      FAR char *base;
      size_t ninterrupt;

      base = (FAR char *)(list_peek_head(&pool->elist) + 1);
      ninterrupt = pool->interruptsize / pool->blocksize;
      if ((FAR char *)blk >= base &&
          (FAR char *)blk < base + ninterrupt * pool->blocksize)
        {
          list_add_head(&pool->ilist, blk);
        }
      else
        {
          list_add_head(&pool->list, blk);
        }
    }
  else
    {
      list_add_head(&pool->list, blk);
    }

  pool->nused--;
  kasan_poison(blk, pool->blocksize);
  spin_unlock_irqrestore(&pool->lock, flags);
  if (pool->wait && pool->expandsize == 0)
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

int mempool_info(FAR struct mempool_s *pool, struct mempoolinfo_s *info)
{
  irqstate_t flags;

  DEBUGASSERT(pool != NULL && info != NULL);

  flags = spin_lock_irqsave(&pool->lock);
  info->ordblks = list_length(&pool->list);
  info->iordblks = list_length(&pool->ilist);
  info->aordblks = pool->nused;
  info->arena = (pool->nused + info->ordblks + info->iordblks) *
                pool->blocksize;
  spin_unlock_irqrestore(&pool->lock, flags);
  info->sizeblks = pool->blocksize;
  if (pool->wait && pool->expandsize == 0)
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
  FAR struct list_node *blk;

  DEBUGASSERT(pool != NULL);

  if (pool->nused != 0)
    {
      return -EBUSY;
    }

#ifndef CONFIG_FS_PROCFS_EXCLUDE_MEMPOOL
  mempool_procfs_unregister(&pool->procfs);
#endif

  while ((blk = list_remove_head(&pool->elist)) != NULL)
    {
      kasan_unpoison(blk, mm_malloc_size(blk));
      mempool_mfree(pool, blk);
    }

  if (pool->wait && pool->expandsize == 0)
    {
      nxsem_destroy(&pool->waitsem);
    }

  return 0;
}
